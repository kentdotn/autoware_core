// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

// Integration test of autoware::gnss_poser::GnssPoserNode, the ROS layer of the GNSS poser.
//
// The pose computation itself (GnssPoser) is unit-tested in test_gnss_poser.cpp without ROS. What
// is checked here, over real topics, is what only the node does: declare the parameters, offer the
// topics, hand the inputs to the logic, publish the right topics for each outcome with the right
// headers, and look up the antenna transform.

#include "gnss_poser_node.hpp"

#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Quaternion.hpp>
#include <tf2_ros/buffer.hpp>
#include <tf2_ros/static_transform_broadcaster.hpp>
#include <tf2_ros/transform_broadcaster.hpp>
#include <tf2_ros/transform_listener.hpp>

#include <autoware_internal_debug_msgs/msg/bool_stamped.hpp>
#include <autoware_map_msgs/msg/map_projector_info.hpp>
#include <autoware_sensing_msgs/msg/gnss_ins_orientation_stamped.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_msgs/msg/tf_message.hpp>

#include <gtest/gtest.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <exception>
#include <functional>
#include <iterator>
#include <memory>
#include <optional>
#include <string>
#include <thread>
#include <vector>

namespace
{
using autoware_internal_debug_msgs::msg::BoolStamped;
using autoware_map_msgs::msg::MapProjectorInfo;
using autoware_sensing_msgs::msg::GnssInsOrientationStamped;
using geometry_msgs::msg::Point;
using geometry_msgs::msg::PoseStamped;
using geometry_msgs::msg::PoseWithCovarianceStamped;
using geometry_msgs::msg::Quaternion;
using geometry_msgs::msg::TransformStamped;
using sensor_msgs::msg::NavSatFix;
using sensor_msgs::msg::NavSatStatus;
using tf2_msgs::msg::TFMessage;
using std::chrono_literals::operator""ms;

// Reference input used throughout (same point as autoware_geography_utils' own tests) and its
// MGRS(54SUE) position, from GeographicLib's UTMUPS::Forward modulo 100 km. The MGRS projection
// passes the altitude through as z.
constexpr double reference_latitude = 35.62426;
constexpr double reference_longitude = 139.74252;
constexpr double reference_altitude = 10.0;
constexpr const char * reference_mgrs_grid = "54SUE";
constexpr double reference_x = 86128.181788820;
constexpr double reference_y = 43002.610125367;
// Tolerances for computed values. A behavior change worth catching moves a position by centimeters
// or more and a heading by about 0.001 rad or more; the floating-point noise of the pipeline is
// many orders of magnitude below both. Values the node copies are compared exactly instead.
constexpr double position_tolerance = 1e-4;  // [m]
constexpr double angle_tolerance = 1e-6;     // [rad]

// Wall-clock budget for the loopback delivery of one published message. The test thread pumps the
// executor itself and only ever one input is in flight, so this only has to cover the delivery.
// It also bounds the window in which an output that is *not* expected would have shown up.
constexpr std::chrono::milliseconds delivery_budget{200};
// Wall-clock budget for anything the test actively waits on (an output that must arrive).
constexpr std::chrono::milliseconds wait_budget{3000};
// Wall-clock budget for pub/sub discovery between the peer and the node under test.
constexpr std::chrono::milliseconds discovery_budget{10000};

struct NodeParams
{
  std::string base_frame = "base_link";
  std::string gnss_base_frame = "gnss_base_link";
  std::string map_frame = "map";
  bool use_gnss_ins_orientation = true;
  int gnss_pose_pub_method = 0;
  int buff_epoch = 1;

  [[nodiscard]] std::vector<rclcpp::Parameter> to_parameters() const
  {
    return {
      rclcpp::Parameter("base_frame", base_frame),
      rclcpp::Parameter("gnss_base_frame", gnss_base_frame),
      rclcpp::Parameter("map_frame", map_frame),
      rclcpp::Parameter("use_gnss_ins_orientation", use_gnss_ins_orientation),
      rclcpp::Parameter("gnss_pose_pub_method", gnss_pose_pub_method),
      rclcpp::Parameter("buff_epoch", buff_epoch),
    };
  }

  [[nodiscard]] rclcpp::NodeOptions to_options() const
  {
    return rclcpp::NodeOptions().parameter_overrides(to_parameters());
  }
};

// Node options with every parameter set except `name`.
rclcpp::NodeOptions options_without(const std::string & name)
{
  std::vector<rclcpp::Parameter> parameters = NodeParams{}.to_parameters();
  parameters.erase(
    std::remove_if(
      parameters.begin(), parameters.end(),
      [&name](const rclcpp::Parameter & parameter) { return parameter.get_name() == name; }),
    parameters.end());
  return rclcpp::NodeOptions().parameter_overrides(parameters);
}

// Node options with every parameter set, `parameter` replacing the one of the same name.
rclcpp::NodeOptions options_with(const rclcpp::Parameter & parameter)
{
  std::vector<rclcpp::Parameter> parameters = NodeParams{}.to_parameters();
  std::replace_if(
    parameters.begin(), parameters.end(),
    [&parameter](const rclcpp::Parameter & existing) {
      return existing.get_name() == parameter.get_name();
    },
    parameter);
  return rclcpp::NodeOptions().parameter_overrides(parameters);
}

builtin_interfaces::msg::Time make_stamp(int32_t sec, uint32_t nanosec)
{
  builtin_interfaces::msg::Time stamp;
  stamp.sec = sec;
  stamp.nanosec = nanosec;
  return stamp;
}

// A fix at the reference point. Its frame is base_link itself, so the node needs no antenna
// transform for it unless a case says otherwise.
NavSatFix make_reference_fix(
  NavSatStatus::_status_type status = NavSatStatus::STATUS_FIX,
  const std::string & frame_id = "base_link")
{
  NavSatFix msg;
  msg.header.stamp = make_stamp(1700000000, 123456789U);
  msg.header.frame_id = frame_id;
  msg.status.status = status;
  msg.status.service = NavSatStatus::SERVICE_GPS;
  msg.latitude = reference_latitude;
  msg.longitude = reference_longitude;
  msg.altitude = reference_altitude;
  msg.position_covariance = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
  msg.position_covariance_type = NavSatFix::COVARIANCE_TYPE_DIAGONAL_KNOWN;
  return msg;
}

NavSatFix make_reference_fix_at(const builtin_interfaces::msg::Time & stamp)
{
  NavSatFix fix = make_reference_fix(NavSatStatus::STATUS_FIX, "my_antenna");
  fix.header.stamp = stamp;
  return fix;
}

MapProjectorInfo make_mgrs_projector_info()
{
  MapProjectorInfo msg;
  msg.projector_type = MapProjectorInfo::MGRS;
  msg.vertical_datum = MapProjectorInfo::WGS84;
  msg.mgrs_grid = reference_mgrs_grid;
  return msg;
}

MapProjectorInfo make_local_projector_info()
{
  MapProjectorInfo msg;
  msg.projector_type = MapProjectorInfo::LOCAL;
  msg.vertical_datum = MapProjectorInfo::WGS84;
  return msg;
}

Quaternion yaw_to_quaternion(double yaw)
{
  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, yaw);
  return tf2::toMsg(q);
}

GnssInsOrientationStamped make_orientation(double yaw)
{
  GnssInsOrientationStamped msg;
  msg.header.stamp = make_stamp(1, 0);  // deliberately unrelated to the fix stamp
  msg.header.frame_id = "ins";
  msg.orientation.orientation = yaw_to_quaternion(yaw);
  msg.orientation.rmse_rotation_x = 0.1;
  msg.orientation.rmse_rotation_y = 0.2;
  msg.orientation.rmse_rotation_z = 0.3;
  return msg;
}

Point make_point(double x, double y, double z)
{
  Point p;
  p.x = x;
  p.y = y;
  p.z = z;
  return p;
}

void expect_position_near(const Point & actual, double x, double y, double z)
{
  EXPECT_NEAR(actual.x, x, position_tolerance);
  EXPECT_NEAR(actual.y, y, position_tolerance);
  EXPECT_NEAR(actual.z, z, position_tolerance);
}

void expect_same_rotation(const Quaternion & actual, const Quaternion & expected)
{
  const double dot =
    actual.x * expected.x + actual.y * expected.y + actual.z * expected.z + actual.w * expected.w;
  const double angle = 2.0 * std::acos(std::min(1.0, std::abs(dot)));
  EXPECT_LE(angle, angle_tolerance) << "rotations differ by " << angle << " rad";
}

// The endpoint gnss_poser owns among `infos`, if discovery has seen it yet.
std::optional<rclcpp::TopicEndpointInfo> gnss_poser_endpoint(
  const std::vector<rclcpp::TopicEndpointInfo> & infos)
{
  const auto it = std::find_if(
    infos.begin(), infos.end(), [](const auto & info) { return info.node_name() == "gnss_poser"; });
  return it == infos.end() ? std::nullopt : std::make_optional(*it);
}

// ---------------------------------------------------------------------------------------
// Test double: a plain rclcpp node that plays the role of every peer of gnss_poser
// (map_projection_loader, GNSS driver, INS driver, robot_state_publisher, and the consumers).
// ---------------------------------------------------------------------------------------
class PeerNode : public rclcpp::Node
{
public:
  PeerNode()
  : rclcpp::Node("gnss_poser_integration_peer"),
    tf_buffer_(get_clock()),
    tf_listener_(tf_buffer_, this, /*spin_thread=*/false)
  {
    projector_pub_ = create_publisher<MapProjectorInfo>(
      "/map/map_projector_info", rclcpp::QoS{1}.transient_local());
    fix_pub_ = create_publisher<NavSatFix>("fix", rclcpp::QoS{1});
    orientation_pub_ =
      create_publisher<GnssInsOrientationStamped>("autoware_orientation", rclcpp::QoS{1});

    pose_sub_ = create_subscription<PoseStamped>(
      "gnss_pose", rclcpp::QoS{100},
      [this](const PoseStamped::ConstSharedPtr msg) { poses.push_back(*msg); });
    pose_cov_sub_ = create_subscription<PoseWithCovarianceStamped>(
      "gnss_pose_cov", rclcpp::QoS{100},
      [this](const PoseWithCovarianceStamped::ConstSharedPtr msg) {
        pose_cov_msgs.push_back(*msg);
      });
    fixed_sub_ = create_subscription<BoolStamped>(
      "gnss_fixed", rclcpp::QoS{100},
      [this](const BoolStamped::ConstSharedPtr msg) { fixed_flags.push_back(*msg); });
    tf_sub_ = create_subscription<TFMessage>(
      "/tf", rclcpp::QoS{100}, [this](const TFMessage::ConstSharedPtr msg) {
        // Only keep what gnss_poser broadcasts; this peer publishes on /tf as well.
        std::copy_if(
          msg->transforms.begin(), msg->transforms.end(), std::back_inserter(broadcast_tfs),
          [this](const TransformStamped & t) { return t.child_frame_id != own_dynamic_tf_child_; });
      });

    static_tf_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(*this);
    dynamic_tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(*this);
  }

  [[nodiscard]] bool all_endpoints_matched() const
  {
    return projector_pub_->get_subscription_count() >= 1 &&
           fix_pub_->get_subscription_count() >= 1 &&
           orientation_pub_->get_subscription_count() >= 1 &&
           pose_sub_->get_publisher_count() >= 1 && pose_cov_sub_->get_publisher_count() >= 1 &&
           fixed_sub_->get_publisher_count() >= 1 &&
           // this peer's own dynamic broadcaster + gnss_poser's broadcaster
           tf_sub_->get_publisher_count() >= 2;
  }

  rclcpp::Publisher<MapProjectorInfo>::SharedPtr projector_pub_;
  rclcpp::Publisher<NavSatFix>::SharedPtr fix_pub_;
  rclcpp::Publisher<GnssInsOrientationStamped>::SharedPtr orientation_pub_;
  rclcpp::Subscription<PoseStamped>::SharedPtr pose_sub_;
  rclcpp::Subscription<PoseWithCovarianceStamped>::SharedPtr pose_cov_sub_;
  rclcpp::Subscription<BoolStamped>::SharedPtr fixed_sub_;
  rclcpp::Subscription<TFMessage>::SharedPtr tf_sub_;
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> dynamic_tf_broadcaster_;
  // Observer buffer: used only to know that a TF we broadcast has propagated. It is filled by the
  // test thread pumping this node, not by a dedicated thread, so it must be asked without a
  // timeout: the overloads that take one refuse to answer at all without such a thread.
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  std::string own_dynamic_tf_child_;

  std::vector<PoseStamped> poses;
  std::vector<PoseWithCovarianceStamped> pose_cov_msgs;
  std::vector<BoolStamped> fixed_flags;
  std::vector<TransformStamped> broadcast_tfs;
};

// Drives the node over its real topics from the test thread. There is no background spin on the
// test side: the executor holding both the peer and the node is pumped only from here, and only one
// input is ever in flight, so a scenario's steps reach the node in the order written. (The node's
// own TF listener runs its usual dedicated thread.)
class GnssPoserNodeIntegration : public ::testing::Test
{
protected:
  void SetUp() override
  {
    peer_ = std::make_shared<PeerNode>();
    executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    executor_->add_node(peer_);
  }

  void TearDown() override
  {
    if (node_) {
      executor_->remove_node(node_->get_node_base_interface());
    }
    executor_->remove_node(peer_);
    node_.reset();
    executor_.reset();
    peer_.reset();
  }

  // Creates the node under test and waits for pub/sub discovery to complete.
  void build_node(const NodeParams & params)
  {
    node_ = std::make_shared<autoware::gnss_poser::GnssPoserNode>(params.to_options());
    executor_->add_node(node_->get_node_base_interface());
    ASSERT_TRUE(pump_until([this] { return peer_->all_endpoints_matched(); }, discovery_budget))
      << "pub/sub discovery between the peer and gnss_poser did not complete";
  }

  // build_node() followed by the MGRS projector info, which every case that expects a pose needs.
  void build_ready_node(const NodeParams & params)
  {
    ASSERT_NO_FATAL_FAILURE(build_node(params));
    send_projector_info(make_mgrs_projector_info());
  }

  // Processes whatever is ready on both nodes for `duration` of wall-clock time.
  void pump(std::chrono::milliseconds duration)
  {
    const auto deadline = std::chrono::steady_clock::now() + duration;
    while (std::chrono::steady_clock::now() < deadline) {
      executor_->spin_some();
      std::this_thread::sleep_for(1ms);
    }
  }

  // Pumps until `predicate` holds or `timeout` expires; returns the predicate's final value.
  bool pump_until(
    const std::function<bool()> & predicate, std::chrono::milliseconds timeout = wait_budget)
  {
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (std::chrono::steady_clock::now() < deadline) {
      executor_->spin_some();
      if (predicate()) {
        return true;
      }
      std::this_thread::sleep_for(1ms);
    }
    return predicate();
  }

  // Inputs. Each helper publishes one message and pumps for the delivery budget, so that the next
  // step of a scenario starts with this input delivered; none of them has an acknowledgement.
  void send_projector_info(const MapProjectorInfo & info)
  {
    peer_->projector_pub_->publish(info);
    pump(delivery_budget);
  }

  void send_orientation(const GnssInsOrientationStamped & msg)
  {
    peer_->orientation_pub_->publish(msg);
    pump(delivery_budget);
  }

  void send_fix(const NavSatFix & fix)
  {
    peer_->fix_pub_->publish(fix);
    pump(delivery_budget);
  }

  // Waits until `gnss_fixed` has been received `count` times. The node publishes it for every fix
  // that passes the projector gates, fixed or not, so it marks that fix as processed. Then pumps
  // for one more delivery budget so that the remaining outputs of the same callback, if any, have
  // landed before a count is asserted.
  void wait_for_gnss_fixed(std::size_t count)
  {
    ASSERT_TRUE(pump_until([this, count] { return peer_->fixed_flags.size() >= count; }))
      << "expected " << count << " gnss_fixed messages, got " << peer_->fixed_flags.size();
    pump(delivery_budget);
  }

  // Waits until every output of an accepted fix (gnss_fixed, gnss_pose, gnss_pose_cov and the
  // TF broadcast) has been received at least `count` times. The four topics arrive in no
  // particular order, so they are waited for together.
  void wait_for_outputs(std::size_t count)
  {
    ASSERT_TRUE(pump_until([this, count] {
      return peer_->fixed_flags.size() >= count && peer_->poses.size() >= count &&
             peer_->pose_cov_msgs.size() >= count && peer_->broadcast_tfs.size() >= count;
    }))
      << "expected " << count << " of each output, got gnss_fixed=" << peer_->fixed_flags.size()
      << " gnss_pose=" << peer_->poses.size() << " gnss_pose_cov=" << peer_->pose_cov_msgs.size()
      << " tf=" << peer_->broadcast_tfs.size();
  }

  // Exact number of messages received so far on each output.
  void expect_output_counts(
    std::size_t fixed, std::size_t pose, std::size_t pose_cov, std::size_t tf) const
  {
    EXPECT_EQ(peer_->fixed_flags.size(), fixed) << "gnss_fixed";
    EXPECT_EQ(peer_->poses.size(), pose) << "gnss_pose";
    EXPECT_EQ(peer_->pose_cov_msgs.size(), pose_cov) << "gnss_pose_cov";
    EXPECT_EQ(peer_->broadcast_tfs.size(), tf) << "/tf";
  }

  // The subscription or publisher gnss_poser has on `topic`, or std::nullopt when discovery has
  // not seen it within the discovery budget.
  std::optional<rclcpp::TopicEndpointInfo> wait_for_subscription(const std::string & topic)
  {
    pump_until(
      [&] {
        return gnss_poser_endpoint(peer_->get_subscriptions_info_by_topic(topic)).has_value();
      },
      discovery_budget);
    return gnss_poser_endpoint(peer_->get_subscriptions_info_by_topic(topic));
  }

  std::optional<rclcpp::TopicEndpointInfo> wait_for_publisher(const std::string & topic)
  {
    pump_until(
      [&] { return gnss_poser_endpoint(peer_->get_publishers_info_by_topic(topic)).has_value(); },
      discovery_budget);
    return gnss_poser_endpoint(peer_->get_publishers_info_by_topic(topic));
  }

  TransformStamped make_tf(
    const std::string & parent, const std::string & child, const Point & translation,
    const Quaternion & rotation, const builtin_interfaces::msg::Time & stamp)
  {
    TransformStamped tf;
    tf.header.stamp = stamp;
    tf.header.frame_id = parent;
    tf.child_frame_id = child;
    tf.transform.translation.x = translation.x;
    tf.transform.translation.y = translation.y;
    tf.transform.translation.z = translation.z;
    tf.transform.rotation = rotation;
    return tf;
  }

  void broadcast_static_tf(
    const std::string & parent, const std::string & child, const Point & translation,
    const Quaternion & rotation)
  {
    peer_->static_tf_broadcaster_->sendTransform(
      make_tf(parent, child, translation, rotation, peer_->now()));
    ASSERT_TRUE(
      pump_until([&] { return peer_->tf_buffer_.canTransform(parent, child, tf2::TimePointZero); }))
      << "static TF " << parent << "->" << child << " did not propagate";
    pump(delivery_budget);  // margin for the node's own (threaded) listener
  }

  // Publishes one time-stamped transform on /tf (as opposed to /tf_static), so that the node's
  // lookup time becomes observable.
  void broadcast_timed_tf(
    const std::string & parent, const std::string & child, const Point & translation,
    const Quaternion & rotation, const builtin_interfaces::msg::Time & stamp)
  {
    peer_->own_dynamic_tf_child_ = child;
    peer_->dynamic_tf_broadcaster_->sendTransform(
      make_tf(parent, child, translation, rotation, stamp));
    ASSERT_TRUE(pump_until(
      [&] { return peer_->tf_buffer_.canTransform(parent, child, tf2_ros::fromMsg(stamp)); }))
      << "timed TF " << parent << "->" << child << " did not propagate";
    pump(delivery_budget);
  }

  // A node whose base_frame is my_base, an INS orientation of `antenna_yaw`, and the static
  // transform my_antenna -> my_base with the translation (1, 2, 0.5) and no rotation.
  void build_node_with_static_antenna_tf(double antenna_yaw)
  {
    NodeParams params;
    params.base_frame = "my_base";
    ASSERT_NO_FATAL_FAILURE(build_ready_node(params));
    send_orientation(make_orientation(antenna_yaw));
    ASSERT_NO_FATAL_FAILURE(broadcast_static_tf(
      "my_antenna", "my_base", make_point(1.0, 2.0, 0.5), yaw_to_quaternion(0.0)));
  }

  // A node whose base_frame is my_base, and the transform my_antenna -> my_base with the
  // translation (1, 0, 0) published on /tf at 1000 s only.
  void build_node_with_antenna_tf_at_1000s()
  {
    NodeParams params;
    params.base_frame = "my_base";
    ASSERT_NO_FATAL_FAILURE(build_ready_node(params));
    send_orientation(make_orientation(0.0));
    ASSERT_NO_FATAL_FAILURE(broadcast_timed_tf(
      "my_antenna", "my_base", make_point(1.0, 0.0, 0.0), yaw_to_quaternion(0.0),
      make_stamp(1000, 0)));
  }

  const PoseStamped & last_pose() const { return peer_->poses.back(); }
  const PoseWithCovarianceStamped & last_pose_cov() const { return peer_->pose_cov_msgs.back(); }
  const BoolStamped & last_fixed() const { return peer_->fixed_flags.back(); }
  const TransformStamped & last_tf() const { return peer_->broadcast_tfs.back(); }

  std::shared_ptr<PeerNode> peer_;
  std::shared_ptr<autoware::gnss_poser::GnssPoserNode> node_;
  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
};

// Constructs nodes without running them, so it needs no peer or executor.
class GnssPoserNodeParameter : public ::testing::TestWithParam<rclcpp::Parameter>
{
};

std::string parameter_name(const ::testing::TestParamInfo<rclcpp::Parameter> & info)
{
  return info.param.get_name();
}

}  // namespace

// =======================================================================================
// Construction and ROS interface
// =======================================================================================

TEST_F(GnssPoserNodeIntegration, IsNamedGnssPoserInRootNamespace)
{
  ASSERT_NO_FATAL_FAILURE(build_node({}));

  EXPECT_STREQ(node_->get_name(), "gnss_poser");
  EXPECT_STREQ(node_->get_namespace(), "/");
}

TEST_F(GnssPoserNodeIntegration, ReadsEveryParameterFromOverrides)
{
  NodeParams params;
  params.base_frame = "my_base";
  params.gnss_base_frame = "my_gnss_base";
  params.map_frame = "my_map";
  params.use_gnss_ins_orientation = false;
  params.gnss_pose_pub_method = 2;
  params.buff_epoch = 7;

  ASSERT_NO_FATAL_FAILURE(build_node(params));

  EXPECT_EQ(node_->get_parameter("base_frame").as_string(), "my_base");
  EXPECT_EQ(node_->get_parameter("gnss_base_frame").as_string(), "my_gnss_base");
  EXPECT_EQ(node_->get_parameter("map_frame").as_string(), "my_map");
  EXPECT_FALSE(node_->get_parameter("use_gnss_ins_orientation").as_bool());
  EXPECT_EQ(node_->get_parameter("gnss_pose_pub_method").as_int(), 2);
  EXPECT_EQ(node_->get_parameter("buff_epoch").as_int(), 7);
}

// The node declares every parameter without a built-in default: a configuration that lacks one
// makes the node fail to start instead of running with a default. Which exception rclcpp throws
// is not part of the contract.
TEST_P(GnssPoserNodeParameter, FailsToStartWhenParameterIsMissing)
{
  const rclcpp::NodeOptions options = options_without(GetParam().get_name());

  EXPECT_THROW(std::make_shared<autoware::gnss_poser::GnssPoserNode>(options), std::exception);
}

// The node declares every parameter with a type: an override of another type makes the node fail
// to start.
TEST_P(GnssPoserNodeParameter, FailsToStartWhenParameterHasWrongType)
{
  const rclcpp::NodeOptions options = options_with(GetParam());

  EXPECT_THROW(std::make_shared<autoware::gnss_poser::GnssPoserNode>(options), std::exception);
}

// Each parameter with a value of the wrong type: an integer for the strings, a string for the
// boolean and for the integers.
INSTANTIATE_TEST_SUITE_P(
  EveryParameter, GnssPoserNodeParameter,
  ::testing::Values(
    rclcpp::Parameter("base_frame", 123), rclcpp::Parameter("gnss_base_frame", 123),
    rclcpp::Parameter("map_frame", 123),
    rclcpp::Parameter("use_gnss_ins_orientation", "not_a_bool"),
    rclcpp::Parameter("gnss_pose_pub_method", "not_an_int"),
    rclcpp::Parameter("buff_epoch", "not_an_int")),
  parameter_name);

// gnss_pose_pub_method accepts exactly 0 (instant), 1 (average) and 2 (median).
TEST_F(GnssPoserNodeIntegration, FailsToStartWhenPubMethodIsAboveRange)
{
  NodeParams params;
  params.gnss_pose_pub_method = 3;

  EXPECT_THROW(
    std::make_shared<autoware::gnss_poser::GnssPoserNode>(params.to_options()), std::exception);
}

TEST_F(GnssPoserNodeIntegration, FailsToStartWhenPubMethodIsNegative)
{
  NodeParams params;
  params.gnss_pose_pub_method = -1;

  EXPECT_THROW(
    std::make_shared<autoware::gnss_poser::GnssPoserNode>(params.to_options()), std::exception);
}

// The node has no built-in defaults: the values README.md and the schema document as defaults live
// in config/gnss_poser.param.yaml, which the launch file loads.
TEST_F(GnssPoserNodeIntegration, LoadsDocumentedDefaultsFromShippedParamFile)
{
  rclcpp::NodeOptions options;
  options.arguments(
    {"--ros-args", "--params-file", GNSS_POSER_CONFIG_DIR "/gnss_poser.param.yaml"});

  const auto node = std::make_shared<autoware::gnss_poser::GnssPoserNode>(options);

  EXPECT_EQ(node->get_parameter("base_frame").as_string(), "base_link");
  EXPECT_EQ(node->get_parameter("gnss_base_frame").as_string(), "gnss_base_link");
  EXPECT_EQ(node->get_parameter("map_frame").as_string(), "map");
  EXPECT_TRUE(node->get_parameter("use_gnss_ins_orientation").as_bool());
  EXPECT_EQ(node->get_parameter("gnss_pose_pub_method").as_int(), 0);
  EXPECT_EQ(node->get_parameter("buff_epoch").as_int(), 1);
}

// Queue depths are not checked: they are not visible through discovery.
TEST_F(GnssPoserNodeIntegration, SubscribesToFixAsReliableVolatile)
{
  ASSERT_NO_FATAL_FAILURE(build_node({}));

  const auto sub = wait_for_subscription("/fix");

  ASSERT_TRUE(sub.has_value());
  EXPECT_EQ(sub->topic_type(), "sensor_msgs/msg/NavSatFix");
  EXPECT_EQ(sub->qos_profile().reliability(), rclcpp::ReliabilityPolicy::Reliable);
  EXPECT_EQ(sub->qos_profile().durability(), rclcpp::DurabilityPolicy::Volatile);
}

TEST_F(GnssPoserNodeIntegration, SubscribesToOrientationAsReliableVolatile)
{
  ASSERT_NO_FATAL_FAILURE(build_node({}));

  const auto sub = wait_for_subscription("/autoware_orientation");

  ASSERT_TRUE(sub.has_value());
  EXPECT_EQ(sub->topic_type(), "autoware_sensing_msgs/msg/GnssInsOrientationStamped");
  EXPECT_EQ(sub->qos_profile().reliability(), rclcpp::ReliabilityPolicy::Reliable);
  EXPECT_EQ(sub->qos_profile().durability(), rclcpp::DurabilityPolicy::Volatile);
}

// The projector info is published once by map_projection_loader, so a late subscriber needs it
// latched.
TEST_F(GnssPoserNodeIntegration, SubscribesToProjectorInfoAsReliableTransientLocal)
{
  ASSERT_NO_FATAL_FAILURE(build_node({}));

  const auto sub = wait_for_subscription("/map/map_projector_info");

  ASSERT_TRUE(sub.has_value());
  EXPECT_EQ(sub->topic_type(), "autoware_map_msgs/msg/MapProjectorInfo");
  EXPECT_EQ(sub->qos_profile().reliability(), rclcpp::ReliabilityPolicy::Reliable);
  EXPECT_EQ(sub->qos_profile().durability(), rclcpp::DurabilityPolicy::TransientLocal);
}

TEST_F(GnssPoserNodeIntegration, PublishesPoseAsReliableVolatile)
{
  ASSERT_NO_FATAL_FAILURE(build_node({}));

  const auto pub = wait_for_publisher("/gnss_pose");

  ASSERT_TRUE(pub.has_value());
  EXPECT_EQ(pub->topic_type(), "geometry_msgs/msg/PoseStamped");
  EXPECT_EQ(pub->qos_profile().reliability(), rclcpp::ReliabilityPolicy::Reliable);
  EXPECT_EQ(pub->qos_profile().durability(), rclcpp::DurabilityPolicy::Volatile);
}

TEST_F(GnssPoserNodeIntegration, PublishesPoseWithCovarianceAsReliableVolatile)
{
  ASSERT_NO_FATAL_FAILURE(build_node({}));

  const auto pub = wait_for_publisher("/gnss_pose_cov");

  ASSERT_TRUE(pub.has_value());
  EXPECT_EQ(pub->topic_type(), "geometry_msgs/msg/PoseWithCovarianceStamped");
  EXPECT_EQ(pub->qos_profile().reliability(), rclcpp::ReliabilityPolicy::Reliable);
  EXPECT_EQ(pub->qos_profile().durability(), rclcpp::DurabilityPolicy::Volatile);
}

TEST_F(GnssPoserNodeIntegration, PublishesGnssFixedAsReliableVolatile)
{
  ASSERT_NO_FATAL_FAILURE(build_node({}));

  const auto pub = wait_for_publisher("/gnss_fixed");

  ASSERT_TRUE(pub.has_value());
  EXPECT_EQ(pub->topic_type(), "autoware_internal_debug_msgs/msg/BoolStamped");
  EXPECT_EQ(pub->qos_profile().reliability(), rclcpp::ReliabilityPolicy::Reliable);
  EXPECT_EQ(pub->qos_profile().durability(), rclcpp::DurabilityPolicy::Volatile);
}

// =======================================================================================
// What the node publishes for each outcome of the pose computation
// =======================================================================================

TEST_F(GnssPoserNodeIntegration, PublishesNothingBeforeProjectorInfo)
{
  ASSERT_NO_FATAL_FAILURE(build_node({}));

  send_fix(make_reference_fix());

  expect_output_counts(0, 0, 0, 0);
}

TEST_F(GnssPoserNodeIntegration, PublishesNothingUnderLocalProjector)
{
  ASSERT_NO_FATAL_FAILURE(build_node({}));
  send_projector_info(make_local_projector_info());

  send_fix(make_reference_fix());

  expect_output_counts(0, 0, 0, 0);
}

// A fix dropped for lack of projector info is not replayed when the projector info arrives.
TEST_F(GnssPoserNodeIntegration, PublishesNothingWhenProjectorInfoArrivesAfterDroppedFix)
{
  ASSERT_NO_FATAL_FAILURE(build_node({}));
  send_fix(make_reference_fix());

  send_projector_info(make_mgrs_projector_info());

  expect_output_counts(0, 0, 0, 0);
}

TEST_F(GnssPoserNodeIntegration, PublishesOnlyGnssFixedFalseWhenNotFixed)
{
  ASSERT_NO_FATAL_FAILURE(build_ready_node({}));

  send_fix(make_reference_fix(NavSatStatus::STATUS_NO_FIX));

  ASSERT_NO_FATAL_FAILURE(wait_for_gnss_fixed(1));
  expect_output_counts(1, 0, 0, 0);
  EXPECT_FALSE(last_fixed().data);
}

TEST_F(GnssPoserNodeIntegration, PublishesOnlyGnssFixedTrueWhileBuffering)
{
  NodeParams params;
  params.gnss_pose_pub_method = 1;  // average
  params.buff_epoch = 2;
  ASSERT_NO_FATAL_FAILURE(build_ready_node(params));

  send_fix(make_reference_fix());

  ASSERT_NO_FATAL_FAILURE(wait_for_gnss_fixed(1));
  expect_output_counts(1, 0, 0, 0);
  EXPECT_TRUE(last_fixed().data);
}

TEST_F(GnssPoserNodeIntegration, PublishesEveryOutputOnceWhenPoseIsComputed)
{
  ASSERT_NO_FATAL_FAILURE(build_ready_node({}));

  send_fix(make_reference_fix());

  ASSERT_NO_FATAL_FAILURE(wait_for_outputs(1));
  pump(delivery_budget);  // room for a duplicate to show up
  expect_output_counts(1, 1, 1, 1);
  EXPECT_TRUE(last_fixed().data);
}

TEST_F(GnssPoserNodeIntegration, StampsEveryOutputWithFixStamp)
{
  ASSERT_NO_FATAL_FAILURE(build_ready_node({}));
  NavSatFix fix = make_reference_fix();
  fix.header.stamp = make_stamp(1234, 5678U);

  send_fix(fix);

  ASSERT_NO_FATAL_FAILURE(wait_for_outputs(1));
  EXPECT_EQ(last_fixed().stamp, make_stamp(1234, 5678U));
  EXPECT_EQ(last_pose().header.stamp, make_stamp(1234, 5678U));
  EXPECT_EQ(last_pose_cov().header.stamp, make_stamp(1234, 5678U));
  EXPECT_EQ(last_tf().header.stamp, make_stamp(1234, 5678U));
}

TEST_F(GnssPoserNodeIntegration, PublishesInFramesFromParameters)
{
  NodeParams params;
  params.map_frame = "my_map";
  params.gnss_base_frame = "my_gnss_base_link";
  ASSERT_NO_FATAL_FAILURE(build_ready_node(params));

  send_fix(make_reference_fix());

  ASSERT_NO_FATAL_FAILURE(wait_for_outputs(1));
  EXPECT_EQ(last_pose().header.frame_id, "my_map");
  EXPECT_EQ(last_pose_cov().header.frame_id, "my_map");
  EXPECT_EQ(last_tf().header.frame_id, "my_map");
  EXPECT_EQ(last_tf().child_frame_id, "my_gnss_base_link");
}

// The pose value is the logic's business (test_gnss_poser.cpp); this shows that the projector info
// and the fix reach the logic unchanged.
TEST_F(GnssPoserNodeIntegration, PublishesProjectedPositionOfFix)
{
  ASSERT_NO_FATAL_FAILURE(build_ready_node({}));

  send_fix(make_reference_fix());

  ASSERT_NO_FATAL_FAILURE(wait_for_outputs(1));
  expect_position_near(last_pose().pose.position, reference_x, reference_y, 10.0);
}

// gnss_pose_cov and the broadcast transform are copies of the published pose, hence exact
// equality.
TEST_F(GnssPoserNodeIntegration, PublishesSamePoseOnEveryOutput)
{
  ASSERT_NO_FATAL_FAILURE(build_ready_node({}));

  send_fix(make_reference_fix());

  ASSERT_NO_FATAL_FAILURE(wait_for_outputs(1));
  const auto & pose = last_pose().pose;
  EXPECT_EQ(last_pose_cov().pose.pose, pose);
  const auto & transform = last_tf().transform;
  EXPECT_EQ(transform.translation.x, pose.position.x);
  EXPECT_EQ(transform.translation.y, pose.position.y);
  EXPECT_EQ(transform.translation.z, pose.position.z);
  EXPECT_EQ(transform.rotation, pose.orientation);
}

// =======================================================================================
// Antenna transform lookup (antenna -> base_link)
// =======================================================================================

// The antenna frame is the fix's header.frame_id. Its transform to base_frame is applied in the
// antenna orientation: facing +y (yaw pi/2), the lever arm (1, 2, 0.5) becomes (-2, 1, 0.5).
TEST_F(GnssPoserNodeIntegration, ComposesPoseWithAntennaTransformOfFixFrame)
{
  ASSERT_NO_FATAL_FAILURE(build_node_with_static_antenna_tf(M_PI / 2.0));

  send_fix(make_reference_fix(NavSatStatus::STATUS_FIX, "my_antenna"));

  ASSERT_NO_FATAL_FAILURE(wait_for_outputs(1));
  expect_position_near(last_pose().pose.position, reference_x - 2.0, reference_y + 1.0, 10.5);
  expect_same_rotation(last_pose().pose.orientation, yaw_to_quaternion(M_PI / 2.0));
}

// A fix stamped in base_frame needs no lookup.
TEST_F(GnssPoserNodeIntegration, PublishesAntennaPoseWhenFixFrameIsBaseFrame)
{
  ASSERT_NO_FATAL_FAILURE(build_node_with_static_antenna_tf(M_PI / 2.0));

  send_fix(make_reference_fix(NavSatStatus::STATUS_FIX, "my_base"));

  ASSERT_NO_FATAL_FAILURE(wait_for_outputs(1));
  expect_position_near(last_pose().pose.position, reference_x, reference_y, 10.0);
}

// NOTE(characterization): a frame_id that matches no TF frame is a configuration error that never
// resolves by waiting, yet the node keeps publishing the antenna position as base_link, with only a
// throttled warning.
TEST_F(GnssPoserNodeIntegration, PublishesAntennaPoseWhenFixFrameHasNoTransform)
{
  ASSERT_NO_FATAL_FAILURE(build_node_with_static_antenna_tf(M_PI / 2.0));

  send_fix(make_reference_fix(NavSatStatus::STATUS_FIX, "some_other_antenna"));

  ASSERT_NO_FATAL_FAILURE(wait_for_outputs(1));
  expect_position_near(last_pose().pose.position, reference_x, reference_y, 10.0);
}

// The antenna sits rigidly on the vehicle and is normally published on /tf_static, where time is
// ignored; a transform on /tf is the only way to observe which time the node asks for. The next
// three cases pin that it is the fix header stamp.
TEST_F(GnssPoserNodeIntegration, AppliesAntennaTransformAtFixStamp)
{
  ASSERT_NO_FATAL_FAILURE(build_node_with_antenna_tf_at_1000s());

  send_fix(make_reference_fix_at(make_stamp(1000, 0)));

  ASSERT_NO_FATAL_FAILURE(wait_for_outputs(1));
  expect_position_near(last_pose().pose.position, reference_x + 1.0, reference_y, 10.0);
}

// A lookup one second after the only transform needs extrapolation, which tf2 refuses.
TEST_F(GnssPoserNodeIntegration, PublishesAntennaPoseWhenAntennaTransformIsOlderThanFix)
{
  ASSERT_NO_FATAL_FAILURE(build_node_with_antenna_tf_at_1000s());

  send_fix(make_reference_fix_at(make_stamp(1001, 0)));

  ASSERT_NO_FATAL_FAILURE(wait_for_outputs(1));
  expect_position_near(last_pose().pose.position, reference_x, reference_y, 10.0);
}

// tf2 treats time 0 as "latest".
TEST_F(GnssPoserNodeIntegration, AppliesLatestAntennaTransformWhenFixStampIsZero)
{
  ASSERT_NO_FATAL_FAILURE(build_node_with_antenna_tf_at_1000s());

  send_fix(make_reference_fix_at(make_stamp(0, 0)));

  ASSERT_NO_FATAL_FAILURE(wait_for_outputs(1));
  expect_position_near(last_pose().pose.position, reference_x + 1.0, reference_y, 10.0);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  const int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
