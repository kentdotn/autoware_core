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

// cspell:ignore SBAS GBAS

// =====================================================================================
// Integration test of autoware::gnss_poser::GnssPoserNode, the ROS layer of the GNSS poser.
//
// The pose computation itself (GnssPoser) is unit-tested in test_gnss_poser.cpp without ROS. What
// is checked here, over real topics, is what only the node does: declare the parameters, offer the
// topics, hand the inputs to the logic, publish the right topics for each outcome with the right
// headers, and look up the antenna transform.
//
// Test naming: <Aspect>_<Condition>_<Behavior>
// =====================================================================================

#include "gnss_poser_node.hpp"

#include <autoware/geography_utils/height.hpp>
#include <autoware/geography_utils/projection.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Matrix3x3.hpp>
#include <tf2/LinearMath/Quaternion.hpp>
#include <tf2_ros/buffer.hpp>
#include <tf2_ros/static_transform_broadcaster.hpp>
#include <tf2_ros/transform_broadcaster.hpp>
#include <tf2_ros/transform_listener.hpp>

#include <autoware_internal_debug_msgs/msg/bool_stamped.hpp>
#include <autoware_map_msgs/msg/map_projector_info.hpp>
#include <autoware_sensing_msgs/msg/gnss_ins_orientation_stamped.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <geographic_msgs/msg/geo_point.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
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
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

namespace
{
using autoware_internal_debug_msgs::msg::BoolStamped;
using autoware_map_msgs::msg::MapProjectorInfo;
using autoware_sensing_msgs::msg::GnssInsOrientationStamped;
using diagnostic_msgs::msg::DiagnosticArray;
using diagnostic_msgs::msg::DiagnosticStatus;
using geometry_msgs::msg::Point;
using geometry_msgs::msg::PoseStamped;
using geometry_msgs::msg::PoseWithCovarianceStamped;
using geometry_msgs::msg::Quaternion;
using geometry_msgs::msg::TransformStamped;
using sensor_msgs::msg::NavSatFix;
using sensor_msgs::msg::NavSatStatus;
using tf2_msgs::msg::TFMessage;
using std::chrono_literals::operator""ms;
using std::chrono_literals::operator""s;

// ---------------------------------------------------------------------------------------
// Reference input used throughout (same point as autoware_geography_utils' own tests).
// ---------------------------------------------------------------------------------------
constexpr double reference_latitude = 35.62426;
constexpr double reference_longitude = 139.74252;
constexpr double reference_altitude = 10.0;
constexpr const char * reference_mgrs_grid = "54SUE";
// Golden values: MGRS(54SUE) projection of (reference_latitude, reference_longitude,
// reference_altitude) as observed from the node output (identity antenna->base TF, WGS84 vertical
// datum). Recorded on 2026-09-03 with the GeographicLib/lanelet2 versions bundled in the autoware
// core-devel jazzy image.
constexpr double golden_x = 86128.181788819958;
constexpr double golden_y = 43002.610125367064;
constexpr double golden_z = reference_altitude;
// Tolerances for computed values. A behavior change worth catching moves a position by centimeters
// or more and a heading by about 0.001 rad or more; the floating-point noise of the pipeline is
// many orders of magnitude below both. Values the node copies are compared exactly instead.
constexpr double position_tolerance = 1e-4;  // [m]
constexpr double angle_tolerance = 1e-6;     // [rad]
// A deterministic, clearly artificial header stamp so that a stamp copied from the input can be
// told apart from one taken from a clock.
constexpr int32_t fix_stamp_sec = 1700000000;
constexpr uint32_t fix_stamp_nanosec = 123456789U;

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
  double antenna_transform_timeout_sec = 0.5;

  // The seven parameters the node declares, in declaration order.
  static const std::vector<std::string> & names()
  {
    static const std::vector<std::string> parameter_names = {
      "base_frame",
      "gnss_base_frame",
      "map_frame",
      "use_gnss_ins_orientation",
      "gnss_pose_pub_method",
      "buff_epoch",
      "antenna_transform_timeout_sec"};
    return parameter_names;
  }

  // Parameter overrides for the node under test. `skip` leaves that one parameter unset.
  [[nodiscard]] rclcpp::NodeOptions to_options(const std::string & skip = "") const
  {
    rclcpp::NodeOptions options;
    const auto add = [&](const std::string & name, const auto & value) {
      if (name != skip) {
        options.append_parameter_override(name, value);
      }
    };
    add("base_frame", base_frame);
    add("gnss_base_frame", gnss_base_frame);
    add("map_frame", map_frame);
    add("use_gnss_ins_orientation", use_gnss_ins_orientation);
    add("gnss_pose_pub_method", gnss_pose_pub_method);
    add("buff_epoch", buff_epoch);
    add("antenna_transform_timeout_sec", antenna_transform_timeout_sec);
    return options;
  }
};

builtin_interfaces::msg::Time make_stamp(int32_t sec, uint32_t nanosec)
{
  builtin_interfaces::msg::Time stamp;
  stamp.sec = sec;
  stamp.nanosec = nanosec;
  return stamp;
}

builtin_interfaces::msg::Time fix_stamp()
{
  return make_stamp(fix_stamp_sec, fix_stamp_nanosec);
}

NavSatFix make_fix(
  double latitude, double longitude, double altitude,
  NavSatStatus::_status_type status = NavSatStatus::STATUS_FIX,
  const std::string & frame_id = "gnss")
{
  NavSatFix msg;
  msg.header.stamp = fix_stamp();
  msg.header.frame_id = frame_id;
  msg.status.status = status;
  msg.status.service = NavSatStatus::SERVICE_GPS;
  msg.latitude = latitude;
  msg.longitude = longitude;
  msg.altitude = altitude;
  msg.position_covariance = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
  msg.position_covariance_type = NavSatFix::COVARIANCE_TYPE_DIAGONAL_KNOWN;
  return msg;
}

NavSatFix make_reference_fix(
  NavSatStatus::_status_type status = NavSatStatus::STATUS_FIX,
  const std::string & frame_id = "gnss")
{
  return make_fix(reference_latitude, reference_longitude, reference_altitude, status, frame_id);
}

MapProjectorInfo make_mgrs_projector_info(
  const std::string & grid = reference_mgrs_grid,
  const std::string & vertical_datum = MapProjectorInfo::WGS84)
{
  MapProjectorInfo msg;
  msg.projector_type = MapProjectorInfo::MGRS;
  msg.vertical_datum = vertical_datum;
  msg.mgrs_grid = grid;
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

double yaw_of(const Quaternion & quaternion)
{
  tf2::Quaternion q;
  tf2::fromMsg(quaternion, q);
  double roll = 0.0;
  double pitch = 0.0;
  double yaw = 0.0;
  tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
  return yaw;
}

GnssInsOrientationStamped make_orientation(
  double yaw, double rmse_x = 0.1, double rmse_y = 0.2, double rmse_z = 0.3)
{
  GnssInsOrientationStamped msg;
  msg.header.stamp = make_stamp(1, 0);  // deliberately unrelated to the fix stamp
  msg.header.frame_id = "ins";
  msg.orientation.orientation = yaw_to_quaternion(yaw);
  msg.orientation.rmse_rotation_x = rmse_x;
  msg.orientation.rmse_rotation_y = rmse_y;
  msg.orientation.rmse_rotation_z = rmse_z;
  return msg;
}

// Antenna position the node is expected to derive from a NavSatFix, computed with the same
// library the node uses (autoware_geography_utils). Used to build exact expectations for the
// buffering / orientation / TF composition logic, which is what these tests characterize.
Point project_antenna(const NavSatFix & fix, const MapProjectorInfo & projector_info)
{
  geographic_msgs::msg::GeoPoint geo_point;
  geo_point.latitude = fix.latitude;
  geo_point.longitude = fix.longitude;
  geo_point.altitude = fix.altitude;
  Point position = autoware::geography_utils::project_forward(geo_point, projector_info);
  position.z = autoware::geography_utils::convert_height(
    position.z, geo_point.latitude, geo_point.longitude, MapProjectorInfo::WGS84,
    projector_info.vertical_datum);
  return position;
}

Point make_point(double x, double y, double z)
{
  Point p;
  p.x = x;
  p.y = y;
  p.z = z;
  return p;
}

void expect_point_near(
  const Point & actual, const Point & expected, double tol = position_tolerance)
{
  EXPECT_NEAR(actual.x, expected.x, tol);
  EXPECT_NEAR(actual.y, expected.y, tol);
  EXPECT_NEAR(actual.z, expected.z, tol);
}

// ---------------------------------------------------------------------------------------
// Test double: a plain rclcpp node that plays the role of every peer of gnss_poser
// (map_projection_loader, GNSS driver, INS driver, robot_state_publisher, and the consumers).
// ---------------------------------------------------------------------------------------
class PeerNode : public rclcpp::Node
{
public:
  PeerNode()
  : rclcpp::Node("gnss_poser_characterization_peer"),
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
    diagnostics_sub_ = create_subscription<DiagnosticArray>(
      "/diagnostics", rclcpp::QoS{100}, [this](const DiagnosticArray::ConstSharedPtr msg) {
        for (const auto & status : msg->status) {
          if (status.hardware_id == "gnss_poser") {
            diagnostics.push_back(status);
          }
        }
      });
    tf_sub_ = create_subscription<TFMessage>(
      "/tf", rclcpp::QoS{100}, [this](const TFMessage::ConstSharedPtr msg) {
        for (const auto & t : msg->transforms) {
          // Only keep what gnss_poser broadcasts; this peer publishes on /tf as well.
          if (t.child_frame_id != own_dynamic_tf_child_) {
            broadcast_tfs.push_back(t);
          }
        }
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
  rclcpp::Subscription<DiagnosticArray>::SharedPtr diagnostics_sub_;
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
  std::vector<DiagnosticStatus> diagnostics;
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

  const PoseStamped & last_pose() const { return peer_->poses.back(); }
  const PoseWithCovarianceStamped & last_pose_cov() const { return peer_->pose_cov_msgs.back(); }
  const BoolStamped & last_fixed() const { return peer_->fixed_flags.back(); }
  const TransformStamped & last_tf() const { return peer_->broadcast_tfs.back(); }

  std::shared_ptr<PeerNode> peer_;
  std::shared_ptr<autoware::gnss_poser::GnssPoserNode> node_;
  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
};

}  // namespace

// =======================================================================================
// 1. Construction and ROS interface
// =======================================================================================

// The node is named `gnss_poser` in the root namespace and declares the six parameters the README
// documents; every override is read back unchanged.
TEST_F(GnssPoserNodeIntegration, Construct_WithAllParameters_DeclaresThemAndIsNamedGnssPoser)
{
  NodeParams params;
  params.base_frame = "my_base";
  params.gnss_base_frame = "my_gnss_base";
  params.map_frame = "my_map";
  params.use_gnss_ins_orientation = false;
  params.gnss_pose_pub_method = 2;
  params.buff_epoch = 7;
  params.antenna_transform_timeout_sec = 1.5;
  ASSERT_NO_FATAL_FAILURE(build_node(params));

  EXPECT_STREQ(node_->get_name(), "gnss_poser");
  EXPECT_STREQ(node_->get_namespace(), "/");

  EXPECT_EQ(node_->get_parameter("base_frame").as_string(), "my_base");
  EXPECT_EQ(node_->get_parameter("gnss_base_frame").as_string(), "my_gnss_base");
  EXPECT_EQ(node_->get_parameter("map_frame").as_string(), "my_map");
  EXPECT_EQ(node_->get_parameter("use_gnss_ins_orientation").as_bool(), false);
  EXPECT_EQ(node_->get_parameter("gnss_pose_pub_method").as_int(), 2);
  EXPECT_EQ(node_->get_parameter("buff_epoch").as_int(), 7);
  EXPECT_DOUBLE_EQ(node_->get_parameter("antenna_transform_timeout_sec").as_double(), 1.5);
}

// The node declares all six parameters without a built-in default: a configuration that lacks any
// of them makes the node fail to start (constructing the component throws) instead of running with
// a default. The values README.md lists as defaults live in config/gnss_poser.param.yaml, which the
// next case pins. Only "fails to start" is pinned; which exception rclcpp throws is not part of the
// contract.
TEST_F(GnssPoserNodeIntegration, Construct_MissingAnyRequiredParameter_FailsToStart)
{
  const NodeParams params;
  for (const auto & missing : NodeParams::names()) {
    EXPECT_THROW(
      std::make_shared<autoware::gnss_poser::GnssPoserNode>(params.to_options(missing)),
      std::exception)
      << "missing parameter: " << missing;
  }
}

// The node declares each parameter with a type: an override of the wrong type (an integer for the
// strings, a string for the boolean and for the integers) makes the node fail to start. As above,
// only "fails to start" is pinned, not the exception rclcpp throws.
TEST_F(GnssPoserNodeIntegration, Construct_WrongParameterType_FailsToStart)
{
  const std::vector<std::pair<std::string, rclcpp::ParameterValue>> wrong_typed = {
    {"base_frame", rclcpp::ParameterValue(123)},
    {"gnss_base_frame", rclcpp::ParameterValue(123)},
    {"map_frame", rclcpp::ParameterValue(123)},
    {"use_gnss_ins_orientation", rclcpp::ParameterValue("not_a_bool")},
    {"gnss_pose_pub_method", rclcpp::ParameterValue("not_an_int")},
    {"buff_epoch", rclcpp::ParameterValue("not_an_int")},
  };
  for (const auto & [name, value] : wrong_typed) {
    rclcpp::NodeOptions options = NodeParams{}.to_options(name);
    options.append_parameter_override(name, value);
    EXPECT_THROW(std::make_shared<autoware::gnss_poser::GnssPoserNode>(options), std::exception)
      << "parameter: " << name;
  }
}

// `gnss_pose_pub_method` accepts exactly 0 (instant), 1 (average) and 2 (median). Any other value
// makes the node fail to start instead of being silently treated as one of them (before this
// validation, every non-zero value buffered and every value other than 1 selected the median).
TEST_F(GnssPoserNodeIntegration, Construct_UnknownPubMethod_FailsToStart)
{
  // Representative invalid values: below the range, just above it, and far outside it.
  for (const int method : {-1, 3, 42}) {
    NodeParams params;
    params.gnss_pose_pub_method = method;
    params.buff_epoch = 3;
    EXPECT_THROW(
      std::make_shared<autoware::gnss_poser::GnssPoserNode>(params.to_options()), std::exception)
      << "gnss_pose_pub_method=" << method;
  }
}

// The node has no built-in defaults: the values README.md and the schema document as defaults live
// in config/gnss_poser.param.yaml, which the launch file loads. That file must keep constructing
// the node and read back exactly as documented.
TEST_F(GnssPoserNodeIntegration, Construct_WithShippedParamFile_MatchesDocumentedDefaults)
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
  EXPECT_DOUBLE_EQ(node->get_parameter("antenna_transform_timeout_sec").as_double(), 0.5);
}

// The node's ROS surface: three subscriptions, three publishers and the /tf broadcast, with their
// message types, reliability and durability. `/map/map_projector_info` is the one transient-local
// subscription.
//
// Queue depths are not pinned: they are not visible through discovery.
TEST_F(GnssPoserNodeIntegration, Interface_TopicsAndQos)
{
  ASSERT_NO_FATAL_FAILURE(build_node({}));

  const auto topics_ready = [this] {
    const auto topics = peer_->get_topic_names_and_types();
    const auto has = [&](const std::string & name, const std::string & type) {
      const auto it = topics.find(name);
      return it != topics.end() &&
             std::find(it->second.begin(), it->second.end(), type) != it->second.end();
    };
    return has("/fix", "sensor_msgs/msg/NavSatFix") &&
           has("/autoware_orientation", "autoware_sensing_msgs/msg/GnssInsOrientationStamped") &&
           has("/map/map_projector_info", "autoware_map_msgs/msg/MapProjectorInfo") &&
           has("/gnss_pose", "geometry_msgs/msg/PoseStamped") &&
           has("/gnss_pose_cov", "geometry_msgs/msg/PoseWithCovarianceStamped") &&
           has("/gnss_fixed", "autoware_internal_debug_msgs/msg/BoolStamped") &&
           has("/tf", "tf2_msgs/msg/TFMessage") &&
           has("/diagnostics", "diagnostic_msgs/msg/DiagnosticArray");
  };
  ASSERT_TRUE(pump_until(topics_ready, 10s));

  const auto node_endpoint = [](const std::vector<rclcpp::TopicEndpointInfo> & infos) {
    for (const auto & info : infos) {
      if (info.node_name() == "gnss_poser") {
        return info;
      }
    }
    throw std::runtime_error("no endpoint owned by gnss_poser");
  };

  // Subscriptions
  {
    const auto sub = node_endpoint(peer_->get_subscriptions_info_by_topic("/fix"));
    EXPECT_EQ(sub.qos_profile().reliability(), rclcpp::ReliabilityPolicy::Reliable);
    EXPECT_EQ(sub.qos_profile().durability(), rclcpp::DurabilityPolicy::Volatile);
  }
  {
    const auto sub = node_endpoint(peer_->get_subscriptions_info_by_topic("/autoware_orientation"));
    EXPECT_EQ(sub.qos_profile().reliability(), rclcpp::ReliabilityPolicy::Reliable);
    EXPECT_EQ(sub.qos_profile().durability(), rclcpp::DurabilityPolicy::Volatile);
  }
  {
    const auto sub =
      node_endpoint(peer_->get_subscriptions_info_by_topic("/map/map_projector_info"));
    EXPECT_EQ(sub.qos_profile().reliability(), rclcpp::ReliabilityPolicy::Reliable);
    EXPECT_EQ(sub.qos_profile().durability(), rclcpp::DurabilityPolicy::TransientLocal);
  }
  // Publishers
  for (const auto & topic : {"/gnss_pose", "/gnss_pose_cov", "/gnss_fixed"}) {
    const auto pub = node_endpoint(peer_->get_publishers_info_by_topic(topic));
    EXPECT_EQ(pub.qos_profile().reliability(), rclcpp::ReliabilityPolicy::Reliable) << topic;
    EXPECT_EQ(pub.qos_profile().durability(), rclcpp::DurabilityPolicy::Volatile) << topic;
  }
}

// =======================================================================================
// 2. What the node publishes for each outcome of the pose computation
// =======================================================================================

// The node maps every outcome of GnssPoser::input_fix() to its topics: a fix before any projector
// info or under a LOCAL projector publishes nothing (not even `gnss_fixed`), a fix without a
// position solution publishes `gnss_fixed = false` only, a fix that only fills the position buffer
// publishes `gnss_fixed = true` only, and a fix that yields a pose publishes all four outputs with
// the fix header stamp, `map_frame` as frame_id and `gnss_base_frame` as the TF child frame. The
// arrival of projector info publishes nothing by itself: dropped fixes are not replayed.
//
// The pose value is the logic's business (test_gnss_poser.cpp); one golden position check remains
// here to show that the projector info and the fix reach the logic unchanged.
TEST_F(GnssPoserNodeIntegration, Outcome_DrivesWhatIsPublished)
{
  NodeParams params;
  params.gnss_pose_pub_method = 1;  // average, so that the buffering outcome can be observed
  params.buff_epoch = 2;
  params.map_frame = "my_map";  // non-default values, so that the frames are shown to come from
  params.gnss_base_frame = "my_gnss_base_link";  // the parameters
  ASSERT_NO_FATAL_FAILURE(build_node(params));

  // Every fix is stamped in base_link, so its antenna transform is available without TF.
  const auto fixed_fix = make_reference_fix(NavSatStatus::STATUS_FIX, "base_link");
  const auto no_fix = make_reference_fix(NavSatStatus::STATUS_NO_FIX, "base_link");

  // NoProjectorInfo: dropped, nothing published.
  send_fix(fixed_fix);
  expect_output_counts(0, 0, 0, 0);

  // LocalProjector: dropped as well.
  send_projector_info(make_local_projector_info());
  send_fix(fixed_fix);
  expect_output_counts(0, 0, 0, 0);

  // NotFixed: gnss_fixed = false and nothing else.
  send_projector_info(make_mgrs_projector_info());
  expect_output_counts(0, 0, 0, 0);
  send_fix(no_fix);
  ASSERT_NO_FATAL_FAILURE(wait_for_gnss_fixed(1));
  expect_output_counts(1, 0, 0, 0);
  EXPECT_FALSE(last_fixed().data);

  // Buffering: gnss_fixed = true and nothing else.
  const auto fix = fixed_fix;
  send_fix(fix);
  ASSERT_NO_FATAL_FAILURE(wait_for_gnss_fixed(2));
  expect_output_counts(2, 0, 0, 0);
  EXPECT_TRUE(last_fixed().data);

  // Published: all four outputs, every one stamped with the fix header stamp.
  send_fix(fix);
  ASSERT_NO_FATAL_FAILURE(wait_for_outputs(1));
  expect_output_counts(3, 1, 1, 1);
  EXPECT_TRUE(last_fixed().data);
  EXPECT_EQ(last_fixed().stamp, fix.header.stamp);

  const auto & pose = last_pose();
  EXPECT_EQ(pose.header.stamp, fix.header.stamp);
  EXPECT_EQ(pose.header.frame_id, "my_map");
  // Two identical fixes average to themselves, so the golden projection applies.
  expect_point_near(pose.pose.position, make_point(golden_x, golden_y, golden_z));

  // gnss_pose_cov carries a copy of the same header and pose, hence exact equality.
  const auto & pose_cov = last_pose_cov();
  EXPECT_EQ(pose_cov.header, pose.header);
  EXPECT_EQ(pose_cov.pose.pose, pose.pose);

  const auto & tf = last_tf();
  EXPECT_EQ(tf.header.stamp, fix.header.stamp);
  EXPECT_EQ(tf.header.frame_id, "my_map");
  EXPECT_EQ(tf.child_frame_id, "my_gnss_base_link");
}

// The diagnostics status `gnss_poser: gnss_poser_status` reflects the input state: WARN while the
// projector info and the fix are missing, with the corresponding keys False, and OK once a fix has
// been published with every input present. Which conditions map to which level is unit-tested in
// test_gnss_poser_diagnostics.cpp; this case checks the wiring to /diagnostics.
TEST_F(GnssPoserNodeIntegration, Diagnostics_ReflectInputState)
{
  ASSERT_NO_FATAL_FAILURE(build_node({}));

  ASSERT_TRUE(pump_until([this] { return !peer_->diagnostics.empty(); }, wait_budget));
  const auto & before = peer_->diagnostics.back();
  EXPECT_EQ(before.name, "gnss_poser: gnss_poser_status");
  EXPECT_EQ(before.level, DiagnosticStatus::WARN);
  const auto value_of = [](const DiagnosticStatus & status, const std::string & key) {
    for (const auto & kv : status.values) {
      if (kv.key == key) {
        return kv.value;
      }
    }
    return std::string("<missing>");
  };
  EXPECT_EQ(value_of(before, "is_arrived_first_fix"), "False");
  EXPECT_EQ(value_of(before, "is_arrived_first_map_projector_info"), "False");
  EXPECT_EQ(value_of(before, "latest_outcome"), "None");

  send_projector_info(make_mgrs_projector_info());
  send_orientation(make_orientation(0.0));
  // The fix is stamped in base_link itself, so the antenna transform is available without TF.
  send_fix(make_reference_fix(NavSatStatus::STATUS_FIX, "base_link"));
  ASSERT_NO_FATAL_FAILURE(wait_for_outputs(1));
  peer_->diagnostics.clear();
  ASSERT_TRUE(pump_until([this] { return !peer_->diagnostics.empty(); }, wait_budget));
  const auto & after = peer_->diagnostics.back();
  EXPECT_EQ(after.level, DiagnosticStatus::OK);
  EXPECT_EQ(value_of(after, "is_arrived_first_fix"), "True");
  EXPECT_EQ(value_of(after, "is_arrived_first_orientation"), "True");
  EXPECT_EQ(value_of(after, "latest_outcome"), "Published");
  EXPECT_EQ(value_of(after, "is_antenna_transform_available"), "True");
  EXPECT_EQ(value_of(after, "is_dropping_fixes_for_missing_transform"), "False");
}

// =======================================================================================
// 3. Antenna transform lookup (antenna -> base_link) and TF broadcast (map -> gnss_base_link)
// =======================================================================================

// The published pose is map->base = map->antenna * antenna->base: the antenna orientation rotates
// the lever arm and the yaws add up. Non-default frame names pin which parameter names which frame.
// The /tf broadcast map_frame -> gnss_base_frame mirrors the pose exactly at the fix stamp, and one
// processed fix yields exactly one of each output.
TEST_F(GnssPoserNodeIntegration, Tf_StaticAntennaToBaseTransform_IsComposedAndBroadcast)
{
  NodeParams params;
  params.gnss_pose_pub_method = 0;
  params.use_gnss_ins_orientation = true;
  params.base_frame = "my_base";
  params.gnss_base_frame = "my_gnss_base";
  params.map_frame = "my_map";
  ASSERT_NO_FATAL_FAILURE(build_node(params));
  const auto projector = make_mgrs_projector_info();
  send_projector_info(projector);
  send_orientation(make_orientation(M_PI / 2.0));

  // TF: my_antenna -> my_base, translation (1, 2, 0.5), rotation yaw +90deg.
  ASSERT_NO_FATAL_FAILURE(broadcast_static_tf(
    "my_antenna", "my_base", make_point(1.0, 2.0, 0.5), yaw_to_quaternion(M_PI / 2.0)));

  const auto fix = make_reference_fix(NavSatStatus::STATUS_FIX, "my_antenna");
  send_fix(fix);
  ASSERT_NO_FATAL_FAILURE(wait_for_outputs(1));

  // map->base = map->antenna * antenna->base. Antenna yaw 90deg rotates (1,2,0.5) into
  // (-2, 1, 0.5); yaws add up to 180deg.
  const auto antenna = project_antenna(fix, projector);
  const auto expected_position = make_point(antenna.x - 2.0, antenna.y + 1.0, antenna.z + 0.5);
  const auto & pose = last_pose();
  expect_point_near(pose.pose.position, expected_position);
  EXPECT_NEAR(std::abs(yaw_of(pose.pose.orientation)), M_PI, angle_tolerance);
  EXPECT_EQ(pose.header.frame_id, "my_map");

  // The broadcast TF mirrors the pose exactly: my_map -> my_gnss_base at the fix stamp.
  const auto & tf = last_tf();
  EXPECT_EQ(tf.header.frame_id, "my_map");
  EXPECT_EQ(tf.child_frame_id, "my_gnss_base");
  EXPECT_EQ(tf.header.stamp, fix.header.stamp);
  // The transform is a field-by-field copy of the pose, hence exact equality on every component.
  EXPECT_EQ(tf.transform.translation.x, pose.pose.position.x);
  EXPECT_EQ(tf.transform.translation.y, pose.pose.position.y);
  EXPECT_EQ(tf.transform.translation.z, pose.pose.position.z);
  EXPECT_EQ(tf.transform.rotation, pose.pose.orientation);

  // Exactly one of each output per processed fix.
  EXPECT_EQ(peer_->fixed_flags.size(), 1U);
  EXPECT_EQ(peer_->poses.size(), 1U);
  EXPECT_EQ(peer_->pose_cov_msgs.size(), 1U);
  EXPECT_EQ(peer_->broadcast_tfs.size(), 1U);
}

// The antenna frame is the fix's `header.frame_id`. A fix in a frame with no transform to
// base_frame publishes `gnss_fixed` only and is held; once a fix newer by more than
// `antenna_transform_timeout_sec` arrives, the held fix is dropped without a pose. A frame equal to
// base_frame needs no TF, and a known frame gets the transform applied.
TEST_F(GnssPoserNodeIntegration, Tf_AntennaFrameComesFromFixHeader_UnknownFrameIsHeldThenDropped)
{
  NodeParams params;
  params.gnss_pose_pub_method = 0;
  params.use_gnss_ins_orientation = true;
  params.base_frame = "my_base";
  params.antenna_transform_timeout_sec = 0.5;
  ASSERT_NO_FATAL_FAILURE(build_node(params));
  const auto projector = make_mgrs_projector_info();
  send_projector_info(projector);
  send_orientation(make_orientation(M_PI / 2.0));
  ASSERT_NO_FATAL_FAILURE(broadcast_static_tf(
    "my_antenna", "my_base", make_point(1.0, 2.0, 0.5), yaw_to_quaternion(0.0)));

  // Unknown frame: gnss_fixed is published, the pose is held.
  auto fix_unknown = make_reference_fix(NavSatStatus::STATUS_FIX, "some_other_antenna");
  fix_unknown.header.stamp = make_stamp(1000, 0);
  send_fix(fix_unknown);
  ASSERT_NO_FATAL_FAILURE(wait_for_gnss_fixed(1));
  expect_output_counts(1, 0, 0, 0);
  EXPECT_TRUE(last_fixed().data);

  // A fix 1 s later in base_frame itself: the held fix expires (no pose for it) and this one is
  // published without a lookup.
  auto fix_base = make_reference_fix(NavSatStatus::STATUS_FIX, "my_base");
  fix_base.header.stamp = make_stamp(1001, 0);
  send_fix(fix_base);
  ASSERT_NO_FATAL_FAILURE(wait_for_outputs(1));
  expect_output_counts(2, 1, 1, 1);
  EXPECT_EQ(last_pose().header.stamp, fix_base.header.stamp);
  expect_point_near(last_pose().pose.position, project_antenna(fix_base, projector));

  // Known frame: the TF is applied (positive control).
  auto fix_known = make_reference_fix(NavSatStatus::STATUS_FIX, "my_antenna");
  fix_known.header.stamp = make_stamp(1001, 0);
  send_fix(fix_known);
  ASSERT_NO_FATAL_FAILURE(wait_for_outputs(2));
  const auto antenna = project_antenna(fix_known, projector);
  expect_point_near(
    last_pose().pose.position, make_point(antenna.x - 2.0, antenna.y + 1.0, antenna.z + 0.5));
}

// The antenna -> base transform is looked up at the fix's header stamp, not at "latest". With a
// transform published on /tf (time-stamped, unlike /tf_static) at exactly t0: a fix stamped t0 gets
// it applied, a fix stamped one second later needs extrapolation, which tf2 refuses, so the node
// falls back to identity, and a fix with a zero stamp means "latest" to tf2 and gets it applied
// again.
//
// The antenna transform is looked up at the fix header stamp: a fix stamped after the latest TF
// sample is held until a sample covering its stamp arrives, and is then composed with the value at
// that stamp, not with the latest one at arrival time.
//
// The antenna sits rigidly on the vehicle and is normally published on /tf_static, where time is
// ignored; a time-stamped transform is the only way to observe which time the node asks for.
TEST_F(GnssPoserNodeIntegration, Tf_LookupIsAtFixHeaderStamp)
{
  NodeParams params;
  params.gnss_pose_pub_method = 0;
  params.use_gnss_ins_orientation = true;
  params.base_frame = "my_base";
  ASSERT_NO_FATAL_FAILURE(build_node(params));
  const auto projector = make_mgrs_projector_info();
  send_projector_info(projector);
  send_orientation(make_orientation(0.0));

  const auto t0 = make_stamp(1000, 0);
  const auto t1 = make_stamp(1000, 400000000U);  // within antenna_transform_timeout_sec of t0
  ASSERT_NO_FATAL_FAILURE(broadcast_timed_tf(
    "my_antenna", "my_base", make_point(1.0, 0.0, 0.0), yaw_to_quaternion(0.0), t0));

  const auto projected = project_antenna(make_reference_fix(), projector);

  // Fix stamped exactly at t0: transform found and applied.
  auto fix_at_t0 = make_reference_fix(NavSatStatus::STATUS_FIX, "my_antenna");
  fix_at_t0.header.stamp = t0;
  send_fix(fix_at_t0);
  ASSERT_NO_FATAL_FAILURE(wait_for_outputs(1));
  expect_point_near(
    last_pose().pose.position, make_point(projected.x + 1.0, projected.y, projected.z));

  // Fix stamped at t1, after the latest TF sample: the lookup would extrapolate, so the fix is
  // held; only gnss_fixed comes out.
  auto fix_at_t1 = fix_at_t0;
  fix_at_t1.header.stamp = t1;
  send_fix(fix_at_t1);
  ASSERT_NO_FATAL_FAILURE(wait_for_gnss_fixed(2));
  pump(delivery_budget);
  expect_output_counts(2, 1, 1, 1);

  // A TF sample at t1 makes the lookup at t1 possible: the held fix is published with that sample.
  ASSERT_NO_FATAL_FAILURE(broadcast_timed_tf(
    "my_antenna", "my_base", make_point(2.0, 0.0, 0.0), yaw_to_quaternion(0.0), t1));
  ASSERT_NO_FATAL_FAILURE(wait_for_outputs(2));
  EXPECT_EQ(last_pose().header.stamp, t1);
  expect_point_near(
    last_pose().pose.position, make_point(projected.x + 2.0, projected.y, projected.z));
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  const int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
