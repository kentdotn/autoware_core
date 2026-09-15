// Copyright 2020 Tier IV, Inc.
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

#include "gnss_poser_node.hpp"

#include "gnss_poser_diagnostics.hpp"

#include <tf2/time.hpp>

#include <autoware_sensing_msgs/msg/gnss_ins_orientation_stamped.hpp>

#include <chrono>
#include <cmath>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace autoware::gnss_poser
{
namespace
{
// How often a fix that waits for its antenna transform is retried. Only used when the antenna
// moves; short enough to stay well inside one fix period at the usual GNSS rates.
constexpr std::chrono::milliseconds transform_update_period{20};

GnssPosePubMethod to_gnss_pose_pub_method(const int value)
{
  switch (value) {
    case 0:
      return GnssPosePubMethod::Instant;
    case 1:
      return GnssPosePubMethod::Average;
    case 2:
      return GnssPosePubMethod::Median;
    default:
      throw std::invalid_argument(
        "gnss_pose_pub_method must be 0 (instant), 1 (average) or 2 (median), got " +
        std::to_string(value));
  }
}
}  // namespace

GnssPoserNode::GnssPoserNode(const rclcpp::NodeOptions & node_options)
: autoware::agnocast_wrapper::Node("gnss_poser", node_options),
  tf2_listener_(tf2_buffer_, *this),
  tf2_broadcaster_(*this),
  base_frame_(declare_parameter<std::string>("base_frame")),
  gnss_poser_(make_gnss_poser())
{
  // Subscribe to map_projector_info topic
  sub_map_projector_info_ = create_subscription<autoware_map_msgs::msg::MapProjectorInfo>(
    "/map/map_projector_info", rclcpp::QoS{1}.transient_local(),
    std::bind(&GnssPoserNode::callback_map_projector_info, this, std::placeholders::_1));

  // Set subscribers and publishers
  nav_sat_fix_sub_ = create_subscription<sensor_msgs::msg::NavSatFix>(
    "fix", rclcpp::QoS{1},
    std::bind(&GnssPoserNode::callback_nav_sat_fix, this, std::placeholders::_1));
  autoware_orientation_sub_ =
    create_subscription<autoware_sensing_msgs::msg::GnssInsOrientationStamped>(
      "autoware_orientation", rclcpp::QoS{1},
      std::bind(
        &GnssPoserNode::callback_gnss_ins_orientation_stamped, this, std::placeholders::_1));

  pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("gnss_pose", rclcpp::QoS{1});
  pose_cov_pub_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "gnss_pose_cov", rclcpp::QoS{1});
  fixed_pub_ =
    create_publisher<autoware_internal_debug_msgs::msg::BoolStamped>("gnss_fixed", rclcpp::QoS{1});

  diagnostics_ = std::make_unique<
    autoware_utils_diagnostics::BasicDiagnosticsInterface<autoware::agnocast_wrapper::Node>>(
    this, "gnss_poser_status");
  diagnostics_timer_ = autoware::agnocast_wrapper::create_timer(
    this, this->get_clock(), std::chrono::milliseconds(100),
    std::bind(&GnssPoserNode::publish_diagnostics, this));
}

std::unique_ptr<GnssPoserInterface> GnssPoserNode::make_gnss_poser()
{
  const GnssPoserParams params = declare_gnss_poser_params();
  const bool is_dynamic = declare_parameter<bool>("antenna_transform_is_dynamic");
  const double timeout_sec = declare_parameter<double>("antenna_transform_timeout_sec");

  if (!is_dynamic) {
    return std::make_unique<StaticGnssPoser>(params, [this](const std::string & gnss_frame) {
      return get_static_transform(gnss_frame, base_frame_);
    });
  }

  // The antenna moves, so a fix waits for the transform stamped like it; the wait is polled
  // because TF arrival is not observable, and it costs the poll period in latency.
  transform_update_timer_ = autoware::agnocast_wrapper::create_timer(
    this, this->get_clock(), transform_update_period,
    [this]() { handle_results(gnss_poser_->input_transform_update()); });
  return std::make_unique<DynamicGnssPoser>(
    params,
    [this](const std::string & gnss_frame, const builtin_interfaces::msg::Time & stamp) {
      return get_transform_at(gnss_frame, base_frame_, stamp);
    },
    timeout_sec);
}

GnssPoserParams GnssPoserNode::declare_gnss_poser_params()
{
  GnssPoserParams params;
  params.gnss_base_frame = declare_parameter<std::string>("gnss_base_frame");
  params.map_frame = declare_parameter<std::string>("map_frame");
  params.use_gnss_ins_orientation = declare_parameter<bool>("use_gnss_ins_orientation");
  params.gnss_pose_pub_method =
    to_gnss_pose_pub_method(declare_parameter<int>("gnss_pose_pub_method"));
  params.buff_epoch = declare_parameter<int>("buff_epoch");
  return params;
}

void GnssPoserNode::callback_map_projector_info(
  const AUTOWARE_MESSAGE_CONST_SHARED_PTR(autoware_map_msgs::msg::MapProjectorInfo) & msg)
{
  gnss_poser_->set_projector_info(*msg);
}

void GnssPoserNode::callback_nav_sat_fix(
  const AUTOWARE_MESSAGE_CONST_SHARED_PTR(sensor_msgs::msg::NavSatFix) & nav_sat_fix_msg_ptr)
{
  latest_fix_stamp_ = nav_sat_fix_msg_ptr->header.stamp;
  antenna_frame_ = nav_sat_fix_msg_ptr->header.frame_id;
  handle_results(gnss_poser_->input_fix(*nav_sat_fix_msg_ptr));
}

void GnssPoserNode::handle_results(const std::vector<GnssPoser::Result> & results)
{
  for (const GnssPoser::Result & result : results) {
    handle_result(result);
  }
}

void GnssPoserNode::handle_result(const GnssPoser::Result & result)
{
  switch (result.outcome) {
    case GnssPoser::Outcome::NoProjectorInfo:
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), std::chrono::milliseconds(1000).count(),
        "map_projector_info has not been received yet. Check if the map_projection_loader is "
        "successfully launched.");
      break;

    case GnssPoser::Outcome::LocalProjector:
      RCLCPP_ERROR_THROTTLE(
        this->get_logger(), *this->get_clock(), std::chrono::milliseconds(5000).count(),
        "map_projector_info is local projector type. Unable to convert GNSS pose.");
      break;

    case GnssPoser::Outcome::NotFixed:
      RCLCPP_WARN_STREAM_THROTTLE(
        this->get_logger(), *this->get_clock(), std::chrono::milliseconds(1000).count(),
        "Not Fixed Topic. Skipping Calculate.");
      break;

    case GnssPoser::Outcome::Buffering:
      RCLCPP_WARN_STREAM_THROTTLE(
        this->get_logger(), *this->get_clock(), std::chrono::milliseconds(1000).count(),
        "Buffering Position. Output Skipped.");
      break;

    case GnssPoser::Outcome::NoAntennaTransform:
      // The receiver has a position solution, which is what gnss_fixed reports; only the pose
      // cannot be derived. get_static_transform() has already warned about the missing transform.
      break;

    case GnssPoser::Outcome::Published:
      break;
  }

  publish_data(result);
}

// The pose computation decides what a fix produces; the node only has to know where each output
// goes.
void GnssPoserNode::publish_data(const GnssPoser::Result & result)
{
  if (result.gnss_fixed) {
    fixed_pub_->publish(*result.gnss_fixed);
  }
  if (result.gnss_pose) {
    pose_pub_->publish(*result.gnss_pose);
  }
  if (result.gnss_pose_cov) {
    pose_cov_pub_->publish(*result.gnss_pose_cov);
  }
  if (result.transform) {
    tf2_broadcaster_.sendTransform(*result.transform);
  }
}

void GnssPoserNode::callback_gnss_ins_orientation_stamped(
  const AUTOWARE_MESSAGE_CONST_SHARED_PTR(autoware_sensing_msgs::msg::GnssInsOrientationStamped) &
  msg)
{
  gnss_poser_->set_ins_orientation(msg->orientation);
}

std::optional<geometry_msgs::msg::Transform> GnssPoserNode::get_static_transform(
  const std::string & target_frame, const std::string & source_frame)
{
  if (target_frame == source_frame) {
    return geometry_msgs::msg::Transform{};  // identity: zero translation, rotation w = 1
  }

  try {
    // tf2::TimePointZero asks for the latest transform. The antenna is rigidly mounted, so every
    // sample of this transform carries the same value and the latest one applies to every fix.
    // Asking for the fix stamp instead would fail whenever the relation is published on /tf rather
    // than /tf_static and the fix is newer than the last /tf message, which tf2 refuses to
    // extrapolate.
    return tf2_buffer_.lookupTransform(target_frame, source_frame, tf2::TimePointZero).transform;
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN_STREAM_THROTTLE(
      this->get_logger(), *this->get_clock(), std::chrono::milliseconds(1000).count(),
      ex.what() << ". Please publish TF " << target_frame << " to " << source_frame
                << ". The fix is skipped.");
    return std::nullopt;
  }
}

std::optional<geometry_msgs::msg::Transform> GnssPoserNode::get_transform_at(
  const std::string & target_frame, const std::string & source_frame,
  const builtin_interfaces::msg::Time & stamp)
{
  if (target_frame == source_frame) {
    return geometry_msgs::msg::Transform{};  // identity: zero translation, rotation w = 1
  }

  try {
    return tf2_buffer_
      .lookupTransform(
        target_frame, source_frame,
        tf2::TimePoint(std::chrono::seconds(stamp.sec) + std::chrono::nanoseconds(stamp.nanosec)))
      .transform;
  } catch (const tf2::TransformException &) {
    // Not available (yet). The fix waits for it, and the queue decides when to give up; the
    // diagnostics report both states, so a throttled log per attempt would only add noise.
    return std::nullopt;
  }
}

void GnssPoserNode::publish_diagnostics()
{
  const GnssPoser::Status status = gnss_poser_->take_status();

  diagnostics_->clear();
  diagnostics_->add_key_value("is_arrived_first_fix", latest_fix_stamp_.has_value());
  diagnostics_->add_key_value(
    "latest_fix_time_stamp",
    latest_fix_stamp_ ? rclcpp::Time(*latest_fix_stamp_).seconds() : std::nan(""));
  diagnostics_->add_key_value(
    "is_arrived_first_map_projector_info", status.projector_info_received);
  diagnostics_->add_key_value("is_arrived_first_orientation", status.ins_orientation_received);
  diagnostics_->add_key_value(
    "latest_outcome",
    status.latest_outcome ? std::string(to_string(*status.latest_outcome)) : std::string("None"));
  diagnostics_->add_key_value("position_buffer_size", status.position_buffer_size);
  diagnostics_->add_key_value("pending_fix_count", status.pending_fix_count);
  diagnostics_->add_key_value(
    "is_dropping_fixes_for_missing_transform", status.fixes_dropped_for_missing_transform);

  DiagnosticsState state;
  state.fix_arrived = latest_fix_stamp_.has_value();
  state.projector_info_received = status.projector_info_received;
  state.projector_is_local = status.projector_is_local;
  state.latest_fix_is_fixed = status.latest_outcome != GnssPoser::Outcome::NotFixed;
  state.use_gnss_ins_orientation = status.use_gnss_ins_orientation;
  state.ins_orientation_received = status.ins_orientation_received;
  state.pending_fix_count = status.pending_fix_count;
  state.fixes_dropped_for_missing_transform = status.fixes_dropped_for_missing_transform;
  state.antenna_frame = antenna_frame_;
  state.base_frame = base_frame_;

  const DiagnosticsResult diagnostics_result = determine_diagnostics(state);
  for (const auto & entry : diagnostics_result.entries) {
    diagnostics_->update_level_and_message(entry.level, entry.message);
  }
  diagnostics_->publish(this->now());
}

}  // namespace autoware::gnss_poser

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::gnss_poser::GnssPoserNode)
