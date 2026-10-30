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

#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <tf2/time.hpp>

#include <autoware_sensing_msgs/msg/gnss_ins_orientation_stamped.hpp>

#include <chrono>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>

namespace autoware::gnss_poser
{
namespace
{
// How long fixes may be skipped for an antenna frame that TF does not know before it is an error:
// right after startup /tf_static may arrive after the first fixes.
constexpr std::chrono::seconds antenna_frame_grace_period{5};

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

OrientationSource to_orientation_source(const std::string & value)
{
  if (value == "ins") {
    return OrientationSource::Ins;
  }
  if (value == "motion") {
    return OrientationSource::Motion;
  }
  if (value == "none") {
    return OrientationSource::None;
  }
  if (value == "ins_or_none") {
    return OrientationSource::InsOrNone;
  }
  throw std::invalid_argument(
    "orientation_source must be ins, motion, none or ins_or_none, got " + value);
}

const char * to_string(const OrientationSource source)
{
  switch (source) {
    case OrientationSource::Ins:
      return "ins";
    case OrientationSource::Motion:
      return "motion";
    case OrientationSource::None:
      return "none";
    case OrientationSource::InsOrNone:
      return "ins_or_none";
  }
  return "unknown";
}

// A parameter that may be left out: std::nullopt when no value is given. A value of another type
// than T makes the node fail to start, as for a parameter declared with its type.
template <typename T>
std::optional<T> declare_optional_parameter(
  autoware::agnocast_wrapper::Node & node, const std::string & name)
{
  rcl_interfaces::msg::ParameterDescriptor descriptor;
  descriptor.dynamic_typing = true;
  const rclcpp::ParameterValue & value =
    node.declare_parameter(name, rclcpp::ParameterValue{}, descriptor);
  if (value.get_type() == rclcpp::ParameterType::PARAMETER_NOT_SET) {
    return std::nullopt;
  }
  return value.get<T>();
}
}  // namespace

GnssPoserNode::GnssPoserNode(const rclcpp::NodeOptions & node_options)
: autoware::agnocast_wrapper::Node("gnss_poser", node_options),
  tf2_listener_(tf2_buffer_, *this),
  tf2_broadcaster_(*this),
  base_frame_(declare_parameter<std::string>("base_frame")),
  orientation_source_(declare_orientation_source()),
  antenna_transform_failure_filter_(antenna_frame_grace_period),
  gnss_poser_(declare_gnss_poser_params(), [this](const std::string & gnss_frame) {
    return get_static_transform(gnss_frame, base_frame_);
  })
{
  RCLCPP_INFO(this->get_logger(), "orientation_source: %s", to_string(orientation_source_));

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
}

GnssPoserParams GnssPoserNode::declare_gnss_poser_params()
{
  GnssPoserParams params;
  params.gnss_base_frame = declare_parameter<std::string>("gnss_base_frame");
  params.map_frame = declare_parameter<std::string>("map_frame");
  params.orientation_source = orientation_source_;
  params.gnss_pose_pub_method =
    to_gnss_pose_pub_method(declare_parameter<int>("gnss_pose_pub_method"));
  params.buff_epoch = declare_parameter<int>("buff_epoch");
  return params;
}

// orientation_source replaces use_gnss_ins_orientation. Configurations written before it set only
// the old parameter, so either one is enough: the old one is read as ins_or_none (true) or motion
// (false), and ignored when orientation_source is set.
OrientationSource GnssPoserNode::declare_orientation_source()
{
  const std::optional<std::string> source =
    declare_optional_parameter<std::string>(*this, "orientation_source");
  const std::optional<bool> use_gnss_ins_orientation =
    declare_optional_parameter<bool>(*this, "use_gnss_ins_orientation");

  if (!source) {
    if (!use_gnss_ins_orientation) {
      throw std::invalid_argument("orientation_source must be set");
    }
    const OrientationSource legacy_source =
      *use_gnss_ins_orientation ? OrientationSource::InsOrNone : OrientationSource::Motion;
    RCLCPP_WARN(
      this->get_logger(),
      "use_gnss_ins_orientation is deprecated; set orientation_source instead. "
      "use_gnss_ins_orientation: %s is read as orientation_source: %s.",
      *use_gnss_ins_orientation ? "true" : "false", to_string(legacy_source));
    return legacy_source;
  }

  if (use_gnss_ins_orientation) {
    RCLCPP_WARN(
      this->get_logger(),
      "use_gnss_ins_orientation is deprecated and ignored because orientation_source is set.");
  }
  const OrientationSource orientation_source = to_orientation_source(*source);
  if (orientation_source == OrientationSource::InsOrNone) {
    RCLCPP_WARN(
      this->get_logger(),
      "orientation_source: ins_or_none is deprecated. Use ins when autoware_orientation is "
      "published, none otherwise.");
  }
  return orientation_source;
}

void GnssPoserNode::callback_map_projector_info(
  const AUTOWARE_MESSAGE_CONST_SHARED_PTR(autoware_map_msgs::msg::MapProjectorInfo) & msg)
{
  gnss_poser_.set_projector_info(*msg);
}

void GnssPoserNode::callback_nav_sat_fix(
  const AUTOWARE_MESSAGE_CONST_SHARED_PTR(sensor_msgs::msg::NavSatFix) & nav_sat_fix_msg_ptr)
{
  const GnssPoser::Result result = gnss_poser_.input_fix(*nav_sat_fix_msg_ptr);

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

    case GnssPoser::Outcome::NoOrientation:
      RCLCPP_WARN_STREAM_THROTTLE(
        this->get_logger(), *this->get_clock(), std::chrono::milliseconds(1000).count(),
        "autoware_orientation has not been received yet. Output Skipped.");
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
  if (orientation_source_ == OrientationSource::None) {
    RCLCPP_WARN_ONCE(
      this->get_logger(),
      "autoware_orientation is received but ignored because orientation_source is none. Set "
      "orientation_source to ins to use it.");
  }
  gnss_poser_.set_ins_orientation(msg->orientation);
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
    const geometry_msgs::msg::Transform transform =
      tf2_buffer_.lookupTransform(target_frame, source_frame, tf2::TimePointZero).transform;
    antenna_transform_failure_filter_.record_success();
    return transform;
  } catch (const tf2::LookupException & ex) {
    report_transform_failure(AntennaTransformFailure::UnknownFrame, ex, target_frame, source_frame);
  } catch (const tf2::TransformException & ex) {
    report_transform_failure(AntennaTransformFailure::Unresolvable, ex, target_frame, source_frame);
  }
  return std::nullopt;
}

void GnssPoserNode::report_transform_failure(
  const AntennaTransformFailure failure, const tf2::TransformException & ex,
  const std::string & target_frame, const std::string & source_frame)
{
  if (!antenna_transform_failure_filter_.record_failure(
        failure, AntennaTransformFailureFilter::Clock::now())) {
    return;
  }
  RCLCPP_ERROR_STREAM_THROTTLE(
    this->get_logger(), *this->get_clock(), std::chrono::milliseconds(1000).count(),
    ex.what() << ". Please publish TF " << target_frame << " to " << source_frame
              << ". The fix is skipped.");
}

}  // namespace autoware::gnss_poser

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::gnss_poser::GnssPoserNode)
