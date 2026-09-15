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

// Unit tests of the rclcpp-free logic in gnss_poser.hpp: what GnssPoser derives from a GNSS fix.
// They run without a ROS context; the node layer is covered by test_gnss_poser_node.cpp and the
// characterization suite.

#include "gnss_poser.hpp"

#include <autoware/geography_utils/height.hpp>
#include <builtin_interfaces/msg/time.hpp>
#include <tf2/LinearMath/Matrix3x3.hpp>
#include <tf2/LinearMath/Quaternion.hpp>

#include <autoware_map_msgs/msg/map_projector_info.hpp>
#include <autoware_sensing_msgs/msg/gnss_ins_orientation.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_with_covariance.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/transform.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/nav_sat_status.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

namespace
{
using autoware::gnss_poser::GnssPosePubMethod;
using autoware::gnss_poser::GnssPoser;
using autoware::gnss_poser::GnssPoserCovarianceDefaults;
using autoware::gnss_poser::GnssPoserParams;
using autoware_map_msgs::msg::MapProjectorInfo;
using autoware_sensing_msgs::msg::GnssInsOrientation;
using geometry_msgs::msg::Point;
using geometry_msgs::msg::Pose;
using geometry_msgs::msg::PoseWithCovariance;
using geometry_msgs::msg::Quaternion;
using geometry_msgs::msg::Transform;
using sensor_msgs::msg::NavSatFix;
using sensor_msgs::msg::NavSatStatus;
using Outcome = GnssPoser::Outcome;

// Reference point in Tokyo (MGRS grid 54SUE) and its projection, taken from the node
// characterization suite so that both suites pin the same numbers.
constexpr double reference_latitude = 35.62426;
constexpr double reference_longitude = 139.74252;
constexpr double reference_altitude = 10.0;
constexpr const char * reference_mgrs_grid = "54SUE";
constexpr double golden_x = 86128.181788819958;
constexpr double golden_y = 43002.610125367064;
// Geoid height at the reference point, from GeographicLib's GeoidEval with egm2008-1: the EGM2008
// altitude of a point is its WGS84 ellipsoid altitude minus this value.
constexpr double reference_geoid_height = 36.12;  // [m]
constexpr double geoid_height_tolerance = 0.01;   // [m]
constexpr double position_tolerance = 1e-4;       // [m]
constexpr double angle_tolerance = 1e-6;          // [rad]
// rmse fields of GnssInsOrientation are float32; their squares are compared with this slack.
constexpr double rmse_squared_tolerance = 1e-6;

constexpr std::size_t cov_xx = 0;
constexpr std::size_t cov_yy = 7;
constexpr std::size_t cov_zz = 14;
constexpr std::size_t cov_roll_roll = 21;
constexpr std::size_t cov_pitch_pitch = 28;
constexpr std::size_t cov_yaw_yaw = 35;

builtin_interfaces::msg::Time make_stamp(int32_t sec, uint32_t nanosec)
{
  builtin_interfaces::msg::Time stamp;
  stamp.sec = sec;
  stamp.nanosec = nanosec;
  return stamp;
}

NavSatFix make_fix(
  double latitude, double longitude, double altitude, int8_t status = NavSatStatus::STATUS_FIX,
  const std::string & frame_id = "gnss_link")
{
  NavSatFix fix;
  fix.header.stamp = make_stamp(1700000000, 123456789U);
  fix.header.frame_id = frame_id;
  fix.status.status = status;
  fix.latitude = latitude;
  fix.longitude = longitude;
  fix.altitude = altitude;
  fix.position_covariance_type = NavSatFix::COVARIANCE_TYPE_UNKNOWN;
  return fix;
}

NavSatFix make_reference_fix(int8_t status = NavSatStatus::STATUS_FIX)
{
  return make_fix(reference_latitude, reference_longitude, reference_altitude, status);
}

MapProjectorInfo make_mgrs_projector_info()
{
  MapProjectorInfo info;
  info.projector_type = MapProjectorInfo::MGRS;
  info.vertical_datum = MapProjectorInfo::WGS84;
  info.mgrs_grid = reference_mgrs_grid;
  return info;
}

MapProjectorInfo make_local_projector_info()
{
  MapProjectorInfo info;
  info.projector_type = MapProjectorInfo::LOCAL;
  return info;
}

MapProjectorInfo make_local_cartesian_utm_projector_info(
  double origin_latitude, double origin_longitude, double origin_altitude)
{
  MapProjectorInfo info;
  info.projector_type = MapProjectorInfo::LOCAL_CARTESIAN_UTM;
  info.vertical_datum = MapProjectorInfo::WGS84;
  info.map_origin.latitude = origin_latitude;
  info.map_origin.longitude = origin_longitude;
  info.map_origin.altitude = origin_altitude;
  return info;
}

// GeographicLib needs the egm2008-1 geoid dataset for the EGM2008 vertical datum; it is not part
// of the build dependencies, so the case that needs it is skipped when it is missing.
bool is_egm2008_dataset_available()
{
  try {
    autoware::geography_utils::convert_wgs84_to_egm2008(0.0, 0.0, 0.0);
    return true;
  } catch (const std::runtime_error &) {
    return false;
  }
}

GnssPoserParams make_params(
  GnssPosePubMethod method = GnssPosePubMethod::Instant, int buff_epoch = 1,
  bool use_gnss_ins_orientation = true)
{
  GnssPoserParams params;
  params.gnss_pose_pub_method = method;
  params.buff_epoch = buff_epoch;
  params.use_gnss_ins_orientation = use_gnss_ins_orientation;
  return params;
}

Quaternion yaw_to_quaternion(double yaw)
{
  tf2::Quaternion quaternion;
  quaternion.setRPY(0.0, 0.0, yaw);
  return tf2::toMsg(quaternion);
}

Transform make_transform(double x, double y, double z, double yaw = 0.0)
{
  Transform transform;
  transform.translation.x = x;
  transform.translation.y = y;
  transform.translation.z = z;
  transform.rotation = yaw_to_quaternion(yaw);
  return transform;
}

// A lookup that always succeeds with the given transform.
GnssPoser::TransformLookup constant_lookup(const Transform & transform)
{
  return [transform](const std::string &) { return std::optional<Transform>(transform); };
}

const GnssPoser::TransformLookup identity_lookup = constant_lookup(Transform{});

// A lookup that never finds a transform.
const GnssPoser::TransformLookup missing_lookup = [](const std::string &) {
  return std::optional<Transform>();
};

// A GnssPoser ready to publish: MGRS projector info received, identity lookup unless given.
GnssPoser make_ready_poser(
  const GnssPoserParams & params = make_params(),
  const GnssPoser::TransformLookup & lookup = identity_lookup)
{
  GnssPoser poser(params, lookup);
  poser.set_projector_info(make_mgrs_projector_info());
  return poser;
}

void expect_point_near(
  const Point & actual, const Point & expected, double tol = position_tolerance)
{
  EXPECT_NEAR(actual.x, expected.x, tol);
  EXPECT_NEAR(actual.y, expected.y, tol);
  EXPECT_NEAR(actual.z, expected.z, tol);
}

void expect_same_rotation(
  const Quaternion & actual, const Quaternion & expected, double tol = angle_tolerance)
{
  const double dot =
    actual.x * expected.x + actual.y * expected.y + actual.z * expected.z + actual.w * expected.w;
  const double angle = 2.0 * std::acos(std::min(1.0, std::abs(dot)));
  EXPECT_LE(angle, tol) << "rotations differ by " << angle << " rad";
}

// The pose of a Published result, taken out of the message that carries it; fails the test (and
// returns an empty pose) otherwise. What the messages around it carry is pinned by
// OutputMessagesAreReadyToPublish below.
PoseWithCovariance published(const GnssPoser::Result & result)
{
  EXPECT_EQ(result.outcome, Outcome::Published);
  if (!result.gnss_pose_cov) {
    return PoseWithCovariance{};
  }
  return result.gnss_pose_cov->pose;
}

// Where a fix lands in the map frame, read back through the Instant method, whose output is
// pinned against golden values by InstantPublishesProjectedAntennaPosition below. The cases that
// are about the buffering or the heading use it so that they can state their expectation in map
// coordinates without projecting anything themselves.
Point projected(const NavSatFix & fix)
{
  GnssPoser oracle = make_ready_poser();
  return published(oracle.input_fix(fix)).pose.position;
}

geometry_msgs::msg::Point make_point(double x, double y, double z)
{
  geometry_msgs::msg::Point point;
  point.x = x;
  point.y = y;
  point.z = z;
  return point;
}

Point mean_of(const Point & a, const Point & b)
{
  return make_point((a.x + b.x) / 2.0, (a.y + b.y) / 2.0, (a.z + b.z) / 2.0);
}

double heading_from(const Point & from, const Point & to)
{
  return std::atan2(to.y - from.y, to.x - from.x);
}

double yaw_of(const geometry_msgs::msg::Quaternion & quaternion)
{
  tf2::Quaternion tf_quaternion;
  tf2::fromMsg(quaternion, tf_quaternion);
  double roll = 0.0;
  double pitch = 0.0;
  double yaw = 0.0;
  tf2::Matrix3x3(tf_quaternion).getRPY(roll, pitch, yaw);
  return yaw;
}
}  // namespace

// ---------------------------------------------------------------------------------------------
// Construction

// A buffer of less than one position is rejected whatever the method, because the average and the
// median cannot be taken over an empty buffer.
TEST(GnssPoser, RejectsBuffEpochBelowOne)
{
  for (const auto method :
       {GnssPosePubMethod::Instant, GnssPosePubMethod::Average, GnssPosePubMethod::Median}) {
    EXPECT_THROW(GnssPoser(make_params(method, 0), identity_lookup), std::invalid_argument);
    EXPECT_THROW(GnssPoser(make_params(method, -1), identity_lookup), std::invalid_argument);
    EXPECT_NO_THROW(GnssPoser(make_params(method, 1), identity_lookup));
  }
}

// ---------------------------------------------------------------------------------------------
// Gates

// Without projector info nothing is derived from the fix, not even the fix status.
TEST(GnssPoser, WithoutProjectorInfoIsNoProjectorInfo)
{
  GnssPoser poser(make_params(), identity_lookup);

  const auto result = poser.input_fix(make_reference_fix());

  EXPECT_EQ(result.outcome, Outcome::NoProjectorInfo);
  EXPECT_FALSE(result.gnss_pose.has_value());
}

// A local projector cannot convert a GNSS fix; the outcome says so for every fix.
TEST(GnssPoser, LocalProjectorIsLocalProjector)
{
  GnssPoser poser(make_params(), identity_lookup);
  poser.set_projector_info(make_local_projector_info());

  const auto result = poser.input_fix(make_reference_fix());

  EXPECT_EQ(result.outcome, Outcome::LocalProjector);
  EXPECT_FALSE(result.gnss_pose.has_value());
}

// A fix whose status is below STATUS_FIX is reported as NotFixed and computes nothing.
TEST(GnssPoser, NotFixedStatusIsNotFixed)
{
  GnssPoser poser = make_ready_poser();

  for (const int8_t status : {NavSatStatus::STATUS_NO_FIX, NavSatStatus::STATUS_UNKNOWN}) {
    const auto result = poser.input_fix(make_reference_fix(status));
    EXPECT_EQ(result.outcome, Outcome::NotFixed) << "status " << static_cast<int>(status);
    EXPECT_FALSE(result.gnss_pose.has_value());
  }
  for (const int8_t status :
       {NavSatStatus::STATUS_FIX, NavSatStatus::STATUS_SBAS_FIX, NavSatStatus::STATUS_GBAS_FIX}) {
    EXPECT_EQ(poser.input_fix(make_reference_fix(status)).outcome, Outcome::Published)
      << "status " << static_cast<int>(status);
  }
}

// ---------------------------------------------------------------------------------------------
// Position: instant, average, median

// Instant: the projected antenna position is published on every fixed fix.
TEST(GnssPoser, InstantPublishesProjectedAntennaPosition)
{
  GnssPoser poser = make_ready_poser();

  const auto pose_with_covariance = published(poser.input_fix(make_reference_fix()));

  expect_point_near(
    pose_with_covariance.pose.position, make_point(golden_x, golden_y, reference_altitude));
}

// Average: nothing until buff_epoch fixes are buffered, then the mean of the last buff_epoch
// projected positions on every fix.
TEST(GnssPoser, AveragePublishesMeanOnceBufferIsFull)
{
  GnssPoser poser = make_ready_poser(make_params(GnssPosePubMethod::Average, 3));
  const std::vector<NavSatFix> fixes = {
    make_fix(reference_latitude, reference_longitude, 10.0),
    make_fix(reference_latitude + 1e-5, reference_longitude, 20.0),
    make_fix(reference_latitude, reference_longitude + 1e-5, 60.0),
    make_fix(reference_latitude - 1e-5, reference_longitude, 40.0),
  };
  std::vector<Point> positions;
  for (const auto & fix : fixes) {
    positions.push_back(projected(fix));
  }
  const auto mean_of = [](const Point & a, const Point & b, const Point & c) {
    return make_point((a.x + b.x + c.x) / 3.0, (a.y + b.y + c.y) / 3.0, (a.z + b.z + c.z) / 3.0);
  };

  EXPECT_EQ(poser.input_fix(fixes[0]).outcome, Outcome::Buffering);
  EXPECT_EQ(poser.input_fix(fixes[1]).outcome, Outcome::Buffering);
  expect_point_near(
    published(poser.input_fix(fixes[2])).pose.position,
    mean_of(positions[0], positions[1], positions[2]));
  // The buffer slides: the fourth fix replaces the first.
  expect_point_near(
    published(poser.input_fix(fixes[3])).pose.position,
    mean_of(positions[1], positions[2], positions[3]));
}

// Median: component-wise median of the buffered projected positions.
TEST(GnssPoser, MedianPublishesComponentWiseMedian)
{
  GnssPoser poser = make_ready_poser(make_params(GnssPosePubMethod::Median, 3));
  const std::vector<NavSatFix> fixes = {
    make_fix(reference_latitude, reference_longitude, 10.0),
    make_fix(reference_latitude + 2e-5, reference_longitude + 1e-5, 30.0),
    make_fix(reference_latitude + 1e-5, reference_longitude + 2e-5, 20.0),
  };

  EXPECT_EQ(poser.input_fix(fixes[0]).outcome, Outcome::Buffering);
  EXPECT_EQ(poser.input_fix(fixes[1]).outcome, Outcome::Buffering);
  const Point position = published(poser.input_fix(fixes[2])).pose.position;

  // Latitude and longitude offsets map to y and x respectively; the middle value of each axis is
  // the one of the third fix for y, the second for x, and 20.0 for z.
  const Point second = projected(fixes[1]);
  const Point third = projected(fixes[2]);
  EXPECT_NEAR(position.x, std::min(second.x, third.x), position_tolerance);
  EXPECT_NEAR(position.y, std::min(second.y, third.y), position_tolerance);
  EXPECT_NEAR(position.z, 20.0, position_tolerance);
}

// Median with an even buffer averages the two central values per axis, which for two buffered
// fixes is their midpoint.
TEST(GnssPoser, MedianWithEvenBufferAveragesTheTwoCentralValues)
{
  GnssPoser poser = make_ready_poser(make_params(GnssPosePubMethod::Median, 2));
  const NavSatFix first = make_fix(reference_latitude, reference_longitude, 10.0);
  const NavSatFix second = make_fix(reference_latitude + 2e-5, reference_longitude + 2e-5, 30.0);
  const Point a = projected(first);
  const Point b = projected(second);

  EXPECT_EQ(poser.input_fix(first).outcome, Outcome::Buffering);
  const Point position = published(poser.input_fix(second)).pose.position;

  expect_point_near(position, make_point((a.x + b.x) / 2.0, (a.y + b.y) / 2.0, (a.z + b.z) / 2.0));
}

// Motion mode: the heading is atan2 of the displacement in the map frame, over all four
// quadrants including the wrap-around at +-pi. The expectation is the direction between the two
// projected positions rather than the compass direction of the latitude / longitude offset: on a
// projected grid the two differ by the meridian convergence, about 0.7 degrees at the reference
// point, which is why the intended direction is only checked coarsely.
TEST(GnssPoser, MotionOrientationCoversTheFourQuadrants)
{
  constexpr double offset = 1e-4;           // ~10 m, well above the projection tolerance
  constexpr double direction_slack = 0.05;  // [rad], room for the grid convergence
  const struct
  {
    const char * name;
    double latitude_offset;
    double longitude_offset;
    double intended_yaw;
  } directions[] = {
    {"east", 0.0, offset, 0.0},
    {"north", offset, 0.0, M_PI / 2.0},
    {"west", 0.0, -offset, M_PI},
    {"south", -offset, 0.0, -M_PI / 2.0},
  };

  for (const auto & direction : directions) {
    GnssPoser poser = make_ready_poser(make_params(GnssPosePubMethod::Instant, 1, false));
    const NavSatFix from = make_reference_fix();
    const NavSatFix to = make_fix(
      reference_latitude + direction.latitude_offset,
      reference_longitude + direction.longitude_offset, reference_altitude);
    const Point a = projected(from);
    const Point b = projected(to);
    const double expected_yaw = std::atan2(b.y - a.y, b.x - a.x);

    EXPECT_EQ(poser.input_fix(from).outcome, Outcome::Published);
    const double yaw = yaw_of(published(poser.input_fix(to)).pose.orientation);

    expect_same_rotation(yaw_to_quaternion(expected_yaw), yaw_to_quaternion(yaw));
    // West is the wrap-around point, where +pi and -pi are the same heading.
    EXPECT_NEAR(std::abs(direction.intended_yaw), std::abs(yaw), direction_slack)
      << "moving " << direction.name;
  }
}

// Instant ignores buff_epoch: every fixed fix is published on its own, even with a large buffer
// size configured.
TEST(GnssPoser, InstantIgnoresBuffEpoch)
{
  GnssPoser poser = make_ready_poser(make_params(GnssPosePubMethod::Instant, 5));
  const std::vector<NavSatFix> fixes = {
    make_fix(reference_latitude, reference_longitude, reference_altitude),
    make_fix(reference_latitude + 0.001, reference_longitude, reference_altitude),
    make_fix(reference_latitude + 0.002, reference_longitude + 0.001, reference_altitude + 1.0),
  };
  for (const auto & fix : fixes) {
    expect_point_near(published(poser.input_fix(fix)).pose.position, projected(fix));
  }
}

// A fix without a fix status does not enter the position buffer.
TEST(GnssPoser, BufferIgnoresNotFixedFixes)
{
  GnssPoser poser = make_ready_poser(make_params(GnssPosePubMethod::Average, 2));
  const NavSatFix fix1 = make_reference_fix();
  const NavSatFix no_fix = make_fix(
    reference_latitude + 0.01, reference_longitude + 0.01, reference_altitude + 100.0,
    NavSatStatus::STATUS_NO_FIX);
  const NavSatFix fix2 =
    make_fix(reference_latitude + 0.001, reference_longitude + 0.001, reference_altitude + 10.0);

  EXPECT_EQ(poser.input_fix(fix1).outcome, Outcome::Buffering);
  EXPECT_EQ(poser.input_fix(no_fix).outcome, Outcome::NotFixed);
  expect_point_near(
    published(poser.input_fix(fix2)).pose.position, mean_of(projected(fix1), projected(fix2)));
}

// Fixes rejected at the projector gates (no projector info yet, or a local projector) never reach
// the position buffer: gates first, then buffer.
TEST(GnssPoser, BufferIgnoresGatedFixes)
{
  GnssPoser poser(make_params(GnssPosePubMethod::Average, 2), identity_lookup);
  const NavSatFix dropped_no_info =
    make_fix(reference_latitude + 0.01, reference_longitude + 0.01, reference_altitude + 100.0);
  const NavSatFix dropped_local =
    make_fix(reference_latitude - 0.01, reference_longitude - 0.01, reference_altitude - 100.0);
  const NavSatFix fix1 = make_reference_fix();
  const NavSatFix fix2 =
    make_fix(reference_latitude + 0.001, reference_longitude + 0.001, reference_altitude + 10.0);

  EXPECT_EQ(poser.input_fix(dropped_no_info).outcome, Outcome::NoProjectorInfo);
  poser.set_projector_info(make_local_projector_info());
  EXPECT_EQ(poser.input_fix(dropped_local).outcome, Outcome::LocalProjector);
  poser.set_projector_info(make_mgrs_projector_info());
  EXPECT_EQ(poser.input_fix(fix1).outcome, Outcome::Buffering);  // one slot filled, not full
  expect_point_near(
    published(poser.input_fix(fix2)).pose.position, mean_of(projected(fix1), projected(fix2)));
}

// ---------------------------------------------------------------------------------------------
// Orientation

// INS mode before any orientation arrived: the identity orientation and
// default_ins_placeholder_rmse (1.0 rad) squared, so 1.0 rad^2 on every rotation axis, stand in
// for the missing message.
TEST(GnssPoser, InsOrientationDefaultsToIdentityWithUnitVariances)
{
  GnssPoser poser = make_ready_poser();

  const auto pose_with_covariance = published(poser.input_fix(make_reference_fix()));

  expect_same_rotation(pose_with_covariance.pose.orientation, Quaternion{});
  EXPECT_DOUBLE_EQ(pose_with_covariance.covariance[cov_roll_roll], 1.0);
  EXPECT_DOUBLE_EQ(pose_with_covariance.covariance[cov_pitch_pitch], 1.0);
  EXPECT_DOUBLE_EQ(pose_with_covariance.covariance[cov_yaw_yaw], 1.0);
}

// INS mode: the latest orientation is used as-is and its rmse values squared become the rotation
// variances: 0.1^2 = 0.01, 0.2^2 = 0.04, 0.3^2 = 0.09.
TEST(GnssPoser, InsOrientationIsUsedWithItsRmseSquared)
{
  GnssPoser poser = make_ready_poser();
  GnssInsOrientation orientation;
  orientation.orientation = yaw_to_quaternion(M_PI / 2.0);
  orientation.rmse_rotation_x = 0.1F;
  orientation.rmse_rotation_y = 0.2F;
  orientation.rmse_rotation_z = 0.3F;
  poser.set_ins_orientation(orientation);

  const auto pose_with_covariance = published(poser.input_fix(make_reference_fix()));

  expect_same_rotation(pose_with_covariance.pose.orientation, yaw_to_quaternion(M_PI / 2.0));
  EXPECT_NEAR(pose_with_covariance.covariance[cov_roll_roll], 0.01, rmse_squared_tolerance);
  EXPECT_NEAR(pose_with_covariance.covariance[cov_pitch_pitch], 0.04, rmse_squared_tolerance);
  EXPECT_NEAR(pose_with_covariance.covariance[cov_yaw_yaw], 0.09, rmse_squared_tolerance);
}

// INS mode: a newer orientation message replaces the one in use, rmse included. Only the latest
// one is kept, so 0.5^2 = 0.25, 0.6^2 = 0.36, 0.7^2 = 0.49 are what the next fix carries.
TEST(GnssPoser, InsOrientationLatestMessageReplacesThePrevious)
{
  GnssPoser poser = make_ready_poser();
  GnssInsOrientation first;
  first.orientation = yaw_to_quaternion(M_PI / 2.0);
  first.rmse_rotation_x = 0.1F;
  first.rmse_rotation_y = 0.2F;
  first.rmse_rotation_z = 0.3F;
  poser.set_ins_orientation(first);
  EXPECT_EQ(poser.input_fix(make_reference_fix()).outcome, Outcome::Published);

  GnssInsOrientation second;
  second.orientation = yaw_to_quaternion(-M_PI / 4.0);
  second.rmse_rotation_x = 0.5F;
  second.rmse_rotation_y = 0.6F;
  second.rmse_rotation_z = 0.7F;
  poser.set_ins_orientation(second);

  const auto pose_with_covariance = published(poser.input_fix(make_reference_fix()));

  expect_same_rotation(pose_with_covariance.pose.orientation, yaw_to_quaternion(-M_PI / 4.0));
  EXPECT_NEAR(pose_with_covariance.covariance[cov_roll_roll], 0.25, rmse_squared_tolerance);
  EXPECT_NEAR(pose_with_covariance.covariance[cov_pitch_pitch], 0.36, rmse_squared_tolerance);
  EXPECT_NEAR(pose_with_covariance.covariance[cov_yaw_yaw], 0.49, rmse_squared_tolerance);
}

// Motion mode: the first fix has no previous position and yields the identity orientation; from
// the second fix on, yaw points from the previous to the current position. The rotation variances
// are default_motion_rotation_variances (0.1, 0.1, 1.0).
TEST(GnssPoser, MotionOrientationIsIdentityFirstThenHeading)
{
  GnssPoser poser = make_ready_poser(make_params(GnssPosePubMethod::Instant, 1, false));
  const NavSatFix first = make_reference_fix();
  const NavSatFix second = make_fix(reference_latitude + 1e-4, reference_longitude, 10.0);

  const auto first_pose = published(poser.input_fix(first));
  expect_same_rotation(first_pose.pose.orientation, Quaternion{});
  EXPECT_DOUBLE_EQ(first_pose.covariance[cov_roll_roll], 0.1);
  EXPECT_DOUBLE_EQ(first_pose.covariance[cov_pitch_pitch], 0.1);
  EXPECT_DOUBLE_EQ(first_pose.covariance[cov_yaw_yaw], 1.0);

  const Point from = projected(first);
  const Point to = projected(second);
  const double expected_yaw = std::atan2(to.y - from.y, to.x - from.x);
  const auto second_pose = published(poser.input_fix(second));
  expect_same_rotation(second_pose.pose.orientation, yaw_to_quaternion(expected_yaw));
}

// Motion mode: a fix without a fix status does not update the previous position, so the heading
// after it is measured from the last published position.
TEST(GnssPoser, MotionOrientationPreviousPositionSurvivesNotFixedFixes)
{
  GnssPoser poser = make_ready_poser(make_params(GnssPosePubMethod::Instant, 1, false));
  const NavSatFix fix1 = make_reference_fix();
  const NavSatFix no_fix = make_fix(
    reference_latitude - 0.01, reference_longitude - 0.01, reference_altitude,
    NavSatStatus::STATUS_NO_FIX);
  const NavSatFix fix2 =
    make_fix(reference_latitude + 0.001, reference_longitude, reference_altitude);

  EXPECT_EQ(poser.input_fix(fix1).outcome, Outcome::Published);
  EXPECT_EQ(poser.input_fix(no_fix).outcome, Outcome::NotFixed);
  const auto second_pose = published(poser.input_fix(fix2));

  const Point p1 = projected(fix1);
  const Point p2 = projected(fix2);
  expect_same_rotation(second_pose.pose.orientation, yaw_to_quaternion(heading_from(p1, p2)));
}

// Motion mode with a buffer: the heading is derived from consecutive averaged positions, not from
// the raw antenna positions.
TEST(GnssPoser, MotionOrientationWithBufferUsesFilteredPositions)
{
  GnssPoser poser = make_ready_poser(make_params(GnssPosePubMethod::Average, 2, false));
  const std::vector<NavSatFix> fixes = {
    make_fix(reference_latitude, reference_longitude, reference_altitude),
    make_fix(reference_latitude + 0.001, reference_longitude, reference_altitude),
    make_fix(reference_latitude + 0.001, reference_longitude + 0.002, reference_altitude),
  };
  std::vector<Point> antenna;
  for (const auto & fix : fixes) {
    antenna.push_back(projected(fix));
  }
  const Point m12 = mean_of(antenna[0], antenna[1]);
  const Point m23 = mean_of(antenna[1], antenna[2]);

  EXPECT_EQ(poser.input_fix(fixes[0]).outcome, Outcome::Buffering);
  const auto first_pose = published(poser.input_fix(fixes[1]));
  expect_same_rotation(first_pose.pose.orientation, Quaternion{});  // first published position
  const auto second_pose = published(poser.input_fix(fixes[2]));
  expect_same_rotation(second_pose.pose.orientation, yaw_to_quaternion(heading_from(m12, m23)));
}

// ---------------------------------------------------------------------------------------------
// Antenna transform

// The lookup receives the fix header frame, and is called only once a pose is composed.
TEST(GnssPoser, LookupReceivesFixFrameOnlyWhenComposing)
{
  std::vector<std::string> calls;
  GnssPoser poser =
    make_ready_poser(make_params(GnssPosePubMethod::Average, 2), [&](const std::string & frame) {
      calls.push_back(frame);
      return std::optional<Transform>(Transform{});
    });
  NavSatFix fix = make_reference_fix();
  fix.header.frame_id = "antenna_frame";

  EXPECT_EQ(
    poser.input_fix(make_reference_fix(NavSatStatus::STATUS_NO_FIX)).outcome, Outcome::NotFixed);
  EXPECT_EQ(poser.input_fix(fix).outcome, Outcome::Buffering);
  EXPECT_TRUE(calls.empty());

  EXPECT_EQ(poser.input_fix(fix).outcome, Outcome::Published);
  ASSERT_EQ(calls.size(), 1U);
  EXPECT_EQ(calls[0], "antenna_frame");
}

// The base_link pose is the antenna pose composed with the looked-up transform: the translation
// is applied in the antenna orientation.
TEST(GnssPoser, ComposesAntennaPoseWithLookedUpTransform)
{
  GnssPoser poser = make_ready_poser(make_params(), constant_lookup(make_transform(1.0, 2.0, 3.0)));
  GnssInsOrientation orientation;
  orientation.orientation = yaw_to_quaternion(M_PI / 2.0);  // antenna faces +y
  poser.set_ins_orientation(orientation);

  const auto pose_with_covariance = published(poser.input_fix(make_reference_fix()));

  // +x in the antenna frame is +y in the map, +y in the antenna frame is -x in the map.
  expect_point_near(
    pose_with_covariance.pose.position,
    make_point(golden_x - 2.0, golden_y + 1.0, reference_altitude + 3.0));
  expect_same_rotation(pose_with_covariance.pose.orientation, yaw_to_quaternion(M_PI / 2.0));
}

// When the lookup cannot provide the transform, the fix yields no pose. It has still been
// observed: it stays in the position buffer, and the next fix is published once the transform
// becomes available.
TEST(GnssPoser, MissingTransformIsNoAntennaTransform)
{
  std::optional<Transform> available;  // no transform until it is assigned
  GnssPoser poser = make_ready_poser(
    make_params(GnssPosePubMethod::Average, 1),
    [&available](const std::string &) { return available; });

  const auto result = poser.input_fix(make_reference_fix());

  EXPECT_EQ(result.outcome, Outcome::NoAntennaTransform);
  EXPECT_FALSE(result.gnss_pose.has_value());
  // The receiver had a position solution; only the pose could not be derived from it.
  ASSERT_TRUE(result.gnss_fixed.has_value());
  EXPECT_TRUE(result.gnss_fixed->data);
  EXPECT_EQ(poser.take_status().position_buffer_size, 1U);

  available = make_transform(1.0, 2.0, 3.0);
  const auto pose_with_covariance = published(poser.input_fix(make_reference_fix()));
  expect_point_near(
    pose_with_covariance.pose.position,
    make_point(golden_x + 1.0, golden_y + 2.0, reference_altitude + 3.0));
}

// ---------------------------------------------------------------------------------------------
// Output messages

// gnss_fixed is stamped like the fix it reports on, whichever outcome that fix produced.
TEST(GnssPoser, GnssFixedCarriesTheFixStamp)
{
  GnssPoser poser = make_ready_poser(make_params(GnssPosePubMethod::Average, 2));
  NavSatFix not_fixed = make_reference_fix(NavSatStatus::STATUS_NO_FIX);
  not_fixed.header.stamp = make_stamp(10, 0);
  NavSatFix fixed = make_reference_fix();
  fixed.header.stamp = make_stamp(11, 0);

  const GnssPoser::Result no_fix = poser.input_fix(not_fixed);
  ASSERT_TRUE(no_fix.gnss_fixed.has_value());
  EXPECT_EQ(no_fix.gnss_fixed->stamp, not_fixed.header.stamp);

  const GnssPoser::Result buffering = poser.input_fix(fixed);
  ASSERT_EQ(buffering.outcome, Outcome::Buffering);
  ASSERT_TRUE(buffering.gnss_fixed.has_value());
  EXPECT_EQ(buffering.gnss_fixed->stamp, fixed.header.stamp);

  const GnssPoser::Result published = poser.input_fix(fixed);
  ASSERT_EQ(published.outcome, Outcome::Published);
  ASSERT_TRUE(published.gnss_fixed.has_value());
  EXPECT_EQ(published.gnss_fixed->stamp, fixed.header.stamp);
}

// A published fix produces the three messages, complete: every one is stamped with the fix header
// stamp, the pose and its covariance are in map_frame and carry the same pose, and the transform
// is that pose again as map_frame -> gnss_base_frame. Both frames come from the parameters.
TEST(GnssPoser, OutputMessagesAreReadyToPublish)
{
  GnssPoserParams params = make_params();
  params.map_frame = "my_map";
  params.gnss_base_frame = "my_gnss_base";
  GnssPoser poser = make_ready_poser(params, constant_lookup(make_transform(1.0, 2.0, 3.0)));
  NavSatFix fix = make_reference_fix();
  fix.header.stamp = make_stamp(1234, 5678U);

  const GnssPoser::Result result = poser.input_fix(fix);

  ASSERT_EQ(result.outcome, Outcome::Published);
  ASSERT_TRUE(result.gnss_fixed.has_value());
  EXPECT_TRUE(result.gnss_fixed->data);
  EXPECT_EQ(result.gnss_fixed->stamp, fix.header.stamp);
  ASSERT_TRUE(result.gnss_pose.has_value());
  ASSERT_TRUE(result.gnss_pose_cov.has_value());
  ASSERT_TRUE(result.transform.has_value());

  EXPECT_EQ(result.gnss_pose->header.stamp, fix.header.stamp);
  EXPECT_EQ(result.gnss_pose->header.frame_id, "my_map");
  expect_point_near(
    result.gnss_pose->pose.position,
    make_point(golden_x + 1.0, golden_y + 2.0, reference_altitude + 3.0));

  // The covariance message carries the same header and the same pose, hence exact equality.
  EXPECT_EQ(result.gnss_pose_cov->header, result.gnss_pose->header);
  EXPECT_EQ(result.gnss_pose_cov->pose.pose, result.gnss_pose->pose);

  // The transform is a field-by-field copy of the pose under a child frame of its own.
  EXPECT_EQ(result.transform->header, result.gnss_pose->header);
  EXPECT_EQ(result.transform->child_frame_id, "my_gnss_base");
  EXPECT_EQ(result.transform->transform.translation.x, result.gnss_pose->pose.position.x);
  EXPECT_EQ(result.transform->transform.translation.y, result.gnss_pose->pose.position.y);
  EXPECT_EQ(result.transform->transform.translation.z, result.gnss_pose->pose.position.z);
  EXPECT_EQ(result.transform->transform.rotation, result.gnss_pose->pose.orientation);
}

// What each outcome produces, so that the caller can publish whatever is there without asking
// why: a pose only for a fix that passed every gate, and the receiver status for every fix that
// reached the fixed check and for no other.
TEST(GnssPoser, WhatIsProducedFollowsTheOutcome)
{
  const auto expect_no_pose = [](const GnssPoser::Result & result) {
    EXPECT_FALSE(result.gnss_pose.has_value());
    EXPECT_FALSE(result.gnss_pose_cov.has_value());
    EXPECT_FALSE(result.transform.has_value());
  };

  GnssPoser without_info(make_params(), identity_lookup);
  const GnssPoser::Result no_info = without_info.input_fix(make_reference_fix());
  expect_no_pose(no_info);
  EXPECT_FALSE(no_info.gnss_fixed.has_value());  // the fix never reached the fixed check

  GnssPoser local(make_params(), identity_lookup);
  local.set_projector_info(make_local_projector_info());
  const GnssPoser::Result local_projector = local.input_fix(make_reference_fix());
  expect_no_pose(local_projector);
  EXPECT_FALSE(local_projector.gnss_fixed.has_value());

  GnssPoser ready = make_ready_poser(make_params(GnssPosePubMethod::Average, 2));
  const GnssPoser::Result not_fixed =
    ready.input_fix(make_reference_fix(NavSatStatus::STATUS_NO_FIX));
  expect_no_pose(not_fixed);
  ASSERT_TRUE(not_fixed.gnss_fixed.has_value());
  EXPECT_FALSE(not_fixed.gnss_fixed->data);
  EXPECT_EQ(not_fixed.gnss_fixed->stamp, make_reference_fix().header.stamp);

  const GnssPoser::Result buffering = ready.input_fix(make_reference_fix());
  expect_no_pose(buffering);
  ASSERT_TRUE(buffering.gnss_fixed.has_value());
  EXPECT_TRUE(buffering.gnss_fixed->data);

  const GnssPoser::Result published = ready.input_fix(make_reference_fix());
  ASSERT_EQ(published.outcome, Outcome::Published);
  ASSERT_TRUE(published.gnss_fixed.has_value());
  EXPECT_TRUE(published.gnss_fixed->data);
}

// ---------------------------------------------------------------------------------------------
// Covariance

// Position variances come from the fix diagonal when the receiver reports a covariance type, and
// are default_unknown_position_variances (10.0 each) otherwise. Off-diagonal terms are always
// zero.
TEST(GnssPoser, PositionVariancesFromFixOrDefault)
{
  GnssPoser poser = make_ready_poser();

  NavSatFix known = make_reference_fix();
  known.position_covariance_type = NavSatFix::COVARIANCE_TYPE_DIAGONAL_KNOWN;
  known.position_covariance = {1.5, 9.0, 9.0, 9.0, 2.5, 9.0, 9.0, 9.0, 3.5};
  const auto from_fix = published(poser.input_fix(known));
  EXPECT_DOUBLE_EQ(from_fix.covariance[cov_xx], 1.5);
  EXPECT_DOUBLE_EQ(from_fix.covariance[cov_yy], 2.5);
  EXPECT_DOUBLE_EQ(from_fix.covariance[cov_zz], 3.5);
  for (std::size_t i = 0; i < 36; ++i) {
    if (i % 7 != 0) {
      EXPECT_DOUBLE_EQ(from_fix.covariance[i], 0.0) << "index " << i;
    }
  }

  const auto defaulted = published(poser.input_fix(make_reference_fix()));
  EXPECT_DOUBLE_EQ(defaulted.covariance[cov_xx], 10.0);
  EXPECT_DOUBLE_EQ(defaulted.covariance[cov_yy], 10.0);
  EXPECT_DOUBLE_EQ(defaulted.covariance[cov_zz], 10.0);
}

// ---------------------------------------------------------------------------------------------
// Status

// take_status() reports the configured orientation source, what has arrived, whether the projector
// is usable, the buffer fill and the latest outcome.
TEST(GnssPoser, TakeStatusReflectsInputsAndLatestOutcome)
{
  GnssPoser poser(make_params(GnssPosePubMethod::Average, 2), identity_lookup);

  const auto initial = poser.take_status();
  EXPECT_TRUE(initial.use_gnss_ins_orientation);
  EXPECT_FALSE(initial.projector_info_received);
  EXPECT_FALSE(initial.projector_is_local);
  EXPECT_FALSE(initial.ins_orientation_received);
  EXPECT_EQ(initial.position_buffer_size, 0U);
  EXPECT_FALSE(initial.latest_outcome.has_value());

  poser.set_projector_info(make_local_projector_info());
  poser.input_fix(make_reference_fix());
  const auto local = poser.take_status();
  EXPECT_TRUE(local.projector_info_received);
  EXPECT_TRUE(local.projector_is_local);
  EXPECT_EQ(local.latest_outcome, Outcome::LocalProjector);

  poser.set_projector_info(make_mgrs_projector_info());
  poser.set_ins_orientation(GnssInsOrientation{});
  poser.input_fix(make_reference_fix());
  const auto buffering = poser.take_status();
  EXPECT_FALSE(buffering.projector_is_local);
  EXPECT_TRUE(buffering.ins_orientation_received);
  EXPECT_EQ(buffering.position_buffer_size, 1U);
  EXPECT_EQ(buffering.latest_outcome, Outcome::Buffering);
}

// ---------------------------------------------------------------------------------------------
// Stage functions

// ---------------------------------------------------------------------------------------------
// Projector variants and injected covariance

// The vertical datum of the projector info is honored: with EGM2008 the WGS84 ellipsoid altitude
// of the fix is converted to a geoid height, about 36.12 m lower at the reference point.
TEST(GnssPoser, Egm2008DatumLowersThePublishedAltitude)
{
  if (!is_egm2008_dataset_available()) {
    GTEST_SKIP() << "egm2008-1 geoid dataset is not installed";
  }
  MapProjectorInfo projector_info = make_mgrs_projector_info();
  projector_info.vertical_datum = MapProjectorInfo::EGM2008;
  GnssPoser poser(make_params(), identity_lookup);
  poser.set_projector_info(projector_info);

  const auto pose_with_covariance = published(poser.input_fix(make_reference_fix()));

  expect_point_near(
    pose_with_covariance.pose.position,
    make_point(golden_x, golden_y, reference_altitude - reference_geoid_height),
    geoid_height_tolerance);
}

// The projector info is forwarded to the projection library as-is: with LOCAL_CARTESIAN_UTM the
// map origin is honored, so a fix at the origin lands at (0, 0, altitude - origin altitude).
TEST(GnssPoser, LocalCartesianUtmProjectorHonorsTheMapOrigin)
{
  GnssPoser poser(make_params(), identity_lookup);
  poser.set_projector_info(
    make_local_cartesian_utm_projector_info(reference_latitude, reference_longitude, -10.0));

  const auto pose_with_covariance = published(poser.input_fix(make_reference_fix()));

  expect_point_near(
    pose_with_covariance.pose.position, make_point(0.0, 0.0, reference_altitude - (-10.0)));
}

// The position variances claimed when the receiver reports none are taken per axis from the
// covariance defaults, and a fix that does report them is used instead. The defaults here are not
// the shipped ones, so the values can only come from the injected struct.
TEST(GnssPoser, InjectedPositionVariancesAreUsedPerAxis)
{
  GnssPoserCovarianceDefaults defaults;
  defaults.unknown_position_variances = {7.0, 8.0, 9.0};
  GnssPoser poser(make_params(), identity_lookup, defaults);
  poser.set_projector_info(make_mgrs_projector_info());

  const auto defaulted = published(poser.input_fix(make_reference_fix()));
  EXPECT_DOUBLE_EQ(defaulted.covariance[cov_xx], 7.0);
  EXPECT_DOUBLE_EQ(defaulted.covariance[cov_yy], 8.0);
  EXPECT_DOUBLE_EQ(defaulted.covariance[cov_zz], 9.0);

  NavSatFix reported = make_reference_fix();
  reported.position_covariance_type = NavSatFix::COVARIANCE_TYPE_APPROXIMATED;
  reported.position_covariance = {1.0, 0.0, 0.0, 0.0, 2.0, 0.0, 0.0, 0.0, 3.0};
  const auto from_fix = published(poser.input_fix(reported));
  EXPECT_DOUBLE_EQ(from_fix.covariance[cov_xx], 1.0);
  EXPECT_DOUBLE_EQ(from_fix.covariance[cov_yy], 2.0);
  EXPECT_DOUBLE_EQ(from_fix.covariance[cov_zz], 3.0);
}
