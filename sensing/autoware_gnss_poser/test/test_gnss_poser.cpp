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

// Unit tests of the rclcpp-free logic in gnss_poser.hpp: what GnssPoser derives from a GNSS fix.
// They run without a ROS context; the node layer is covered by test_gnss_poser_node.cpp and the
// characterization suite.

#include "gnss_poser.hpp"

#include <autoware/geography_utils/height.hpp>
#include <builtin_interfaces/msg/time.hpp>
#include <tf2/LinearMath/Quaternion.hpp>

#include <autoware_map_msgs/msg/map_projector_info.hpp>
#include <autoware_sensing_msgs/msg/gnss_ins_orientation.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose_with_covariance.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/transform.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/nav_sat_status.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>
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
using geometry_msgs::msg::PoseWithCovariance;
using geometry_msgs::msg::Quaternion;
using geometry_msgs::msg::Transform;
using sensor_msgs::msg::NavSatFix;
using sensor_msgs::msg::NavSatStatus;
using Outcome = GnssPoser::Outcome;

// Reference point in Tokyo (MGRS grid 54SUE).
constexpr double reference_latitude = 35.62426;
constexpr double reference_longitude = 139.74252;
constexpr double reference_altitude = 10.0;
constexpr const char * reference_mgrs_grid = "54SUE";
// Fixes used by the cases, as offsets from the reference point, and their MGRS positions. The
// positions are UTM coordinates modulo 100 km from GeographicLib's UTMUPS::Forward, computed
// outside of this package so that no expectation goes through the projection under test. A
// latitude offset moves a point mostly along y and a longitude offset mostly along x; the grid is
// rotated by the meridian convergence (-0.73 degrees here), so both coordinates change a little.
constexpr double reference_x = 86128.181788820;
constexpr double reference_y = 43002.610125367;
// x of reference + (2e-5, 1e-5) and y of reference + (1e-5, 2e-5) in (latitude, longitude), the
// fixes of the median case.
constexpr double near_north_x = 86129.115754038;
constexpr double near_east_y = 43003.696131575;
constexpr double small_offset = 1e-4;  // [deg], about 10 m
// Yaw from the reference point to the points small_offset away in each direction: the compass
// direction plus the meridian convergence, so not exactly 0, pi/2, pi and -pi/2.
constexpr double yaw_to_east = -0.012784370;
constexpr double yaw_to_north = 1.558011432;
constexpr double yaw_to_west = 3.128807266;
constexpr double yaw_to_south = -1.583581190;

// Geoid height at the reference point, from GeographicLib's GeoidEval with egm2008-1: the EGM2008
// altitude of a point is its WGS84 ellipsoid altitude minus this value.
constexpr double reference_geoid_height = 36.12;  // [m]
constexpr double geoid_height_tolerance = 0.01;   // [m]
constexpr double position_tolerance = 1e-4;       // [m]
constexpr double angle_tolerance = 1e-6;          // [rad]
// rmse fields of GnssInsOrientation are float32; their squares are compared with this slack.
constexpr double rmse_squared_tolerance = 1e-6;

constexpr std::size_t cov_xx = 0;
constexpr std::size_t cov_xy = 1;
constexpr std::size_t cov_xz = 2;
constexpr std::size_t cov_yy = 7;
constexpr std::size_t cov_yz = 8;
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
  double latitude, double longitude, double altitude, int8_t status = NavSatStatus::STATUS_FIX)
{
  NavSatFix fix;
  fix.header.stamp = make_stamp(1700000000, 123456789U);
  fix.header.frame_id = "gnss_link";
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

// A fix at the reference latitude and longitude, so that only its altitude, which the MGRS
// projection passes through as z, differs. The buffering cases use it to state their expectations
// as plain arithmetic on altitudes.
NavSatFix make_fix_at_altitude(double altitude, int8_t status = NavSatStatus::STATUS_FIX)
{
  return make_fix(reference_latitude, reference_longitude, altitude, status);
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

GnssPoserParams make_params(GnssPosePubMethod method, int buff_epoch)
{
  GnssPoserParams params;
  params.gnss_pose_pub_method = method;
  params.buff_epoch = buff_epoch;
  return params;
}

GnssPoserParams make_motion_orientation_params(
  GnssPosePubMethod method = GnssPosePubMethod::Instant, int buff_epoch = 1)
{
  GnssPoserParams params = make_params(method, buff_epoch);
  params.use_gnss_ins_orientation = false;
  return params;
}

Quaternion yaw_to_quaternion(double yaw)
{
  tf2::Quaternion quaternion;
  quaternion.setRPY(0.0, 0.0, yaw);
  return tf2::toMsg(quaternion);
}

Transform make_transform(double x, double y, double z)
{
  Transform transform;
  transform.translation.x = x;
  transform.translation.y = y;
  transform.translation.z = z;
  return transform;
}

GnssInsOrientation make_ins_orientation(
  double yaw, float rmse_rotation_x, float rmse_rotation_y, float rmse_rotation_z)
{
  GnssInsOrientation orientation;
  orientation.orientation = yaw_to_quaternion(yaw);
  orientation.rmse_rotation_x = rmse_rotation_x;
  orientation.rmse_rotation_y = rmse_rotation_y;
  orientation.rmse_rotation_z = rmse_rotation_z;
  return orientation;
}

// A lookup that always succeeds with the given transform.
GnssPoser::TransformLookup constant_lookup(const Transform & transform)
{
  return [transform](const std::string &, const builtin_interfaces::msg::Time &) {
    return std::optional<Transform>(transform);
  };
}

const GnssPoser::TransformLookup identity_lookup = constant_lookup(Transform{});

// A lookup that never finds a transform.
const GnssPoser::TransformLookup missing_lookup =
  [](const std::string &, const builtin_interfaces::msg::Time &) {
    return std::optional<Transform>();
  };

// A GnssPoser ready to publish: MGRS projector info received, identity lookup unless given.
GnssPoser make_ready_poser(
  const GnssPoserParams & params = {}, const GnssPoser::TransformLookup & lookup = identity_lookup)
{
  GnssPoser poser(params, lookup);
  poser.set_projector_info(make_mgrs_projector_info());
  return poser;
}

// The pose with covariance of a result that carries one; std::optional::value() throws, and so
// fails the test, when the result carries none.
PoseWithCovariance pose_of(const GnssPoser::Result & result)
{
  return result.gnss_pose_cov.value().pose;
}

void expect_position_near(
  const Point & actual, double x, double y, double z, double tol = position_tolerance)
{
  EXPECT_NEAR(actual.x, x, tol);
  EXPECT_NEAR(actual.y, y, tol);
  EXPECT_NEAR(actual.z, z, tol);
}

void expect_same_rotation(const Quaternion & actual, const Quaternion & expected)
{
  const double dot =
    actual.x * expected.x + actual.y * expected.y + actual.z * expected.z + actual.w * expected.w;
  const double angle = 2.0 * std::acos(std::min(1.0, std::abs(dot)));
  EXPECT_LE(angle, angle_tolerance) << "rotations differ by " << angle << " rad";
}

void expect_no_pose(const GnssPoser::Result & result)
{
  EXPECT_FALSE(result.gnss_pose.has_value());
  EXPECT_FALSE(result.gnss_pose_cov.has_value());
  EXPECT_FALSE(result.transform.has_value());
}

// The EGM2008 case needs GeographicLib's egm2008-1 geoid dataset, which is not a build dependency;
// the case is skipped where it is not installed.
class GnssPoserWithEgm2008Dataset : public ::testing::Test
{
protected:
  void SetUp() override
  {
    try {
      autoware::geography_utils::convert_wgs84_to_egm2008(0.0, 0.0, 0.0);
    } catch (const std::runtime_error &) {
      GTEST_SKIP() << "egm2008-1 geoid dataset is not installed";
    }
  }
};
}  // namespace

// ---------------------------------------------------------------------------------------------
// Construction

TEST(GnssPoser, ThrowsWhenBuffEpochIsZero)
{
  const GnssPoserParams params = make_params(GnssPosePubMethod::Average, 0);

  EXPECT_THROW(GnssPoser(params, identity_lookup), std::invalid_argument);
}

// Instant does not use the buffer, but buff_epoch is still validated.
TEST(GnssPoser, ThrowsWhenBuffEpochIsZeroEvenForInstantMethod)
{
  const GnssPoserParams params = make_params(GnssPosePubMethod::Instant, 0);

  EXPECT_THROW(GnssPoser(params, identity_lookup), std::invalid_argument);
}

// ---------------------------------------------------------------------------------------------
// Gates

// Without projector info nothing is derived from the fix, not even the fix status.
TEST(GnssPoser, ReturnsNoProjectorInfoAndNoOutputBeforeProjectorInfoIsSet)
{
  GnssPoser poser({}, identity_lookup);

  const auto result = poser.input_fix(make_reference_fix());

  EXPECT_EQ(result.outcome, Outcome::NoProjectorInfo);
  EXPECT_FALSE(result.gnss_fixed.has_value());
  expect_no_pose(result);
}

TEST(GnssPoser, ReturnsLocalProjectorAndNoOutputWhenProjectorIsLocal)
{
  GnssPoser poser({}, identity_lookup);
  poser.set_projector_info(make_local_projector_info());

  const auto result = poser.input_fix(make_reference_fix());

  EXPECT_EQ(result.outcome, Outcome::LocalProjector);
  EXPECT_FALSE(result.gnss_fixed.has_value());
  expect_no_pose(result);
}

// The receiver status is reported for a fix without a position solution; the pose is not.
TEST(GnssPoser, ReturnsNotFixedAndReportsFixStatusFalseWhenStatusIsNoFix)
{
  GnssPoser poser = make_ready_poser();
  NavSatFix fix = make_reference_fix(NavSatStatus::STATUS_NO_FIX);
  fix.header.stamp = make_stamp(10, 0);

  const auto result = poser.input_fix(fix);

  EXPECT_EQ(result.outcome, Outcome::NotFixed);
  ASSERT_TRUE(result.gnss_fixed.has_value());
  EXPECT_FALSE(result.gnss_fixed->data);
  EXPECT_EQ(result.gnss_fixed->stamp, make_stamp(10, 0));
  expect_no_pose(result);
}

// ---------------------------------------------------------------------------------------------
// Position: instant, average, median

TEST(GnssPoser, PublishesProjectedAntennaPositionWithInstantMethod)
{
  GnssPoser poser = make_ready_poser();

  const auto result = poser.input_fix(make_reference_fix());

  EXPECT_EQ(result.outcome, Outcome::Published);
  expect_position_near(pose_of(result).pose.position, reference_x, reference_y, 10.0);
}

TEST(GnssPoser, PublishesEveryFixWithInstantMethodWhateverTheBuffEpoch)
{
  GnssPoser poser = make_ready_poser(make_params(GnssPosePubMethod::Instant, 5));

  const auto result = poser.input_fix(make_fix_at_altitude(20.0));

  expect_position_near(pose_of(result).pose.position, reference_x, reference_y, 20.0);
}

// The receiver status is still reported while the buffer fills.
TEST(GnssPoser, ReturnsBufferingAndReportsFixStatusTrueUntilBufferIsFull)
{
  GnssPoser poser = make_ready_poser(make_params(GnssPosePubMethod::Average, 3));
  poser.input_fix(make_fix_at_altitude(10.0));

  const auto result = poser.input_fix(make_fix_at_altitude(20.0));

  EXPECT_EQ(result.outcome, Outcome::Buffering);
  ASSERT_TRUE(result.gnss_fixed.has_value());
  EXPECT_TRUE(result.gnss_fixed->data);
  expect_no_pose(result);
}

TEST(GnssPoser, PublishesMeanOfBufferedPositionsWithAverageMethod)
{
  GnssPoser poser = make_ready_poser(make_params(GnssPosePubMethod::Average, 3));
  poser.input_fix(make_fix_at_altitude(10.0));
  poser.input_fix(make_fix_at_altitude(20.0));

  const auto result = poser.input_fix(make_fix_at_altitude(60.0));

  // (10 + 20 + 60) / 3
  expect_position_near(pose_of(result).pose.position, reference_x, reference_y, 30.0);
}

TEST(GnssPoser, PublishesMeanOfLatestPositionsOnceBufferSlides)
{
  GnssPoser poser = make_ready_poser(make_params(GnssPosePubMethod::Average, 3));
  poser.input_fix(make_fix_at_altitude(10.0));
  poser.input_fix(make_fix_at_altitude(20.0));
  poser.input_fix(make_fix_at_altitude(60.0));

  const auto result = poser.input_fix(make_fix_at_altitude(40.0));

  // (20 + 60 + 40) / 3: the first fix has left the buffer.
  expect_position_near(pose_of(result).pose.position, reference_x, reference_y, 40.0);
}

// The median is taken per axis, so the published position need not be one of the buffered ones:
// x comes from the near-north fix, y from the near-east fix and z from the third altitude.
TEST(GnssPoser, PublishesComponentWiseMedianWithMedianMethod)
{
  GnssPoser poser = make_ready_poser(make_params(GnssPosePubMethod::Median, 3));
  poser.input_fix(make_fix(reference_latitude, reference_longitude, 10.0));
  poser.input_fix(make_fix(reference_latitude + 2e-5, reference_longitude + 1e-5, 30.0));

  const auto result =
    poser.input_fix(make_fix(reference_latitude + 1e-5, reference_longitude + 2e-5, 20.0));

  expect_position_near(pose_of(result).pose.position, near_north_x, near_east_y, 20.0);
}

TEST(GnssPoser, PublishesMeanOfTwoCentralValuesWithMedianMethodAndEvenBuffer)
{
  GnssPoser poser = make_ready_poser(make_params(GnssPosePubMethod::Median, 2));
  poser.input_fix(make_fix_at_altitude(10.0));

  const auto result = poser.input_fix(make_fix_at_altitude(30.0));

  expect_position_near(pose_of(result).pose.position, reference_x, reference_y, 20.0);
}

TEST(GnssPoser, KeepsNotFixedFixesOutOfPositionBuffer)
{
  GnssPoser poser = make_ready_poser(make_params(GnssPosePubMethod::Average, 2));
  poser.input_fix(make_fix_at_altitude(10.0));
  poser.input_fix(make_fix_at_altitude(1000.0, NavSatStatus::STATUS_NO_FIX));

  const auto result = poser.input_fix(make_fix_at_altitude(30.0));

  expect_position_near(pose_of(result).pose.position, reference_x, reference_y, 20.0);
}

TEST(GnssPoser, KeepsFixesBeforeProjectorInfoOutOfPositionBuffer)
{
  GnssPoser poser(make_params(GnssPosePubMethod::Average, 2), identity_lookup);
  poser.input_fix(make_fix_at_altitude(1000.0));
  poser.set_projector_info(make_mgrs_projector_info());
  poser.input_fix(make_fix_at_altitude(10.0));

  const auto result = poser.input_fix(make_fix_at_altitude(30.0));

  expect_position_near(pose_of(result).pose.position, reference_x, reference_y, 20.0);
}

TEST(GnssPoser, KeepsFixesUnderLocalProjectorOutOfPositionBuffer)
{
  GnssPoser poser(make_params(GnssPosePubMethod::Average, 2), identity_lookup);
  poser.set_projector_info(make_local_projector_info());
  poser.input_fix(make_fix_at_altitude(1000.0));
  poser.set_projector_info(make_mgrs_projector_info());
  poser.input_fix(make_fix_at_altitude(10.0));

  const auto result = poser.input_fix(make_fix_at_altitude(30.0));

  expect_position_near(pose_of(result).pose.position, reference_x, reference_y, 20.0);
}

// ---------------------------------------------------------------------------------------------
// Orientation from the INS

// Until the first INS orientation arrives, the identity orientation and the placeholder rmse of
// 1.0 rad, squared to 1.0 rad^2, stand in for it.
TEST(GnssPoser, UsesIdentityWithUnitVariancesBeforeInsOrientationArrives)
{
  GnssPoser poser = make_ready_poser();

  const auto pose = pose_of(poser.input_fix(make_reference_fix()));

  expect_same_rotation(pose.pose.orientation, Quaternion{});
  EXPECT_DOUBLE_EQ(pose.covariance[cov_roll_roll], 1.0);
  EXPECT_DOUBLE_EQ(pose.covariance[cov_pitch_pitch], 1.0);
  EXPECT_DOUBLE_EQ(pose.covariance[cov_yaw_yaw], 1.0);
}

TEST(GnssPoser, UsesInsOrientationWithSquaredRmseAsRotationVariances)
{
  GnssPoser poser = make_ready_poser();
  poser.set_ins_orientation(make_ins_orientation(M_PI / 2.0, 0.1F, 0.2F, 0.3F));

  const auto pose = pose_of(poser.input_fix(make_reference_fix()));

  expect_same_rotation(pose.pose.orientation, yaw_to_quaternion(M_PI / 2.0));
  EXPECT_NEAR(pose.covariance[cov_roll_roll], 0.01, rmse_squared_tolerance);
  EXPECT_NEAR(pose.covariance[cov_pitch_pitch], 0.04, rmse_squared_tolerance);
  EXPECT_NEAR(pose.covariance[cov_yaw_yaw], 0.09, rmse_squared_tolerance);
}

TEST(GnssPoser, UsesLatestInsOrientation)
{
  GnssPoser poser = make_ready_poser();
  poser.set_ins_orientation(make_ins_orientation(M_PI / 2.0, 0.1F, 0.2F, 0.3F));
  poser.set_ins_orientation(make_ins_orientation(-M_PI / 4.0, 0.5F, 0.6F, 0.7F));

  const auto pose = pose_of(poser.input_fix(make_reference_fix()));

  expect_same_rotation(pose.pose.orientation, yaw_to_quaternion(-M_PI / 4.0));
  EXPECT_NEAR(pose.covariance[cov_roll_roll], 0.25, rmse_squared_tolerance);
  EXPECT_NEAR(pose.covariance[cov_pitch_pitch], 0.36, rmse_squared_tolerance);
  EXPECT_NEAR(pose.covariance[cov_yaw_yaw], 0.49, rmse_squared_tolerance);
}

// ---------------------------------------------------------------------------------------------
// Orientation from motion

TEST(GnssPoser, UsesIdentityForFirstFixWhenOrientationComesFromMotion)
{
  GnssPoser poser = make_ready_poser(make_motion_orientation_params());

  const auto pose = pose_of(poser.input_fix(make_reference_fix()));

  expect_same_rotation(pose.pose.orientation, Quaternion{});
}

TEST(GnssPoser, UsesMotionRotationVariancesWhenOrientationComesFromMotion)
{
  GnssPoser poser = make_ready_poser(make_motion_orientation_params());

  const auto pose = pose_of(poser.input_fix(make_reference_fix()));

  EXPECT_DOUBLE_EQ(pose.covariance[cov_roll_roll], 0.1);
  EXPECT_DOUBLE_EQ(pose.covariance[cov_pitch_pitch], 0.1);
  EXPECT_DOUBLE_EQ(pose.covariance[cov_yaw_yaw], 1.0);
}

// The heading is the direction of the displacement in the map frame, in all four quadrants.
TEST(GnssPoser, HeadsEastWhenMovingEast)
{
  GnssPoser poser = make_ready_poser(make_motion_orientation_params());
  poser.input_fix(make_reference_fix());

  const auto result = poser.input_fix(
    make_fix(reference_latitude, reference_longitude + small_offset, reference_altitude));

  expect_same_rotation(pose_of(result).pose.orientation, yaw_to_quaternion(yaw_to_east));
}

TEST(GnssPoser, HeadsNorthWhenMovingNorth)
{
  GnssPoser poser = make_ready_poser(make_motion_orientation_params());
  poser.input_fix(make_reference_fix());

  const auto result = poser.input_fix(
    make_fix(reference_latitude + small_offset, reference_longitude, reference_altitude));

  expect_same_rotation(pose_of(result).pose.orientation, yaw_to_quaternion(yaw_to_north));
}

// West is where the yaw wraps around between +pi and -pi.
TEST(GnssPoser, HeadsWestWhenMovingWest)
{
  GnssPoser poser = make_ready_poser(make_motion_orientation_params());
  poser.input_fix(make_reference_fix());

  const auto result = poser.input_fix(
    make_fix(reference_latitude, reference_longitude - small_offset, reference_altitude));

  expect_same_rotation(pose_of(result).pose.orientation, yaw_to_quaternion(yaw_to_west));
}

TEST(GnssPoser, HeadsSouthWhenMovingSouth)
{
  GnssPoser poser = make_ready_poser(make_motion_orientation_params());
  poser.input_fix(make_reference_fix());

  const auto result = poser.input_fix(
    make_fix(reference_latitude - small_offset, reference_longitude, reference_altitude));

  expect_same_rotation(pose_of(result).pose.orientation, yaw_to_quaternion(yaw_to_south));
}

// A fix without a fix status does not move the previous position, so the heading is measured
// from the last fixed position: north, not from far south-west.
TEST(GnssPoser, MeasuresHeadingFromLastFixedPositionAcrossNotFixedFix)
{
  GnssPoser poser = make_ready_poser(make_motion_orientation_params());
  poser.input_fix(make_reference_fix());
  poser.input_fix(make_fix(
    reference_latitude - 0.01, reference_longitude - 0.01, reference_altitude,
    NavSatStatus::STATUS_NO_FIX));

  const auto result = poser.input_fix(
    make_fix(reference_latitude + small_offset, reference_longitude, reference_altitude));

  expect_same_rotation(pose_of(result).pose.orientation, yaw_to_quaternion(yaw_to_north));
}

// With a buffer the heading is measured between consecutive averaged positions. The raw antenna
// moves east (yaw -0.0128) between the last two fixes, the averages move north-east.
TEST(GnssPoser, MeasuresHeadingBetweenAveragedPositions)
{
  GnssPoser poser = make_ready_poser(make_motion_orientation_params(GnssPosePubMethod::Average, 2));
  poser.input_fix(make_reference_fix());
  poser.input_fix(make_fix(reference_latitude + 1e-3, reference_longitude, reference_altitude));

  const auto result = poser.input_fix(
    make_fix(reference_latitude + 1e-3, reference_longitude + 2e-3, reference_altitude));

  // atan2 between the means (86128.890862090, 43058.068296369) and (86220.159081204,
  // 43112.369509462) of the projected positions.
  expect_same_rotation(pose_of(result).pose.orientation, yaw_to_quaternion(0.536707589));
}

// ---------------------------------------------------------------------------------------------
// Antenna transform

TEST(GnssPoser, LooksUpAntennaTransformAtFixFrameAndStamp)
{
  std::vector<std::pair<std::string, builtin_interfaces::msg::Time>> lookups;
  GnssPoser poser = make_ready_poser(
    {}, [&lookups](const std::string & frame, const builtin_interfaces::msg::Time & stamp) {
      lookups.emplace_back(frame, stamp);
      return std::optional<Transform>(Transform{});
    });
  NavSatFix fix = make_reference_fix();
  fix.header.frame_id = "antenna_frame";
  fix.header.stamp = make_stamp(1234, 5678U);

  poser.input_fix(fix);

  ASSERT_EQ(lookups.size(), 1U);
  EXPECT_EQ(lookups[0].first, "antenna_frame");
  EXPECT_EQ(lookups[0].second, make_stamp(1234, 5678U));
}

TEST(GnssPoser, DoesNotLookUpAntennaTransformWhileBuffering)
{
  int lookup_count = 0;
  GnssPoser poser = make_ready_poser(
    make_params(GnssPosePubMethod::Average, 2),
    [&lookup_count](const std::string &, const builtin_interfaces::msg::Time &) {
      ++lookup_count;
      return std::optional<Transform>(Transform{});
    });

  poser.input_fix(make_reference_fix());

  EXPECT_EQ(lookup_count, 0);
}

// The translation of the antenna transform is applied in the antenna orientation: facing +y, the
// antenna's +x is the map's +y and its +y is the map's -x.
TEST(GnssPoser, ComposesAntennaPoseWithLookedUpTransform)
{
  GnssPoser poser = make_ready_poser({}, constant_lookup(make_transform(1.0, 2.0, 3.0)));
  poser.set_ins_orientation(make_ins_orientation(M_PI / 2.0, 1.0F, 1.0F, 1.0F));

  const auto pose = pose_of(poser.input_fix(make_reference_fix()));

  expect_position_near(pose.pose.position, reference_x - 2.0, reference_y + 1.0, 13.0);
  expect_same_rotation(pose.pose.orientation, yaw_to_quaternion(M_PI / 2.0));
}

TEST(GnssPoser, PublishesAntennaPoseWhenAntennaTransformIsMissing)
{
  GnssPoser poser = make_ready_poser({}, missing_lookup);

  const auto pose = pose_of(poser.input_fix(make_reference_fix()));

  expect_position_near(pose.pose.position, reference_x, reference_y, 10.0);
}

// ---------------------------------------------------------------------------------------------
// Output messages

TEST(GnssPoser, StampsEveryOutputWithFixStamp)
{
  GnssPoser poser = make_ready_poser();
  NavSatFix fix = make_reference_fix();
  fix.header.stamp = make_stamp(1234, 5678U);

  const auto result = poser.input_fix(fix);

  EXPECT_EQ(result.gnss_fixed.value().stamp, make_stamp(1234, 5678U));
  EXPECT_EQ(result.gnss_pose.value().header.stamp, make_stamp(1234, 5678U));
  EXPECT_EQ(result.gnss_pose_cov.value().header.stamp, make_stamp(1234, 5678U));
  EXPECT_EQ(result.transform.value().header.stamp, make_stamp(1234, 5678U));
}

TEST(GnssPoser, PublishesInFramesFromParameters)
{
  GnssPoserParams params;
  params.map_frame = "my_map";
  params.gnss_base_frame = "my_gnss_base";
  GnssPoser poser = make_ready_poser(params);

  const auto result = poser.input_fix(make_reference_fix());

  EXPECT_EQ(result.gnss_pose.value().header.frame_id, "my_map");
  EXPECT_EQ(result.gnss_pose_cov.value().header.frame_id, "my_map");
  EXPECT_EQ(result.transform.value().header.frame_id, "my_map");
  EXPECT_EQ(result.transform.value().child_frame_id, "my_gnss_base");
}

TEST(GnssPoser, ReportsFixStatusTrueWhenPublishing)
{
  GnssPoser poser = make_ready_poser();

  const auto result = poser.input_fix(make_reference_fix());

  EXPECT_TRUE(result.gnss_fixed.value().data);
}

// The pose, the pose with covariance and the transform are copies of one pose, hence exact
// equality.
TEST(GnssPoser, PublishesSamePoseInEveryOutput)
{
  GnssPoser poser = make_ready_poser({}, constant_lookup(make_transform(1.0, 2.0, 3.0)));

  const auto result = poser.input_fix(make_reference_fix());

  const auto & pose = result.gnss_pose.value().pose;
  EXPECT_EQ(result.gnss_pose_cov.value().pose.pose, pose);
  const auto & transform = result.transform.value().transform;
  EXPECT_EQ(transform.translation.x, pose.position.x);
  EXPECT_EQ(transform.translation.y, pose.position.y);
  EXPECT_EQ(transform.translation.z, pose.position.z);
  EXPECT_EQ(transform.rotation, pose.orientation);
}

// ---------------------------------------------------------------------------------------------
// Position covariance

TEST(GnssPoser, UsesFixDiagonalAsPositionVariancesWhenCovarianceIsKnown)
{
  GnssPoser poser = make_ready_poser();
  NavSatFix fix = make_reference_fix();
  fix.position_covariance_type = NavSatFix::COVARIANCE_TYPE_DIAGONAL_KNOWN;
  fix.position_covariance = {1.5, 0.0, 0.0, 0.0, 2.5, 0.0, 0.0, 0.0, 3.5};

  const auto pose = pose_of(poser.input_fix(fix));

  EXPECT_DOUBLE_EQ(pose.covariance[cov_xx], 1.5);
  EXPECT_DOUBLE_EQ(pose.covariance[cov_yy], 2.5);
  EXPECT_DOUBLE_EQ(pose.covariance[cov_zz], 3.5);
}

TEST(GnssPoser, DropsOffDiagonalPositionCovariance)
{
  GnssPoser poser = make_ready_poser();
  NavSatFix fix = make_reference_fix();
  fix.position_covariance_type = NavSatFix::COVARIANCE_TYPE_KNOWN;
  fix.position_covariance = {1.5, 9.0, 9.0, 9.0, 2.5, 9.0, 9.0, 9.0, 3.5};

  const auto pose = pose_of(poser.input_fix(fix));

  EXPECT_DOUBLE_EQ(pose.covariance[cov_xy], 0.0);
  EXPECT_DOUBLE_EQ(pose.covariance[cov_xz], 0.0);
  EXPECT_DOUBLE_EQ(pose.covariance[cov_yz], 0.0);
}

TEST(GnssPoser, UsesDefaultPositionVariancesWhenCovarianceIsUnknown)
{
  GnssPoser poser = make_ready_poser();

  const auto pose = pose_of(poser.input_fix(make_reference_fix()));

  EXPECT_DOUBLE_EQ(pose.covariance[cov_xx], 10.0);
  EXPECT_DOUBLE_EQ(pose.covariance[cov_yy], 10.0);
  EXPECT_DOUBLE_EQ(pose.covariance[cov_zz], 10.0);
}

// The injected values are not the shipped ones, so they can only come from the injected struct.
TEST(GnssPoser, UsesInjectedPositionVariancesWhenCovarianceIsUnknown)
{
  GnssPoserCovarianceDefaults defaults;
  defaults.unknown_position_variances = {7.0, 8.0, 9.0};
  GnssPoser poser({}, identity_lookup, defaults);
  poser.set_projector_info(make_mgrs_projector_info());

  const auto pose = pose_of(poser.input_fix(make_reference_fix()));

  EXPECT_DOUBLE_EQ(pose.covariance[cov_xx], 7.0);
  EXPECT_DOUBLE_EQ(pose.covariance[cov_yy], 8.0);
  EXPECT_DOUBLE_EQ(pose.covariance[cov_zz], 9.0);
}

// ---------------------------------------------------------------------------------------------
// Projector variants

// With EGM2008 the WGS84 ellipsoid altitude of the fix is converted to a geoid height:
// 10.0 - 36.12 at the reference point.
TEST_F(GnssPoserWithEgm2008Dataset, ConvertsAltitudeToGeoidHeightWhenVerticalDatumIsEgm2008)
{
  MapProjectorInfo projector_info = make_mgrs_projector_info();
  projector_info.vertical_datum = MapProjectorInfo::EGM2008;
  GnssPoser poser({}, identity_lookup);
  poser.set_projector_info(projector_info);

  const auto pose = pose_of(poser.input_fix(make_reference_fix()));

  expect_position_near(
    pose.pose.position, reference_x, reference_y, 10.0 - reference_geoid_height,
    geoid_height_tolerance);
}

// A fix at the map origin lands at (0, 0, altitude - origin altitude).
TEST(GnssPoser, MeasuresPositionFromMapOriginWithLocalCartesianUtmProjector)
{
  MapProjectorInfo projector_info;
  projector_info.projector_type = MapProjectorInfo::LOCAL_CARTESIAN_UTM;
  projector_info.vertical_datum = MapProjectorInfo::WGS84;
  projector_info.map_origin.latitude = reference_latitude;
  projector_info.map_origin.longitude = reference_longitude;
  projector_info.map_origin.altitude = -10.0;
  GnssPoser poser({}, identity_lookup);
  poser.set_projector_info(projector_info);

  const auto pose = pose_of(poser.input_fix(make_reference_fix()));

  expect_position_near(pose.pose.position, 0.0, 0.0, 20.0);
}
