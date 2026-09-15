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

// Unit tests of DynamicGnssPoser: the pose computation with a PendingFixQueue in front of it, for
// an antenna that moves relative to base_link. They run without a ROS context; what the node does
// with the results is covered by test_gnss_poser_node.cpp.

#include "dynamic_gnss_poser.hpp"
#include "gnss_poser.hpp"

#include <autoware_map_msgs/msg/map_projector_info.hpp>
#include <geometry_msgs/msg/transform.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/nav_sat_status.hpp>

#include <gtest/gtest.h>

#include <map>
#include <stdexcept>
#include <string>
#include <vector>

namespace
{
using autoware::gnss_poser::DynamicGnssPoser;
using autoware::gnss_poser::GnssPosePubMethod;
using autoware::gnss_poser::GnssPoser;
using autoware::gnss_poser::GnssPoserParams;
using autoware::gnss_poser::PendingFixQueue;
using autoware::gnss_poser::project_to_map;
using autoware_map_msgs::msg::MapProjectorInfo;
using geometry_msgs::msg::Transform;
using sensor_msgs::msg::NavSatFix;
using sensor_msgs::msg::NavSatStatus;
using Outcome = GnssPoser::Outcome;

constexpr double reference_latitude = 35.6762;
constexpr double reference_longitude = 139.6503;
constexpr double reference_altitude = 40.0;
constexpr double position_tolerance = 1e-4;  // [m]
constexpr double default_timeout_sec = 0.5;
// For the cases where the fixes must all wait rather than expire against each other.
constexpr double long_timeout_sec = 10.0;
constexpr const char * antenna_frame = "gnss_antenna";

MapProjectorInfo make_projector_info()
{
  MapProjectorInfo info;
  info.projector_type = MapProjectorInfo::MGRS;
  info.mgrs_grid = "54SUE";
  info.vertical_datum = MapProjectorInfo::WGS84;
  return info;
}

NavSatFix make_fix(int32_t sec)
{
  NavSatFix fix;
  fix.header.frame_id = antenna_frame;
  fix.header.stamp.sec = sec;
  fix.status.status = NavSatStatus::STATUS_FIX;
  fix.latitude = reference_latitude;
  fix.longitude = reference_longitude;
  fix.altitude = reference_altitude;
  return fix;
}

Transform make_transform(double x)
{
  Transform transform;
  transform.translation.x = x;
  transform.rotation.w = 1.0;
  return transform;
}

GnssPoserParams make_params(
  const GnssPosePubMethod method = GnssPosePubMethod::Instant, const int buff_epoch = 1)
{
  GnssPoserParams params;
  params.gnss_pose_pub_method = method;
  params.buff_epoch = buff_epoch;
  params.use_gnss_ins_orientation = true;  // identity until an orientation arrives
  return params;
}

// A lookup whose answers the test sets per fix stamp. Unknown stamps have no transform, which is
// how a fix is made to wait.
class Transforms
{
public:
  void set(const int32_t sec, const double x) { by_sec_[sec] = make_transform(x); }

  PendingFixQueue::TransformLookup lookup()
  {
    return [this](
             const std::string & frame,
             const builtin_interfaces::msg::Time & stamp) -> std::optional<Transform> {
      ++calls;
      EXPECT_EQ(frame, antenna_frame);
      const auto found = by_sec_.find(stamp.sec);
      if (found == by_sec_.end()) {
        return std::nullopt;
      }
      return found->second;
    };
  }

  int calls = 0;

private:
  std::map<int32_t, Transform> by_sec_;
};

// The x of the pose of a Published result, relative to the projected antenna position: with an
// identity antenna orientation that is the translation the transform contributed.
double lever_arm_x(const GnssPoser::Result & result, const NavSatFix & fix)
{
  EXPECT_EQ(result.outcome, Outcome::Published);
  const auto position =
    result.gnss_pose ? result.gnss_pose->pose.position : geometry_msgs::msg::Point{};
  return position.x - project_to_map(fix, make_projector_info()).x;
}
}  // namespace

// A negative timeout is a configuration error, like an out-of-range buff_epoch.
TEST(DynamicGnssPoser, RejectsNegativeTimeout)
{
  Transforms transforms;
  EXPECT_THROW(DynamicGnssPoser(make_params(), transforms.lookup(), -0.1), std::invalid_argument);
  EXPECT_NO_THROW(DynamicGnssPoser(make_params(), transforms.lookup(), 0.0));
}

// A fix whose transform is already available is answered straight away, like the static poser.
TEST(DynamicGnssPoser, FixWithItsTransformIsAnsweredAtOnce)
{
  Transforms transforms;
  transforms.set(10, 1.0);
  DynamicGnssPoser poser(make_params(), transforms.lookup(), default_timeout_sec);
  poser.set_projector_info(make_projector_info());

  const NavSatFix fix = make_fix(10);
  const std::vector<GnssPoser::Result> results = poser.input_fix(fix);

  ASSERT_EQ(results.size(), 1U);
  EXPECT_EQ(results[0].gnss_pose->header.stamp, fix.header.stamp);
  EXPECT_NEAR(lever_arm_x(results[0], fix), 1.0, position_tolerance);
  EXPECT_EQ(poser.take_status().pending_fix_count, 0U);
}

// A fix whose transform is not available yet produces nothing and waits; it is answered on the
// transform update that finds it, and it keeps its own stamp.
TEST(DynamicGnssPoser, FixWaitsUntilItsTransformArrives)
{
  Transforms transforms;
  DynamicGnssPoser poser(make_params(), transforms.lookup(), default_timeout_sec);
  poser.set_projector_info(make_projector_info());

  const NavSatFix fix = make_fix(10);
  EXPECT_TRUE(poser.input_fix(fix).empty());
  EXPECT_TRUE(poser.input_transform_update().empty());
  EXPECT_EQ(poser.take_status().pending_fix_count, 1U);

  transforms.set(10, 1.0);
  const std::vector<GnssPoser::Result> results = poser.input_transform_update();

  ASSERT_EQ(results.size(), 1U);
  EXPECT_EQ(results[0].gnss_pose->header.stamp, fix.header.stamp);
  EXPECT_NEAR(lever_arm_x(results[0], fix), 1.0, position_tolerance);
  EXPECT_EQ(poser.take_status().pending_fix_count, 0U);
}

// Every fix is computed with the transform that belongs to it, not with the newest one: that is
// the whole point of waiting when the antenna moves. Two fixes wait, then both transforms arrive
// and both come out at once, oldest first.
TEST(DynamicGnssPoser, EachFixUsesTheTransformOfItsOwnStamp)
{
  Transforms transforms;
  DynamicGnssPoser poser(make_params(), transforms.lookup(), long_timeout_sec);
  poser.set_projector_info(make_projector_info());

  const NavSatFix first = make_fix(10);
  const NavSatFix second = make_fix(11);
  static_assert(long_timeout_sec > 1.0, "the two fixes must not expire while they wait");
  EXPECT_TRUE(poser.input_fix(first).empty());
  EXPECT_TRUE(poser.input_fix(second).empty());
  EXPECT_EQ(poser.take_status().pending_fix_count, 2U);

  transforms.set(10, 1.0);
  transforms.set(11, 2.0);
  const std::vector<GnssPoser::Result> results = poser.input_transform_update();

  ASSERT_EQ(results.size(), 2U);
  EXPECT_EQ(results[0].gnss_pose->header.stamp, first.header.stamp);
  EXPECT_EQ(results[1].gnss_pose->header.stamp, second.header.stamp);
  EXPECT_NEAR(lever_arm_x(results[0], first), 1.0, position_tolerance);
  EXPECT_NEAR(lever_arm_x(results[1], second), 2.0, position_tolerance);
}

// A fix whose transform never arrives is dropped once a fix newer than it by more than the
// timeout has arrived. It leaves no result behind: it never became computable, so it was never
// received. The status says that it was dropped, and stops saying so as soon as a fix is computed.
TEST(DynamicGnssPoser, FixWithoutItsTransformIsDroppedAfterTheTimeout)
{
  Transforms transforms;
  DynamicGnssPoser poser(make_params(), transforms.lookup(), default_timeout_sec);
  poser.set_projector_info(make_projector_info());

  EXPECT_TRUE(poser.input_fix(make_fix(10)).empty());
  EXPECT_FALSE(poser.take_status().fixes_dropped_for_missing_transform);

  transforms.set(11, 1.0);
  const NavSatFix newer = make_fix(11);
  const std::vector<GnssPoser::Result> results = poser.input_fix(newer);

  ASSERT_EQ(results.size(), 1U);  // only the newer fix, the older one is gone
  EXPECT_EQ(results[0].gnss_pose->header.stamp, newer.header.stamp);
  EXPECT_EQ(poser.take_status().pending_fix_count, 0U);
  EXPECT_FALSE(poser.take_status().fixes_dropped_for_missing_transform);
}

// A dropped fix does not enter the position buffer either: the average is taken over the fixes
// that were received, and a fix that never became computable was not one of them.
TEST(DynamicGnssPoser, DroppedFixDoesNotEnterThePositionBuffer)
{
  Transforms transforms;
  DynamicGnssPoser poser(
    make_params(GnssPosePubMethod::Average, 2), transforms.lookup(), default_timeout_sec);
  poser.set_projector_info(make_projector_info());

  EXPECT_TRUE(poser.input_fix(make_fix(10)).empty());  // waits, then expires

  transforms.set(11, 1.0);
  const std::vector<GnssPoser::Result> buffering = poser.input_fix(make_fix(11));
  ASSERT_EQ(buffering.size(), 1U);
  EXPECT_EQ(buffering[0].outcome, Outcome::Buffering);
  EXPECT_EQ(poser.take_status().position_buffer_size, 1U);
}

// take_status() reports the queue depth while fixes wait and the drop once one gave up, so the
// node can tell "the transform has not arrived yet" from "it never will".
TEST(DynamicGnssPoser, StatusReportsWaitingAndDropping)
{
  Transforms transforms;
  DynamicGnssPoser poser(make_params(), transforms.lookup(), default_timeout_sec);
  poser.set_projector_info(make_projector_info());

  EXPECT_TRUE(poser.input_fix(make_fix(10)).empty());
  const GnssPoser::Status waiting = poser.take_status();
  EXPECT_EQ(waiting.pending_fix_count, 1U);
  EXPECT_FALSE(waiting.fixes_dropped_for_missing_transform);

  EXPECT_TRUE(poser.input_fix(make_fix(11)).empty());  // drops the first, holds the second
  const GnssPoser::Status dropped = poser.take_status();
  EXPECT_EQ(dropped.pending_fix_count, 1U);
  EXPECT_TRUE(dropped.fixes_dropped_for_missing_transform);
}

// The gates of the pose computation still apply, and they apply when the fix is computed, not when
// it arrives: a fix that waited through the arrival of the projector info is projected with it.
TEST(DynamicGnssPoser, GatesApplyWhenTheFixIsComputedNotWhenItArrives)
{
  Transforms transforms;
  DynamicGnssPoser poser(make_params(), transforms.lookup(), default_timeout_sec);

  const NavSatFix fix = make_fix(10);
  EXPECT_TRUE(poser.input_fix(fix).empty());

  poser.set_projector_info(make_projector_info());
  transforms.set(10, 1.0);
  const std::vector<GnssPoser::Result> results = poser.input_transform_update();

  ASSERT_EQ(results.size(), 1U);
  EXPECT_EQ(results[0].outcome, Outcome::Published);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
