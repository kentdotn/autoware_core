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

// Unit tests of PendingFixQueue: it holds fixes until their antenna transform can be looked up.
// It needs no ROS context; transforms reach it through the lookup it was given.

#include "pending_fix_queue.hpp"

#include <builtin_interfaces/msg/time.hpp>

#include <geometry_msgs/msg/transform.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>

#include <gtest/gtest.h>

#include <optional>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace
{
using autoware::gnss_poser::PendingFixQueue;
using builtin_interfaces::msg::Time;
using geometry_msgs::msg::Transform;
using sensor_msgs::msg::NavSatFix;

constexpr double timeout_sec = 0.5;

Time make_stamp(int32_t sec, uint32_t nanosec = 0)
{
  Time stamp;
  stamp.sec = sec;
  stamp.nanosec = nanosec;
  return stamp;
}

NavSatFix make_fix(int32_t sec, uint32_t nanosec = 0, const std::string & frame_id = "gnss_link")
{
  NavSatFix fix;
  fix.header.stamp = make_stamp(sec, nanosec);
  fix.header.frame_id = frame_id;
  return fix;
}

Transform make_transform(double x)
{
  Transform transform;
  transform.translation.x = x;
  transform.rotation.w = 1.0;
  return transform;
}

// A lookup that succeeds only while `available` is true.
PendingFixQueue::TransformLookup switchable_lookup(const bool & available)
{
  return [&available](const std::string &, const Time &) {
    return available ? std::optional<Transform>(make_transform(1.0)) : std::optional<Transform>();
  };
}

const PendingFixQueue::TransformLookup missing_lookup = [](const std::string &, const Time &) {
  return std::optional<Transform>();
};
}  // namespace

// A negative timeout is rejected at construction; zero is allowed.
TEST(PendingFixQueue, RejectsNegativeTimeout)
{
  EXPECT_THROW(PendingFixQueue(missing_lookup, -0.1), std::invalid_argument);
  EXPECT_NO_THROW(PendingFixQueue(missing_lookup, 0.0));
}

// A fix whose transform is available comes back out at once, with that transform, and the queue is
// empty again.
TEST(PendingFixQueue, HandsOutAFixWhoseTransformIsAvailable)
{
  bool available = true;
  PendingFixQueue queue(switchable_lookup(available), timeout_sec);

  queue.push(make_fix(1000));
  const auto item = queue.next();

  ASSERT_TRUE(item.has_value());
  EXPECT_EQ(item->fix.header.stamp, make_stamp(1000));
  ASSERT_TRUE(item->antenna_to_base_link.has_value());
  EXPECT_DOUBLE_EQ(item->antenna_to_base_link->translation.x, 1.0);
  EXPECT_EQ(queue.size(), 0U);
  EXPECT_FALSE(queue.next().has_value());
}

// The lookup is asked for the frame and the stamp of the fix at the head of the queue.
TEST(PendingFixQueue, AsksForTheFixFrameAndStamp)
{
  std::vector<std::pair<std::string, Time>> calls;
  PendingFixQueue queue(
    [&](const std::string & frame, const Time & stamp) {
      calls.emplace_back(frame, stamp);
      return std::optional<Transform>(Transform{});
    },
    timeout_sec);

  queue.push(make_fix(1234, 5678U, "antenna_frame"));
  ASSERT_TRUE(queue.next().has_value());

  ASSERT_EQ(calls.size(), 1U);
  EXPECT_EQ(calls[0].first, "antenna_frame");
  EXPECT_EQ(calls[0].second, make_stamp(1234, 5678U));
}

// While the transform is missing the fix stays in the queue and nothing comes out; once the
// transform is there the same fix comes out with it.
TEST(PendingFixQueue, HoldsAFixUntilItsTransformIsAvailable)
{
  bool available = false;
  PendingFixQueue queue(switchable_lookup(available), timeout_sec);

  queue.push(make_fix(1000));
  EXPECT_FALSE(queue.next().has_value());
  EXPECT_EQ(queue.size(), 1U);

  available = true;
  const auto item = queue.next();
  ASSERT_TRUE(item.has_value());
  EXPECT_EQ(item->fix.header.stamp, make_stamp(1000));
  EXPECT_TRUE(item->antenna_to_base_link.has_value());
}

// Fixes come out in arrival order: one that arrives while an older one is held waits behind it,
// even though its own transform is available.
TEST(PendingFixQueue, KeepsArrivalOrder)
{
  bool available = false;
  PendingFixQueue queue(switchable_lookup(available), timeout_sec);

  queue.push(make_fix(1000, 0));
  queue.push(make_fix(1000, 100000000U));
  available = true;

  const auto first = queue.next();
  const auto second = queue.next();
  ASSERT_TRUE(first.has_value());
  ASSERT_TRUE(second.has_value());
  EXPECT_EQ(first->fix.header.stamp, make_stamp(1000, 0));
  EXPECT_EQ(second->fix.header.stamp, make_stamp(1000, 100000000U));
  EXPECT_FALSE(queue.next().has_value());
}

// A held fix is dropped once a fix newer than it by more than the timeout has arrived, oldest
// first; a fix that is still within the timeout keeps waiting. A dropped fix comes out without a
// transform, so that the caller can report it.
TEST(PendingFixQueue, DropsFixesOlderThanTheTimeout)
{
  PendingFixQueue queue(missing_lookup, timeout_sec);

  queue.push(make_fix(1000, 0));
  queue.push(make_fix(1000, 200000000U));
  EXPECT_FALSE(queue.next().has_value());  // nothing is older than 0.5 s yet

  queue.push(make_fix(1001, 0));
  const auto first = queue.next();
  const auto second = queue.next();
  ASSERT_TRUE(first.has_value());
  ASSERT_TRUE(second.has_value());
  EXPECT_EQ(first->fix.header.stamp, make_stamp(1000, 0));
  EXPECT_FALSE(first->antenna_to_base_link.has_value());
  EXPECT_EQ(second->fix.header.stamp, make_stamp(1000, 200000000U));
  EXPECT_FALSE(second->antenna_to_base_link.has_value());
  EXPECT_FALSE(queue.next().has_value());  // the newest fix is still waiting
  EXPECT_EQ(queue.size(), 1U);
}

// The timeout is measured against the newest stamp seen, not against the order of arrival: a fix
// that arrives out of order does not expire the ones before it.
TEST(PendingFixQueue, MeasuresTheTimeoutAgainstTheNewestStamp)
{
  PendingFixQueue queue(missing_lookup, timeout_sec);

  queue.push(make_fix(1001, 0));
  queue.push(make_fix(1000, 900000000U));  // older than the one before it
  EXPECT_FALSE(queue.next().has_value());  // 1001 - 1001 = 0 s, within the timeout

  queue.push(make_fix(1002, 0));
  const auto first = queue.next();
  ASSERT_TRUE(first.has_value());
  EXPECT_EQ(first->fix.header.stamp, make_stamp(1001, 0));
  EXPECT_FALSE(first->antenna_to_base_link.has_value());
}
