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

// Unit tests of PendingMessageQueue: it holds stamped messages until the transform from their own
// frame can be looked up. It needs no ROS context; transforms reach it through the lookup it was
// given. The GNSS fix is the message it was written for; a second message type is exercised here
// to keep it usable for anything with a header.

#include "pending_message_queue.hpp"

#include <builtin_interfaces/msg/time.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
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
using autoware::gnss_poser::DropReason;
using autoware::gnss_poser::PendingMessageQueue;
using builtin_interfaces::msg::Time;
using geometry_msgs::msg::PoseStamped;
using geometry_msgs::msg::Transform;
using sensor_msgs::msg::NavSatFix;
using FixQueue = PendingMessageQueue<NavSatFix>;
using Limits = FixQueue::Limits;

constexpr double timeout_sec = 0.5;
// The limits of every case that is not about the limits themselves.
const Limits limits{timeout_sec, 32};

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
FixQueue::TransformLookup switchable_lookup(const bool & available)
{
  return [&available](const std::string &, const Time &) {
    return available ? std::optional<Transform>(make_transform(1.0)) : std::optional<Transform>();
  };
}

const FixQueue::TransformLookup missing_lookup = [](const std::string &, const Time &) {
  return std::optional<Transform>();
};
}  // namespace

// Limits out of range are rejected at construction; a zero age is allowed, a zero queue is not.
TEST(PendingMessageQueue, RejectsLimitsOutOfRange)
{
  EXPECT_THROW(FixQueue(missing_lookup, Limits{-0.1, 32}), std::invalid_argument);
  EXPECT_THROW(FixQueue(missing_lookup, Limits{timeout_sec, 0}), std::invalid_argument);
  EXPECT_NO_THROW(FixQueue(missing_lookup, Limits{0.0, 1}));
}

// A fix whose transform is available comes back out at once, with that transform, and the queue is
// empty again.
TEST(PendingMessageQueue, HandsOutAFixWhoseTransformIsAvailable)
{
  bool available = true;
  FixQueue queue(switchable_lookup(available), limits);

  queue.push(make_fix(1000));
  const auto item = queue.next();

  ASSERT_TRUE(item.has_value());
  EXPECT_EQ(item->message.header.stamp, make_stamp(1000));
  ASSERT_TRUE(item->transform.has_value());
  EXPECT_DOUBLE_EQ(item->transform->translation.x, 1.0);
  EXPECT_EQ(queue.size(), 0U);
  EXPECT_FALSE(queue.next().has_value());
}

// The lookup is asked for the frame and the stamp of the fix at the head of the queue.
TEST(PendingMessageQueue, AsksForTheFixFrameAndStamp)
{
  std::vector<std::pair<std::string, Time>> calls;
  FixQueue queue(
    [&](const std::string & frame, const Time & stamp) {
      calls.emplace_back(frame, stamp);
      return std::optional<Transform>(Transform{});
    },
    limits);

  queue.push(make_fix(1234, 5678U, "antenna_frame"));
  ASSERT_TRUE(queue.next().has_value());

  ASSERT_EQ(calls.size(), 1U);
  EXPECT_EQ(calls[0].first, "antenna_frame");
  EXPECT_EQ(calls[0].second, make_stamp(1234, 5678U));
}

// While the transform is missing the fix stays in the queue and nothing comes out; once the
// transform is there the same fix comes out with it.
TEST(PendingMessageQueue, HoldsAFixUntilItsTransformIsAvailable)
{
  bool available = false;
  FixQueue queue(switchable_lookup(available), limits);

  queue.push(make_fix(1000));
  EXPECT_FALSE(queue.next().has_value());
  EXPECT_EQ(queue.size(), 1U);

  available = true;
  const auto item = queue.next();
  ASSERT_TRUE(item.has_value());
  EXPECT_EQ(item->message.header.stamp, make_stamp(1000));
  EXPECT_TRUE(item->transform.has_value());
}

// Fixes come out in arrival order: one that arrives while an older one is held waits behind it,
// even though its own transform is available.
TEST(PendingMessageQueue, KeepsArrivalOrder)
{
  bool available = false;
  FixQueue queue(switchable_lookup(available), limits);

  queue.push(make_fix(1000, 0));
  queue.push(make_fix(1000, 100000000U));
  available = true;

  const auto first = queue.next();
  const auto second = queue.next();
  ASSERT_TRUE(first.has_value());
  ASSERT_TRUE(second.has_value());
  EXPECT_EQ(first->message.header.stamp, make_stamp(1000, 0));
  EXPECT_EQ(second->message.header.stamp, make_stamp(1000, 100000000U));
  EXPECT_FALSE(queue.next().has_value());
}

// A held fix is dropped once a fix newer than it by more than the timeout has arrived, oldest
// first; a fix that is still within the timeout keeps waiting. A dropped fix comes out without a
// transform, so that the caller can report it.
TEST(PendingMessageQueue, DropsFixesOlderThanTheTimeout)
{
  FixQueue queue(missing_lookup, limits);

  queue.push(make_fix(1000, 0));
  queue.push(make_fix(1000, 200000000U));
  EXPECT_FALSE(queue.next().has_value());  // nothing is older than 0.5 s yet

  queue.push(make_fix(1001, 0));
  const auto first = queue.next();
  const auto second = queue.next();
  ASSERT_TRUE(first.has_value());
  ASSERT_TRUE(second.has_value());
  EXPECT_EQ(first->message.header.stamp, make_stamp(1000, 0));
  EXPECT_FALSE(first->transform.has_value());
  EXPECT_EQ(first->drop_reason, DropReason::TransformUnavailable);
  EXPECT_EQ(second->message.header.stamp, make_stamp(1000, 200000000U));
  EXPECT_FALSE(second->transform.has_value());
  EXPECT_FALSE(queue.next().has_value());  // the newest fix is still waiting
  EXPECT_EQ(queue.size(), 1U);
}

// The timeout is measured against the newest stamp seen, not against the order of arrival: a fix
// that arrives out of order does not expire the ones before it.
TEST(PendingMessageQueue, MeasuresTheTimeoutAgainstTheNewestStamp)
{
  FixQueue queue(missing_lookup, limits);

  queue.push(make_fix(1001, 0));
  queue.push(make_fix(1000, 900000000U));  // older than the one before it
  EXPECT_FALSE(queue.next().has_value());  // 1001 - 1001 = 0 s, within the timeout

  queue.push(make_fix(1002, 0));
  const auto first = queue.next();
  ASSERT_TRUE(first.has_value());
  EXPECT_EQ(first->message.header.stamp, make_stamp(1001, 0));
  EXPECT_FALSE(first->transform.has_value());
}

// A queue that is full pushes its oldest message off the back and reports it as QueueFull, so that
// messages whose stamps do not advance cannot grow it without end. The bound is on the count, like
// the queue size of tf2_ros::MessageFilter.
TEST(PendingMessageQueue, DropsTheOldestMessageWhenTheQueueIsFull)
{
  FixQueue queue(missing_lookup, Limits{timeout_sec, 2});

  queue.push(make_fix(1000, 0));
  queue.push(make_fix(1000, 1U));
  queue.push(make_fix(1000, 2U));  // same stamps, so nothing expires; the oldest is pushed off

  const auto dropped = queue.next();
  ASSERT_TRUE(dropped.has_value());
  EXPECT_EQ(dropped->message.header.stamp, make_stamp(1000, 0));
  EXPECT_FALSE(dropped->transform.has_value());
  EXPECT_EQ(dropped->drop_reason, DropReason::QueueFull);
  EXPECT_EQ(queue.size(), 2U);
  EXPECT_FALSE(queue.next().has_value());  // the two that are left are still waiting
}

// Nothing in the queue is specific to a GNSS fix: any message with a header works, and the stamp
// and the frame are read through MessageHeaderTraits.
TEST(PendingMessageQueue, HoldsAnyStampedMessage)
{
  bool available = false;
  PendingMessageQueue<PoseStamped> queue(
    [&available](const std::string & frame, const Time &) {
      EXPECT_EQ(frame, "lidar_link");
      return available ? std::optional<Transform>(make_transform(2.0)) : std::optional<Transform>();
    },
    limits);

  PoseStamped pose;
  pose.header.stamp = make_stamp(1000);
  pose.header.frame_id = "lidar_link";
  pose.pose.position.x = 7.0;
  queue.push(pose);
  EXPECT_FALSE(queue.next().has_value());

  available = true;
  const auto item = queue.next();
  ASSERT_TRUE(item.has_value());
  EXPECT_DOUBLE_EQ(item->message.pose.position.x, 7.0);
  ASSERT_TRUE(item->transform.has_value());
  EXPECT_DOUBLE_EQ(item->transform->translation.x, 2.0);
}
