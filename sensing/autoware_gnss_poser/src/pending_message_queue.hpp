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
#ifndef PENDING_MESSAGE_QUEUE_HPP_
#define PENDING_MESSAGE_QUEUE_HPP_

#include <builtin_interfaces/msg/time.hpp>

#include <geometry_msgs/msg/transform.hpp>

#include <cstddef>
#include <deque>
#include <functional>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>

namespace autoware::gnss_poser
{
/// \brief Where the stamp and the frame of a queued message are. Specialize it for a message that
/// keeps them somewhere other than a standard header.
///
/// This mirrors message_filters::message_traits, which is what tf2_ros::MessageFilter uses, so
/// that a queue can be exchanged for that filter without changing which messages are accepted.
/// The traits are written out here rather than taken from message_filters so that this stays free
/// of rclcpp: message_filters answers with rclcpp::Time.
template <typename MessageT>
struct MessageHeaderTraits
{
  static const builtin_interfaces::msg::Time & stamp(const MessageT & message)
  {
    return message.header.stamp;
  }
  static const std::string & frame_id(const MessageT & message) { return message.header.frame_id; }
};

/// \brief When PendingMessageQueue gives up on a message.
enum class DropReason {
  /// The queue was full and this message was the oldest one in it. Same condition as
  /// tf2_ros::filter_failure_reasons::QueueFull.
  QueueFull,
  /// Its transform did not become available before the queue moved past it in message time.
  TransformUnavailable,
};

/// \brief When PendingMessageQueue gives up on a message that waits.
struct PendingMessageQueueLimits
{
  /// A waiting message is dropped once a message newer than it by more than this has arrived.
  /// Measured between message stamps, which is what no ROS mechanism offers: tf2_ros::MessageFilter
  /// bounds its queue by message count, and its optional buffer timeout runs on the node clock.
  double max_stamp_age_sec = 0.5;
  /// Hard bound on the queue, so that messages whose stamps do not advance cannot grow it without
  /// end. This is what tf2_ros::MessageFilter calls its queue size.
  std::size_t max_queued = 32;
};

/// \brief Keeps stamped messages until the transform from their own frame to a fixed target frame
/// can be looked up, so that a consumer always receives a message together with that transform.
///
/// Messages are handed out in arrival order, each with the transform that was resolved for its own
/// stamp, and dropped when they wait too long or when the queue overflows. Ages are measured
/// between message stamps, so the queue holds no clock, and it does not know where transforms come
/// from: resolving one is the caller's business. Both properties are what make it testable without
/// a ROS context.
///
/// It is the pull-shaped counterpart of tf2_ros::MessageFilter: call next() until it answers
/// nothing, after every push() and whenever new transforms may have arrived.
template <typename MessageT>
class PendingMessageQueue
{
public:
  using TransformLookup = std::function<std::optional<geometry_msgs::msg::Transform>(
    const std::string & source_frame, const builtin_interfaces::msg::Time & stamp)>;
  using Limits = PendingMessageQueueLimits;
  using Traits = MessageHeaderTraits<MessageT>;

  /// \brief What came out of the queue: a message with its transform, or a message that was
  /// dropped and why.
  struct Item
  {
    MessageT message;
    /// The transform from the message's own frame to the target frame. Empty exactly when the
    /// message was dropped.
    std::optional<geometry_msgs::msg::Transform> transform;
    /// Set exactly when `transform` is empty.
    std::optional<DropReason> drop_reason;
  };

  /// \param lookup_transform asked for the oldest queued message on every next() call.
  /// \throw std::invalid_argument when a limit is out of range.
  explicit PendingMessageQueue(TransformLookup lookup_transform, const Limits & limits = {})
  : lookup_transform_(std::move(lookup_transform)), limits_(limits)
  {
    if (limits.max_stamp_age_sec < 0.0) {
      throw std::invalid_argument(
        "max_stamp_age_sec must not be negative, got " + std::to_string(limits.max_stamp_age_sec));
    }
    if (limits.max_queued == 0) {
      throw std::invalid_argument("max_queued must be at least 1");
    }
  }

  /// \brief Queue one message. When the queue is full the oldest one is pushed off the back and
  /// comes out of the next next() call with DropReason::QueueFull.
  void push(const MessageT & message)
  {
    const builtin_interfaces::msg::Time & stamp = Traits::stamp(message);
    if (!newest_stamp_ || seconds_between(*newest_stamp_, stamp) > 0.0) {
      newest_stamp_ = stamp;
    }
    if (messages_.size() >= limits_.max_queued) {
      overflowed_ = std::move(messages_.front());
      messages_.pop_front();
    }
    messages_.push_back(message);
  }

  /// \brief Take the oldest message that can be handed out, resolved or dropped. Empty while the
  /// oldest one is still waiting for its transform, or while nothing is queued.
  std::optional<Item> next()
  {
    if (overflowed_) {
      Item item{std::move(*overflowed_), std::nullopt, DropReason::QueueFull};
      overflowed_.reset();
      return item;
    }
    if (messages_.empty()) {
      return std::nullopt;
    }
    const MessageT & message = messages_.front();

    if (auto transform = lookup_transform_(Traits::frame_id(message), Traits::stamp(message))) {
      Item item{message, std::move(transform), std::nullopt};
      messages_.pop_front();
      return item;
    }
    if (is_expired(message)) {
      Item item{message, std::nullopt, DropReason::TransformUnavailable};
      messages_.pop_front();
      return item;
    }
    // Still waiting. The messages behind it wait too, so that they stay in order.
    return std::nullopt;
  }

  [[nodiscard]] std::size_t size() const { return messages_.size(); }

private:
  static double seconds_between(
    const builtin_interfaces::msg::Time & older, const builtin_interfaces::msg::Time & newer)
  {
    return (static_cast<double>(newer.sec) - static_cast<double>(older.sec)) +
           (static_cast<double>(newer.nanosec) - static_cast<double>(older.nanosec)) * 1e-9;
  }

  [[nodiscard]] bool is_expired(const MessageT & message) const
  {
    return newest_stamp_ &&
           seconds_between(Traits::stamp(message), *newest_stamp_) > limits_.max_stamp_age_sec;
  }

  TransformLookup lookup_transform_;
  Limits limits_;
  std::deque<MessageT> messages_;
  // Pushed off the back by an overflow, waiting to be reported by next().
  std::optional<MessageT> overflowed_;
  // Newest message stamp seen so far; the reference the age is measured against.
  std::optional<builtin_interfaces::msg::Time> newest_stamp_;
};
}  // namespace autoware::gnss_poser

#endif  // PENDING_MESSAGE_QUEUE_HPP_
