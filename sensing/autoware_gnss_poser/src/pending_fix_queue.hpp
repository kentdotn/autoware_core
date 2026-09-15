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
#ifndef PENDING_FIX_QUEUE_HPP_
#define PENDING_FIX_QUEUE_HPP_

#include <builtin_interfaces/msg/time.hpp>

#include <geometry_msgs/msg/transform.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>

#include <cstddef>
#include <deque>
#include <functional>
#include <optional>
#include <string>

namespace autoware::gnss_poser
{
/// \brief Keeps GNSS fixes until the transform from the antenna frame named in their header to
/// base_link can be looked up, so that the pose computation always receives a fix together with
/// that transform.
///
/// The transform is normally static and published once at start-up, so a fix that arrives before it
/// only waits for the first message. A fix is handed out as soon as its transform resolves, in
/// arrival order, and dropped once a fix newer than it by more than `timeout_sec` has arrived; that
/// is how a frame which never resolves (a configuration error) is told from one whose transform has
/// simply not arrived yet. Ages are measured between fix header stamps, so this class holds no
/// clock, and it does not know where transforms come from: resolving one is the caller's business.
class PendingFixQueue
{
public:
  using TransformLookup = std::function<std::optional<geometry_msgs::msg::Transform>(
    const std::string & antenna_frame, const builtin_interfaces::msg::Time & stamp)>;

  /// \brief What came out of the queue.
  struct Item
  {
    sensor_msgs::msg::NavSatFix fix;
    /// The transform from the fix's antenna frame to base_link, or nothing when the fix was
    /// dropped because its transform did not become available in time.
    std::optional<geometry_msgs::msg::Transform> antenna_to_base_link;
  };

  /// \param lookup_antenna_to_base_link asked for the oldest queued fix on every next() call.
  /// \throw std::invalid_argument when timeout_sec is negative.
  PendingFixQueue(TransformLookup lookup_antenna_to_base_link, double timeout_sec);

  void push(const sensor_msgs::msg::NavSatFix & fix);

  /// \brief Take the oldest fix that can be handed out, resolved or dropped. Empty while the oldest
  /// one is still waiting for its transform, or while nothing is queued. Call it after every push()
  /// and whenever new transforms may have arrived, until it returns nothing.
  std::optional<Item> next();

  [[nodiscard]] std::size_t size() const { return fixes_.size(); }

private:
  [[nodiscard]] bool is_expired(const sensor_msgs::msg::NavSatFix & fix) const;

  TransformLookup lookup_antenna_to_base_link_;
  double timeout_sec_;
  std::deque<sensor_msgs::msg::NavSatFix> fixes_;
  // Newest fix header stamp seen so far; the reference the timeout is measured against.
  std::optional<builtin_interfaces::msg::Time> newest_stamp_;
};
}  // namespace autoware::gnss_poser

#endif  // PENDING_FIX_QUEUE_HPP_
