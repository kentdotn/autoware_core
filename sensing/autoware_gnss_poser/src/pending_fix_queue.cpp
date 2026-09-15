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

#include "pending_fix_queue.hpp"

#include <stdexcept>
#include <string>
#include <utility>

namespace autoware::gnss_poser
{
namespace
{
double seconds_between(
  const builtin_interfaces::msg::Time & older, const builtin_interfaces::msg::Time & newer)
{
  return (static_cast<double>(newer.sec) - static_cast<double>(older.sec)) +
         (static_cast<double>(newer.nanosec) - static_cast<double>(older.nanosec)) * 1e-9;
}
}  // namespace

PendingFixQueue::PendingFixQueue(TransformLookup lookup_antenna_to_base_link, double timeout_sec)
: lookup_antenna_to_base_link_(std::move(lookup_antenna_to_base_link)), timeout_sec_(timeout_sec)
{
  if (timeout_sec < 0.0) {
    throw std::invalid_argument(
      "antenna_transform_timeout_sec must not be negative, got " + std::to_string(timeout_sec));
  }
}

void PendingFixQueue::push(const sensor_msgs::msg::NavSatFix & fix)
{
  if (!newest_stamp_ || seconds_between(*newest_stamp_, fix.header.stamp) > 0.0) {
    newest_stamp_ = fix.header.stamp;
  }
  fixes_.push_back(fix);
}

std::optional<PendingFixQueue::Item> PendingFixQueue::next()
{
  if (fixes_.empty()) {
    return std::nullopt;
  }
  const sensor_msgs::msg::NavSatFix & fix = fixes_.front();

  if (auto transform = lookup_antenna_to_base_link_(fix.header.frame_id, fix.header.stamp)) {
    Item item{fix, std::move(transform)};
    fixes_.pop_front();
    return item;
  }
  if (is_expired(fix)) {
    Item item{fix, std::nullopt};
    fixes_.pop_front();
    return item;
  }
  // Still waiting. The fixes behind it wait too, so that they stay in order.
  return std::nullopt;
}

bool PendingFixQueue::is_expired(const sensor_msgs::msg::NavSatFix & fix) const
{
  return newest_stamp_ && seconds_between(fix.header.stamp, *newest_stamp_) > timeout_sec_;
}
}  // namespace autoware::gnss_poser
