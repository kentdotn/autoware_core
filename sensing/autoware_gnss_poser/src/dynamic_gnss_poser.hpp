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
#ifndef DYNAMIC_GNSS_POSER_HPP_
#define DYNAMIC_GNSS_POSER_HPP_

#include "gnss_poser.hpp"
#include "gnss_poser_interface.hpp"
#include "pending_message_queue.hpp"
#include "static_gnss_poser.hpp"

#include <autoware_map_msgs/msg/map_projector_info.hpp>
#include <autoware_sensing_msgs/msg/gnss_ins_orientation.hpp>
#include <geometry_msgs/msg/transform.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>

#include <memory>
#include <optional>
#include <vector>

namespace autoware::gnss_poser
{
/// \brief StaticGnssPoser with a PendingFixQueue in front of it, for an antenna that moves
/// relative to base_link.
///
/// A moving antenna needs the transform stamped like the fix, so a fix that arrives before its
/// transform cannot be answered yet; it waits in the queue and is handed to the pose computation
/// when the transform resolves, or dropped when it does not resolve within the timeout. The pose
/// computation itself is untouched and never learns about the waiting: it is given the transform
/// the queue resolved, through the lookup it already takes.
///
/// The price is the delay of the waiting, which is why the node only uses this when it is told
/// that the antenna moves.
class DynamicGnssPoser : public GnssPoserInterface
{
public:
  using PendingFixes = PendingMessageQueue<sensor_msgs::msg::NavSatFix>;

  /// \param lookup_antenna_to_base_link resolves the transform for a fix's antenna frame at the
  /// fix's own stamp, or nothing while TF cannot provide it.
  /// \param timeout_sec a queued fix is dropped once a fix newer than it by more than this has
  /// arrived; ages are measured between fix header stamps.
  /// \throw std::invalid_argument when params.buff_epoch is smaller than 1 or timeout_sec is
  /// negative.
  DynamicGnssPoser(
    const GnssPoserParams & params, PendingFixes::TransformLookup lookup_antenna_to_base_link,
    double timeout_sec, const GnssPoserCovarianceDefaults & covariance_defaults = {});

  // Non-copyable and non-movable: the pose computation holds a lookup that reads this object.
  DynamicGnssPoser(const DynamicGnssPoser &) = delete;
  DynamicGnssPoser & operator=(const DynamicGnssPoser &) = delete;
  DynamicGnssPoser(DynamicGnssPoser &&) = delete;
  DynamicGnssPoser & operator=(DynamicGnssPoser &&) = delete;
  ~DynamicGnssPoser() override = default;

  void set_projector_info(const autoware_map_msgs::msg::MapProjectorInfo & info) override;
  void set_ins_orientation(
    const autoware_sensing_msgs::msg::GnssInsOrientation & orientation) override;

  /// \brief Queue the fix and answer with the results of every fix that can be computed now,
  /// oldest first. Empty while the oldest queued fix is still waiting for its transform.
  std::vector<GnssPoser::Result> input_fix(const sensor_msgs::msg::NavSatFix & fix) override;

  /// \brief Answer with the results of the fixes whose transform has just arrived.
  std::vector<GnssPoser::Result> input_transform_update() override;

  /// \brief The pose computation's status, with the queue depth and whether the last fix that
  /// left the queue was dropped for lack of its transform.
  [[nodiscard]] GnssPoser::Status take_status() const override;

private:
  std::vector<GnssPoser::Result> drain();

  PendingFixes pending_fixes_;
  // The transform the queue resolved for the fix being computed. The pose computation reads it
  // through its lookup, so it receives the transform that belongs to that fix rather than
  // whatever TF holds by the time the fix is computed.
  std::optional<geometry_msgs::msg::Transform> resolved_transform_;
  bool last_fix_was_dropped_ = false;
  std::unique_ptr<StaticGnssPoser> gnss_poser_;
};
}  // namespace autoware::gnss_poser

#endif  // DYNAMIC_GNSS_POSER_HPP_
