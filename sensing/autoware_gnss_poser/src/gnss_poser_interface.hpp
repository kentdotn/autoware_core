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
#ifndef GNSS_POSER_INTERFACE_HPP_
#define GNSS_POSER_INTERFACE_HPP_

#include "gnss_poser.hpp"

#include <autoware_map_msgs/msg/map_projector_info.hpp>
#include <autoware_sensing_msgs/msg/gnss_ins_orientation.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>

#include <vector>

namespace autoware::gnss_poser
{
/// \brief What the node drives, whichever way the antenna transform is obtained.
///
/// A fix counts as received once its position in the map frame can be computed, not when it
/// arrives: with a rigidly mounted antenna the two coincide, while an antenna that moves relative
/// to base_link needs the transform stamped like the fix, which may arrive later. Both
/// implementations therefore answer with the results that became available now, which is none,
/// one, or several.
///
/// \see StaticGnssPoser, DynamicGnssPoser
class GnssPoserInterface
{
public:
  virtual ~GnssPoserInterface() = default;

  virtual void set_projector_info(const autoware_map_msgs::msg::MapProjectorInfo & info) = 0;
  virtual void set_ins_orientation(
    const autoware_sensing_msgs::msg::GnssInsOrientation & orientation) = 0;

  /// \brief Take one fix and answer with what became computable. Exactly one result when the
  /// transform is always at hand, none or several when fixes may wait for it.
  virtual std::vector<GnssPoser::Result> input_fix(const sensor_msgs::msg::NavSatFix & fix) = 0;

  /// \brief Answer with what became computable because transforms changed. Call it whenever new
  /// transforms may have arrived; empty when no fix was waiting.
  virtual std::vector<GnssPoser::Result> input_transform_update() = 0;

  [[nodiscard]] virtual GnssPoser::Status take_status() const = 0;
};
}  // namespace autoware::gnss_poser

#endif  // GNSS_POSER_INTERFACE_HPP_
