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
#ifndef STATIC_GNSS_POSER_HPP_
#define STATIC_GNSS_POSER_HPP_

#include "gnss_poser.hpp"
#include "gnss_poser_interface.hpp"

#include <autoware_map_msgs/msg/map_projector_info.hpp>
#include <autoware_sensing_msgs/msg/gnss_ins_orientation.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>

#include <vector>

namespace autoware::gnss_poser
{
/// \brief GnssPoser as the node drives it when the antenna is rigidly mounted: every fix is
/// answered at once, because the transform to base_link is the same at every time and is either
/// already known or never will be.
class StaticGnssPoser : public GnssPoserInterface
{
public:
  StaticGnssPoser(
    const GnssPoserParams & params, GnssPoser::TransformLookup lookup_antenna_to_base_link,
    const GnssPoserCovarianceDefaults & covariance_defaults = {});

  void set_projector_info(const autoware_map_msgs::msg::MapProjectorInfo & info) override;
  void set_ins_orientation(
    const autoware_sensing_msgs::msg::GnssInsOrientation & orientation) override;

  /// \brief The one result of the fix.
  std::vector<GnssPoser::Result> input_fix(const sensor_msgs::msg::NavSatFix & fix) override;

  /// \brief Always empty: no fix is ever waiting for a transform.
  std::vector<GnssPoser::Result> input_transform_update() override;

  [[nodiscard]] GnssPoser::Status take_status() const override;

private:
  GnssPoser gnss_poser_;
};
}  // namespace autoware::gnss_poser

#endif  // STATIC_GNSS_POSER_HPP_
