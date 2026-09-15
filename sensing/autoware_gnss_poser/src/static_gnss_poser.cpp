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

#include "static_gnss_poser.hpp"

#include <utility>
#include <vector>

namespace autoware::gnss_poser
{
StaticGnssPoser::StaticGnssPoser(
  const GnssPoserParams & params, GnssPoser::TransformLookup lookup_antenna_to_base_link,
  const GnssPoserCovarianceDefaults & covariance_defaults)
: gnss_poser_(params, std::move(lookup_antenna_to_base_link), covariance_defaults)
{
}

void StaticGnssPoser::set_projector_info(const autoware_map_msgs::msg::MapProjectorInfo & info)
{
  gnss_poser_.set_projector_info(info);
}

void StaticGnssPoser::set_ins_orientation(
  const autoware_sensing_msgs::msg::GnssInsOrientation & orientation)
{
  gnss_poser_.set_ins_orientation(orientation);
}

std::vector<GnssPoser::Result> StaticGnssPoser::input_fix(const sensor_msgs::msg::NavSatFix & fix)
{
  return {gnss_poser_.input_fix(fix)};
}

std::vector<GnssPoser::Result> StaticGnssPoser::input_transform_update()
{
  return {};
}

GnssPoser::Status StaticGnssPoser::take_status() const
{
  return gnss_poser_.take_status();
}
}  // namespace autoware::gnss_poser
