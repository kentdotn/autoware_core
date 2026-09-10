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

#include "gnss_poser_diagnostics.hpp"

#include <algorithm>
#include <string>

namespace autoware::gnss_poser
{
DiagnosticsResult determine_diagnostics(const DiagnosticsState & state)
{
  using diagnostic_msgs::msg::DiagnosticStatus;

  DiagnosticsResult result;
  const auto raise = [&result](const int8_t level, const std::string & message) {
    result.entries.push_back({level, message});
    result.level = std::max(result.level, level);
    result.log_message += message;
    result.log_message += "; ";
  };

  if (!state.fix_arrived) {
    raise(DiagnosticStatus::WARN, "NavSatFix has not been received yet.");
  }
  if (!state.projector_info_received) {
    raise(
      DiagnosticStatus::WARN,
      "map_projector_info has not been received yet. Check if the map_projection_loader is "
      "successfully launched.");
  } else if (state.projector_is_local) {
    raise(
      DiagnosticStatus::ERROR,
      "map_projector_info is local projector type. Unable to convert GNSS pose.");
  }
  if (!state.latest_fix_is_fixed) {
    raise(DiagnosticStatus::WARN, "The latest NavSatFix has no position solution (not fixed).");
  }
  if (state.use_gnss_ins_orientation && !state.ins_orientation_received) {
    raise(
      DiagnosticStatus::WARN,
      "autoware_orientation has not been received yet. The identity orientation with an rmse of "
      "1.0 rad is used.");
  }
  if (!state.antenna_transform_available) {
    raise(
      DiagnosticStatus::ERROR, "Please publish TF " + state.antenna_frame + " to " +
                                 state.base_frame +
                                 ". The antenna pose is published as the base_link pose.");
  }
  return result;
}
}  // namespace autoware::gnss_poser
