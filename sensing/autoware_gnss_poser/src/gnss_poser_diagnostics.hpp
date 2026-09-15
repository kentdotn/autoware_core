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
#ifndef GNSS_POSER_DIAGNOSTICS_HPP_
#define GNSS_POSER_DIAGNOSTICS_HPP_

#include <diagnostic_msgs/msg/diagnostic_status.hpp>

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace autoware::gnss_poser
{
/// \brief Inputs needed to decide the gnss_poser diagnostics level and messages.
struct DiagnosticsState
{
  bool fix_arrived = false;
  bool projector_info_received = false;
  bool projector_is_local = false;
  bool latest_fix_is_fixed = true;  ///< false when the most recent fix had no position solution
  bool use_gnss_ins_orientation = true;
  bool ins_orientation_received = false;
  std::size_t pending_fix_count = 0;  ///< fixes held until their antenna transform arrives
  /// The last fix that was evaluated produced no pose for lack of its antenna transform.
  bool fixes_dropped_for_missing_transform = false;
  std::string antenna_frame;
  std::string base_frame;
};

struct DiagnosticsEntry
{
  int8_t level = diagnostic_msgs::msg::DiagnosticStatus::OK;
  std::string message;
};

/// \brief Result of evaluating the diagnostics state: one entry per triggered condition, the
/// maximum level across them, and every message concatenated for logging.
struct DiagnosticsResult
{
  std::vector<DiagnosticsEntry> entries;
  int8_t level = diagnostic_msgs::msg::DiagnosticStatus::OK;
  std::string log_message;
};

/// \brief Evaluate the diagnostics state. Pure function: no node, clock or interface dependency.
///
/// Missing inputs (fix, map projector info, INS orientation), a fix without a position solution
/// and fixes waiting for their antenna transform are WARN: they are expected transients. A local
/// projector and fixes dropped for lack of their antenna transform are ERROR: the first makes
/// every fix unusable, the second means TF does not carry the antenna frame of the fixes.
DiagnosticsResult determine_diagnostics(const DiagnosticsState & state);
}  // namespace autoware::gnss_poser

#endif  // GNSS_POSER_DIAGNOSTICS_HPP_
