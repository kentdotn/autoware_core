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

// Unit tests of the diagnostics decision in gnss_poser_diagnostics.hpp, a pure function.

#include "gnss_poser_diagnostics.hpp"

#include <diagnostic_msgs/msg/diagnostic_status.hpp>

#include <gtest/gtest.h>

#include <string>

namespace
{
using autoware::gnss_poser::determine_diagnostics;
using autoware::gnss_poser::DiagnosticsResult;
using autoware::gnss_poser::DiagnosticsState;
using diagnostic_msgs::msg::DiagnosticStatus;

// Every input present and usable: the state the node is in while it publishes poses.
DiagnosticsState healthy_state()
{
  DiagnosticsState state;
  state.fix_arrived = true;
  state.projector_info_received = true;
  state.projector_is_local = false;
  state.latest_fix_is_fixed = true;
  state.use_gnss_ins_orientation = true;
  state.ins_orientation_received = true;
  state.antenna_transform_available = true;
  state.antenna_frame = "gnss_link";
  state.base_frame = "base_link";
  return state;
}

bool has_message_containing(const DiagnosticsResult & result, const std::string & needle)
{
  for (const auto & entry : result.entries) {
    if (entry.message.find(needle) != std::string::npos) {
      return true;
    }
  }
  return false;
}
}  // namespace

// Nothing to report while every input is present and usable.
TEST(GnssPoserDiagnostics, OkWhenHealthy)
{
  const DiagnosticsResult result = determine_diagnostics(healthy_state());

  EXPECT_EQ(result.level, DiagnosticStatus::OK);
  EXPECT_TRUE(result.entries.empty());
  EXPECT_TRUE(result.log_message.empty());
}

// Inputs that have not arrived yet are warnings: fix, map projector info, and the INS orientation
// when it is the configured source.
TEST(GnssPoserDiagnostics, WarnWhileInputsHaveNotArrived)
{
  DiagnosticsState state = healthy_state();
  state.fix_arrived = false;
  state.projector_info_received = false;
  state.ins_orientation_received = false;

  const DiagnosticsResult result = determine_diagnostics(state);

  EXPECT_EQ(result.level, DiagnosticStatus::WARN);
  EXPECT_EQ(result.entries.size(), 3U);
  EXPECT_TRUE(has_message_containing(result, "NavSatFix"));
  EXPECT_TRUE(has_message_containing(result, "map_projector_info"));
  EXPECT_TRUE(has_message_containing(result, "autoware_orientation"));
}

// A missing INS orientation is not reported when the orientation comes from motion.
TEST(GnssPoserDiagnostics, MissingInsOrientationIgnoredInMotionMode)
{
  DiagnosticsState state = healthy_state();
  state.use_gnss_ins_orientation = false;
  state.ins_orientation_received = false;

  const DiagnosticsResult result = determine_diagnostics(state);

  EXPECT_EQ(result.level, DiagnosticStatus::OK);
}

// A fix without a position solution is a warning, not an error: the receiver may recover.
TEST(GnssPoserDiagnostics, WarnWhenLatestFixIsNotFixed)
{
  DiagnosticsState state = healthy_state();
  state.latest_fix_is_fixed = false;

  const DiagnosticsResult result = determine_diagnostics(state);

  EXPECT_EQ(result.level, DiagnosticStatus::WARN);
  EXPECT_TRUE(has_message_containing(result, "not fixed"));
}

// A local projector makes every fix unusable: error. It supersedes the not-received warning.
TEST(GnssPoserDiagnostics, ErrorOnLocalProjector)
{
  DiagnosticsState state = healthy_state();
  state.projector_is_local = true;

  const DiagnosticsResult result = determine_diagnostics(state);

  EXPECT_EQ(result.level, DiagnosticStatus::ERROR);
  EXPECT_EQ(result.entries.size(), 1U);
  EXPECT_TRUE(has_message_containing(result, "local projector"));
}

// An antenna transform TF cannot provide is an error, and the message names both frames.
TEST(GnssPoserDiagnostics, ErrorOnMissingAntennaTransform)
{
  DiagnosticsState state = healthy_state();
  state.antenna_transform_available = false;

  const DiagnosticsResult result = determine_diagnostics(state);

  EXPECT_EQ(result.level, DiagnosticStatus::ERROR);
  EXPECT_TRUE(has_message_containing(result, "gnss_link to base_link"));
}

// Several conditions at once: one entry each, the level is the maximum, the log joins them.
TEST(GnssPoserDiagnostics, AggregatesToMaxSeverity)
{
  DiagnosticsState state = healthy_state();
  state.ins_orientation_received = false;     // WARN
  state.antenna_transform_available = false;  // ERROR

  const DiagnosticsResult result = determine_diagnostics(state);

  EXPECT_EQ(result.level, DiagnosticStatus::ERROR);
  EXPECT_EQ(result.entries.size(), 2U);
  EXPECT_EQ(result.entries[0].level, DiagnosticStatus::WARN);
  EXPECT_EQ(result.entries[1].level, DiagnosticStatus::ERROR);
  EXPECT_NE(result.log_message.find("; "), std::string::npos);
}
