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

// Unit tests of the diagnostics decision in gnss_poser_diagnostics.hpp, a pure function. The
// messages are for humans and not checked; the levels and the number of conditions reported are.
//
// Test naming: <Aspect>_<Condition>_<ObservedBehavior>

#include "gnss_poser.hpp"
#include "gnss_poser_diagnostics.hpp"

#include <diagnostic_msgs/msg/diagnostic_status.hpp>

#include <gtest/gtest.h>

#include <cstdint>
#include <string>

namespace
{
using autoware::gnss_poser::determine_diagnostics;
using autoware::gnss_poser::DiagnosticsState;
using autoware::gnss_poser::GnssPoser;
using diagnostic_msgs::msg::DiagnosticStatus;

// Every input present and usable: the state the node is in while it publishes poses.
DiagnosticsState healthy_state()
{
  DiagnosticsState state;
  state.fix_arrived = true;
  state.projector_info_received = true;
  state.projector_is_local = false;
  state.latest_outcome = GnssPoser::Outcome::Published;
  state.use_gnss_ins_orientation = true;
  state.ins_orientation_received = true;
  state.antenna_transform_available = true;
  state.antenna_frame = "gnss_link";
  state.base_frame = "base_link";
  return state;
}

// One condition broken in an otherwise healthy state, and the level it is reported with.
struct SingleCondition
{
  const char * name;
  void (*apply)(DiagnosticsState & state);
  int8_t level;
};

class GnssPoserDiagnosticsSingleCondition : public ::testing::TestWithParam<SingleCondition>
{
};
}  // namespace

TEST(GnssPoserDiagnostics, Level_EveryInputPresent_ReportsOk)
{
  // Arrange
  const DiagnosticsState state = healthy_state();

  // Act
  const auto result = determine_diagnostics(state);

  // Assert
  EXPECT_EQ(result.level, DiagnosticStatus::OK);
  EXPECT_TRUE(result.entries.empty());
}

// Inputs that have not arrived yet and a fix without a position solution are expected transients,
// hence warnings; the receiver may recover. A local projector and an antenna transform that TF
// cannot provide make the fixes unusable, hence errors.
TEST_P(GnssPoserDiagnosticsSingleCondition, Level_SingleCondition_ReportsItsLevel)
{
  // Arrange
  DiagnosticsState state = healthy_state();
  GetParam().apply(state);

  // Act
  const auto result = determine_diagnostics(state);

  // Assert
  EXPECT_EQ(result.level, GetParam().level);
  EXPECT_EQ(result.entries.size(), 1U);
}

INSTANTIATE_TEST_SUITE_P(
  EveryCondition, GnssPoserDiagnosticsSingleCondition,
  ::testing::Values(
    SingleCondition{
      "FixNotArrived", [](DiagnosticsState & state) { state.fix_arrived = false; },
      DiagnosticStatus::WARN},
    SingleCondition{
      "ProjectorInfoNotArrived",
      [](DiagnosticsState & state) { state.projector_info_received = false; },
      DiagnosticStatus::WARN},
    SingleCondition{
      "InsOrientationNotArrived",
      [](DiagnosticsState & state) { state.ins_orientation_received = false; },
      DiagnosticStatus::WARN},
    SingleCondition{
      "LatestFixNotFixed",
      [](DiagnosticsState & state) { state.latest_outcome = GnssPoser::Outcome::NotFixed; },
      DiagnosticStatus::WARN},
    SingleCondition{
      "LocalProjector", [](DiagnosticsState & state) { state.projector_is_local = true; },
      DiagnosticStatus::ERROR},
    SingleCondition{
      "AntennaTransformMissing",
      [](DiagnosticsState & state) { state.antenna_transform_available = false; },
      DiagnosticStatus::ERROR}),
  [](const ::testing::TestParamInfo<SingleCondition> & info) {
    return std::string(info.param.name);
  });

TEST(GnssPoserDiagnostics, Level_InsOrientationMissingWhileOrientationFromMotion_ReportsOk)
{
  // Arrange
  DiagnosticsState state = healthy_state();
  state.use_gnss_ins_orientation = false;
  state.ins_orientation_received = false;

  // Act
  const auto result = determine_diagnostics(state);

  // Assert
  EXPECT_EQ(result.level, DiagnosticStatus::OK);
  EXPECT_TRUE(result.entries.empty());
}

// Several conditions at once: one entry each, in the order they are checked, and the level is the
// highest of them.
TEST(GnssPoserDiagnostics, Level_SeveralConditions_ReportsOneEntryEachAndTheHighestLevel)
{
  // Arrange
  DiagnosticsState state = healthy_state();
  state.ins_orientation_received = false;     // WARN
  state.antenna_transform_available = false;  // ERROR

  // Act
  const auto result = determine_diagnostics(state);

  // Assert
  EXPECT_EQ(result.level, DiagnosticStatus::ERROR);
  ASSERT_EQ(result.entries.size(), 2U);
  EXPECT_EQ(result.entries[0].level, DiagnosticStatus::WARN);
  EXPECT_EQ(result.entries[1].level, DiagnosticStatus::ERROR);
}
