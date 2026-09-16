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

#include "dynamic_gnss_poser.hpp"

#include <cstddef>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace autoware::gnss_poser
{
namespace
{
// Hard bound on the queue, so that fixes whose stamps do not advance cannot grow it without end.
// Well above what any GNSS rate produces within the timeout.
constexpr std::size_t max_pending_fixes = 32;
}  // namespace

DynamicGnssPoser::DynamicGnssPoser(
  const GnssPoserParams & params, PendingFixes::TransformLookup lookup_antenna_to_base_link,
  const double timeout_sec, const GnssPoserCovarianceDefaults & covariance_defaults)
: pending_fixes_(
    std::move(lookup_antenna_to_base_link), PendingFixes::Limits{timeout_sec, max_pending_fixes}),
  gnss_poser_(
    std::make_unique<StaticGnssPoser>(
      params, [this](const std::string &) { return resolved_transform_; }, covariance_defaults))
{
}

void DynamicGnssPoser::set_projector_info(const autoware_map_msgs::msg::MapProjectorInfo & info)
{
  gnss_poser_->set_projector_info(info);
}

void DynamicGnssPoser::set_ins_orientation(
  const autoware_sensing_msgs::msg::GnssInsOrientation & orientation)
{
  gnss_poser_->set_ins_orientation(orientation);
}

std::vector<GnssPoser::Result> DynamicGnssPoser::input_fix(const sensor_msgs::msg::NavSatFix & fix)
{
  pending_fixes_.push(fix);
  return drain();
}

std::vector<GnssPoser::Result> DynamicGnssPoser::input_transform_update()
{
  return drain();
}

std::vector<GnssPoser::Result> DynamicGnssPoser::drain()
{
  std::vector<GnssPoser::Result> results;
  while (const std::optional<PendingFixes::Item> item = pending_fixes_.next()) {
    last_fix_was_dropped_ = !item->transform;
    if (last_fix_was_dropped_) {
      // The fix never became computable, so it was never received: it leaves no result behind,
      // and it does not enter the position buffer either. take_status() reports it instead.
      continue;
    }
    resolved_transform_ = item->transform;
    for (GnssPoser::Result & result : gnss_poser_->input_fix(item->message)) {
      results.push_back(std::move(result));
    }
  }
  resolved_transform_.reset();
  return results;
}

GnssPoser::Status DynamicGnssPoser::take_status() const
{
  GnssPoser::Status status = gnss_poser_->take_status();
  status.pending_fix_count = pending_fixes_.size();
  status.fixes_dropped_for_missing_transform = last_fix_was_dropped_;
  return status;
}
}  // namespace autoware::gnss_poser
