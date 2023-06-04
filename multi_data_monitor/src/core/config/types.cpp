// Copyright 2022 Takagi, Isamu
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

#include "types.hpp"
#include <memory>

namespace multi_data_monitor
{

NodeTrack NodeTrack::Create(const std::string & start)
{
  return NodeTrack(start);
}

NodeTrack NodeTrack::Create(const std::string & start, size_t index)
{
  return NodeTrack(start + "-" + std::to_string(index));
}

NodeTrack::NodeTrack(const std::string & start)
{
  start_ = start;
  input_ = 0;
}

NodeTrack NodeTrack::panel() const
{
  NodeTrack track(*this);
  track.extra_ += "-panel";
  return track;
}

NodeTrack NodeTrack::apply() const
{
  NodeTrack track(*this);
  track.extra_ += "-apply";
  return track;
}

NodeTrack NodeTrack::rules() const
{
  NodeTrack track(*this);
  track.rules_ += "-rules";
  return track;
}

NodeTrack NodeTrack::input() const
{
  NodeTrack track(*this);
  track.input_ += 1;
  return track;
}
NodeTrack NodeTrack::items(size_t index) const
{
  NodeTrack track(*this);
  track.items_ += "-" + std::to_string(index);
  return track;
}

NodeTrack NodeTrack::rules(size_t index) const
{
  NodeTrack track(*this);
  track.rules_ += "-" + std::to_string(index);
  return track;
}

std::string NodeTrack::text() const
{
  const auto input_depth = input_ ? "-input-" + std::to_string(input_) : "";
  return start_ + items_ + input_depth + rules_ + extra_;
}

CommonData::CommonData(const NodeClass & klass, const NodeTrack & track, const NodeLabel & label) : klass(klass), track(track), label(label)
{
  ++created;
}

CommonData::~CommonData()
{
  ++removed;
}

FilterLink ConfigData::create_filter(const NodeClass & klass, const NodeTrack & track, const NodeLabel & label, YAML::Node yaml)
{
  const auto data = std::make_shared<FilterData>(klass, track, label);
  data->yaml = yaml;
  return filters.emplace_back(data);
}

StreamLink ConfigData::create_stream(const NodeClass & klass, const NodeTrack & track, const NodeLabel & label, YAML::Node yaml)
{
  const auto data = std::make_shared<StreamData>(klass, track, label);
  data->yaml = yaml;
  return streams.emplace_back(data);
}

WidgetLink ConfigData::create_widget(const NodeClass & klass, const NodeTrack & track, const NodeLabel & label, YAML::Node yaml)
{
  const auto data = std::make_shared<WidgetData>(klass, track, label);
  data->yaml = yaml;
  return widgets.emplace_back(data);
}

}  // namespace multi_data_monitor
