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

#include "resolve_relation.hpp"
#include "common/exceptions.hpp"
#include "common/util.hpp"
#include "common/yaml.hpp"
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace multi_data_monitor
{

template <class NodeLink>
void create_label(const NodeLink & node, std::unordered_map<std::string, NodeLink> & labels)
{
  const auto & label = node->label;
  if (label.empty())
  {
    return;
  }
  if (labels.count(label))
  {
    throw ConfigError::LabelConflict(label, NodeTraits<NodeLink>::Name);
  }
  labels[label] = node;
}

template <class NodeLink>
void connect_label(const NodeLink & node, const std::unordered_map<std::string, NodeLink> & labels)
{
  if (node->klass == builtin::relay)
  {
    const auto refer = yaml::take_required(node->yaml, "refer").template as<std::string>();
    if (labels.count(refer) == 0)
    {
      throw ConfigError::LabelNotFound(refer, NodeTraits<NodeLink>::Name);
    }
    node->refer = labels.at(refer);
  }
}

template <class NodeLink>
NodeLink resolve_link(const NodeLink & node, std::unordered_set<NodeLink> & visit)
{
  if (!node->refer)
  {
    return node;
  }
  if (visit.count(node))
  {
    std::vector<std::string> labels;
    for (const NodeLink & node : visit)
    {
      if (!node->label.empty()) labels.push_back(node->label);
    }
    throw ConfigError("circular link is detected: " + util::join(labels));
  }
  visit.insert(node);
  node->refer = resolve_link(node->refer, visit);
  return node->refer;
}

template <class NodeLink>
void resolve_node(const NodeLink & node)
{
  if (node->refer)
  {
    std::unordered_set<decltype(node->refer)> visit;
    node->refer = resolve_link(node->refer, visit);
  }
  if (node->input)
  {
    std::unordered_set<decltype(node->input)> visit;
    node->input = resolve_link(node->input, visit);
  }
}

ConfigData ConnectRelation::execute(const ConfigData & input)
{
  std::unordered_map<std::string, StreamLink> stream_labels;
  std::unordered_map<std::string, WidgetLink> widget_labels;

  for (const auto & stream : input.streams) create_label(stream, stream_labels);
  for (const auto & widget : input.widgets) create_label(widget, widget_labels);
  for (const auto & stream : input.streams) connect_label(stream, stream_labels);
  for (const auto & widget : input.widgets) connect_label(widget, widget_labels);
  return input;
}

ConfigData ResolveRelation::execute(const ConfigData & input)
{
  for (const auto & stream : input.streams) resolve_node(stream);
  for (const auto & widget : input.widgets) resolve_node(widget);
  return input;
}

ConfigData ReleaseRelation::execute(const ConfigData & input)
{
  std::unordered_map<StreamLink, int> stream_used;
  std::unordered_map<WidgetLink, int> widget_used;
  for (const auto & stream : input.streams)
  {
    if (stream->refer) ++stream_used[stream->refer];
    if (stream->input) ++stream_used[stream->input];
  }
  for (const auto & widget : input.widgets)
  {
    if (widget->refer) ++widget_used[widget->refer];
    if (widget->input) ++stream_used[widget->input];
  }

  std::vector<StreamLink> output_streams;
  std::vector<WidgetLink> output_widgets;
  for (const auto & stream : input.streams)
  {
    if (stream->klass == builtin::relay || stream->klass == builtin::subscription)
    {
      if (stream_used[stream] == 0) continue;
      throw LogicError("ReleaseRelation: unintended stream reverse reference");
    }
    output_streams.push_back(stream);
  }
  for (const auto & widget : input.widgets)
  {
    if (widget->klass == builtin::relay)
    {
      if (widget_used[widget] == 0) continue;
      throw LogicError("ReleaseRelation: unintended widget reverse reference");
    }
    output_widgets.push_back(widget);
  }

  ConfigData output = input;
  output.streams = output_streams;
  output.widgets = output_widgets;
  return output;
}

}  // namespace multi_data_monitor
