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
#include "common/graph.hpp"
#include "common/util.hpp"
#include "common/yaml.hpp"
#include <functional>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

//
#include <iostream>

namespace multi_data_monitor
{

using StreamGraph = graph::Graph<StreamLink>;
using WidgetGraph = graph::Graph<WidgetLink>;

StreamGraph create_stream_graph(const StreamList & streams)
{
  StreamGraph graph;
  for (const auto & stream : streams)
  {
    graph[stream] = stream->input ? StreamList{stream->input} : StreamList{};
  }
  return graph;
}

WidgetGraph create_widget_graph(const WidgetList & widgets)
{
  WidgetGraph graph;
  for (const auto & widget : widgets)
  {
    const auto item_link = [](const WidgetItem & item) { return item.link; };
    graph[widget] = util::map<WidgetItem, WidgetLink>(widget->items, item_link);
  }
  return graph;
}

template <class T>
std::vector<T> normalize_graph(const graph::Graph<T> & graph, bool tree)
{
  const auto result = graph::topological_sort(graph);
  if (result.size() != graph.size())
  {
    throw GraphCirculation("graph loop is detected");
  }
  if (tree && !graph::is_tree(graph))
  {
    throw GraphIsNotTree("graph is not tree");
  }
  return result;
}

struct ConfigLinks
{
  std::vector<std::reference_wrapper<StreamLink>> to_streams;
  std::vector<std::reference_wrapper<WidgetLink>> to_widgets;
};

ConfigLinks create_link_list(const ConfigData & input)
{
  std::vector<std::reference_wrapper<StreamLink>> to_streams;
  std::vector<std::reference_wrapper<WidgetLink>> to_widgets;
  for (const auto & stream : input.streams)
  {
    to_streams.push_back(std::ref(stream->input));
    to_streams.push_back(std::ref(stream->refer));
    to_widgets.push_back(std::ref(stream->panel));
  }
  for (const auto & widget : input.widgets)
  {
    ;
    to_widgets.push_back(std::ref(widget->refer));
    for (auto & item : widget->items)
    {
      to_widgets.push_back(std::ref(item.link));
    }
  }

  ConfigLinks links;
  const auto has_ref = [](const auto & ref) { return static_cast<bool>(ref.get()); };
  links.to_streams = util::filter<std::reference_wrapper<StreamLink>>(to_streams, has_ref);
  links.to_widgets = util::filter<std::reference_wrapper<WidgetLink>>(to_widgets, has_ref);
  return links;
}

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
NodeLink resolve_node(const NodeLink & node, std::unordered_set<NodeLink> & visit)
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
    throw LabelCirculation("label loop is detected: " + util::join(labels));
  }
  visit.insert(node);
  node->refer = resolve_node(node->refer, visit);
  return node->refer;
}

template <class NodeLink>
void resolve_link(std::reference_wrapper<NodeLink> link)
{
  std::unordered_set<NodeLink> visit;
  link.get() = resolve_node(link.get(), visit);
}

template <class NodeLink, class NodeList = std::vector<NodeLink>, class NodeSet = std::unordered_set<NodeLink>>
NodeList filter_unused(const NodeList & nodes, const NodeSet & node_used, const std::string & klass)
{
  NodeList result;
  for (const auto & node : nodes)
  {
    if (node->klass == klass)
    {
      if (node_used.count(node) == 0) continue;
      throw LogicError("ReleaseRelation: unintended stream reverse reference");
    }
    result.push_back(node);
  }
  return result;
}

ConfigData ConnectRelation::execute(const ConfigData & input)
{
  std::unordered_map<std::string, StreamLink> stream_labels;
  std::unordered_map<std::string, WidgetLink> widget_labels;

  for (const auto & node : input.streams) create_label(node, stream_labels);
  for (const auto & node : input.widgets) create_label(node, widget_labels);
  for (const auto & node : input.streams) connect_label(node, stream_labels);
  for (const auto & node : input.widgets) connect_label(node, widget_labels);
  return input;
}

ConfigData ResolveRelation::execute(const ConfigData & input)
{
  const auto links = create_link_list(input);
  for (const auto & link : links.to_streams) resolve_link(link);
  for (const auto & link : links.to_widgets) resolve_link(link);
  return input;
}

ConfigData ReleaseRelation::execute(const ConfigData & input)
{
  std::unordered_set<StreamLink> stream_used;
  std::unordered_set<WidgetLink> widget_used;

  const auto links = create_link_list(input);
  for (const auto & link : links.to_streams) stream_used.insert(link.get());
  for (const auto & link : links.to_widgets) widget_used.insert(link.get());

  ConfigData output = input;
  output.streams = filter_unused<StreamLink>(output.streams, stream_used, builtin::subscription);
  output.streams = filter_unused<StreamLink>(output.streams, stream_used, builtin::relay);
  output.widgets = filter_unused<WidgetLink>(output.widgets, widget_used, builtin::relay);
  return output;
}

ConfigData NormalizeRelation::execute(const ConfigData & input)
{
  const auto stream_graph = create_stream_graph(input.streams);
  const auto widget_graph = create_widget_graph(input.widgets);

  ConfigData output;
  output.streams = normalize_graph(stream_graph, false);
  output.widgets = normalize_graph(widget_graph, true);
  return output;
}

}  // namespace multi_data_monitor
