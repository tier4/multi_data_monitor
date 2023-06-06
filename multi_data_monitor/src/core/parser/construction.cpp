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

#include "construction.hpp"
#include "common/yaml.hpp"
#include <multi_data_monitor/config.hpp>
#include <multi_data_monitor/errors.hpp>
#include <functional>
#include <string>
#include <utility>
#include <vector>

namespace multi_data_monitor
{

using RootFunc = void (ParseBasicObject::*)(YAML::Node, const NodeTrack & track);

auto enumerate(YAML::Node sequence)
{
  std::vector<std::pair<size_t, YAML::Node>> result;
  for (size_t i = 0; i < sequence.size(); ++i)
  {
    result.emplace_back(i, sequence[i]);
  }
  return result;
}

void parse(YAML::Node & yaml, const std::string & name, ParseBasicObject * self, RootFunc func)
{
  const auto nodes = yaml::take_optional(yaml, name);
  if (!nodes.IsDefined())
  {
    return;
  }
  if (!nodes.IsSequence())
  {
    throw ConfigError("config section '" + name + "' is not a sequence");
  }
  for (const auto & [i, node] : enumerate(nodes))
  {
    std::invoke(func, self, node, NodeTrack::Create(name, i));
  }
}

ConfigData ParseBasicObject::execute(ConfigFile & file)
{
  parse(file.yaml, "subscriptions", this, &ParseBasicObject::parse_subscription);
  parse(file.yaml, "filters", this, &ParseBasicObject::parse_filter_root);
  parse(file.yaml, "streams", this, &ParseBasicObject::parse_stream_root);
  parse(file.yaml, "widgets", this, &ParseBasicObject::parse_widget_root);
  return data_;
}

void ParseBasicObject::parse_filter_root(YAML::Node yaml, const NodeTrack & track)
{
  parse_filter_yaml(yaml, track);
}

void ParseBasicObject::parse_stream_root(YAML::Node yaml, const NodeTrack & track)
{
  parse_stream_yaml(yaml, track);
}

void ParseBasicObject::parse_widget_root(YAML::Node yaml, const NodeTrack & track)
{
  parse_widget_yaml(yaml, track);
}

void ParseBasicObject::parse_subscription(YAML::Node topic, const NodeTrack & track)
{
  if (!topic.IsMap())
  {
    // TODO(Takagi, Isamu): error message
    throw ConfigError("is not map");
  }

  const auto fields = yaml::take_required(topic, "fields");
  if (!fields.IsSequence())
  {
    // TODO(Takagi, Isamu): error message
    throw ConfigError("topic fields is not sequence");
  }

  const auto topic_name = yaml::take_required(topic, "name");
  const auto topic_type = yaml::take_optional(topic, "type");
  const auto topic_qos = yaml::take_optional(topic, "qos");
  yaml::check_empty(topic);

  for (YAML::Node field : fields)
  {
    const auto label = yaml::take_optional(field, "label").as<std::string>("");
    const auto field_name = yaml::take_required(field, "name");
    const auto field_type = yaml::take_optional(field, "type");
    yaml::check_empty(field);

    YAML::Node yaml;
    yaml["topic"] = topic_name;
    yaml["field"] = field_name;
    yaml["topic-type"] = topic_type;
    yaml["field-type"] = field_type;
    yaml["qos"] = topic_qos;

    const auto stream = data_.create_stream(builtin::subscription, track, label, yaml);
    stream->system = true;
  }
}

FilterLink ParseBasicObject::parse_filter_yaml(YAML::Node yaml, const NodeTrack & track)
{
  if (yaml.IsScalar())
  {
    return parse_filter_link(yaml, track);
  }
  if (yaml.IsMap())
  {
    return parse_filter_dict(yaml, track);
  }
  if (yaml.IsSequence())
  {
    return parse_filter_list(yaml, track);
  }
  // TODO(Takagi, Isamu): error message
  throw ConfigError("unexpected filter format");
}

StreamLink ParseBasicObject::parse_stream_yaml(YAML::Node yaml, const NodeTrack & track)
{
  if (yaml.IsScalar())
  {
    return parse_stream_link(yaml, track);
  }
  if (yaml.IsMap())
  {
    return parse_stream_dict(yaml, track);
  }
  // TODO(Takagi, Isamu): error message
  throw ConfigError("unexpected stream format");
}

WidgetLink ParseBasicObject::parse_widget_yaml(YAML::Node yaml, const NodeTrack & track)
{
  if (yaml.IsScalar())
  {
    return parse_widget_link(yaml, track);
  }
  if (yaml.IsMap())
  {
    return parse_widget_dict(yaml, track);
  }
  // TODO(Takagi, Isamu): error message
  throw ConfigError("unexpected widget format");
}

FilterLink ParseBasicObject::parse_filter_link(YAML::Node yaml, const NodeTrack & track)
{
  FilterLink filter = data_.create_filter(builtin::relay, track);
  filter->system = true;
  filter->yaml["refer"] = yaml.as<std::string>();
  return filter;
}

StreamLink ParseBasicObject::parse_stream_link(YAML::Node yaml, const NodeTrack & track)
{
  StreamLink stream = data_.create_stream(builtin::relay, track);
  stream->system = true;
  stream->yaml["refer"] = yaml.as<std::string>();
  return stream;
}

WidgetLink ParseBasicObject::parse_widget_link(YAML::Node yaml, const NodeTrack & track)
{
  WidgetLink widget = data_.create_widget(builtin::relay, track);
  widget->system = true;
  widget->yaml["refer"] = yaml.as<std::string>();
  return widget;
}

FilterLink ParseBasicObject::parse_filter_dict(YAML::Node yaml, const NodeTrack & track)
{
  ConfigObject object(yaml, track, false);
  const auto klass = object.take_required_data<std::string>("class");
  const auto label = object.take_optional_data<std::string>("label", "");
  FilterLink filter = data_.create_filter(klass, track, label, yaml);

  if (klass == builtin::function)
  {
    YAML::Node rules = object.take_required_node("rules");
    if (!rules.IsSequence())
    {
      throw InvalidNodeType("rules", track, "sequence");
    }
    for (const auto & [i, rule] : enumerate(rules))
    {
      filter->items.push_back(parse_filter_yaml(rule, track.rules(i)));
    }
  }
  return filter;
}

StreamLink ParseBasicObject::parse_stream_dict(YAML::Node yaml, const NodeTrack & track)
{
  ConfigObject object(yaml, track, false);
  const auto klass = object.take_required_data<std::string>("class");
  const auto label = object.take_optional_data<std::string>("label", "");
  StreamLink stream = data_.create_stream(klass, track, label, yaml);

  const auto input = object.take_optional_node("input");
  if (input)
  {
    stream->items = StreamList{parse_stream_yaml(input, track.input())};
  }
  if (klass == builtin::apply)
  {
    const auto rules = object.take_required_node("rules");
    if (rules)
    {
      stream->apply = parse_filter_yaml(rules, track.rules());
    }
  }
  return stream;
}

WidgetLink ParseBasicObject::parse_widget_dict(YAML::Node yaml, const NodeTrack & track)
{
  ConfigObject object(yaml, track, false);
  const auto klass = object.take_required_data<std::string>("class");
  const auto label = object.take_optional_data<std::string>("label", "");
  WidgetLink widget = data_.create_widget(klass, track, label, yaml);

  const auto items = object.take_optional_node("items");
  if (items)
  {
    if (!items.IsSequence())
    {
      throw InvalidNodeType("items", track, "sequence");
    }
    for (const auto & [i, item] : enumerate(items))
    {
      widget->items.push_back(parse_widget_yaml(item, track.items(i)));
    }
  }

  const auto input = object.take_optional_node("input");
  if (input)
  {
    StreamLink panel = data_.create_stream(builtin::panel, track.panel());
    panel->system = true;
    panel->panel = widget;
    panel->items = StreamList{parse_stream_yaml(input, track.input())};

    const auto rules = object.take_optional_node("rules");
    if (rules)
    {
      StreamLink apply = data_.create_stream(builtin::apply, track.apply());
      apply->system = true;
      apply->apply = parse_filter_yaml(rules, track.rules());
      apply->items = panel->items;
      panel->items = StreamList{apply};
    }
  }
  return widget;
}

FilterLink ParseBasicObject::parse_filter_list(YAML::Node yaml, const NodeTrack & track)
{
  FilterLink filter = data_.create_filter(builtin::function, track);
  for (const auto & [i, node] : enumerate(yaml))
  {
    filter->items.push_back(parse_filter_yaml(node, track.rules(i)));
  }
  return filter;
}

}  // namespace multi_data_monitor
