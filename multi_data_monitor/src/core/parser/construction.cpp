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
#include "common/exceptions.hpp"
#include "common/yaml.hpp"
#include <functional>
#include <string>

namespace multi_data_monitor
{

using RootFunc = void (ParseBasicObject::*)(YAML::Node);

void parse(YAML::Node & yaml, const std::string & name, ParseBasicObject * self, RootFunc func)
{
  const auto nodes = yaml::take_optional(yaml, name);
  if (!nodes.IsSequence())
  {
    throw ConfigError("config seccion '" + name + "' is not a sequence");
  }
  for (const auto & node : nodes)
  {
    std::invoke(func, self, node);
  }
}

ConfigData ParseBasicObject::execute(ConfigFile & file)
{
  parse(file.yaml, "subscriptions", this, &ParseBasicObject::parse_subscription);
  parse(file.yaml, "streams", this, &ParseBasicObject::parse_stream_root);
  parse(file.yaml, "widgets", this, &ParseBasicObject::parse_widget_root);
  return data_;
}

void ParseBasicObject::parse_stream_root(YAML::Node yaml)
{
  parse_stream_yaml(yaml);
}

void ParseBasicObject::parse_widget_root(YAML::Node yaml)
{
  parse_widget_yaml(yaml);
}

void ParseBasicObject::parse_subscription(YAML::Node topic)
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

    const auto stream = data_.create_stream(builtin::subscription, label, yaml);
    stream->system = true;
  }
}

StreamLink ParseBasicObject::parse_stream_yaml(YAML::Node yaml)
{
  if (yaml.IsScalar())
  {
    return parse_stream_link(yaml);
  }
  if (yaml.IsMap())
  {
    return parse_stream_dict(yaml);
  }
  // TODO(Takagi, Isamu): error message
  throw ConfigError("unexpected stream format");
}

WidgetLink ParseBasicObject::parse_widget_yaml(YAML::Node yaml)
{
  if (yaml.IsScalar())
  {
    return parse_widget_link(yaml);
  }
  if (yaml.IsMap())
  {
    return parse_widget_dict(yaml);
  }
  // TODO(Takagi, Isamu): error message
  throw ConfigError("unexpected widget format");
}

StreamLink ParseBasicObject::parse_stream_link(YAML::Node yaml)
{
  StreamLink stream = data_.create_stream(builtin::relay);
  stream->system = true;
  stream->yaml["refer"] = yaml.as<std::string>();
  return stream;
}

WidgetLink ParseBasicObject::parse_widget_link(YAML::Node yaml)
{
  WidgetLink widget = data_.create_widget(builtin::relay);
  widget->yaml["refer"] = yaml.as<std::string>();
  return widget;
}

StreamLink ParseBasicObject::parse_stream_dict(YAML::Node yaml)
{
  const auto klass = yaml::take_required(yaml, "class").as<std::string>("");
  const auto label = yaml::take_optional(yaml, "label").as<std::string>("");
  const auto input = yaml::take_optional(yaml, "input");

  StreamLink stream = data_.create_stream(klass, label, yaml);
  if (input)
  {
    stream->input = parse_stream_yaml(input);
  }
  return stream;
}

WidgetLink ParseBasicObject::parse_widget_dict(YAML::Node yaml)
{
  const auto klass = yaml::take_required(yaml, "class").as<std::string>("");
  const auto label = yaml::take_optional(yaml, "label").as<std::string>("");
  const auto input = yaml::take_optional(yaml, "input");

  WidgetLink widget = data_.create_widget(klass, label, yaml);
  if (input)
  {
    StreamLink stream = data_.create_stream(builtin::panel);
    stream->system = true;
    stream->input = parse_stream_yaml(input);
    widget->input = stream;
  }
  return widget;
}

}  // namespace multi_data_monitor
