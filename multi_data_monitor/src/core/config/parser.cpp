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

#include "parser.hpp"
#include "exceptions.hpp"
#include "types.hpp"
#include "utils.hpp"
#include <filesystem>
#include <iostream>
#include <memory>

// temp
#include <vector>

namespace multi_data_monitor
{

YAML::Node take_optional(YAML::Node & yaml, const std::string & name)
{
  const auto node = yaml[name];
  yaml.remove(name);
  return node;
}

YAML::Node take_required(YAML::Node & yaml, const std::string & name)
{
  const auto node = yaml[name];
  if (!node)
  {
    // TODO(Takagi, Isamu): add debug info
    throw ConfigError("required key: " + name);
  }
  yaml.remove(name);
  return node;
}

void check_empty(YAML::Node & yaml)
{
  std::vector<std::string> fields;
  for (const auto pair : yaml)
  {
    fields.push_back(pair.first.as<std::string>());
  }
  if (!fields.empty())
  {
    // TODO(Takagi, Isamu): use join
    std::string text;
    for (const auto & field : fields)
    {
      text += " " + field;
    }
    throw ConfigError("not empty:" + text);
  }
}

ConfigFile ConfigLoader::operator()(const std::string & input)
{
  const auto path = std::filesystem::path(path::resolve(input));
  if (!std::filesystem::exists(path))
  {
    throw FilePathError("file not found '" + path.string() + "'");
  }

  // TODO(Takagi, Isamu): handle yaml error
  auto yaml = YAML::LoadFile(path);

  // Check version.
  const auto version = take_optional(yaml, "version").as<std::string>("undefined");
  if (version != "2.0")
  {
    throw ConfigError("not supported version '" + version + "'");
  }

  const auto streams = take_optional(yaml, "streams");
  if (!streams.IsSequence())
  {
    throw ConfigError("streams config is not sequence");
  }
  for (const auto & stream : streams)
  {
    output_.streams.push_back(stream);
  }

  const auto subscriptions = take_optional(yaml, "subscriptions");
  if (!subscriptions.IsSequence())
  {
    throw ConfigError("subscriptions config is not sequence");
  }
  for (const auto & subscription : subscriptions)
  {
    output_.subscriptions.push_back(subscription);
  }

  return output_;
}

StreamList ConstructSubscriptions::operator()(const ConfigFile & input)
{
  for (const auto & node : input.subscriptions)
  {
    parse_subscription(node);
  }
  return output_;
}

void ConstructSubscriptions::parse_subscription(YAML::Node topic)
{
  if (!topic.IsMap())
  {
    // TODO(Takagi, Isamu): error message
    throw ConfigError("is not map");
  }

  const auto fields = take_optional(topic, "fields");
  if (!fields.IsSequence())
  {
    // TODO(Takagi, Isamu): error message
    throw ConfigError("is not sequence");
  }

  const auto topic_name = take_required(topic, "name");
  const auto topic_type = take_optional(topic, "type");
  const auto topic_qos = take_optional(topic, "qos");
  check_empty(topic);

  for (YAML::Node field : fields)
  {
    const auto label = take_optional(field, "label").as<std::string>("");
    const auto field_name = take_required(field, "name");
    const auto field_type = take_optional(field, "type");
    check_empty(field);

    YAML::Node yaml;
    yaml["topic"] = topic_name;
    yaml["field"] = field_name;
    yaml["topic-type"] = topic_type;
    yaml["field-type"] = field_type;
    yaml["qos"] = topic_qos;

    output_.emplace_back(StreamData::Create(builtin::subscription, label, yaml));
  }
}

StreamList ConstructStream::operator()(const ConfigFile & input)
{
  for (const auto & stream : input.streams)
  {
    parse_stream_yaml(stream);
  }

  return output_;
}

StreamLink ConstructStream::parse_stream_yaml(YAML::Node yaml)
{
  if (yaml.IsScalar())
  {
    // TODO(Takagi, Isamu): parse_stream_link
  }
  if (yaml.IsMap())
  {
    return parse_stream_dict(yaml);
  }

  // TODO(Takagi, Isamu): error message
  throw ConfigError("unexpected stream format");
}

StreamLink ConstructStream::parse_stream_dict(YAML::Node yaml)
{
  const auto klass = take_required(yaml, "class").as<std::string>("");
  const auto label = take_optional(yaml, "label").as<std::string>("");
  const auto input = take_optional(yaml, "input");

  StreamLink node = StreamData::Create(klass, label, yaml);
  output_.emplace_back(node);

  if (input)
  {
    node->input = parse_stream_yaml(input);
  }
  return node;
}

StreamList NodeTransformer::operator()(const StreamList & input)
{
  for (const auto & stream : input)
  {
    transform_stream_common(stream);
  }
  return output_;
}

void NodeTransformer::transform_stream_common(const std::shared_ptr<StreamData> & stream)
{
  if (stream->klass == "subscription")
  {
    return transform_stream_subscription(stream);
  }
  output_.push_back(stream);
}

void NodeTransformer::transform_stream_subscription(const std::shared_ptr<StreamData> & stream)
{
  StreamLink topic;
  {
    YAML::Node yaml;
    yaml["name"] = take_required(stream->yaml, "topic");
    yaml["qos"] = take_optional(stream->yaml, "qos");
    topic = StreamData::Create("topic", yaml);
  }

  StreamLink field;
  {
    YAML::Node yaml;
    yaml["name"] = take_required(stream->yaml, "field");
    yaml["label"] = stream->label;
    field = StreamData::Create("field", stream->label, yaml);
  }

  stream->refer = field;
  field->input = topic;
  output_.push_back(stream);
  output_.push_back(topic);
  output_.push_back(field);
}

StreamList InterfaceHandler::operator()(const StreamList & input)
{
  for (const auto & stream : input)
  {
    output_.push_back(stream);
  }
  return output_;
}

StreamList ResolveConnection::operator()(const StreamList & input)
{
  for (const auto & stream : input)
  {
    if (stream->input)
    {
      stream->input = resolve(stream->input);
    }

    if (stream->refer == nullptr)
    {
      output_.push_back(stream);
    }
  }
  return output_;
}

StreamLink ResolveConnection::resolve(const StreamLink & stream)
{
  return stream->refer ? resolve(stream->refer) : stream;
}

}  // namespace multi_data_monitor
