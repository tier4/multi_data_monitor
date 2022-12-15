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
#include "common/exceptions.hpp"
#include "common/yaml.hpp"
#include "types.hpp"
#include "utils.hpp"
#include <filesystem>
#include <iostream>
#include <memory>

// temp
#include <vector>

namespace multi_data_monitor
{

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
  const auto version = yaml::take_optional(yaml, "version").as<std::string>("undefined");
  if (version != "2.0")
  {
    throw ConfigError("not supported version '" + version + "'");
  }

  const auto streams = yaml::take_optional(yaml, "streams");
  if (!streams.IsSequence())
  {
    throw ConfigError("streams config is not sequence");
  }
  for (const auto & stream : streams)
  {
    output_.streams.push_back(stream);
  }

  const auto subscriptions = yaml::take_optional(yaml, "subscriptions");
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

StreamList ConstructSubscription::operator()(const ConfigFile & input)
{
  for (const auto & node : input.subscriptions)
  {
    parse_subscription(node);
  }
  return output_;
}

void ConstructSubscription::parse_subscription(YAML::Node topic)
{
  if (!topic.IsMap())
  {
    // TODO(Takagi, Isamu): error message
    throw ConfigError("is not map");
  }

  const auto fields = yaml::take_optional(topic, "fields");
  if (!fields.IsSequence())
  {
    // TODO(Takagi, Isamu): error message
    throw ConfigError("is not sequence");
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

    output_.push_back(StreamData::Create(builtin::subscription, label, yaml));
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
  const auto klass = yaml::take_required(yaml, "class").as<std::string>("");
  const auto label = yaml::take_optional(yaml, "label").as<std::string>("");
  const auto input = yaml::take_optional(yaml, "input");

  StreamLink node = StreamData::Create(klass, label, yaml);
  output_.push_back(node);

  if (input)
  {
    node->input = parse_stream_yaml(input);
  }
  return node;
}

StreamList CheckSpecialClass::operator()(const StreamList & input)
{
  for (const auto & stream : input)
  {
    const auto & klass = stream->klass;
    if (!klass.empty() && klass[0] == '@')
    {
      throw ConfigError("class name is reserved: " + klass);
    }
  }
  return input;
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
