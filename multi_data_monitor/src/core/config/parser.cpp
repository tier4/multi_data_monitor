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
    throw ConfigError("streams config is node sequence");
  }
  for (const auto & stream : streams)
  {
    output_.streams.push_back(stream);
  }

  return output_;
}

ConfigData NodeConstructor::operator()(const ConfigFile & input)
{
  for (const auto & stream : input.streams)
  {
    parse_stream_yaml(stream);
  }

  return output_;
}

StreamLink NodeConstructor::parse_stream_yaml(YAML::Node yaml)
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

StreamLink NodeConstructor::parse_stream_dict(YAML::Node yaml)
{
  const auto klass = take_required(yaml, "class").as<std::string>("");
  const auto label = take_optional(yaml, "label").as<std::string>("");
  const auto input = take_optional(yaml, "input");

  StreamLink node = StreamData::Create(klass, label, yaml);
  output_.streams.emplace_back(node);

  if (input)
  {
    node->input = parse_stream_yaml(input);
  }
  return node;
}

ConfigData NodeTransformer::operator()(const ConfigData & input)
{
  for (const auto & stream : input.streams)
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
  output_.streams.push_back(stream);
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

  field->input = topic;
  output_.streams.push_back(topic);
  output_.streams.push_back(field);
}

ConfigData InterfaceHandler::operator()(const ConfigData & input)
{
  for (const auto & stream : input.streams)
  {
    output_.streams.push_back(stream);
  }
  return output_;
}

}  // namespace multi_data_monitor
