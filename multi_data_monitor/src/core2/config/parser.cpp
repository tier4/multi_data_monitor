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
    throw ConfigError("required key: " + name);
  }
  yaml.remove(name);
  return node;
}

ConfigLoader::ConfigLoader(const std::string & path) : path_(path)
{
}

void ConfigLoader::parse()
{
  const auto path = std::filesystem::path(path::resolve(path_));
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

  auto streams = take_optional(yaml, "streams");
  if (!streams.IsSequence())
  {
    // TODO(Takagi, Isamu): error message
    throw ConfigError("IsSequence");
  }
  for (const auto & stream : streams)
  {
    parse_stream_yaml(stream);
  }

  for (const auto & stream : config_.streams)
  {
    stream->dump();
  }
}

StreamLink ConfigLoader::parse_stream_yaml(YAML::Node yaml)
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

StreamLink ConfigLoader::parse_stream_dict(YAML::Node yaml)
{
  const auto klass = take_required(yaml, "class").as<std::string>("");
  const auto label = take_optional(yaml, "label").as<std::string>("");
  const auto input = take_optional(yaml, "input");

  // TODO(Takagi, Isamu): if (klass == "link")

  std::optional<StreamLink> link;
  if (input)
  {
    link = parse_stream_yaml(input);
  }

  const auto data = std::make_shared<StreamData>(StreamData{klass, label, link, yaml});
  config_.streams.push_back(data);
  return StreamLink{"", data, {}};
}

}  // namespace multi_data_monitor
