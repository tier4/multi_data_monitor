// Copyright 2021 Takagi, Isamu
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

#include "loader.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <string>

namespace monitors
{

std::string Join(const std::vector<std::string> & input, const std::string & delimiter)
{
  std::string result;
  for (size_t i = 0; i < input.size(); ++i)
  {
    result += (i ? delimiter : "") + input[i];
  }
  return result;
}

std::pair<std::string, size_t> ParseElement(const std::string & path, size_t & base)
{
  const size_t pos1 = path.find('(', base);
  const size_t pos2 = path.find(')', base);
  if (pos1 != base + 1 || pos2 == std::string::npos)
  {
    return {"$", 1};
  }
  const auto expr = path.substr(base + 2, pos2 - base - 2);
  if (expr.substr(0, 15) != "find-pkg-share ")
  {
    return {"$", 1};
  }
  return {ament_index_cpp::get_package_share_directory(expr.substr(15)), pos2 - base + 1};
}

std::string ParsePath(const std::string & path)
{
  std::string parsed;
  size_t base = 0;
  while (true)
  {
    const size_t pos = path.find('$', base);
    parsed += path.substr(base, pos - base);
    if (pos == std::string::npos)
    {
      break;
    }
    const auto [str, len] = ParseElement(path, base);
    parsed += str;
    base = pos + len;
  }
  return parsed;
}

YAML::Node LoadYAML(const std::string & path)
{
  try
  {
    return YAML::LoadFile(ParsePath(path));
  }
  catch(YAML::BadFile & error)
  {
    throw ConfigError(error.what());
  }
}

void ConfigLoader::Load(const std::string & path)
{
  YAML::Node config = LoadYAML(path);
  version_ = config["version"].as<std::string>();

  for (const auto & pair : config["monitors"])
  {
    monitors_.emplace(pair.first.as<std::string>(), pair.second);
  }
}

std::string ConfigLoader::GetVersion() const
{
  return version_;
}

std::vector<TopicConfig> ConfigLoader::GetTopics() const
{
  struct TopicConfigMerge
  {
    std::unordered_set<std::string> type;
    std::unordered_set<int> depth;
    std::unordered_set<std::string> reliability;
    std::unordered_set<std::string> durability;
  };

  std::unordered_map<std::string, TopicConfigMerge> topic_merge;
  for (const auto & pair : monitors_)
  {
    if (!pair.second.topic) { continue; }
    const auto & topic = pair.second.topic.value();

    topic_merge[topic.name].type.insert(topic.type);
    topic_merge[topic.name].depth.insert(topic.depth);
    topic_merge[topic.name].reliability.insert(topic.reliability);
    topic_merge[topic.name].durability.insert(topic.durability);
  }

  std::vector<TopicConfig> topics;
  for (const auto & [name, merge] : topic_merge)
  {
    if (merge.type.size() != 1) { throw ConfigError("topic type is not unique: " + name); }
    if (merge.depth.size() != 1) { throw ConfigError("topic depth is not unique: " + name); }
    if (merge.reliability.size() != 1) { throw ConfigError("topic reliability is not unique: " + name); }
    if (merge.durability.size() != 1) { throw ConfigError("topic durability is not unique: " + name); }

    TopicConfig topic;
    topic.name = name;
    topic.type = *merge.type.begin();
    topic.depth = *merge.depth.begin();
    topic.reliability = *merge.reliability.begin();
    topic.durability = *merge.durability.begin();
    topics.push_back(topic);
  }
  return topics;
}

}  // namespace monitors
