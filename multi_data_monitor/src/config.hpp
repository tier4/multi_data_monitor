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

#ifndef CONFIG_HPP_
#define CONFIG_HPP_

#include <string>
#include <unordered_map>

namespace multi_data_monitor
{

struct ConfigFile
{
  ConfigFile(const std::string & package, const std::string & path);
};

struct FieldConfig
{
  using Key = std::string;
  using Map = std::unordered_map<Key, FieldConfig>;
  std::string path;
};

struct TopicConfig
{
  using Key = std::pair<std::string, std::string>;
  using Map = std::unordered_map<Key, TopicConfig>;
  std::string name;
  std::string type;
  FieldConfig::Map fields;
};

struct InputConfig
{
  // Input Object
  // Field Object
};

}  // namespace multi_data_monitor

#endif  // CONFIG_HPP_