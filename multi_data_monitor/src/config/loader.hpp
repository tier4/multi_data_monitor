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

#ifndef CONFIG__LOADER_HPP_
#define CONFIG__LOADER_HPP_

#include "config.hpp"
#include <string>
#include <unordered_map>
#include <unordered_set>

namespace monitors
{

class ConfigLoader
{
public:
  void Load(const std::string & path);
  std::string GetVersion() const;

  std::vector<TopicConfig> GetTopics() const;

private:
  std::string version_;
  std::unordered_map<std::string, MonitorConfig> monitors_;
};

}  // namespace monitors

#endif  // CONFIG__LOADER_HPP_
