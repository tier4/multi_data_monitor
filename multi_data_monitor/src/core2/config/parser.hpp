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

#ifndef CORE2__CONFIG__PARSER_HPP_
#define CORE2__CONFIG__PARSER_HPP_

#include "types.hpp"
#include <yaml-cpp/yaml.h>
#include <string>

namespace multi_data_monitor
{

class ConfigLoader
{
public:
  explicit ConfigLoader(const std::string & path);
  void parse();
  StreamLink parse_stream_link(YAML::Node yaml);
  StreamLink parse_stream_yaml(YAML::Node yaml);
  StreamLink parse_stream_dict(YAML::Node yaml);

private:
  const std::string path_;
  ConfigData config_;
};

class StyleSheetConfig
{
};

class StyleSheetStore
{
};

}  // namespace multi_data_monitor

#endif  // CORE2__CONFIG__PARSER_HPP_
