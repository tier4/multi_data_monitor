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

#ifndef CORE__LOADER__CONFIG_LOADER_HPP_
#define CORE__LOADER__CONFIG_LOADER_HPP_

#include "config/types.hpp"
#include <memory>
#include <string>
#include <vector>

namespace multi_data_monitor
{

class ConfigLoader final
{
public:
  using HookFunction = std::function<void(int, const std::string &, const ConfigData &)>;
  static ConfigData Execute(const std::string & path);
  static ConfigData Execute(const std::string & path, HookFunction function);

private:
  ConfigLoader();
  ConfigData execute(const std::string & path) const;
  void hook(HookFunction function);

  std::vector<std::shared_ptr<ConfigParserInterface>> parsers_;
  HookFunction function_;
};

}  // namespace multi_data_monitor

#endif  // CORE__LOADER__CONFIG_LOADER_HPP_
