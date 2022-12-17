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

#ifndef CORE__PARSER__CHECK_SYSTEM_CLASS_HPP_
#define CORE__PARSER__CHECK_SYSTEM_CLASS_HPP_

#include "config/types.hpp"
#include <string>

namespace multi_data_monitor
{

class CheckSystemClass : public ConfigParserInterface
{
public:
  std::string name() { return "check-system-class"; }
  ConfigData execute(const ConfigData & input) override;
};

}  // namespace multi_data_monitor

#endif  // CORE__PARSER__CHECK_SYSTEM_CLASS_HPP_
