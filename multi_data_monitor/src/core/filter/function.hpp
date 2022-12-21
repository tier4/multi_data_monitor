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

#ifndef CORE__FILTER__FUNCTION_HPP_
#define CORE__FILTER__FUNCTION_HPP_

#include "common/typedef.hpp"
#include <multi_data_monitor/action.hpp>

namespace multi_data_monitor
{

struct FunctionAction : public BasicAction
{
public:
  explicit FunctionAction(Action action);
  void setup(YAML::Node yaml) override;
  void apply(Packet & packet) override;

private:
  Action action_;
};

}  // namespace multi_data_monitor

#endif  // CORE__FILTER__FUNCTION_HPP_
