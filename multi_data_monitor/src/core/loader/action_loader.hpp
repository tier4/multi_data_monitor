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

#ifndef CORE__LOADER__ACTION_LOADER_HPP_
#define CORE__LOADER__ACTION_LOADER_HPP_

#include "common/typedef.hpp"
#include <multi_data_monitor/action.hpp>
#include <pluginlib/class_loader.hpp>
#include <vector>

namespace multi_data_monitor
{

class ActionLoader final
{
public:
  ActionLoader();
  ActionMaps create(const ActionList & configs);
  void release();

private:
  Action create_action(const ActionLink & config, const ActionMaps & mapping);

  // The plugin loader must be first for release order.
  pluginlib::ClassLoader<BasicAction> plugins_;
  std::vector<Action> actions_;
};

}  // namespace multi_data_monitor

#endif  // CORE__LOADER__ACTION_LOADER_HPP_
