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

#include "action_loader.hpp"
#include "common/exceptions.hpp"
#include "config/types.hpp"
#include "filter/function.hpp"
#include <memory>
#include <string>
#include <unordered_map>

// DEBUG
#include <iostream>

namespace multi_data_monitor
{

// TODO(Takagi, Isamu): merge widget loader
std::string get_full_plugin_name(const std::string & klass)
{
  if (klass.find("::") != std::string::npos)
  {
    return klass;
  }
  return plugin::name::package + std::string("::") + klass;
}

ActionLoader::ActionLoader() : plugins_(plugin::name::package, plugin::name::action)
{
}

ActionMaps ActionLoader::create(const ActionList & configs)
{
  ActionMaps mapping;
  for (const auto & config : configs)
  {
    const auto action = create_action(config, mapping);
    mapping[config] = actions_.emplace_back(action);
    action->setup(config->yaml);
  }
  return mapping;
}

Action ActionLoader::create_action(const ActionLink & config, const ActionMaps & mapping)
{
  if (config->klass == builtin::function)
  {
    for (const auto & item : config->items)
    {
      std::cout << config->klass << " " << item << " " << mapping.at(item) << std::endl;
    }
    const auto action = std::make_shared<FunctionAction>(nullptr);
    return action;
  }

  // Search in default plugins if namespace is omitted.
  std::string klass = get_full_plugin_name(config->klass);
  if (!plugins_.isClassAvailable(klass))
  {
    throw ConfigError("unknown action type: " + config->klass);
  }
  return plugins_.createSharedInstance(klass);
}

void ActionLoader::release()
{
  // TODO(Takagi, Isamu): check use count
  // Release shared_ptr to unload plugins.
  actions_.clear();
}

}  // namespace multi_data_monitor
