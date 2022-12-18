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

#include "widget_loader.hpp"
#include "common/exceptions.hpp"
#include <string>
#include <unordered_map>
#include <vector>

// DEBUG
#include <iostream>

namespace multi_data_monitor
{

WidgetLoader::WidgetLoader() : plugins_(plugin::name::package, plugin::name::widget)
{
}

WidgetLoader::Mapping WidgetLoader::create(const WidgetList & configs)
{
  std::unordered_map<WidgetLink, Widget> mapping;
  for (const auto & config : configs)
  {
    const auto widget = widgets_.emplace_back(create_widget(config));
    widget->setup(config->yaml, std::vector<YAML::Node>());
  }
  return mapping;
}

Widget WidgetLoader::create_widget(const WidgetLink config)
{
  // Search in default plugins if namespace is omitted.
  std::string klass = config->klass;
  if (klass.find("::") == std::string::npos)
  {
    klass = plugin::name::package + std::string("::") + klass;
  }
  if (!plugins_.isClassAvailable(klass))
  {
    throw ConfigError("unknown widget type: " + config->klass);
  }
  return plugins_.createSharedInstance(klass);
}

void WidgetLoader::release()
{
  // TODO(Takagi, Isamu): check use count
  // Release shared_ptr to unload plugins.
  widgets_.clear();
}

}  // namespace multi_data_monitor
