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

WidgetLoader::WidgetLoader(const WidgetList & configs) : plugins_(plugin::name::package, plugin::name::widget)
{
  std::unordered_map<WidgetLink, Widget> mapping;
  for (const auto & config : configs)
  {
    const auto widget = create_widget(config);
    /*
    widget->setup(config->yaml, std::vector<YAML::Node>());
    mapping[config] = widgets_.emplace_back(widget);
    */
  }

  /*
  for (const auto & [config, widget] : mapping)
  {
    if (config->input)
    {
      mapping[config->input]->connect(widget);
    }
  }
  */
}

Widget WidgetLoader::create_widget(const WidgetLink config)
{
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

}  // namespace multi_data_monitor
