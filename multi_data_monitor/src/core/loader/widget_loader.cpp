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
#include <QWidget>
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

WidgetLoader::~WidgetLoader()
{
}

WidgetLoader::Mapping WidgetLoader::create(const WidgetList & configs)
{
  QWidget dummy_root_widget;
  std::unordered_map<WidgetLink, SetupWidget> containers;
  std::unordered_map<WidgetLink, Widget> mapping;

  for (const auto & config : configs)
  {
    const auto widget = create_widget(config);
    mapping[config] = widgets_.emplace_back(widget);

    std::vector<ChildWidget> children;
    for (const auto & item : config->items)
    {
      // TODO(Takagi, Isamu): check no child widget
      children.push_back({containers.at(item.link).main, item.yaml});
    }
    const auto result = widget->setup(config->yaml, children);
    containers[config] = result;
    result.main->setParent(&dummy_root_widget);
  }

  if (!configs.empty())
  {
    QWidget * widget = containers.at(configs.back()).main;
    widget->setParent(nullptr);
    root_widget_.reset(widget);
  }

  for (const auto & [config, setup] : containers)
  {
    QWidget * widget = setup.main;
    if (widget && widget->parent() == &dummy_root_widget)
    {
      // TODO(Takagi, Isamu): exception
      std::cerr << "unused widget is detected: " << widget << std::endl;
    }
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
  root_widget_.reset();
}

QWidget * WidgetLoader::take_root_widget()
{
  return root_widget_.release();
}

}  // namespace multi_data_monitor
