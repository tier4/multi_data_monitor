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

#include "manager.hpp"
#include "matrix.hpp"
#include "simple.hpp"
#include "titled.hpp"
#include <string>

#include <iostream>

namespace multi_data_monitor
{

void CreateDefault(const DefaultConfig & config)
{
  std::cout << "create default: " << config.klass << std::endl;

  if (config.klass == "titled")
  {
    Titled::default_style_title_ = StyleDefinition(config.temp["style"]["title"]);
    Titled::default_style_value_ = StyleDefinition(config.temp["style"]["value"]);
  }
  if (config.klass == "simple")
  {
    Simple::default_style_ = StyleDefinition(config.temp["style"]);
  }
}

std::unique_ptr<Monitor> CreateMonitor(const std::string & name, const ObjectConfig & config)
{
  std::cout << "create monitor: " << name << "  " << config.klass << std::endl;

  if (config.klass == "matrix")
  {
    return std::make_unique<Matrix>(config);
  }
  if (config.klass == "titled")
  {
    return std::make_unique<Titled>(config);
  }
  if (config.klass == "simple")
  {
    return std::make_unique<Simple>(config);
  }

  throw ConfigError("unknown monitor type" + config.klass);
}

void Manager::Load(const std::string & path, rclcpp::Node::SharedPtr node)
{
  ConfigLoader config; // TODO: support clear/retry when error
  try
  {
    config.Load(path);
    RCLCPP_INFO_STREAM(node->get_logger(), "format version: " << config.GetVersion());
  }
  catch(const ConfigError& error)
  {
    RCLCPP_ERROR_STREAM(node->get_logger(), error.what());
  }

  for (const auto & config : config.GetDefaults())
  {
    CreateDefault(config);
  }

  for (const auto & topic : config.GetTopics())
  {
    subscriptions_.emplace(topic.name, topic);
  }

  for (const auto & field : config.GetFields())
  {
    subscriptions_.at(field.topic).AddField(field);
  }

  for (const auto & pair : config.GetMonitors())
  {
    monitors_.emplace(pair.first, CreateMonitor(pair.first, pair.second.object));
    if (pair.second.topic)
    {
      const auto & topic = pair.second.topic.value();
      const auto & field = pair.second.field.value();
      subscriptions_.at(topic.name).GetField(field.name).monitors.push_back(monitors_.at(pair.first).get());
    }
  }
  root_ = monitors_.at(config.GetRoot()).get();
}

void Manager::Start(const rclcpp::Node::SharedPtr & node)
{
  for (auto & pair : subscriptions_)
  {
    pair.second.Start(node);
  }
}

void Manager::Build(QWidget * panel)
{
  root_->Build(monitors_);

  const auto widget = root_->GetWidget();
  std::cout << "widget: " << widget << std::endl;
  if (widget)
  {
    // TODO: use dummy layout
    // panel->Widget(widget)
  }

  const auto layout = root_->GetLayout();
  std::cout << "layout: " << layout << std::endl;
  if (layout)
  {
    panel->setLayout(layout);
  }
}

}  // namespace multi_data_monitor
