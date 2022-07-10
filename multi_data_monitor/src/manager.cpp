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
#include "util/parser.hpp"
#include <string>
/*
#include "matrix.hpp"
#include "simple.hpp"
#include "titled.hpp"
#include <QWidget>
*/

#include <iostream>


namespace monitors
{

void Manager::Load(const std::string & path, rclcpp::Node::SharedPtr node)
{
  YAML::Node config;
  try
  {
    config = YAML::LoadFile(ParsePath(path));
  }
  catch(YAML::BadFile & error)
  {
    RCLCPP_ERROR_STREAM(node->get_logger(), error.what());
  }

  RCLCPP_INFO_STREAM(node->get_logger(), "format version: " << config["version"].as<std::string>());



}


/*
void Manager::CreateMonitors()
{
  const auto CreateMonitor = [](const std::string & name, const YAML::Node & yaml) -> std::unique_ptr<Monitor>
  {
    const auto type = yaml["class"].as<std::string>();
    std::cout << "create node: " << name << "  " << type << std::endl;

    if (type == "matrix")
    {
      return std::make_unique<Matrix>(name, yaml);
    }
    if (type == "titled")
    {
      return std::make_unique<Titled>(name, yaml);
    }
    if (type == "simple")
    {
      return std::make_unique<Simple>(name, yaml);
    }

    std::cout << "  unknown type: " << type << std::endl;
    return nullptr;
  };

  for (const auto & monitor : yaml_["defaults"])
  {
    const auto type = monitor["class"].as<std::string>();
    std::cout << "set default: " << type << std::endl;

    if (type == "titled")
    {
      Titled::default_style_title_ = StyleDefinition(monitor["style"]["title"]);
      Titled::default_style_value_ = StyleDefinition(monitor["style"]["value"]);
    }
    if (type == "simple")
    {
      Simple::default_style_ = StyleDefinition(monitor["style"]);
    }
  }

  for (const auto & monitor : yaml_["monitors"])
  {
    const auto name = monitor.first.as<std::string>();
    monitors_[name] = CreateMonitor(name, monitor.second);
  }
}

void Manager::CreateSubscription(const rclcpp::Node::SharedPtr & node)
{
  std::map<std::string, MonitorList> topics;
  for (auto & [_, monitor] : monitors_)
  {
    const auto topic = monitor->GetTopic();
    if (!topic) { continue; }
    const auto type = topic["type"].as<std::string>();
    const auto name = topic["name"].as<std::string>();

    std::cout << monitor->GetName() << " " <<  type << " " << name << std::endl;

    if (!supports_.count(type))
    {
      std::cout << "create support: " << type << std::endl;
      supports_[type] = std::make_unique<const GenericMessageSupport>(type);
    }
    const GenericMessageSupport * support = supports_[type].get();

    if (!subscriptions_.count(name))
    {
      std::cout << "create subscriptions: " << name << std::endl;
      subscriptions_[name] = std::make_unique<TopicSubscription>(name, support);
    }
    TopicSubscription * subscription = subscriptions_[name].get();

    // TODO: CreateMonitorsと処理をまとめる。Monitorが余分なYAMLを持たなくて済む。
    monitor->SetTypeSupport(support);
    monitor->ValidateField();
    subscription->Add(monitor.get(), topic["qos"]);

    std::cout << std::endl;
  }

  for (auto & pair : subscriptions_)
  {
    pair.second->Start(node);
  }
}

void Manager::Build(QWidget * panel)
{
  const auto name = yaml_["root"].as<std::string>();
  const auto root = monitors_.at(name).get();
  root->Build(monitors_);

  const auto widget = root->GetWidget();
  std::cout << "widget: " << widget << std::endl;
  if (widget)
  {
    // TODO: use dummy layout
    // panel->Widget(widget)
  }

  const auto layout = root->GetLayout();
  std::cout << "layout: " << layout << std::endl;
  if (layout)
  {
    panel->setLayout(layout);
  }
}
*/

}  // namespace monitors
