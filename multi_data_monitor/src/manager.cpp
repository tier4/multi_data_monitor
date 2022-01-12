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
#include <QWidget>
#include <string>

#include <iostream>
#include <ament_index_cpp/get_package_share_directory.hpp>  // TODO: move parser

namespace monitors
{

std::pair<std::string, size_t> parse(const std::string & path, size_t & base)
{
  const size_t pos1 = path.find('(', base);
  const size_t pos2 = path.find(')', base);
  if (pos1 != base + 1 || pos2 == std::string::npos)
  {
    return {"$", 1};
  }
  const auto expr = path.substr(base + 2, pos2 - base - 2);
  if (expr.substr(0, 15) != "find-pkg-share ")
  {
    return {"$", 1};
  }
  return {ament_index_cpp::get_package_share_directory(expr.substr(15)), pos2 - base + 1};
}

void Manager::Load(const std::string & path)
{
  std::string parsed;
  size_t base = 0;
  while (true)
  {
    const size_t pos = path.find('$', base);
    parsed += path.substr(base, pos - base);
    if (pos == std::string::npos)
    {
      break;
    }
    const auto [str, len] = parse(path, base);
    parsed += str;
    base = pos + len;
  }

  yaml_ = YAML::LoadFile(parsed);
  std::cout << "format version: " << yaml_["version"].as<std::string>() << std::endl;
}

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

}  // namespace monitors
