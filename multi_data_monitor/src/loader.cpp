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

#include "loader.hpp"
#include "config.hpp"
#include "stream.hpp"
#include "topic.hpp"
#include <QLayout>
#include <QWidget>
#include <multi_data_monitor/action.hpp>
#include <multi_data_monitor/design.hpp>
#include <pluginlib/class_loader.hpp>
#include <unordered_map>
#include <utility>
#include <vector>

// clang-format off
#include <iostream>
#include <fmt/format.h>
using std::cout;
using std::endl;
// clang-format on

namespace multi_data_monitor
{

struct DesignInstance
{
  DesignInstance(QLayout * layout, QWidget * widget)
  {
    this->layout = layout;
    this->widget = widget;

    if (layout == nullptr && widget == nullptr)
    {
      throw LogicError("design create 1");  // TODO(Takagi, Isamu): message
    }
    if (layout != nullptr && widget != nullptr)
    {
      throw LogicError("design create 2");  // TODO(Takagi, Isamu): message
    }
  }

  QLayout * layout;
  QWidget * widget;
};

struct Loader::Impl
{
  std::unique_ptr<pluginlib::ClassLoader<Action>> action_loader;
  std::unique_ptr<pluginlib::ClassLoader<Design>> design_loader;
  std::vector<std::unique_ptr<Topic>> topics;
  std::vector<std::unique_ptr<Stream>> streams;
  // filters
  std::vector<std::unique_ptr<Design>> designs;
};

Loader::Loader(rclcpp::Node::SharedPtr node)
{
  constexpr char package[] = "multi_data_monitor";
  constexpr char action[] = "multi_data_monitor::Action";
  constexpr char design[] = "multi_data_monitor::Design";

  impl_ = std::make_unique<Impl>();
  node_ = node;
  impl_->action_loader = std::make_unique<pluginlib::ClassLoader<Action>>(package, action);
  impl_->design_loader = std::make_unique<pluginlib::ClassLoader<Design>>(package, design);
}

Loader::~Loader()
{
  // because of the forward declaration
}

template <class NodeT>
std::string NodeInfo(const NodeT & node)
{
  return fmt::format("{}, {}, {}, {}", node->mode, node->type, node->name, node->data);
}

void Dump(const std::vector<std::unique_ptr<NodeConfig>> & nodes, bool children = false)
{
  for (const auto & node : nodes)
  {
    const auto ptr1 = fmt::format("{}", static_cast<void *>(node.get()));
    const auto ptr2 = fmt::format("{}", static_cast<void *>(node->target));
    const auto info = NodeInfo(node);
    cout << fmt::format("{:50} [{} => {}] ({})", node->path, ptr1, ptr2, info) << endl;
    if (children)
    {
      if (node->stream)
      {
        cout << fmt::format(" - {}", node->stream->path) << endl;
      }
      for (const auto child : node->children)
      {
        cout << fmt::format(" - {}", child->path) << endl;
      }
    }
  }
}

QWidget * Loader::Reload(const std::string & package, const std::string & path)
{
  const auto config = ConfigFile(package, path);

  // create field stream
  std::unordered_map<std::string, std::unordered_map<std::string, Field *>> subscriptions;
  for (const auto & config : config.GetTopics())
  {
    Topic * topic = impl_->topics.emplace_back(std::make_unique<Topic>(config)).get();
    for (auto & field : topic->GetFields())
    {
      subscriptions[topic->GetName()][field->GetData()] = field.get();
    }
  }

  // create streams
  std::unordered_map<const NodeConfig *, Stream *> streams;
  for (const auto & node : config.GetNodes("data"))
  {
    if (node->type == "topic")
    {
      streams[node] = subscriptions[node->name][node->data];
    }
    if (node->type == "filter")
    {
      std::vector<std::unique_ptr<Action>> actions;
      for (const auto & rule : node->children)
      {
        auto action = std::unique_ptr<Action>(impl_->action_loader->createUnmanagedInstance(rule->type));
        actions.push_back(std::move(action));
      }
      auto stream = std::make_unique<FilterStream>(std::move(actions));
      streams[node] = impl_->streams.emplace_back(std::move(stream)).get();
    }
  }

  // create designs
  std::unordered_map<const NodeConfig *, Design *> designs;
  for (const auto & node : config.GetNodes("view"))
  {
    auto design = std::unique_ptr<Design>(impl_->design_loader->createUnmanagedInstance(node->type));
    designs[node] = impl_->designs.emplace_back(std::move(design)).get();

    if (node->stream)
    {
      auto stream = std::make_unique<WidgetStream>(designs[node]);
      streams[node] = impl_->streams.emplace_back(std::move(stream)).get();
    }
  }

  // connect streams
  for (const auto & node : config.GetNodes())
  {
    if (node->stream)
    {
      Stream * src = streams[node->stream];
      Stream * dst = streams[node];
      if (src == nullptr || dst == nullptr)
      {
        throw LogicError("connect streams");
      }
      src->Register(dst);
    }
  }

  // create widgets
  std::unordered_map<const NodeConfig *, DesignInstance> instances;
  for (const auto & [node, design] : designs)
  {
    const auto stylesheet = config.GetStyleSheet(node->type);
    auto * layout = design->CreateLayout(node->yaml);
    auto * widget = design->CreateWidget(node->yaml);
    if (widget)
    {
      widget->setStyleSheet(QString::fromStdString(stylesheet));
    }
    instances.emplace(node, DesignInstance(layout, widget));
  }

  // connect widgets
  for (const auto [node, design] : designs)
  {
    for (const auto child : node->children)
    {
      const auto instance = instances.at(child);
      if (instance.layout)
      {
        design->AddLayout(instance.layout, YAML::Node());
      }
      if (instance.widget)
      {
        design->AddWidget(instance.widget, YAML::Node());
      }
    }
  }

  DesignInstance root_instance = instances.at(config.GetRoot());
  QWidget * root = nullptr;

  if (root_instance.layout)
  {
    root = new QWidget();
    root->setLayout(root_instance.layout);
  }
  if (root_instance.widget)
  {
    root = root_instance.widget;
  }

  const auto stylesheet = config.GetStyleSheet();
  root->setStyleSheet(QString::fromStdString(stylesheet));

  // place the dummy root object in stack memory to automatically release Qt objects
  {
    QWidget dummy_root_widget;
    for (const auto [node, instance] : instances)
    {
      if (instance.layout && instance.layout->parent() == nullptr)
      {
        instance.layout->setParent(&dummy_root_widget);
      }
      if (instance.widget && instance.widget->parent() == nullptr)
      {
        instance.widget->setParent(&dummy_root_widget);
      }
    }
    root->setParent(nullptr);
  }

  // start subscription
  for (auto & topic : impl_->topics)
  {
    topic->Subscribe(node_);
  }

  return root;

  // TODO(Takagi, Isamu): handle exception
  // catch(pluginlib::PluginlibException& ex)
}

}  // namespace multi_data_monitor
