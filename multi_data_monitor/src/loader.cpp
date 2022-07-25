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

struct Loader::Impl
{
  Impl();
  std::unique_ptr<pluginlib::ClassLoader<Action>> action_loader;
  std::unique_ptr<pluginlib::ClassLoader<Design>> design_loader;
  std::vector<std::unique_ptr<Topic>> topics;
  std::vector<std::unique_ptr<Stream>> streams;
  std::vector<std::unique_ptr<Design>> designs;
};

Loader::Impl::Impl()
{
  constexpr char package[] = "multi_data_monitor";
  constexpr char action[] = "multi_data_monitor::Action";
  constexpr char design[] = "multi_data_monitor::Design";
  action_loader = std::make_unique<pluginlib::ClassLoader<Action>>(package, action);
  design_loader = std::make_unique<pluginlib::ClassLoader<Design>>(package, design);
}

Loader::Loader()
{
  impl_ = std::make_unique<Impl>();
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

QWidget * Loader::Reload(const std::string & path)
{
  const auto config = ConfigFile(path);

  // release old resources
  impl_ = std::make_unique<Impl>();

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
        action->Initialize(rule->yaml);
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
  std::unordered_map<const NodeConfig *, Design::Instance> instances;
  for (const auto & [node, design] : designs)
  {
    const auto instance = design->Create(node->yaml);
    if (std::holds_alternative<QWidget *>(instance))
    {
      // TODO(Takagi, Isamu): warning when layout
      const auto stylesheet = config.GetStyleSheet(node->type);
      std::get<QWidget *>(instance)->setStyleSheet(QString::fromStdString(stylesheet));
    }
    instances.emplace(node, instance);
  }

  // connect widgets
  for (const auto [node, design] : designs)
  {
    for (const auto child : node->children)
    {
      struct AddInstance
      {
        void operator()(QLayout * layout) { design->AddLayout(layout, YAML::Node()); }
        void operator()(QWidget * widget) { design->AddWidget(widget, YAML::Node()); }
        Design * design;
      };
      const auto instance = instances.at(child);
      std::visit(AddInstance{design}, instance);
    }
  }

  QWidget * root = nullptr;
  {
    Design::Instance instance = instances.at(config.GetRoot());
    struct CreateRootWidget
    {
      QWidget * operator()(QLayout * layout)
      {
        QWidget * widget = new QWidget();
        widget->setLayout(layout);
        return widget;
      }
      QWidget * operator()(QWidget * widget) { return widget; }
    };
    const auto stylesheet = config.GetStyleSheet();
    root = std::visit(CreateRootWidget{}, instance);
    root->setStyleSheet(QString::fromStdString(stylesheet));
  }

  // place the dummy root object in stack memory to automatically release Qt objects
  {
    QWidget dummy;
    const auto connect = [&dummy](auto & instance)
    {
      if (instance->parent() == nullptr)
      {
        instance->setParent(&dummy);
      }
    };
    for (const auto [node, instance] : instances)
    {
      std::visit(connect, instance);
    }
    root->setParent(nullptr);
  }

  return root;

  // TODO(Takagi, Isamu): handle exception
  // catch(pluginlib::PluginlibException& ex)
}

void Loader::Subscribe(rclcpp::Node::SharedPtr & node)
{
  for (auto & topic : impl_->topics)
  {
    topic->Subscribe(node);
  }
}

void Loader::Unsubscribe()
{
  for (auto & topic : impl_->topics)
  {
    topic->Unsubscribe();
  }
}

}  // namespace multi_data_monitor
