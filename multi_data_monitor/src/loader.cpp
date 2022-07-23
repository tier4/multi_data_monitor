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
#include <multi_data_monitor/design.hpp>
#include <multi_data_monitor/filter.hpp>
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
  DesignInstance(QWidget * parent, QLayout * layout, QWidget * widget)
  {
    this->layout = layout;
    this->widget = widget;

    if (layout)
    {
      layout->setParent(parent);
    }
    if (widget)
    {
      widget->setParent(parent);
    }

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
  std::unique_ptr<pluginlib::ClassLoader<Design>> design_loader;
  std::vector<std::unique_ptr<Topic>> topics;
  std::vector<std::unique_ptr<Stream>> streams;
  // filters
  std::vector<std::unique_ptr<Design>> designs;
};

Loader::Loader(QWidget * rviz, rclcpp::Node::SharedPtr node)
{
  constexpr char package[] = "multi_data_monitor";
  constexpr char design[] = "multi_data_monitor::Design";

  impl_ = std::make_unique<Impl>();
  rviz_ = rviz;
  node_ = node;
  impl_->design_loader = std::make_unique<pluginlib::ClassLoader<Design>>(package, design);
}

Loader::~Loader()
{
  // define the destructor here for unique_ptr.
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

void Loader::Reload(const std::string & package, const std::string & path)
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
  std::unordered_map<NodeConfig *, Stream *> streams;
  for (const auto & node : config.GetNodes("data"))
  {
    if (node->type == "topic")
    {
      streams[node] = subscriptions[node->name][node->data];
    }
    if (node->type == "filter")
    {
      auto stream = std::make_unique<FilterStream>();
      streams[node] = impl_->streams.emplace_back(std::move(stream)).get();
    }
  }

  // pluginlib::ClassLoader<Filter> filter_loader("multi_data_monitor", "multi_data_monitor::Filter");
  // std::unordered_map<NodeConfig *, Filter *> filters;
  for (const auto & node : config.GetNodes("rule"))
  {
    // const auto rule = filter_loader.createUniqueInstance("multi_data_monitor::TestFilter");
    // cout << "TestFilter: " << rule.get() << endl;
    // cout << rule->Apply(YAML::Node(123)) << endl;
  }

  // create designs
  std::unordered_map<NodeConfig *, Design *> designs;
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

  // place the root in stack memory to automatically release Qt objects
  QWidget dummy_root_widget;

  // build widget

  std::unordered_map<NodeConfig *, DesignInstance> instances;
  for (const auto & [node, design] : designs)
  {
    auto * layout = design->CreateLayout(node->yaml);
    auto * widget = design->CreateWidget(node->yaml);
    instances.emplace(node, DesignInstance(&dummy_root_widget, layout, widget));
  }

  for (const auto & [node, instance] : instances)
  {
    cout << instance.layout << endl;
    if (instance.layout)
    {
      instance.layout->dumpObjectInfo();
      rviz_->setLayout(instance.layout);
      break;
    }
  }
  dummy_root_widget.dumpObjectTree();

  /*
  mainWidget->layout()->removeWidget( your_widget );
  delete mainWidget->layout();
  */

  // start subscription
  for (auto & topic : impl_->topics)
  {
    topic->Subscribe(node_);
  }

  // TODO(Takagi, Isamu): handle exception
  /*
  catch(pluginlib::PluginlibException& ex)
  {
    printf("The plugin failed to load for some reason. Error: %s\n", ex.what());
  }
  */
}

}  // namespace multi_data_monitor
