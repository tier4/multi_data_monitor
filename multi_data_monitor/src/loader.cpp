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
  std::vector<std::unique_ptr<Topic>> topics;
  std::vector<std::unique_ptr<Stream>> streams;
};

Loader::Loader(QWidget * rviz, rclcpp::Node::SharedPtr node)
{
  impl_ = std::make_unique<Impl>();
  rviz_ = rviz;
  node_ = node;
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

  // extract view and data
  // extract rule

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
  for (const auto & node : config.GetNodes())
  {
    if (node->mode == "data")
    {
      if (node->type == "topic")
      {
        streams[node.get()] = subscriptions[node->name][node->data];
      }
      if (node->type == "filter")
      {
        Stream * stream = impl_->streams.emplace_back(std::make_unique<FilterStream>()).get();
        streams[node.get()] = stream;
      }
    }

    if (node->mode == "view")
    {
      // TODO(Takagi, Isamu): create widget
      if (node->stream)
      {
        Stream * stream = impl_->streams.emplace_back(std::make_unique<WidgetStream>()).get();
        streams[node.get()] = stream;
      }
    }
    // rule
  }

  // connect streams
  for (const auto & node : config.GetNodes())
  {
    if (node->stream)
    {
      Stream * src = streams[node->stream];
      Stream * dst = streams[node.get()];
      if (src == nullptr || dst == nullptr)
      {
        throw LogicError("connect streams");
      }
      src->Register(dst);
    }
  }

  cout << "===============================" << endl;

  for (auto & topic : impl_->topics)
  {
    topic->Subscribe(node_);
  }
}

}  // namespace multi_data_monitor
