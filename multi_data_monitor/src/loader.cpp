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

void Dump(const std::vector<std::unique_ptr<ConfigNode>> & nodes, bool children = false)
{
  for (const auto & node : nodes)
  {
    cout << fmt::format("Node     {:50} ({}, {}, {})", node->path, node->type, node->name, node->data) << endl;
    if (children)
    {
      if (node->input)
      {
        cout << fmt::format("  Input  {}", node->input->path) << endl;
      }
      for (const auto child : node->children)
      {
        cout << fmt::format("  Child  {}", child->path) << endl;
      }
    }
  }
}

void Loader::Reload(const std::string & package, const std::string & path)
{
  const auto config = ConfigFile(package, path);

  for (auto & config : config.GetTopics())
  {
    impl_->topics.emplace_back(std::make_unique<Topic>(config))->Subscribe(node_);
  }

  Dump(config.GetNodes());
}

}  // namespace multi_data_monitor
