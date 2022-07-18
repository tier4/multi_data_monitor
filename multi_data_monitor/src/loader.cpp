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
  std::vector<Stream> streams;
};

Loader::Loader(QWidget * rviz, rclcpp::Node::ConstSharedPtr node)
{
  impl_ = std::make_unique<Impl>();
  rviz_ = rviz;
  node_ = node;
}

Loader::~Loader()
{
  // define the destructor here for unique_ptr.
}

void Loader::Reload(const std::string & package, const std::string & path)
{
  const auto config = ConfigFile(package, path);

  cout << "========================================================" << endl;
  for (const auto & node : config.nodes_)
  {
    cout << fmt::format("Node     {:50} ({}, {}, {})", node->path, node->type, node->name, node->data) << endl;
    if (node->input)
    {
      cout << fmt::format("  Input  {}", node->input->path) << endl;
    }
    for (const auto child : node->children)
    {
      cout << fmt::format("  Child  {}", child->path) << endl;
    }
  }
  cout << "========================================================" << endl;

  for (auto & topic : config.topics_)
  {
    cout << "===== " << endl;
    cout << topic.name << endl;
    cout << topic.type << endl;
    cout << topic.depth << endl;
    cout << topic.reliability << endl;
    cout << topic.durability << endl;
    cout << "[ ";
    for (const auto & field : topic.fields)
    {
      cout << field.data << " ";
    }
    cout << "]" << endl;
  }
}

}  // namespace multi_data_monitor
