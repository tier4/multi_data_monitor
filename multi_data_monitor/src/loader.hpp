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

#ifndef LOADER_HPP_
#define LOADER_HPP_

#include "config.hpp"
#include <rclcpp/rclcpp.hpp>
#include <memory>
#include <string>

class QWidget;

namespace multi_data_monitor
{

class Loader
{
public:
  explicit Loader(rclcpp::Node::SharedPtr node);
  ~Loader();
  QWidget * Reload(const std::string & package, const std::string & path);

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
  QWidget * rviz_;
  rclcpp::Node::SharedPtr node_;
};

}  // namespace multi_data_monitor

#endif  // LOADER_HPP_
