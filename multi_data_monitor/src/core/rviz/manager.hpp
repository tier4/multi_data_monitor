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

#ifndef CORE__RVIZ__MANAGER_HPP_
#define CORE__RVIZ__MANAGER_HPP_

#include "common/rclcpp.hpp"
#include "runner/widget_runner.hpp"
#include <memory>
#include <string>

class QWidget;

namespace multi_data_monitor
{

class RvizManager final
{
public:
  ~RvizManager();
  QWidget * build(const std::string & path, ros::Node node);

private:
  WidgetRunner runner_;
};

}  // namespace multi_data_monitor

#endif  // CORE__RVIZ__MANAGER_HPP_
