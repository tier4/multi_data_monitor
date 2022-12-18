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

#include "rclcpp_runner.hpp"
#include <rclcpp/rclcpp.hpp>
#include <utility>

namespace multi_data_monitor
{

RclcppRunner::RclcppRunner(const StreamLoader::SharedPtr loader)
{
  loader_ = std::move(loader);
}

void RclcppRunner::start(ros::Node node)
{
  const auto rate = rclcpp::Rate(1.0);
  timer_ = rclcpp::create_timer(node, node->get_clock(), rate.period(), [this, node]() { on_timer(node); });
}

void RclcppRunner::shutdown()
{
  timer_->cancel();
  for (auto & topic : loader_->topics()) topic->shutdown();
}

void RclcppRunner::on_timer(ros::Node node)
{
  for (auto & topic : loader_->topics()) topic->update(node);
}

}  // namespace multi_data_monitor
