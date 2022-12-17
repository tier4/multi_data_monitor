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

#include "stream_runner.hpp"
#include <rclcpp/rclcpp.hpp>

namespace multi_data_monitor
{

StreamRunner::StreamRunner(const StreamList & configs) : loader_(configs)
{
}

void StreamRunner::start(ros::Node node)
{
  node_ = node;

  const auto rate = rclcpp::Rate(1.0);
  timer_ = rclcpp::create_timer(node_, node_->get_clock(), rate.period(), [this]() { on_timer(); });
}

void StreamRunner::on_timer()
{
  for (auto & topic : loader_.topics())
  {
    topic->update(node_);
  }
}

}  // namespace multi_data_monitor