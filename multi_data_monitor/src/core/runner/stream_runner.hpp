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

#ifndef CORE__RUNNER__STREAM_RUNNER_HPP_
#define CORE__RUNNER__STREAM_RUNNER_HPP_

#include "common/rclcpp.hpp"
#include "runner/stream_loader.hpp"
#include <memory>

namespace multi_data_monitor
{

class StreamRunner final
{
public:
  using SharedPtr = std::shared_ptr<StreamRunner>;
  explicit StreamRunner(const StreamLoader::SharedPtr loader);
  void start(ros::Node node);
  void shutdown();

private:
  void on_timer(ros::Node node);
  ros::Node node_;
  ros::Timer timer_;
  StreamLoader::SharedPtr loader_;
};

}  // namespace multi_data_monitor

#endif  // CORE__RUNNER__STREAM_RUNNER_HPP_
