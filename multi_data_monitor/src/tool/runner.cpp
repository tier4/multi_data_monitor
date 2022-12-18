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

#include "runner/config_loader.hpp"
#include "runner/stream_loader.hpp"
#include "runner/stream_runner.hpp"
#include "runner/widget_loader.hpp"
#include <rclcpp/rclcpp.hpp>
#include <iostream>

namespace multi_data_monitor
{

struct Loaders
{
  StreamLoader::SharedPtr stream;
  WidgetLoader::SharedPtr widget;
};

Loaders create_loaders(const std::string & path)
{
  const auto config = ConfigLoader().execute(path);
  const auto stream = std::make_shared<StreamLoader>(config.streams);
  const auto widget = std::make_shared<WidgetLoader>(config.widgets);
  return Loaders{stream, widget};
}

}  // namespace multi_data_monitor

int main(int argc, char ** argv)
{
  if (argc != 3)
  {
    std::cerr << "usage: command <scheme> <config-file-path>" << std::endl;
    return 1;
  }

  const auto scheme = std::string(argv[1]);
  const auto config = std::string(argv[2]);
  auto loader = multi_data_monitor::create_loaders(scheme + "://" + config);
  return 0;
  auto runner = std::make_shared<multi_data_monitor::StreamRunner>(loader.stream);

  rclcpp::init(argc, argv);
  rclcpp::executors::SingleThreadedExecutor executor;
  auto node = std::make_shared<rclcpp::Node>("runner");
  runner->start(node);
  executor.add_node(node);
  executor.spin();
  executor.remove_node(node);
  rclcpp::shutdown();
}
