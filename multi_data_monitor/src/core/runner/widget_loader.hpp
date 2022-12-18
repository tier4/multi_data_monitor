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

#ifndef CORE__RUNNER__WIDGET_LOADER_HPP_
#define CORE__RUNNER__WIDGET_LOADER_HPP_

#include "config/types.hpp"
#include <multi_data_monitor/widget.hpp>
#include <pluginlib/class_loader.hpp>
#include <memory>
#include <vector>

namespace multi_data_monitor
{

class WidgetLoader final
{
public:
  using SharedPtr = std::shared_ptr<WidgetLoader>;
  explicit WidgetLoader(const WidgetList & configs);

private:
  Widget create_widget(const WidgetLink config);

  // The plugin loader must be written first for release order.
  pluginlib::ClassLoader<BasicWidget> plugins_;
  std::vector<Widget> widgets_;
};

}  // namespace multi_data_monitor

#endif  // CORE__RUNNER__WIDGET_LOADER_HPP_
