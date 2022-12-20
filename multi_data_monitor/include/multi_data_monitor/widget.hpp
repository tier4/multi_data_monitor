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

#ifndef MULTI_DATA_MONITOR__WIDGET_HPP_
#define MULTI_DATA_MONITOR__WIDGET_HPP_

#include <multi_data_monitor/packet.hpp>
#include <memory>
#include <vector>

class QWidget;

namespace multi_data_monitor
{

struct SetupWidget
{
  QWidget * main;
};

struct ChildWidget
{
  QWidget * widget;
  YAML::Node yaml;
};

class BasicWidget
{
public:
  // DEBUG
  static inline int created = 0;
  static inline int removed = 0;
  BasicWidget() { ++created; }
  virtual ~BasicWidget() { ++removed; }

  // virtual ~BasicWidget() = default;
  virtual SetupWidget setup(YAML::Node yaml, const std::vector<ChildWidget> & children) = 0;
  virtual void message([[maybe_unused]] const Packet & packet) {}
};

using Widget = std::shared_ptr<BasicWidget>;

}  // namespace multi_data_monitor

#endif  // MULTI_DATA_MONITOR__WIDGET_HPP_
