// Copyright 2021 Takagi, Isamu
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

#ifndef MONITOR_HPP_
#define MONITOR_HPP_

#include "function.hpp"
#include "config/config.hpp"
#include <string>
#include <unordered_map>

class QWidget;
class QLayout;

namespace monitors
{

class Monitor;
using MonitorDict = std::unordered_map<std::string, std::unique_ptr<Monitor>>;

class Monitor
{
public:
  Monitor(const ObjectConfig & config);
  virtual ~Monitor() = default;

  // TODO: merge (Build, GetWidget, GetLayout)
  QWidget * GetWidget() {return widget_;}
  QLayout * GetLayout() {return layout_;}
  virtual void Build(MonitorDict & monitors) = 0;
  virtual void Callback([[maybe_unused]] const YAML::Node & field) {}

protected:
  ObjectConfig config_;
  FunctionRules rules_;
  QWidget * widget_ = nullptr;
  QLayout * layout_ = nullptr;
};

}  // namespace monitors

#endif  // MONITOR_HPP_
