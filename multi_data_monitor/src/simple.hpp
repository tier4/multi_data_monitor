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

#ifndef MONITORS__WIDGET__SIMPLE_HPP_
#define MONITORS__WIDGET__SIMPLE_HPP_

#include "monitor.hpp"
#include "function.hpp"

// TODO: style class
#include <map>
#include <string>
#include <vector>

class QLabel;

namespace monitors
{

class Simple : public Monitor
{
public:
  using Monitor::Monitor;
  void Build(MonitorDict & monitors) override;
  void Callback(const YAML::Node & message) override;

private:
  QLabel * label;
  FunctionRules rules_;    // TODO: move base class
  StyleDefinition style_;  // TODO: move base class
  std::string prev_;
  std::string title_;
public:
  static StyleDefinition default_style_;  // TODO: private and setter
};

}  // namespace monitors

#endif  // MONITORS__WIDGET__SIMPLE_HPP_
