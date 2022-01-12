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

#ifndef MONITORS__MONITOR_HPP_
#define MONITORS__MONITOR_HPP_

#include <yaml-cpp/yaml.h>
#include <string>
#include <map>
#include <memory>

#include <iostream>  // DEBUG
#include <generic_type_support/generic_type_support.hpp>  // DEBUG, move src

class QWidget;
class QLayout;

namespace monitors
{

class Monitor;
using MonitorList = std::vector<std::shared_ptr<Monitor>>;
using MonitorDict = std::map<std::string, std::unique_ptr<Monitor>>;

class Monitor
{
public:
  Monitor(const std::string & name, const YAML::Node & yaml);
  virtual ~Monitor() = default;

  // TODO: merge (Build, GetWidget, GetLayout)
  QWidget * GetWidget() {return widget_;}
  QLayout * GetLayout() {return layout_;}
  virtual void Build(MonitorDict & monitors) = 0;
  virtual void Callback([[maybe_unused]] const YAML::Node & message) {}

  std::string GetName() { return name_; }

  // TODO: Remove temporary methods
  YAML::Node GetTopic() { return yaml_["topic"]; }
  void ValidateField() { access_.Validate(support_->GetClass()); };
  void SetTypeSupport(const generic_type_support::GenericMessageSupport * support) { support_ = support; }
  const generic_type_support::GenericMessageSupport * GetTypeSupport() const { return support_; }

protected:
  std::string name_;
  YAML::Node yaml_;
  QWidget * widget_ = nullptr;
  QLayout * layout_ = nullptr;
  generic_type_support::GenericTypeAccess access_;

  // TODO: const generic_type_support::GenericMessageSupport * const support_;
  const generic_type_support::GenericMessageSupport * support_;
};

}  // namespace monitors

#endif  // MONITORS__MONITOR_HPP_
