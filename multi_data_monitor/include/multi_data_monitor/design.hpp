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

#ifndef MULTI_DATA_MONITOR__DESIGN_HPP_
#define MULTI_DATA_MONITOR__DESIGN_HPP_

#include <QLayout>
#include <QWidget>
#include <yaml-cpp/yaml.h>

namespace multi_data_monitor
{

// default style

class Design
{
public:
  virtual ~Design() = default;
  virtual QWidget * CreateWidget(const YAML::Node) { return nullptr; }
  virtual QLayout * CreateLayout(const YAML::Node) { return nullptr; }
  virtual void AddWidget(QWidget *, const YAML::Node)
  {
    // throw ImplementError
  }
  virtual void AddLayout(QLayout *, const YAML::Node)
  {
    // throw ImplementError
  }
};

}  // namespace multi_data_monitor

#endif  // MULTI_DATA_MONITOR__DESIGN_HPP_
