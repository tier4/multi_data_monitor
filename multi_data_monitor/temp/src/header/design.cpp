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

#include <QStyle>
#include <QVariant>
#include <QWidget>
#include <multi_data_monitor/design.hpp>

namespace multi_data_monitor
{

void Design::AddWidget(QWidget *, const YAML::Node)
{
  throw std::runtime_error("Design::AddWidget");  // TODO(Takagi, Isamu): change exception
}
void Design::AddLayout(QLayout *, const YAML::Node)
{
  throw std::runtime_error("Design::AddWidget");  // TODO(Takagi, Isamu): change exception
}

void Design::UpdateProperties(const MonitorValues & input, QWidget * widget)
{
  // TODO(Takagi, Isamu): execute only when changed
  for (const auto & [name, attr] : input.attrs)
  {
    widget->setProperty(name.c_str(), QVariant(attr.c_str()));
  }
  widget->style()->unpolish(widget);
  widget->style()->polish(widget);
}

}  // namespace multi_data_monitor
