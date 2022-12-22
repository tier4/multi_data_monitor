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

#include "common/exceptions.hpp"
#include <multi_data_monitor/widget.hpp>
#include <QWidget>

namespace multi_data_monitor
{

QWidget * BasicWidget::system_get_widget()
{
  return root_;
}

void BasicWidget::system_setup(YAML::Node yaml, const std::vector<QWidget *> & items)
{
  setup(yaml, items);
}

void BasicWidget::system_apply(const Packet & packet)
{
  apply(packet);
}

void BasicWidget::register_root_widget(QWidget * widget)
{
  if (root_)
  {
    throw PluginError("register_root_widget is called multiple times");
  }
  root_ = widget;
}

void BasicWidget::register_root_layout(QLayout * layout)
{
  if (root_)
  {
    throw PluginError("register_root_layout is called multiple times");
  }
  root_ = new QWidget();
  root_->setLayout(layout);
}

void BasicWidget::register_stylesheet_widget(QWidget * widget)
{
  (void)widget;
}

}  // namespace multi_data_monitor
