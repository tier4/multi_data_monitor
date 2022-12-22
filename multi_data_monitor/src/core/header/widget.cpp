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
#include <QStyle>
#include <QVariant>
#include <QWidget>
#include <unordered_map>
#include <unordered_set>

namespace
{

using StylesheetWidgets = std::vector<QWidget *>;
using Attributes = std::unordered_map<std::string, std::string>;

bool validate_stylesheet_widgets(const StylesheetWidgets & widgets)
{
  std::unordered_set<QWidget *> collection(widgets.begin(), widgets.end());
  return widgets.size() == collection.size();
}

void update_property(const StylesheetWidgets & widgets, const Attributes & attributes)
{
  for (const auto & widget : widgets)
  {
    for (const auto & [name, attr] : attributes)
    {
      widget->setProperty(name.c_str(), attr.c_str());
    }
    widget->style()->unpolish(widget);
    widget->style()->polish(widget);
  }
}

}  // namespace

namespace multi_data_monitor
{

QWidget * BasicWidget::system_get_widget()
{
  return root_;
}

void BasicWidget::system_setup(YAML::Node yaml, const std::vector<QWidget *> & items)
{
  setup(yaml, items);

  if (!root_)
  {
    throw PluginError("register_root_widget has never been called");
  }
  if (!validate_stylesheet_widgets(stylesheet_widgets_))
  {
    throw PluginError("register_stylesheet_widget was called for the same object");
  }
}

void BasicWidget::system_apply(const Packet & packet)
{
  apply(packet);
  update_property(stylesheet_widgets_, packet.attrs);
}

void BasicWidget::system_set_stylesheet(const QString & stylesheet)
{
  for (const auto & widget : stylesheet_widgets_)
  {
    widget->setStyleSheet(stylesheet);
  }
}

void BasicWidget::register_root_widget(QWidget * widget)
{
  if (root_)
  {
    throw PluginError("register_root_widget was called multiple times");
  }
  root_ = widget;
}

void BasicWidget::register_root_layout(QLayout * layout)
{
  QWidget * widget = new QWidget();
  widget->setLayout(layout);
  register_root_widget(widget);
}

void BasicWidget::register_stylesheet_widget(QWidget * widget, const std::string & target)
{
  if (!target.empty())
  {
    widget->setProperty("class", QString::fromStdString(target));
  }
  stylesheet_widgets_.push_back(widget);
}

}  // namespace multi_data_monitor
