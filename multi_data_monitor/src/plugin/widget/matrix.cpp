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

#include <multi_data_monitor/widget.hpp>
#include <QGridLayout>
#include <QWidget>

// DEBUG
#include <QLabel>
#include <iostream>

namespace multi_data_monitor
{

class Matrix : public BasicWidget
{
public:
  SetupWidget setup(YAML::Node yaml, const std::vector<ChildWidget> & children) override;
};

SetupWidget Matrix::setup(YAML::Node yaml, const std::vector<ChildWidget> & children)
{
  int cols = 1;
  int rows = 0;
  int dx = 1;
  int dy = 0;

  if (yaml["cols"])
  {
    cols = yaml["cols"].as<int>();
    rows = 0;
    dx = 1;
    dy = 0;
  }
  if (yaml["rows"])
  {
    cols = 0;
    rows = yaml["rows"].as<int>();
    dx = 0;
    dy = 1;
  }

  const auto widget = new QWidget();
  const auto layout = new QGridLayout();
  layout->setContentsMargins(0, 0, 0, 0);
  widget->setLayout(layout);

  int x = 0;
  int y = 0;
  for (const auto & child : children)
  {
    layout->addWidget(child.widget, y, x);
    x += dx;
    y += dy;
    if (cols) y += (x / cols);
    if (rows) x += (y / rows);
    if (cols) x %= cols;
    if (rows) y %= rows;
  }

  SetupWidget setup;
  setup.main = widget;
  return setup;
}

}  // namespace multi_data_monitor

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(multi_data_monitor::Matrix, multi_data_monitor::BasicWidget)
