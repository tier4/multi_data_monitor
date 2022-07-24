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

#include <QGridLayout>
#include <multi_data_monitor/design.hpp>

namespace multi_data_monitor
{

class Matrix : public multi_data_monitor::Design
{
public:
  QLayout * CreateLayout(const YAML::Node) override
  {
    layout_ = new QGridLayout();
    return layout_;
  }

  void AddWidget(QWidget * widget, const YAML::Node) override { layout_->addWidget(widget); }

  void AddLayout(QLayout * layout, const YAML::Node) override { layout_->addLayout(layout, 1, 1); }

  void Callback(const MonitorValues &) override {}

private:
  QGridLayout * layout_;
};

/*
void Matrix::Build(MonitorDict & monitors)
{
  layout_ = grid = new QGridLayout();

  int cols = config_.custom["cols"].as<int>();
  int rows = config_.custom["rows"].as<int>();
  int x = 0;
  int y = 0;

  for (const auto & node : config_.custom["children"])
  {
    const auto & child = monitors[node.as<std::string>()];
    if (child)
    {
      child->Build(monitors);
      const auto widget = child->GetWidget();
      const auto layout = child->GetLayout();
      if (widget) { grid->addWidget(widget, y, x); }
      if (layout) { grid->addLayout(layout, y, x); }
    }

    x += 1;
    y += x / cols;
    x %= cols;
    y %= rows;
  }
}
*/

}  // namespace multi_data_monitor

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(multi_data_monitor::Matrix, multi_data_monitor::Design)