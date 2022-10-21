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
private:
  QGridLayout * layout_;
  int cols_;
  int rows_;
  int x_;
  int y_;
  void Increment()
  {
    x_ += 1;
    y_ += x_ / cols_;
    x_ %= cols_;
    y_ %= rows_;
  }

public:
  Instance Create(const YAML::Node yaml) override
  {
    cols_ = yaml["cols"].as<int>(1000);
    rows_ = yaml["rows"].as<int>(1000);
    x_ = 0;
    y_ = 0;
    layout_ = new QGridLayout();
    return layout_;
  }
  void AddWidget(QWidget * widget, const YAML::Node) override
  {
    layout_->addWidget(widget, y_, x_);
    Increment();
  }
  void AddLayout(QLayout * layout, const YAML::Node) override
  {
    layout_->addLayout(layout, y_, x_);
    Increment();
  }
  void Callback(const MonitorValues &) override {}  // TODO(Takagi, Isamu): remove
};

}  // namespace multi_data_monitor

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(multi_data_monitor::Matrix, multi_data_monitor::Design)
