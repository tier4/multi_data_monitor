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

#include <QLabel>
#include <QVBoxLayout>
#include <QVariant>
#include <QWidget>
#include <multi_data_monitor/design.hpp>

namespace multi_data_monitor
{

class Title : public multi_data_monitor::Design
{
private:
  QLabel * value_;
  QLabel * title_;

public:
  Instance Create(const YAML::Node params) override
  {
    value_ = new QLabel();
    title_ = new QLabel(QString::fromStdString(params["title"].as<std::string>("")));
    value_->setAlignment(Qt::AlignCenter);
    title_->setAlignment(Qt::AlignCenter);
    value_->setProperty("class", "value");
    title_->setProperty("class", "title");

    auto * layout = new QVBoxLayout();
    layout->addWidget(value_);
    layout->addWidget(title_);
    layout->setSpacing(0);
    layout->setContentsMargins(0, 0, 0, 0);

    auto * widget = new QWidget();
    widget->setToolTip(QString::fromStdString(params["notes"].as<std::string>("")));
    widget->setLayout(layout);
    return widget;
  }

  void Callback(const MonitorValues & input) override
  {
    const auto value = QString::fromStdString(input.value.as<std::string>());
    value_->setText(value);
    UpdateProperties(input, value_);
  }
};

}  // namespace multi_data_monitor

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(multi_data_monitor::Title, multi_data_monitor::Design)
