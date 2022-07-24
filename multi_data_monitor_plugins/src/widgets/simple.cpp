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
#include <QVariant>  // TODO(Takagi, Isamu): debug
#include <multi_data_monitor/design.hpp>

namespace multi_data_monitor
{

class Simple : public multi_data_monitor::Design
{
public:
  QWidget * CreateWidget(const YAML::Node) override
  {
    label_ = new QLabel(QString::fromStdString("Simple"));
    label_->setAlignment(Qt::AlignCenter);
    // title_ = config_.custom["title"].as<std::string>("");
    // label->setToolTip("Simple Widget");
    return label_;
  }

  void Callback(const MonitorValues & input) override
  {
    label_->setText(QString::fromStdString(input.value.as<std::string>()));

    for (const auto & [name, attr] : input.attrs)
    {
      label_->setProperty(name.c_str(), QVariant(attr.c_str()));
    }
  }

private:
  QLabel * label_;
};

}  // namespace multi_data_monitor

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(multi_data_monitor::Simple, multi_data_monitor::Design)