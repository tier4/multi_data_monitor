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
#include <QLabel>

namespace multi_data_monitor
{

class Simple : public BasicWidget
{
public:
  SetupWidget setup(YAML::Node yaml, const std::vector<ChildWidget> & children) override;
  void message(const Packet & packet) override;
};

SetupWidget Simple::setup(YAML::Node yaml, const std::vector<ChildWidget> & children)
{
  (void)yaml;
  (void)children;
  return {};
}

void Simple::message(const Packet & packet)
{
  (void)packet;
}

/*
class Simple : public BasicWidget
{
private:
  QLabel * label_;
  QString title_;

public:
  Instance Create(const YAML::Node params) override
  {
    title_ = QString::fromStdString(params["title"].as<std::string>(""));
    label_ = new QLabel(title_);
    label_->setAlignment(Qt::AlignCenter);
    label_->setToolTip(QString::fromStdString(params["notes"].as<std::string>("")));
    return label_;
  }

  void Callback(const MonitorValues & input) override
  {
    const auto value = QString::fromStdString(input.value.as<std::string>());
    label_->setText(title_ + value);
    UpdateProperties(input, label_);
  }
};
*/

}  // namespace multi_data_monitor

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(multi_data_monitor::Simple, multi_data_monitor::BasicWidget)
