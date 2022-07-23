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
#include <multi_data_monitor/design.hpp>

namespace multi_data_monitor
{

class Simple : public multi_data_monitor::Design
{
public:
  QWidget * CreateWidget(const YAML::Node) override;

private:
};

QWidget * Simple::CreateWidget(const YAML::Node)
{
  constexpr auto kStyleSheet = "border-width: 1px 1px 1px 1px; border-style: solid;";

  // title_ = config_.custom["title"].as<std::string>("");
  const auto label = new QLabel(QString::fromStdString("Simple"));
  label->setAlignment(Qt::AlignCenter);
  label->setStyleSheet(kStyleSheet);
  label->setToolTip("Simple Widget");
  return label;
}

/*
void Simple::Callback(const YAML::Node & field)
{
  const auto data = YAML::Clone(field);
  const auto text = data.as<std::string>();
  if (prev_ != text)
  {
    FunctionResult result = rules_.Apply(FunctionResult{data, style_});

    label->setText(QString::fromStdString(title_ + result.value.as<std::string>()));
    label->setStyleSheet(QString::fromStdString(kStyleSheet + result.style.GetStyleSheet()));  // TODO: workload
reduction

    prev_ = text;
  }
}
*/

}  // namespace multi_data_monitor

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(multi_data_monitor::Simple, multi_data_monitor::Design)
