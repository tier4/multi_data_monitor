// Copyright 2021 Takagi, Isamu
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

#include "simple.hpp"
#include <QLabel>

#include <iostream>

namespace monitors
{

constexpr auto kStyleSheet = "border-width: 1px 1px 1px 1px; border-style: solid;";

StyleDefinition Simple::default_style_;

void Simple::Build([[maybe_unused]] MonitorDict & monitors)
{
  title_ = yaml_["title"].as<std::string>("");

  rules_.Load(yaml_["rules"]);
  style_ = default_style_.Merge(StyleDefinition(yaml_["style"]));

  widget_ = label = new QLabel(QString::fromStdString(title_));
  label->setAlignment(Qt::AlignCenter);
  label->setStyleSheet(QString::fromStdString(kStyleSheet + style_.GetStyleSheet()));
  // setToolTip();
}

void Simple::Callback(const YAML::Node & message)
{
  const auto data = YAML::Clone(access_.Get(message));
  const auto text = data.as<std::string>();
  if (prev_ != text)
  {
    FunctionResult result = rules_.Apply(FunctionResult{data, style_});

    label->setText(QString::fromStdString(title_ + result.value.as<std::string>()));
    label->setStyleSheet(QString::fromStdString(kStyleSheet + result.style.GetStyleSheet()));  // TODO: workload reduction

    prev_ = text;
  }
}

}  // namespace monitors
