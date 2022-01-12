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

#include "matrix.hpp"
#include <QGridLayout>

#include <iostream>

namespace monitors
{

void Matrix::Build(MonitorDict & monitors)
{
  layout_ = grid = new QGridLayout();

  int cols = yaml_["cols"].as<int>();
  int rows = yaml_["rows"].as<int>();
  int x = 0;
  int y = 0;

  for (const auto & node : yaml_["children"])
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

}  // namespace monitors
