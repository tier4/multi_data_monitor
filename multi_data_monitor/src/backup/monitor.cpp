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

#include "monitor.hpp"
#include <string>

namespace multi_data_monitor
{

Monitor::Monitor(const ObjectConfig & config)
{
  config_ = config;
}

/*
void NodeBase::AddChild(QWidget * parent, const std::unique_ptr<NodeBase> & base)
{
  auto layout = base->GetLayout();
  auto widget = base->GetWidget();
  std::cout << widget << " " << layout << std::endl;

  if (layout)
  {
    parent->setLayout(layout);
  }
}

void NodeBase::AddChild(QLayout * parent, const std::unique_ptr<NodeBase> & base)
{
  (void)parent;
  (void)base;
}
*/

}  // namespace multi_data_monitor
