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

#include "panel.hpp"
#include <rviz_common/display_context.hpp>
#include <rviz_common/ros_integration/ros_node_abstraction_iface.hpp>

MultiDataMonitor::MultiDataMonitor(QWidget * parent) : rviz_common::Panel(parent)
{

}

void MultiDataMonitor::save(rviz_common::Config config) const
{
  Panel::save(config);
  config.mapSetValue("File", path_);
}

void MultiDataMonitor::load(const rviz_common::Config & config)
{
  Panel::load(config);
  config.mapGetString("File", &path_);

  const auto node = getDisplayContext()->getRosNodeAbstraction();
  manager_.Load(path_.toStdString(), node.lock()->get_raw_node());
}

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(MultiDataMonitor, rviz_common::Panel)
