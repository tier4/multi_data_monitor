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
#include <QMouseEvent>
#include <QFormLayout>
#include <QStackedLayout>
#include <QLineEdit>

#include <QLabel>
#include <QGridLayout>

MultiDataMonitor::MultiDataMonitor(QWidget * parent) : rviz_common::Panel(parent)
{
  const auto stacked = new QStackedLayout();
  monitor_ = new MonitorWidget(this);
  setting_ = new SettingWidget(this);

  stacked->addWidget(monitor_);
  stacked->addWidget(setting_);
  setLayout(stacked);
}

void MultiDataMonitor::save(rviz_common::Config config) const
{
  Panel::save(config);
  setting_->save(config);
}

void MultiDataMonitor::load(const rviz_common::Config & config)
{
  Panel::load(config);
  setting_->load(config);

  const auto node = getDisplayContext()->getRosNodeAbstraction();

  const auto logger = node.lock()->get_raw_node()->get_logger();
  RCLCPP_INFO_STREAM(logger, setting_->getPackage());
  RCLCPP_INFO_STREAM(logger, setting_->getPath());

/*
  manager_.Load(path_.toStdString(), node.lock()->get_raw_node());
  manager_.Build(this);
  manager_.Start(node.lock()->get_raw_node());
*/
}

void MultiDataMonitor::onInitialize()
{

}

void MultiDataMonitor::mousePressEvent([[maybe_unused]] QMouseEvent * event)
{
  printf("mousePressEvent\n");

  if (event->modifiers() & Qt::ControlModifier)
  {
    const auto layout = dynamic_cast<QStackedLayout *>(this->layout());
    if (layout)
    {
      layout->setCurrentIndex(1 - layout->currentIndex());
    }
  }
}



MonitorWidget::MonitorWidget(MultiDataMonitor * parent) : QWidget(parent)
{
  const auto widget = new QLabel("monitor");
  const auto layout = new QGridLayout();
  layout->addWidget(widget);
  setLayout(layout);
}



SettingWidget::SettingWidget(MultiDataMonitor * parent) : QWidget(parent)
{
  const auto layout = new QFormLayout();
  package_ = new QLineEdit();
  path_ = new QLineEdit();
  connect(package_, &QLineEdit::editingFinished, parent, &MultiDataMonitor::configChanged);
  connect(path_, &QLineEdit::editingFinished, parent, &MultiDataMonitor::configChanged);

  layout->addRow("Package", package_);
  layout->addRow("Path", path_);
  setLayout(layout);
}

void SettingWidget::save(rviz_common::Config config) const
{
  config.mapSetValue("Package", package_->text());
  config.mapSetValue("Path", path_->text());
}

void SettingWidget::load(const rviz_common::Config & config)
{
  package_->setText(config.mapGetChild("Package").getValue().toString());
  path_->setText(config.mapGetChild("Path").getValue().toString());
}

std::string SettingWidget::getPackage() const
{
  return package_->text().toStdString();
}

std::string SettingWidget::getPath() const
{
  return path_->text().toStdString();
}

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(MultiDataMonitor, rviz_common::Panel)
