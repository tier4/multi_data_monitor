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

#include "widget.hpp"
#include <QDockWidget>
#include <QFormLayout>  // debug
#include <QGridLayout>
#include <QLabel>  // debug
#include <QLineEdit>
#include <QMouseEvent>
#include <QStackedLayout>
#include <QVBoxLayout>
#include <rviz_common/display_context.hpp>
#include <rviz_common/ros_integration/ros_node_abstraction_iface.hpp>
#include <memory>
#include <string>

namespace multi_data_monitor
{

MonitorWidget::MonitorWidget(rviz_common::Panel * panel) : QWidget(panel)
{
  constexpr auto kStyleSheet = "border-width: 1px 1px 1px 1px; border-style: solid;";
  const auto layout = new MyGrid();
  for (size_t i = 0; i < 6; ++i)
  {
    const auto widget = new MyLabel(std::to_string(i + 1).c_str());
    widget->setAlignment(Qt::AlignCenter);
    widget->setStyleSheet(kStyleSheet);
    layout->addWidget(widget, i / 3, i % 3);
  }
  layout->setContentsMargins(3, 1, 3, 1);
  setLayout(layout);
}

SettingWidget::SettingWidget(rviz_common::Panel * panel) : QWidget(panel)
{
  const auto layout = new QFormLayout();
  package_ = new QLineEdit();
  path_ = new QLineEdit();
  connect(package_, &QLineEdit::editingFinished, panel, &MultiDataMonitor::configChanged);
  connect(path_, &QLineEdit::editingFinished, panel, &MultiDataMonitor::configChanged);

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

MultiDataMonitor::MultiDataMonitor(QWidget * parent) : rviz_common::Panel(parent)
{
  const auto stacked = new QStackedLayout();
  monitor_ = new MonitorWidget(this);
  setting_ = new SettingWidget(this);

  stacked->addWidget(setting_);
  stacked->addWidget(monitor_);
  stacked->setCurrentWidget(monitor_);
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
  reload();
}

void MultiDataMonitor::onInitialize()
{
  const auto parent = dynamic_cast<QDockWidget *>(this->parent());
  if (parent)
  {
    const auto widget = new QWidget();
    const auto layout = new QVBoxLayout();
    layout->addWidget(parent->titleBarWidget());
    layout->setMargin(0);
    widget->setLayout(layout);
    parent->setTitleBarWidget(widget);
  }

  auto rviz_node = getDisplayContext()->getRosNodeAbstraction().lock();
  loader_ = std::make_unique<Loader>(rviz_node->get_raw_node());
}

void MultiDataMonitor::mousePressEvent(QMouseEvent * event)
{
  if (event->modifiers() & Qt::ControlModifier)
  {
    const auto layout = dynamic_cast<QStackedLayout *>(this->layout());
    if (layout)
    {
      layout->setCurrentIndex(1 - layout->currentIndex());
    }
  }

  if (event->modifiers() & Qt::ShiftModifier)
  {
    const auto parent = dynamic_cast<QDockWidget *>(this->parent());
    if (parent)
    {
      const auto title = parent->titleBarWidget()->layout()->itemAt(0)->widget();
      title->setVisible(!title->isVisible());
    }
  }
}

void MultiDataMonitor::reload()
{
  QWidget * widget = loader_->Reload(setting_->getPackage(), setting_->getPath());
  if (widget)
  {
    QStackedLayout * stacked = dynamic_cast<QStackedLayout *>(layout());
    delete monitor_;
    monitor_ = widget;
    stacked->addWidget(monitor_);
    stacked->setCurrentWidget(monitor_);
    setLayout(stacked);
  }
}

}  // namespace multi_data_monitor

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(multi_data_monitor::MultiDataMonitor, rviz_common::Panel)
