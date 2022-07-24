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
#include <QGridLayout>
#include <QLabel>
#include <QLineEdit>
#include <QMouseEvent>
#include <QPushButton>
#include <QStackedLayout>
#include <QVBoxLayout>
#include <rviz_common/display_context.hpp>
#include <rviz_common/ros_integration/ros_node_abstraction_iface.hpp>
#include <memory>
#include <string>

namespace multi_data_monitor
{

SettingWidget::SettingWidget(rviz_common::Panel * panel) : QWidget(panel)
{
  const auto layout = new QGridLayout();
  package_ = new QLineEdit();
  path_ = new QLineEdit();
  button_ = new QPushButton("Reload");
  connect(package_, &QLineEdit::editingFinished, panel, &MultiDataMonitor::configChanged);
  connect(path_, &QLineEdit::editingFinished, panel, &MultiDataMonitor::configChanged);

  layout->addWidget(new QLabel("Package"), 0, 0);
  layout->addWidget(new QLabel("Path"), 1, 0);
  layout->addWidget(package_, 0, 1);
  layout->addWidget(path_, 1, 1);
  layout->addWidget(button_, 2, 0, 1, 2);
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
  monitor_ = nullptr;
  // setting_ = new SettingWidget(this);
  // stacked->addWidget(setting_);
  setLayout(stacked);
}

void MultiDataMonitor::save(rviz_common::Config config) const
{
  Panel::save(config);
  // setting_->save(config);

  // TODO(Takagi, Isamu): temporary
  config.mapSetValue("Package", QString::fromStdString(package_));
  config.mapSetValue("Path", QString::fromStdString(path_));
}

void MultiDataMonitor::load(const rviz_common::Config & config)
{
  Panel::load(config);
  // setting_->load(config);

  // TODO(Takagi, Isamu): temporary
  package_ = config.mapGetChild("Package").getValue().toString().toStdString();
  path_ = config.mapGetChild("Path").getValue().toString().toStdString();

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

  rviz_node_ = getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node();
  loader_ = std::make_unique<Loader>();
}

void MultiDataMonitor::mousePressEvent(QMouseEvent * event)
{
  /*
  if (event->modifiers() & Qt::ControlModifier)
  {
    const auto layout = dynamic_cast<QStackedLayout *>(this->layout());
    if (layout)
    {
      layout->setCurrentIndex(1 - layout->currentIndex());
    }
  }
  */

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
  QWidget * widget = nullptr;
  try
  {
    // widget = loader_->Reload(setting_->getPackage(), setting_->getPath());
    widget = loader_->Reload(package_, path_);
    loader_->Subscribe(rviz_node_);
  }
  catch (const std::exception & error)
  {
    widget = new QLabel(error.what());
    RCLCPP_ERROR_STREAM(rviz_node_->get_logger(), error.what());
  }

  if (widget)
  {
    QStackedLayout * stacked = dynamic_cast<QStackedLayout *>(layout());
    delete monitor_;
    monitor_ = widget;
    stacked->addWidget(monitor_);
    stacked->setCurrentWidget(monitor_);
  }
}

}  // namespace multi_data_monitor

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(multi_data_monitor::MultiDataMonitor, rviz_common::Panel)
