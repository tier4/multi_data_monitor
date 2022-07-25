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
#include <QTextEdit>
#include <rviz_common/display_context.hpp>
#include <rviz_common/ros_integration/ros_node_abstraction_iface.hpp>
#include <memory>
#include <string>

namespace multi_data_monitor
{

SettingWidget::SettingWidget(MultiDataMonitor * panel) : QWidget(panel)
{
  const auto layout = new QGridLayout();
  button_ = new QPushButton("Reload");
  path_ = new QLineEdit();
  path_->setPlaceholderText("package://<package>/<path>  or  file://<path>");
  connect(path_, &QLineEdit::editingFinished, panel, &MultiDataMonitor::configChanged);
  connect(button_, &QPushButton::clicked, panel, &MultiDataMonitor::reload);

  layout->addWidget(new QLabel("Path"), 0, 0);
  layout->addWidget(path_, 0, 1);
  layout->addWidget(button_, 1, 0, 1, 2);
  layout->setContentsMargins(0, 0, 0, 0);
  setLayout(layout);
}

void SettingWidget::save(rviz_common::Config config) const
{
  config.mapSetValue("Path", path_->text());
}

void SettingWidget::load(const rviz_common::Config & config)
{
  path_->setText(config.mapGetChild("Path").getValue().toString());
}

std::string SettingWidget::getPath() const
{
  return path_->text().toStdString();
}

MultiDataMonitor::MultiDataMonitor(QWidget * parent) : rviz_common::Panel(parent)
{
  const auto layout = new QGridLayout();
  setting_ = new SettingWidget(this);
  monitor_ = new QLabel();
  monitor_->setVisible(false);
  layout->addWidget(monitor_);
  layout->addWidget(setting_);
  setLayout(layout);
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
    const auto layout = new QGridLayout();
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
  if (event->modifiers() & Qt::ControlModifier)
  {
    setting_->setVisible(!setting_->isVisible());
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
  QWidget * widget = nullptr;
  try
  {
    // TODO(Takagi, Isamu): subscribe automatically
    loader_->Unsubscribe();
    widget = loader_->Reload(setting_->getPath());
    loader_->Subscribe(rviz_node_);
    setting_->setVisible(false);
  }
  catch (const std::exception & error)
  {
    RCLCPP_ERROR_STREAM(rviz_node_->get_logger(), error.what());
    QTextEdit * text = new QTextEdit();
    text->setReadOnly(true);
    text->setText(error.what());
    widget = text;
    setting_->setVisible(true);
  }

  if (widget)
  {
    layout()->replaceWidget(monitor_, widget);
    delete monitor_;
    monitor_ = widget;
    monitor_->setVisible(true);
  }
}

}  // namespace multi_data_monitor

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(multi_data_monitor::MultiDataMonitor, rviz_common::Panel)
