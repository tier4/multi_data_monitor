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

#ifndef PANEL_HPP_
#define PANEL_HPP_

#include "manager.hpp"
#include <rviz_common/panel.hpp>

class QMouseEvent;
class QLineEdit;
class MonitorWidget;
class SettingWidget;

class MultiDataMonitor : public rviz_common::Panel
{
Q_OBJECT

public:
  explicit MultiDataMonitor(QWidget * parent = nullptr);
  void save(rviz_common::Config config) const override;
  void load(const rviz_common::Config & config) override;
  void onInitialize() override;
  void mousePressEvent(QMouseEvent * event) override;

private:
  monitors::Manager manager_;
  MonitorWidget * monitor_;
  SettingWidget * setting_;
};


class MonitorWidget : public QWidget
{
Q_OBJECT

public:
  explicit MonitorWidget(MultiDataMonitor * parent);
};


class SettingWidget : public QWidget
{
Q_OBJECT

public:
  explicit SettingWidget(MultiDataMonitor * parent);
  void save(rviz_common::Config config) const;
  void load(const rviz_common::Config & config);
  std::string getPackage() const;
  std::string getPath() const;

private:
  QLineEdit * package_;
  QLineEdit * path_;
};

#endif  // PANEL_HPP_
