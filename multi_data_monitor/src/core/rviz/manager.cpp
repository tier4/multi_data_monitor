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

#include "manager.hpp"
#include "runner/config_loader.hpp"
#include "runner/stream_runner.hpp"
#include "runner/widget_loader.hpp"
#include <QLabel>

// DEBUG
#include <iostream>

namespace multi_data_monitor
{

class RvizManager::Impl final
{
public:
  ~Impl();
  void setup(const std::string & path);
  void start(ros::Node node);

private:
  StreamRunner::SharedPtr stream_runner_;
  StreamLoader::SharedPtr stream_loader_;
  WidgetLoader::SharedPtr widget_loader_;
};

RvizManager::Impl::~Impl()
{
  // Stop the runner first
  if (stream_runner_)
  {
    stream_runner_->shutdown();
  }

  // Releases the object after the runner stops
  stream_runner_.reset();
  stream_loader_.reset();
  widget_loader_.reset();
}

void RvizManager::Impl::setup(const std::string & path)
{
  const auto data = ConfigLoader().execute(path);
  stream_loader_ = std::make_shared<StreamLoader>(data.streams);
  widget_loader_ = std::make_shared<WidgetLoader>(data.widgets);
}

void RvizManager::Impl::start(ros::Node node)
{
  stream_runner_ = std::make_shared<StreamRunner>(stream_loader_);
  stream_runner_->start(node);
}

QWidget * RvizManager::build(const std::string & path, ros::Node node)
{
  impl = std::make_unique<Impl>();

  impl->setup(path);
  impl->start(node);

  return new QLabel("TEST");
}

RvizManager::RvizManager()
{
}

RvizManager::~RvizManager()
{
}

}  // namespace multi_data_monitor
