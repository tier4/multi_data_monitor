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

#ifndef STREAM_HPP_
#define STREAM_HPP_

#include "config.hpp"
#include <rclcpp/rclcpp.hpp>
#include <yaml-cpp/yaml.h>
#include <unordered_set>

namespace multi_data_monitor
{

class Stream
{
public:
  virtual ~Stream() = default;
  virtual void Callback(const YAML::Node & yaml) = 0;
  virtual void Register(Stream * output) = 0;
};

class TopicStream : public Stream
{
public:
  explicit TopicStream(const TopicConfig & config);
  void Callback(const YAML::Node & yaml) override;
  void Register(Stream * output) override;
  void Subscribe(rclcpp::Node::ConstSharedPtr node);
  void Unsubscribe();

private:
  TopicConfig config_;
  rclcpp::GenericSubscription::ConstSharedPtr subscription_;
  std::unordered_set<Stream *> outputs_;
};

class FilterStream : public Stream
{
public:
  void Callback(const YAML::Node & yaml) override;
  void Register(Stream * output) override;

private:
  std::unordered_set<Stream *> outputs_;
};

}  // namespace multi_data_monitor

#endif  // STREAM_HPP_
