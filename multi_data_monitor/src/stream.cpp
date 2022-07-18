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

#include "stream.hpp"
#include "config.hpp"
#include <memory>

namespace multi_data_monitor
{

TopicStream::TopicStream(const TopicConfig & config)
{
  config_ = config;
}

void TopicStream::Callback(const YAML::Node & yaml)
{
  throw ConfigError::Logic("TopicStream::Callback");
}

void TopicStream::Register(Stream * output)
{
  outputs_.insert(output);
}

void TopicStream::Subscribe(rclcpp::Node::ConstSharedPtr node)
{
  auto qos = rclcpp::QoS(config_.depth);

  const auto callback = [this](const std::shared_ptr<rclcpp::SerializedMessage> serialized)
  {
    // const auto yaml = message_->ConvertYAML(*serialized);
    // const auto node = access_->Access(yaml);
  };

  subscription_ = node->create_generic_subscription(config_.name, config_.type, qos, callback);
}

void TopicStream::Unsubscribe()
{
  subscription_.reset();
}

}  // namespace multi_data_monitor
