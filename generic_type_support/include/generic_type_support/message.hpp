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

#ifndef GENERIC_TYPE_SUPPORT__MESSAGE_HPP_
#define GENERIC_TYPE_SUPPORT__MESSAGE_HPP_

#include <yaml-cpp/yaml.h>
#include <rclcpp/serialization.hpp>
#include <memory>
#include <string>

#include <generic_type_support/errors.hpp>

namespace generic_type_support
{

class GenericMessage
{
public:
  GenericMessage(const std::string & type_name);
  ~GenericMessage();
  std::string GetTypeName() const;
  YAML::Node ConvertYAML(const rclcpp::SerializedMessage & serialized) const;

  class GenericAccess;
  std::shared_ptr<GenericAccess> GetAccess(const std::string & path) const;

private:
  struct Data;
  std::shared_ptr<Data> data_;
};

class GenericMessage::GenericAccess
{
public:
  GenericAccess(const GenericMessage & generic, const std::string & path);
  ~GenericAccess();
  const YAML::Node Access(const YAML::Node & yaml) const;

  bool IsMessage() const;
  std::string GetTypeName() const;
  std::string GetFullPath() const;

public:
  struct Data;
  std::shared_ptr<Data> data_;
};

}  // namespace generic_type_support

#endif  // GENERIC_TYPE_SUPPORT__MESSAGE_HPP_
