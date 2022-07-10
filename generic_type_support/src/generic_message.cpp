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

#include "generic_type_support/generic_type_support.hpp"
#include "impl/library.hpp"
#include "impl/message.hpp"
#include "impl/convert.hpp"

namespace generic_type_support
{

struct GenericMessage::Data
{
  Data(const std::string & type_name);
  const TypeSupportLibrary introspection_library;
  const TypeSupportLibrary serialization_library;
  const TypeSupportMessage introspection;
  const rclcpp::SerializationBase serialization;
};

GenericMessage::Data::Data(const std::string & type_name)
: introspection_library(TypeSupportLibrary::LoadIntrospection(type_name)),
  serialization_library(TypeSupportLibrary::LoadTypeSupport(type_name)),
  introspection(introspection_library.GetMessage()),
  serialization(serialization_library.CreateSerialization())
{

}

GenericMessage::GenericMessage(const std::string & type_name)
{
  data_ = std::make_unique<Data>(type_name);
}

GenericMessage::~GenericMessage()
{
  // define the destructor here to delete members.
}

std::string GenericMessage::GetTypeName() const
{
  return data_->introspection.GetTypeName();
}

YAML::Node GenericMessage::ConvertYAML(const rclcpp::SerializedMessage & serialized) const
{
  void * memory = nullptr;
  data_->introspection.CreateMemory(memory);
  data_->serialization.deserialize_message(&serialized, memory);
  auto yaml = GetMessageYAML(data_->introspection, memory);
  data_->introspection.DeleteMemory(memory);
  return yaml;
}

}  // namespace generic_type_support
