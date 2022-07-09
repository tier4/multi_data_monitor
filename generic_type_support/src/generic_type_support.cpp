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


namespace generic_type_support
{

struct GenericMessage::Data
{
  std::string type_;  // TODO: remove (introspection_->GetTypeName)
  //const TypeSupportMessage introspection_;
  //const TypeSupportSerialization serialization_;
};

GenericMessage::GenericMessage(const std::string & type_name)
{
  data_ = std::make_unique<Data>();
  data_->type_ = type_name;
}

GenericMessage::~GenericMessage()
{
  // define the destructor here for the unique_ptr.
}

std::string GenericMessage::GetTypeName() const
{
  return data_->type_;
}

YAML::Node GenericMessage::DeserializeYAML(const rclcpp::SerializedMessage & serialized) const
{
  (void)serialized;
  return {};
}

/*
YAML::Node GenericMessageSupport::DeserializeYAML(const rclcpp::SerializedMessage & serialized) const
{
  TypeSupportMessageMemory memory(introspection_);
  serialization_.GetSerialization().deserialize_message(&serialized, memory.GetData());
  return introspection_.GetClass().Get<YAML::Node>(memory.GetData());
}
*/

}  // namespace generic_type_support
