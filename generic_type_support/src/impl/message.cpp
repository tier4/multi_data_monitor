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

#include "message.hpp"
#include "field.hpp"
#include "generic_type_support/errors.hpp"
#include <iostream>

namespace generic_type_support
{

TypeSupportMessage::TypeSupportMessage(const IntrospectionMessage * message)
{
  message_ = message;
  for (uint32_t i = 0; i < message_->member_count_; ++i)
  {
    fields_.emplace_back(message_->members_ + i);
  }
}

TypeSupportMessage::TypeSupportMessage(const  IntrospectionHandle * handle)
{
  message_ = reinterpret_cast<const IntrospectionMessage *>(handle->data);
  for (uint32_t i = 0; i < message_->member_count_; ++i)
  {
    fields_.emplace_back(message_->members_ + i);
  }
}

TypeSupportMessage::~TypeSupportMessage()
{
  // define the destructor here to delete members.
}

const std::string TypeSupportMessage::GetTypeName() const
{
  std::string name = message_->message_namespace_;
  size_t pos = name.find(':');
  return name.substr(0, pos) + "/" + name.substr(pos + 2) + "/" + message_->message_name_;
}

const std::vector<TypeSupportField> & TypeSupportMessage::GetFields() const
{
  return fields_;
}

const TypeSupportField TypeSupportMessage::GetField(const std::string name) const
{
  for (const auto field : fields_)
  {
    if (name == field.GetDataName()) { return field; }
  }
  throw FieldError("Field '" + name +"' is not a member of '" + GetTypeName() + "'");
}

void TypeSupportMessage::CreateMemory(void *& data) const
{
  data = std::malloc(message_->size_of_);
  message_->init_function(data, rosidl_runtime_cpp::MessageInitialization::DEFAULTS_ONLY);
}

void TypeSupportMessage::DeleteMemory(void *& data) const
{
  message_->fini_function(data);
  std::free(data);
}

/*
void TypeSupportClass::Dump() const
{
  std::cout << "namespace     : " << message_->message_namespace_ << std::endl;
  std::cout << "name          : " << message_->message_name_ << std::endl;
  std::cout << "member_count  : " << message_->member_count_ << std::endl;
  std::cout << "size_of       : " << message_->size_of_ << std::endl;
  std::cout << "members       : " << message_->members_ << std::endl;
  std::cout << "init_function : " << reinterpret_cast<void *>(message_->init_function) << std::endl;
  std::cout << "fini_function : " << reinterpret_cast<void *>(message_->fini_function) << std::endl;
}
*/

}  // namespace generic_type_support
