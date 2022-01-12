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

#include "generic_type_support/typesupport.hpp"

#include <iostream>
#include <memory>
#include <string>

#include <yaml-cpp/yaml.h>
#include <rosidl_typesupport_introspection_cpp/field_types.hpp>

namespace generic_type_support
{

TypeSupportLibrary TypeSupportLibrary::LoadTypeSupport(const std::string & type)
{
  constexpr auto identifier = "rosidl_typesupport_cpp";
  const auto library = rclcpp::get_typesupport_library(type, identifier);
  const auto handle = rclcpp::get_typesupport_handle(type, identifier, *library);
  return TypeSupportLibrary{handle, library};
}

TypeSupportLibrary TypeSupportLibrary::LoadIntrospection(const std::string & type)
{
  constexpr auto identifier = "rosidl_typesupport_introspection_cpp";
  const auto library = rclcpp::get_typesupport_library(type, identifier);
  const auto handle = rclcpp::get_typesupport_handle(type, identifier, *library);
  return TypeSupportLibrary{handle, library};
}

TypeSupportField::TypeSupportField(const IntrospectionField * field) : field_(field)
{
}

void TypeSupportField::Dump() const
{
  std::cout << "name               : " << field_->name_ << std::endl;
  std::cout << "type_id            : " << static_cast<uint32_t>(field_->type_id_) << std::endl;
  std::cout << "string_upper_bound : " << field_->string_upper_bound_ << std::endl;
  std::cout << "members            : " << field_->members_ << std::endl;
  std::cout << "is_array           : " << field_->is_array_ << std::endl;
  std::cout << "array_size         : " << field_->array_size_ << std::endl;
  std::cout << "is_upper_bound     : " << field_->is_upper_bound_ << std::endl;
  std::cout << "offset             : " << field_->offset_ << std::endl;
  std::cout << "default_value      : " << field_->default_value_ << std::endl;
  std::cout << "size_function      : " << field_->size_function << std::endl;
  std::cout << "get_const_function : " << field_->get_const_function << std::endl;
  std::cout << "get_function       : " << field_->get_function << std::endl;
  std::cout << "resize_function    : " << field_->resize_function << std::endl;
}

bool TypeSupportField::IsClass() const
{
  return field_->type_id_ == rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE;
}

bool TypeSupportField::IsArray() const
{
  return field_->is_array_;
}

bool TypeSupportField::HasIndex(size_t index) const
{
  if (field_->array_size_ == 0)
  {
    return true;  // dynamic
  }
  return index < field_->array_size_;  // fixed or bounded
}

TypeSupportClass TypeSupportField::GetClass() const
{
  if (IsClass())
  {
    return TypeSupportClass(field_->members_);
  }
  // TODO: class name for improvement
  throw std::runtime_error(field_->name_ + std::string(" is not class"));
}

template<>
YAML::Node TypeSupportClass::Get(const void * data) const;

YAML::Node GetFieldValue(const void * data, const IntrospectionField & field_)
{
  using namespace rosidl_typesupport_introspection_cpp;

  switch (field_.type_id_)
  {
    case ROS_TYPE_FLOAT:
      return YAML::Node(*reinterpret_cast<const float*>(data));
    case ROS_TYPE_DOUBLE:
      return YAML::Node(*reinterpret_cast<const double*>(data));
    case ROS_TYPE_LONG_DOUBLE:
      return YAML::Node(*reinterpret_cast<const long double*>(data));
    case ROS_TYPE_CHAR:
      return YAML::Node(*reinterpret_cast<const char*>(data));
    case ROS_TYPE_WCHAR:
      return YAML::Node("[WCHAR IS NOT IMPLEMENTED]");
    case ROS_TYPE_BOOLEAN:
      return YAML::Node(*reinterpret_cast<const bool*>(data));
    case ROS_TYPE_OCTET:
      return YAML::Node(static_cast<uint32_t>(*reinterpret_cast<const uint8_t*>(data)));
    case ROS_TYPE_UINT8:
      return YAML::Node(static_cast<uint32_t>(*reinterpret_cast<const uint8_t*>(data)));
    case ROS_TYPE_INT8:
      return YAML::Node(static_cast<int32_t>(*reinterpret_cast<const int8_t*>(data)));
    case ROS_TYPE_UINT16:
      return YAML::Node(*reinterpret_cast<const uint16_t*>(data));
    case ROS_TYPE_INT16:
      return YAML::Node(*reinterpret_cast<const int16_t*>(data));
    case ROS_TYPE_UINT32:
      return YAML::Node(*reinterpret_cast<const uint32_t*>(data));
    case ROS_TYPE_INT32:
      return YAML::Node(*reinterpret_cast<const int32_t*>(data));
    case ROS_TYPE_UINT64:
      return YAML::Node(*reinterpret_cast<const uint64_t*>(data));
    case ROS_TYPE_INT64:
      return YAML::Node(*reinterpret_cast<const int64_t*>(data));
    case ROS_TYPE_STRING:
      return YAML::Node(*reinterpret_cast<const std::string*>(data));
    case ROS_TYPE_WSTRING:
      return YAML::Node("[WSTRING IS NOT IMPLEMENTED]");
    case ROS_TYPE_MESSAGE:
      return TypeSupportClass(field_.members_).Get<YAML::Node>(data);
  }
  return YAML::Node("[PARSE_ERROR]");
}

YAML::Node GetFieldArray(const void * data, const IntrospectionField & field_)
{
  YAML::Node node;
  size_t size = field_.size_function(data);
  for (size_t i = 0; i < size; ++i)
  {
    const void * element = field_.get_const_function(data, i);
    node.push_back(GetFieldValue(element, field_));
  }
  return node;
}

template<>
YAML::Node TypeSupportField::Get(const void * data) const
{
  data = static_cast<const uint8_t*>(data) + field_->offset_;
  if (field_->is_array_)
  {
    return GetFieldArray(data, *field_);
  }
  return GetFieldValue(data, *field_);
}

TypeSupportClass::TypeSupportClass(const IntrospectionMessage * message) : message_(message)
{
  for (uint32_t i = 0; i < message_->member_count_; ++i)
  {
    fields_.emplace_back(message_->members_ + i);
  }
}

TypeSupportClass::TypeSupportClass(const TypeSupportHandle * handle)
: TypeSupportClass(reinterpret_cast<const IntrospectionMessage *>(handle->data))
{
}

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

std::string TypeSupportClass::GetFullName() const
{
  return message_->message_namespace_ + std::string("::") +  message_->message_name_;
}

void TypeSupportClass::CreateMemory(void *& data)
{
  data = std::malloc(message_->size_of_);
  message_->init_function(data, rosidl_runtime_cpp::MessageInitialization::DEFAULTS_ONLY);
}

void TypeSupportClass::DeleteMemory(void *& data)
{
  message_->fini_function(data);
  std::free(data);
  data = nullptr;
}

bool TypeSupportClass::HasField(const std::string & name) const
{
  // TODO: improve algorithm
  for (const auto field : fields_)
  {
    if (name == field.GetName()) { return true; }
  }
  return false;
}

TypeSupportField TypeSupportClass::GetField(const std::string & name) const
{
  // TODO: improve algorithm
  for (const auto field : fields_)
  {
    if (name == field.GetName()) { return field; }
  }
  throw std::runtime_error("Field '" + name +"' is not a member of '" + GetFullName() + "'");
}

template<>
YAML::Node TypeSupportClass::Get(const void * data) const
{
  YAML::Node node;
  for (const auto field : fields_)
  {
    node[field.GetName()] = field.Get<YAML::Node>(data);
  }
  return node;
}

TypeSupportMessage::TypeSupportMessage(const std::string & type)
: library_(TypeSupportLibrary::LoadIntrospection(type))
{
}

TypeSupportClass TypeSupportMessage::GetClass() const
{
  return TypeSupportClass(library_.handle);
}

TypeSupportSerialization::TypeSupportSerialization(const std::string & type)
: library_(TypeSupportLibrary::LoadIntrospection(type)), serialization_(library_.handle)
{
}

const rclcpp::SerializationBase & TypeSupportSerialization::GetSerialization() const
{
  return serialization_;
}

TypeSupportMessageMemory::TypeSupportMessageMemory(const TypeSupportMessage & message)
: message_(message)
{
  message_.GetClass().CreateMemory(data_);
}

TypeSupportMessageMemory::~TypeSupportMessageMemory()
{
  message_.GetClass().DeleteMemory(data_);
}

}  // namespace generic_type_support
