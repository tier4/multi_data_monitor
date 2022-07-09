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

#ifndef GENERIC_TYPE_SUPPORT__TYPESUPPORT_HPP_
#define GENERIC_TYPE_SUPPORT__TYPESUPPORT_HPP_

#include "typesupport.hpp"

#include <rclcpp/serialization.hpp>
#include <rclcpp/typesupport_helpers.hpp>
#include <rosidl_typesupport_introspection_cpp/message_introspection.hpp>
#include <rosidl_typesupport_introspection_cpp/service_introspection.hpp>
#include <string>
#include <vector>

namespace generic_type_support
{

class TypeSupportMessage;
class TypeSupportService;
class TypeSupportSerialization;
class TypeSupportMessageMemory;

class TypeSupportClass;
class TypeSupportField;
using TypeSupportHandle = rosidl_message_type_support_t;
using IntrospectionMessage = rosidl_typesupport_introspection_cpp::MessageMembers;
using IntrospectionField = rosidl_typesupport_introspection_cpp::MessageMember;
using IntrospectionService = rosidl_typesupport_introspection_cpp::ServiceMembers;

// Do not create this class directly.
struct TypeSupportLibrary
{
  static TypeSupportLibrary LoadTypeSupport(const std::string & type);
  static TypeSupportLibrary LoadIntrospection(const std::string & type);

  const rosidl_message_type_support_t * handle;
  const std::shared_ptr<rcpputils::SharedLibrary> library;
};

// Do not create this class directly.
class TypeSupportField
{
public:
  TypeSupportField(const IntrospectionField * field);
  void Dump() const;
  std::string GetName() const { return field_->name_; }
  bool IsClass() const;
  bool IsArray() const;
  bool HasIndex(size_t index) const;
  TypeSupportClass GetClass() const;

  template<class T>
  T Get(const void * data) const;

private:
  const IntrospectionField * field_;
};

// Do not create this class directly.
class TypeSupportClass
{
public:
  TypeSupportClass(const IntrospectionMessage * message);
  TypeSupportClass(const TypeSupportHandle * handle);
  void Dump() const;
  std::string GetFullName() const;
  void CreateMemory(void *& data);
  void DeleteMemory(void *& data);
  bool HasField(const std::string & name) const;
  TypeSupportField GetField(const std::string & name) const;

  // const auto begin() const { return fields_.begin(); }
  // const auto end() const { return fields_.end(); }

  template<class T>
  T Get(const void * data) const;

private:
  const IntrospectionMessage * message_;
  std::vector<TypeSupportField> fields_;
};

// This is the interface class.
class TypeSupportMessage
{
public:
  TypeSupportMessage(const std::string & type);
  TypeSupportClass GetClass() const;

private:
  const TypeSupportLibrary library_;
  // const TypeSupportClass and GetClass returns reference
};

// This is the interface class.
class TypeSupportSerialization
{
public:
  TypeSupportSerialization(const std::string & type);
  const rclcpp::SerializationBase & GetSerialization() const;

private:
  const TypeSupportLibrary library_;
  const rclcpp::SerializationBase serialization_;
};

class TypeSupportMessageMemory
{
public:
  TypeSupportMessageMemory(const TypeSupportMessage & message);
  ~TypeSupportMessageMemory();
  void * GetData() { return data_; };

private:
  void * data_;
  const TypeSupportMessage message_;
};

}  // namespace generic_type_support

#endif  // GENERIC_TYPE_SUPPORT__TYPESUPPORT_HPP_
