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



// Do not create this class directly.

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
