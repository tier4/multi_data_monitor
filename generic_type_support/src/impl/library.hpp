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

#ifndef GENERIC_TYPE_SUPPORT__IMPL__LIBRARY_HPP_
#define GENERIC_TYPE_SUPPORT__IMPL__LIBRARY_HPP_

#include "util/types.hpp"
#include <rclcpp/typesupport_helpers.hpp>
#include <rclcpp/serialization.hpp>

namespace generic_type_support
{

class TypeSupportLibrary
{
public:
  TypeSupportLibrary(const std::string & type_name, const std::string & identifier);
  static TypeSupportLibrary LoadTypeSupport(const std::string & type_name);
  static TypeSupportLibrary LoadIntrospection(const std::string & type_name);
  TypeSupportMessage GetMessage() const;
  rclcpp::SerializationBase CreateSerialization() const;

private:
  const rosidl_message_type_support_t * handle_;
  std::shared_ptr<rcpputils::SharedLibrary> library_;
};

}  // namespace generic_type_support

#endif  // GENERIC_TYPE_SUPPORT__IMPL__LIBRARY_HPP_
