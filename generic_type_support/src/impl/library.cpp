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

#include "library.hpp"
#include "message.hpp"

namespace generic_type_support
{

constexpr char typesupport_identifier[] = "rosidl_typesupport_cpp";
constexpr char introspection_identifier[] = "rosidl_typesupport_introspection_cpp";

TypeSupportLibrary::TypeSupportLibrary(const std::string & type_name, const std::string & identifier)
{
  library_ = rclcpp::get_typesupport_library(type_name, identifier);
  handle_ = rclcpp::get_typesupport_handle(type_name, identifier, *library_);
}

TypeSupportLibrary TypeSupportLibrary::LoadTypeSupport(const std::string & type_name)
{
  return TypeSupportLibrary(type_name, typesupport_identifier);
}

TypeSupportLibrary TypeSupportLibrary::LoadIntrospection(const std::string & type_name)
{
  return TypeSupportLibrary(type_name, introspection_identifier);
}

TypeSupportMessage TypeSupportLibrary::GetMessage() const
{
  if (std::string(handle_->typesupport_identifier) == std::string(introspection_identifier))
  {
    return TypeSupportMessage(handle_);
  }
  throw TypeSupportIdentifierError(handle_->typesupport_identifier);
}

rclcpp::SerializationBase TypeSupportLibrary::CreateSerialization() const
{
  if (std::string(handle_->typesupport_identifier) == std::string(typesupport_identifier))
  {
    return rclcpp::SerializationBase(handle_);
  }
  throw TypeSupportIdentifierError(handle_->typesupport_identifier);
}

}  // namespace generic_type_support
