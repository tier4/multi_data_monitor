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

namespace generic_type_support
{

TypeSupportLibrary::TypeSupportLibrary(const std::string & type_name, const std::string & identifier)
{
  library_ = rclcpp::get_typesupport_library(type_name, identifier);
  handle_ = rclcpp::get_typesupport_handle(type_name, identifier, *library_);
}

TypeSupportLibrary TypeSupportLibrary::LoadTypeSupport(const std::string & type_name)
{
  return TypeSupportLibrary(type_name, "rosidl_typesupport_cpp");
}

TypeSupportLibrary TypeSupportLibrary::LoadIntrospection(const std::string & type_name)
{
  return TypeSupportLibrary(type_name, "rosidl_typesupport_introspection_cpp");
}

}  // namespace generic_type_support
