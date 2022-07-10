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

#include "field.hpp"
#include "message.hpp"
#include <iostream>

namespace generic_type_support
{

TypeSupportField::TypeSupportField(const IntrospectionField * field)
{
  field_ = field;
}

const std::string TypeSupportField::GetDataName() const
{
  return field_->name_;
}

uint32_t TypeSupportField::GetMemoryOffset() const
{
  return field_->offset_;
}

uint8_t TypeSupportField::GetTypeID() const
{
  return field_->type_id_;
}

TypeSupportMessage TypeSupportField::GetMessage() const
{
  return TypeSupportMessage(field_->members_);
}

bool TypeSupportField::IsArray() const
{
  return field_->is_array_;
}

std::vector<const void *> TypeSupportField::GetConstArray(const void * data) const
{
  std::vector<const void *> array;
  for (size_t i = 0, n = field_->size_function(data); i < n; ++i)
  {
    array.push_back(field_->get_const_function(data, i));
  }
  return array;
}

/*
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

bool TypeSupportField::HasIndex(size_t index) const
{
  if (field_->array_size_ == 0)
  {
    return true;  // dynamic
  }
  return index < field_->array_size_;  // fixed or bounded
}
*/

}  // namespace generic_type_support
