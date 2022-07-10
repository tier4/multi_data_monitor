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

#ifndef GENERIC_TYPE_SUPPORT__IMPL__FIELD_HPP_
#define GENERIC_TYPE_SUPPORT__IMPL__FIELD_HPP_

#include "util/types.hpp"
#include <string>
#include <vector>

namespace generic_type_support
{

class TypeSupportField
{
public:
  TypeSupportField(const IntrospectionField * field);
  const std::string GetDataName() const;
  const std::string GetTypeName() const;
  uint32_t GetMemoryOffset() const;
  uint8_t GetTypeID() const;

  bool IsMessage() const;
  TypeSupportMessage GetMessage() const;

  bool IsArray() const;
  std::vector<const void *> GetConstArray(const void * data) const;

private:
  const IntrospectionField * field_;
};

}  // namespace generic_type_support

#endif  // GENERIC_TYPE_SUPPORT__IMPL__FIELD_HPP_
