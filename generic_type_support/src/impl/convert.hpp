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

#ifndef IMPL__CONVERT_HPP_
#define IMPL__CONVERT_HPP_

#include "util/types.hpp"
#include <yaml-cpp/yaml.h>

namespace generic_type_support
{

YAML::Node GetMessageYAML(const TypeSupportMessage & message, const void * data);
YAML::Node GetFieldYAML(const TypeSupportField & field, const void * data);
YAML::Node GetFieldArray(const TypeSupportField & field, const void * data);
YAML::Node GetFieldValue(const TypeSupportField & field, const void * data);

}  // namespace generic_type_support

#endif  // IMPL__CONVERT_HPP_
