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

#include "convert.hpp"
#include "field.hpp"
#include "message.hpp"
#include <rosidl_typesupport_introspection_cpp/field_types.hpp>
#include <string>

namespace generic_type_support
{

YAML::Node GetMessageYAML(const TypeSupportMessage & message, const void * data)
{
  YAML::Node node;
  for (const auto & field : message.GetFields())
  {
    node[field.GetDataName()] = GetFieldYAML(field, data);
  }
  return node;
}

YAML::Node GetFieldYAML(const TypeSupportField & field, const void * data)
{
  data = static_cast<const uint8_t *>(data) + field.GetMemoryOffset();
  if (field.IsArray())
  {
    return GetFieldArray(field, data);
  }
  return GetFieldValue(field, data);
}

YAML::Node GetFieldArray(const TypeSupportField & field, const void * data)
{
  YAML::Node node;
  for (const void * element : field.GetConstArray(data))
  {
    node.push_back(GetFieldValue(field, element));
  }
  return node;
}

YAML::Node GetFieldValue(const TypeSupportField & field, const void * data)
{
  using namespace rosidl_typesupport_introspection_cpp;  // NOLINT(build/namespaces)

  switch (field.GetTypeID())
  {
    case ROS_TYPE_FLOAT:
      return YAML::Node(*reinterpret_cast<const float *>(data));
    case ROS_TYPE_DOUBLE:
      return YAML::Node(*reinterpret_cast<const double *>(data));
    case ROS_TYPE_LONG_DOUBLE:
      return YAML::Node(*reinterpret_cast<const long double *>(data));
    case ROS_TYPE_CHAR:
      return YAML::Node(*reinterpret_cast<const char *>(data));
    case ROS_TYPE_WCHAR:
      return YAML::Node("[WCHAR IS NOT SUPPORTED]");
    case ROS_TYPE_BOOLEAN:
      return YAML::Node(*reinterpret_cast<const bool *>(data));
    case ROS_TYPE_OCTET:
      return YAML::Node(static_cast<uint32_t>(*reinterpret_cast<const uint8_t *>(data)));
    case ROS_TYPE_UINT8:
      return YAML::Node(static_cast<uint32_t>(*reinterpret_cast<const uint8_t *>(data)));
    case ROS_TYPE_INT8:
      return YAML::Node(static_cast<int32_t>(*reinterpret_cast<const int8_t *>(data)));
    case ROS_TYPE_UINT16:
      return YAML::Node(*reinterpret_cast<const uint16_t *>(data));
    case ROS_TYPE_INT16:
      return YAML::Node(*reinterpret_cast<const int16_t *>(data));
    case ROS_TYPE_UINT32:
      return YAML::Node(*reinterpret_cast<const uint32_t *>(data));
    case ROS_TYPE_INT32:
      return YAML::Node(*reinterpret_cast<const int32_t *>(data));
    case ROS_TYPE_UINT64:
      return YAML::Node(*reinterpret_cast<const uint64_t *>(data));
    case ROS_TYPE_INT64:
      return YAML::Node(*reinterpret_cast<const int64_t *>(data));
    case ROS_TYPE_STRING:
      return YAML::Node(*reinterpret_cast<const std::string *>(data));
    case ROS_TYPE_WSTRING:
      return YAML::Node("[WSTRING IS NOT SUPPORTED]");
    case ROS_TYPE_MESSAGE:
      return GetMessageYAML(field.GetMessage(), data);
  }
  return YAML::Node("[PARSE_ERROR]");
}

}  // namespace generic_type_support
