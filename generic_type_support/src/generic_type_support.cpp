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

#include "generic_type_support/generic_type_support.hpp"
#include "impl/library.hpp"
#include "impl/convert.hpp"
#include "impl/message.hpp"
#include "impl/field.hpp"

namespace generic_type_support
{

struct GenericMessage::Data
{
  Data(const std::string & type_name);
  const TypeSupportLibrary introspection_library;
  const TypeSupportLibrary serialization_library;
  const TypeSupportMessage introspection;
  const rclcpp::SerializationBase serialization;
};

GenericMessage::Data::Data(const std::string & type_name)
: introspection_library(TypeSupportLibrary::LoadIntrospection(type_name)),
  serialization_library(TypeSupportLibrary::LoadTypeSupport(type_name)),
  introspection(introspection_library.GetMessage()),
  serialization(serialization_library.CreateSerialization())
{

}

GenericMessage::GenericMessage(const std::string & type_name)
{
  data_ = std::make_unique<Data>(type_name);
}

GenericMessage::~GenericMessage()
{
  // define the destructor here to delete members.
}

std::string GenericMessage::GetTypeName() const
{
  return data_->introspection.GetTypeName();
}

YAML::Node GenericMessage::ConvertYAML(const rclcpp::SerializedMessage & serialized) const
{
  void * memory = nullptr;
  data_->introspection.CreateMemory(memory);
  data_->serialization.deserialize_message(&serialized, memory);
  auto yaml = GetMessageYAML(data_->introspection, memory);
  data_->introspection.DeleteMemory(memory);
  return yaml;
}

GenericMessage::GenericField GenericMessage::GetField(const std::string & path)
{
  return GenericField(*this, path);
}



std::vector<std::string> split(const std::string & input, char delimiter)
{
  std::vector<std::string> result;
  size_t found = input.find(delimiter);
  size_t start = 0;
  while(found != std::string::npos)
  {
    result.push_back(input.substr(start, found - start));
    start = found + 1;
    found = input.find(delimiter, start);
  }
  result.push_back(input.substr(start));
  return result;
}

std::string join(const std::vector<std::string> & input, const std::string & delimiter)
{
  std::string result;
  for (size_t i = 0; i < input.size(); ++i)
  {
    result += (i ? delimiter : "") + input[i];
  }
  return result;
}

struct GenericFieldElement
{
  GenericFieldElement(const std::string & element);

  int index;
  std::string name;
};

GenericFieldElement::GenericFieldElement(const std::string & element)
{
  const auto token = split(element, '@');
  name = token[0];
  index = token.size() == 2 ? std::stoi(token[1]) : -1;
}



struct GenericMessage::GenericField::Data
{
  std::vector<GenericFieldElement> elements;
};

GenericMessage::GenericField::GenericField(const GenericMessage & generic, const std::string & path)
{
  data_ = std::make_unique<Data>();
  for (const auto & element : split(path, '.'))
  {
    data_->elements.emplace_back(element);
  }

  // TODO: check array type
  TypeSupportField field = generic.data_->introspection.GetField(data_->elements.at(0).name);
  for (size_t i = 1, n = data_->elements.size(); i < n; ++i)
  {
    const auto & element = data_->elements[i];
    if (!field.IsMessage())
    {
      throw FieldError("Field '" + element.name +"' is not a member of '" + field.GetTypeName() + "'");
    }
    field = field.GetMessage().GetField(data_->elements[i].name);
  }
}

GenericMessage::GenericField::~GenericField()
{
  // define the destructor here to delete members.
}

const YAML::Node GenericMessage::GenericField::Access(const YAML::Node & yaml) const
{
  YAML::Node node = yaml;
  for (const auto & element : data_->elements)
  {
    if (element.index < 0)
    {
      node.reset(node[element.name]);
    }
    else
    {
      node.reset(node[element.name][element.index]);
    }
  }
  return node;
}

/*
bool GenericTypeAccess::Validate(const TypeSupportClass & support) const
{
    //std::string last_type = "primitive";
    //if (support_field.IsClass())
    //{

    if (support_field.IsClass())
    {
      if(type_.empty())
      {
        throw std::runtime_error("Field '" + path_ +"' in '" + support.GetFullName() + "' is not a primitive type");
      }
      else
      {
        if (type_ != support_field.GetClass().GetFullName())
        {
          throw std::runtime_error("Field '" + path_ +"' in '" + support.GetFullName() + "' is not '" + type_ + "'");
        }
      }
    }

  }
  return true;
}
*/

}  // namespace generic_type_support
