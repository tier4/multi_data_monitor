// Copyright 2022 Takagi, Isamu
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

#include "plantuml.hpp"
#include <fstream>
#include <sstream>
#include <vector>

namespace multi_data_monitor::plantuml
{

std::string Diagram::convert(const ConfigData & data) const
{
  std::ostringstream ss;
  ss << "@startuml debug" << std::endl;

  for (const auto & stream : data.streams)
  {
    ss << "card " << stream << " #AAFFFF [" << std::endl;
    ss << stream->klass;
    if (!stream->label.empty())
    {
      ss << " [" << stream->label << "]";
    }
    ss << std::endl;
    ss << "---" << std::endl;
    ss << YAML::Dump(stream->yaml) << std::endl;
    ss << "]" << std::endl;
  }

  for (const auto & widget : data.widgets)
  {
    ss << "card " << widget << " #FFAAFF [" << std::endl;
    ss << widget->klass;
    if (!widget->label.empty())
    {
      ss << " [" << widget->label << "]";
    }
    ss << std::endl;
    ss << "---" << std::endl;
    ss << YAML::Dump(widget->yaml) << std::endl;
    ss << "]" << std::endl;
  }

  for (const auto & stream : data.streams)
  {
    if (stream->input)
    {
      ss << stream << " --> " << stream->input << std::endl;
    }
    if (stream->refer)
    {
      ss << stream << " --> " << stream->refer << " #line.dashed" << std::endl;
    }
  }

  for (const auto & widget : data.widgets)
  {
    ss << "'items" << std::endl;
    for (const auto & item : widget->items)
    {
      ss << widget << " --> " << item.link << std::endl;
    }
    ss << "'end" << std::endl;
    if (widget->input)
    {
      ss << widget << " --> " << widget->input << std::endl;
    }
    if (widget->refer)
    {
      ss << widget << " --> " << widget->refer << " #line.dashed" << std::endl;
    }
  }

  ss << "@enduml" << std::endl;
  return ss.str();
}

void Diagram::write(const ConfigData & data, const std::string & path) const
{
  std::ofstream ofs(path);
  ofs << convert(data) << std::endl;
}

}  // namespace multi_data_monitor::plantuml
