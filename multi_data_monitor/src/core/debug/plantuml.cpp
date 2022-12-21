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

template <class T>
void dump_node(std::ostringstream & ss, const T & data, const std::string & color)
{
  const auto label = data->label.empty() ? "" : " [" + data->label + "]";

  ss << "card " << data << " " << color << " [" << std::endl;
  ss << data->klass << label << std::endl;
  ss << "---" << std::endl;
  ss << YAML::Dump(data->yaml) << std::endl;
  ss << "]" << std::endl;
}

std::string Diagram::convert(const ConfigData & data) const
{
  std::ostringstream ss;
  ss << "@startuml debug" << std::endl;

  for (const auto & stream : data.streams)
  {
    dump_node(ss, stream, "#AAFFFF");
  }
  for (const auto & action : data.actions)
  {
    dump_node(ss, action, "#FFFFAA");
  }
  for (const auto & widget : data.widgets)
  {
    dump_node(ss, widget, "#FFAAFF");
  }

  for (const auto & stream : data.streams)
  {
    if (stream->refer)
    {
      ss << stream << " --> " << stream->refer << " #line.dashed" << std::endl;
    }
    if (stream->input)
    {
      ss << stream << " --> " << stream->input << std::endl;
    }
    if (stream->apply)
    {
      ss << stream << " --> " << stream->apply << std::endl;
    }
    if (stream->panel)
    {
      ss << stream->panel << " --> " << stream << std::endl;
    }
  }

  for (const auto & action : data.actions)
  {
    if (action->refer)
    {
      ss << action << " --> " << action->refer << " #line.dashed" << std::endl;
    }
    for (const auto & rule : action->rules)
    {
      ss << action << " --> " << rule << std::endl;
    }
  }

  for (const auto & widget : data.widgets)
  {
    if (widget->refer)
    {
      ss << widget << " --> " << widget->refer << " #line.dashed" << std::endl;
    }
    for (const auto & item : widget->items)
    {
      ss << widget << " --> " << item.link << std::endl;
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
