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

std::string Diagram::convert(const StreamList & streams)
{
  std::ostringstream ss;
  ss << "@startuml debug" << std::endl;

  for (const auto & stream : streams)
  {
    ss << "card " << stream << " [" << std::endl;
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

  for (const auto & stream : streams)
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

  ss << "@enduml" << std::endl;
  return ss.str();
}

void Diagram::write(const StreamList & input, const std::string & path)
{
  std::ofstream ofs(path);
  ofs << convert(input) << std::endl;
}

}  // namespace multi_data_monitor::plantuml
