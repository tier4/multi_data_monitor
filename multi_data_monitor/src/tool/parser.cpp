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

#include "debug/plantuml.hpp"
#include "loader/config_loader.hpp"
#include <iostream>

namespace multi_data_monitor
{

ConfigData load(const std::string & path)
{
  const auto diagram = plantuml::Diagram();
  const auto loader = ConfigLoader();
  const auto parsers = loader.parsers();

  auto data = loader.partial_construct(path);
  diagram.write(data, "graphs/step0-partial-construct.plantuml");
  for (size_t i = 0; i < parsers.size(); ++i)
  {
    const auto filename = std::to_string(i + 1) + "-" + parsers[i]->name();
    data = parsers[i]->execute(data);
    diagram.write(data, "graphs/step" + filename + ".plantuml");
  }
  return data;
}

}  // namespace multi_data_monitor

int main(int argc, char ** argv)
{
  if (argc != 3)
  {
    std::cerr << "usage: command <scheme> <config-file-path>" << std::endl;
    return 1;
  }

  const auto scheme = std::string(argv[1]);
  const auto config = std::string(argv[2]);
  {
    const auto data = multi_data_monitor::load(scheme + "://" + config);
    std::cout << multi_data_monitor::CommonData::created << std::endl;
    std::cout << multi_data_monitor::CommonData::removed << std::endl;
  }
  std::cout << multi_data_monitor::CommonData::created << std::endl;
  std::cout << multi_data_monitor::CommonData::removed << std::endl;
}
