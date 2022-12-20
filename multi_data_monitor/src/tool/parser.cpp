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
#include "loader/stream_loader.hpp"
#include <iostream>
using namespace multi_data_monitor;  // NOLINT

ConfigData load(const std::string & path)
{
  const auto func = [](int step, const std::string & name, const ConfigData & data)
  {
    const auto diagram = plantuml::Diagram();
    const auto filename = std::to_string(step) + "-" + name;
    diagram.write(data, "graphs/step" + filename + ".plantuml");
  };
  return ConfigLoader::Execute(path, func);
}

int main(int argc, char ** argv)
{
  if (argc != 3)
  {
    std::cerr << "usage: command <scheme> <config-file-path>" << std::endl;
    return 1;
  }

  StreamLoader stream_loader;
  {
    const auto scheme = std::string(argv[1]);
    const auto config = std::string(argv[2]);
    const auto data = load(scheme + "://" + config);
  }

  std::cout << CommonData::created << std::endl;
  std::cout << CommonData::removed << std::endl;
  std::cout << DesignData::created << std::endl;
  std::cout << DesignData::removed << std::endl;
}
