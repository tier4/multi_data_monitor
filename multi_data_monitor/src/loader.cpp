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

#include "config/parser.hpp"
#include "debug/plantuml.hpp"
#include <iostream>

int main(int argc, char ** argv)
{
  namespace mdm = multi_data_monitor;
  if (argc != 3)
  {
    std::cerr << "usage: command <scheme> <config-file-path>" << std::endl;
    return 1;
  }

  const auto scheme = std::string(argv[1]);
  const auto config = std::string(argv[2]);

  auto step1 = mdm::ConfigLoader();
  auto step2 = mdm::NodeConstructor();
  auto step3 = mdm::NodeTransformer();
  auto step4 = mdm::InterfaceHandler();

  const auto data0 = scheme + "://" + config;
  const auto data1 = step1(data0);
  const auto data2 = step2(data1);
  const auto data3 = step3(data2);
  const auto data4 = step4(data3);

  std::cout << "========================================" << std::endl;
  for (const auto & stream : data3.streams)
  {
    stream->dump();
  }
  std::cout << "========================================" << std::endl;
  for (const auto & stream : data4.streams)
  {
    stream->dump();
  }
  std::cout << "========================================" << std::endl;

  auto diagram = mdm::plantuml::Diagram();
  diagram.write(data3, "diagram1.plantuml");
  diagram.write(data4, "diagram2.plantuml");
}
