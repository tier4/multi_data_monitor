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

void load(const std::string & input)
{
  auto step0 = multi_data_monitor::ConfigLoader();
  auto data0 = step0(input);

  auto step1a = multi_data_monitor::ConstructSubscriptions();
  auto step1b = multi_data_monitor::ConstructStream();
  auto data1a = step1a(data0);
  auto data1b = step1b(data0);

  auto data1 = multi_data_monitor::StreamList();
  data1.insert(data1.end(), data1a.begin(), data1a.end());
  data1.insert(data1.end(), data1b.begin(), data1b.end());

  // auto step2 = multi_data_monitor::NodeTransformer();
  // auto step3 = multi_data_monitor::InterfaceHandler();
  // auto step4 = multi_data_monitor::ResolveConnection();

  // const auto data2 = step2(data1);
  // const auto data3 = step3(data2);

  const auto & debug1 = data1;
  const auto & debug2 = data1b;

  std::cout << "========================================" << std::endl;
  for (const auto & stream : debug1)
  {
    stream->dump();
  }
  std::cout << "========================================" << std::endl;
  for (const auto & stream : debug2)
  {
    stream->dump();
  }
  std::cout << "========================================" << std::endl;

  auto diagram = multi_data_monitor::plantuml::Diagram();
  diagram.write(debug1, "diagram1.plantuml");
  diagram.write(debug2, "diagram2.plantuml");
}

int main(int argc, char ** argv)
{
  if (argc != 3)
  {
    std::cerr << "usage: command <scheme> <config-file-path>" << std::endl;
    return 1;
  }

  const auto scheme = std::string(argv[1]);
  const auto config = std::string(argv[2]);
  load(scheme + "://" + config);
}
