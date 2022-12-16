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
#include "loader/stream_loader.hpp"
#include "parser/subscription.hpp"
#include <iostream>

namespace multi_data_monitor
{

StreamList load(const std::string & input)
{
  auto diagram = plantuml::Diagram();

  auto step0 = ConfigLoader();
  auto data0 = step0(input);

  auto step1a = ConstructSubscription();
  auto step1b = ConstructStream();
  auto data1a = step1a(data0);
  auto data1b = step1b(data0);

  auto data1 = StreamList();
  data1.insert(data1.end(), data1a.begin(), data1a.end());
  data1.insert(data1.end(), data1b.begin(), data1b.end());

  auto step2 = CheckSpecialClass();
  auto step3 = MergeSubscription();
  auto step4 = ResolveConnection();

  const auto data2 = step2(data1);
  const auto data3 = step3(data2);
  diagram.write(data3, "diagram1.plantuml");
  const auto data4 = step4(data3);
  diagram.write(data4, "diagram2.plantuml");

  return data4;
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

  auto streams = multi_data_monitor::load(scheme + "://" + config);

  auto loader = multi_data_monitor::StreamLoader(streams);
  (void)loader;
}
