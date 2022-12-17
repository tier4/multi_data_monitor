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

#ifndef TOOL__LOADER_HPP_
#define TOOL__LOADER_HPP_

#include "debug/plantuml.hpp"
#include "parser/check_system_class.hpp"
#include "parser/construction.hpp"
#include "parser/file.hpp"
#include "parser/resolve_relation.hpp"
#include "parser/subscription.hpp"
#include <memory>
#include <string>
#include <vector>

namespace multi_data_monitor
{

ConfigData load(const std::string & path)
{
  auto diagram = plantuml::Diagram();
  auto file = ConfigFileLoader().execute(path);
  auto data = ParseBasicObject().execute(file);

  std::vector<std::shared_ptr<ConfigParserInterface>> parsers;
  parsers.push_back(std::make_shared<MergeSubscription>());
  parsers.push_back(std::make_shared<CheckSystemClass>());
  parsers.push_back(std::make_shared<ConnectRelation>());
  parsers.push_back(std::make_shared<ResolveRelation>());
  parsers.push_back(std::make_shared<ReleaseRelation>());

  diagram.write(data, "graphs/step0-parse-basic-object.plantuml");
  for (size_t i = 0; i < parsers.size(); ++i)
  {
    const auto filename = std::to_string(i + 1) + "-" + parsers[i]->name();
    data = parsers[i]->execute(data);
    diagram.write(data, "graphs/step" + filename + ".plantuml");
  }
  return data;
}

}  // namespace multi_data_monitor

#endif  // TOOL__LOADER_HPP_
