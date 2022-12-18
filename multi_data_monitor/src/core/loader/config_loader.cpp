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

#include "config_loader.hpp"
#include "parser/check_system_class.hpp"
#include "parser/construction.hpp"
#include "parser/file.hpp"
#include "parser/resolve_relation.hpp"
#include "parser/subscription.hpp"

namespace multi_data_monitor
{

ConfigLoader::ConfigLoader()
{
  parsers_.push_back(std::make_shared<MergeSubscription>());
  parsers_.push_back(std::make_shared<CheckSystemClass>());
  parsers_.push_back(std::make_shared<ConnectRelation>());
  parsers_.push_back(std::make_shared<ResolveRelation>());
  parsers_.push_back(std::make_shared<ReleaseRelation>());
}

ConfigData ConfigLoader::execute(const std::string & path) const
{
  return partial_execute(partial_construct(path));
}

ConfigData ConfigLoader::partial_construct(const std::string & path) const
{
  auto file = ConfigFileLoader().execute(path);
  auto data = ParseBasicObject().execute(file);
  return data;
}

ConfigData ConfigLoader::partial_execute(const ConfigData & data) const
{
  ConfigData result = data;
  for (const auto & parser : parsers_)
  {
    result = parser->execute(result);
  }
  return result;
}

}  // namespace multi_data_monitor
