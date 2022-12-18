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

#include "stream_loader.hpp"
#include "common/exceptions.hpp"
#include <iostream>
#include <memory>
#include <unordered_map>
#include <utility>
#include <vector>

namespace multi_data_monitor
{

StreamLoader::StreamLoader()
{
}

StreamLoader::Mapping StreamLoader::create(const StreamList & configs)
{
  std::unordered_map<StreamLink, Stream> mapping;
  for (const auto & config : configs)
  {
    const auto stream = create_stream(config);
    stream->setting(config->yaml);
    mapping[config] = streams_.emplace_back(stream);
  }
  for (const auto & [config, stream] : mapping)
  {
    if (config->input)
    {
      mapping[config->input]->connect(stream);
    }
  }
  return mapping;
}

Stream StreamLoader::create_stream(const StreamLink config)
{
  if (config->klass == builtin::panel)
  {
    return panels_.emplace_back(std::make_shared<PanelStream>());
  }
  if (config->klass == builtin::topic)
  {
    return topics_.emplace_back(std::make_shared<TopicStream>());
  }
  if (config->klass == builtin::field)
  {
    return std::make_shared<FieldStream>();
  }
  if (config->klass == builtin::print)
  {
    return std::make_shared<PrintStream>();
  }
  throw ConfigError("unknown stream type: " + config->klass);
}

void StreamLoader::release()
{
  // TODO(Takagi, Isamu): check use count
  // Release shared_ptr to unload plugins.
  panels_.clear();
  topics_.clear();
  streams_.clear();
}

}  // namespace multi_data_monitor
