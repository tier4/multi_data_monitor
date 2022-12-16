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
#include <utility>
#include <vector>

namespace multi_data_monitor
{

StreamLoader::StreamLoader(const StreamList & streams)
{
  std::vector<std::pair<Stream, StreamLink>> stream_mapping_;
  for (const auto & data : streams)
  {
    const auto node = create_stream(data);
    streams_.push_back(node);
    stream_mapping_.push_back({node, data});
  }

  for (const auto & [node, link] : stream_mapping_)
  {
    std::cout << link->klass << ": " << node << std::endl;

    Packet packet;
    packet.value = YAML::Node("aaaaaaaaaaaaaaaa");
    node->message(packet);
  }
}

Stream StreamLoader::create_stream(const StreamLink data)
{
  if (data->klass == builtin::panel)
  {
    return panels_.emplace_back(std::make_shared<PanelStream>());
  }
  if (data->klass == builtin::topic)
  {
    return topics_.emplace_back(std::make_shared<TopicStream>());
  }
  if (data->klass == builtin::field)
  {
    return std::make_shared<FieldStream>();
  }
  if (data->klass == builtin::debug)
  {
    return std::make_shared<DebugStream>();
  }
  throw ConfigError("unknown stream type: " + data->klass);
}

}  // namespace multi_data_monitor
