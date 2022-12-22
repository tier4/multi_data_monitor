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
#include "config/types.hpp"
#include "stream/apply.hpp"
#include "stream/field.hpp"
#include "stream/panel.hpp"
#include "stream/print.hpp"
#include "stream/topic.hpp"
#include <memory>
#include <unordered_map>
#include <utility>
#include <vector>

// DEBUG
#include <iostream>

namespace multi_data_monitor
{

StreamLoader::StreamLoader()
{
}

StreamMaps StreamLoader::create(const StreamList & configs, const FilterMaps & filters)
{
  return create(configs, filters, WidgetMaps());
}

StreamMaps StreamLoader::create(const StreamList & configs, const FilterMaps & filters, const WidgetMaps & widgets)
{
  StreamMaps mapping;
  for (const auto & config : configs)
  {
    const auto stream = create_stream(config, filters, widgets);
    mapping[config] = streams_.emplace_back(stream);
    stream->setting(config->yaml);
  }
  for (const auto & [config, stream] : mapping)
  {
    for (const auto & item : config->items)
    {
      mapping[item]->connect(stream);
    }
  }
  return mapping;
}

Stream StreamLoader::create_stream(const StreamLink & config, const FilterMaps & filters, const WidgetMaps & widgets)
{
  if (config->klass == builtin::topic)
  {
    const auto stream = std::make_shared<TopicStream>();
    return topics_.emplace_back(stream);
  }
  if (config->klass == builtin::apply)
  {
    const auto filter = filters.get(config->apply, nullptr);
    return std::make_shared<ApplyStream>(filter);
  }
  if (config->klass == builtin::panel)
  {
    const auto widget = widgets.get(config->panel, nullptr);
    return std::make_shared<PanelStream>(widget);
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
  topics_.clear();
  streams_.clear();
}

}  // namespace multi_data_monitor
