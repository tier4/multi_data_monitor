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

#include "parser.hpp"
#include "common/exceptions.hpp"
#include "common/yaml.hpp"
#include "types.hpp"
#include <iostream>
#include <memory>

// temp
#include <vector>

namespace multi_data_monitor
{

StreamList CheckSpecialClass::operator()(const StreamList & input)
{
  for (const auto & stream : input)
  {
    const auto & klass = stream->klass;
    if (!klass.empty() && klass[0] == '@')
    {
      throw ConfigError("class name is reserved: " + klass);
    }
  }
  return input;
}

StreamList InterfaceHandler::operator()(const StreamList & input)
{
  for (const auto & stream : input)
  {
    output_.push_back(stream);
  }
  return output_;
}

StreamList ResolveConnection::operator()(const StreamList & input)
{
  for (const auto & stream : input)
  {
    if (stream->input)
    {
      stream->input = resolve(stream->input);
    }

    if (stream->refer == nullptr)
    {
      output_.push_back(stream);
    }
  }
  return output_;
}

StreamLink ResolveConnection::resolve(const StreamLink & stream)
{
  return stream->refer ? resolve(stream->refer) : stream;
}

}  // namespace multi_data_monitor
