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

#include "stream.hpp"
#include "errors.hpp"

// clang-format off
#include <iostream>
#include <fmt/format.h>
using std::cout;
using std::endl;
// clang-format on

namespace multi_data_monitor
{

void Stream::Register([[maybe_unused]] Stream * stream)
{
  throw LogicError("Stream::Register");
}

void OutputStream::Callback(const YAML::Node & yaml)
{
  for (auto & stream : streams_)
  {
    stream->Callback(yaml);
  }
}

void OutputStream::Register(Stream * stream)
{
  streams_.push_back(stream);
}

void WidgetStream::Callback(const YAML::Node & yaml)
{
  cout << "========== widget ==========" << endl;
  cout << yaml << endl;
}

void FilterStream::Callback(const YAML::Node & yaml)
{
  cout << "========== filter ==========" << endl;
  cout << yaml << endl;
}

}  // namespace multi_data_monitor
