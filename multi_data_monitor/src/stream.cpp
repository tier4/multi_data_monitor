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
#include <multi_data_monitor/action.hpp>
#include <multi_data_monitor/design.hpp>

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

void OutputStream::Callback(const MonitorValues & input)
{
  for (auto & stream : streams_)
  {
    stream->Callback(input);
  }
}

void OutputStream::Register(Stream * stream)
{
  streams_.push_back(stream);
}

WidgetStream::WidgetStream(Design * design)
{
  design_ = design;
}

void WidgetStream::Callback(const MonitorValues & input)
{
  // cout << "========== widget ==========" << endl;
  // cout << input.value << endl;

  design_->Callback(input);
}

FilterStream::FilterStream(std::vector<std::unique_ptr<Action>> && actions)
{
  actions_.swap(actions);
}

FilterStream::~FilterStream()
{
  // because of the forward declaration
}

void FilterStream::Callback(const MonitorValues & input)
{
  // cout << "========== filter ==========" << endl;
  // cout << input.value << endl;

  MonitorValues output = input.Clone();
  for (const auto & action : actions_)
  {
    output = action->Apply(output);
  }
  OutputStream::Callback(output);
}

}  // namespace multi_data_monitor
