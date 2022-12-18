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

#ifndef CORE__COMMON__TYPEDEF_HPP_
#define CORE__COMMON__TYPEDEF_HPP_

#include <memory>
#include <vector>

namespace multi_data_monitor
{

struct StreamConfig;
struct WidgetConfig;
using StreamData = std::shared_ptr<StreamConfig>;
using WidgetData = std::shared_ptr<WidgetConfig>;
using StreamDataList = std::vector<StreamData>;
using WidgetDataList = std::vector<WidgetData>;

struct BasicStream;
struct BasicWidget;
using StreamNode = std::shared_ptr<BasicStream>;
using WidgetNode = std::shared_ptr<BasicWidget>;
using StreamNodeList = std::vector<StreamNode>;
using WidgetNodeList = std::vector<WidgetNode>;

}  // namespace multi_data_monitor

#endif  // CORE__COMMON__TYPEDEF_HPP_
