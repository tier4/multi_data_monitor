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

#include "types.hpp"
#include <iostream>

namespace multi_data_monitor
{

StreamLink StreamData::Create(const NodeClass & klass, YAML::Node yaml)
{
  const auto data = StreamData{klass, "", yaml, nullptr, nullptr};
  return std::make_shared<StreamData>(data);
}

StreamLink StreamData::Create(const NodeClass & klass, const NodeLabel & label, YAML::Node yaml)
{
  const auto data = StreamData{klass, label, yaml, nullptr, nullptr};
  return std::make_shared<StreamData>(data);
}

void StreamData::dump() const
{
  std::cout << "stream: " << klass << " [" << this << "]" << std::endl;
  if (!label.empty())
  {
    std::cout << " - label: " << label << std::endl;
  }
  if (input)
  {
    std::cout << " - input: " << input << std::endl;
  }
}

}  // namespace multi_data_monitor
