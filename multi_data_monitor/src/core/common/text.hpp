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

#ifndef CORE__COMMON__TEXT_HPP_
#define CORE__COMMON__TEXT_HPP_

#include <string>

namespace multi_data_monitor::text
{

template <class Iterable>
std::string join(const Iterable & iterable, const std::string & delimiter)
{
  const auto head = iterable.begin();
  const auto tail = iterable.end();
  std::string result;
  for (auto iter = head; iter != tail; ++iter)
  {
    result += (iter != head ? delimiter : "") + (*iter);
  }
  return result;
}

}  // namespace multi_data_monitor::text

#endif  // CORE__COMMON__TEXT_HPP_
