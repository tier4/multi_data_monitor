// Copyright 2021 Takagi, Isamu
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
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <string>

namespace monitors
{

std::pair<std::string, size_t> ParseElement(const std::string & path, size_t & base)
{
  const size_t pos1 = path.find('(', base);
  const size_t pos2 = path.find(')', base);
  if (pos1 != base + 1 || pos2 == std::string::npos)
  {
    return {"$", 1};
  }
  const auto expr = path.substr(base + 2, pos2 - base - 2);
  if (expr.substr(0, 15) != "find-pkg-share ")
  {
    return {"$", 1};
  }
  return {ament_index_cpp::get_package_share_directory(expr.substr(15)), pos2 - base + 1};
}

std::string ParsePath(const std::string & path)
{
  std::string parsed;
  size_t base = 0;
  while (true)
  {
    const size_t pos = path.find('$', base);
    parsed += path.substr(base, pos - base);
    if (pos == std::string::npos)
    {
      break;
    }
    const auto [str, len] = ParseElement(path, base);
    parsed += str;
    base = pos + len;
  }
  return parsed;
}

}  // namespace monitors
