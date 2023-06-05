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

#include "file.hpp"
#include "common/path.hpp"
#include <multi_data_monitor/config.hpp>
#include <multi_data_monitor/errors.hpp>
#include <filesystem>
#include <string>

namespace multi_data_monitor
{

ConfigFile ConfigFileLoader::execute(const std::string & input)
{
  const auto path = std::filesystem::path(path::resolve(input));
  if (!std::filesystem::exists(path))
  {
    throw FilePathError("config file not found '" + path.string() + "'");
  }

  // TODO(Takagi, Isamu): handle yaml error
  ConfigObject object(YAML::LoadFile(path), "config-file");

  // Check version.
  const auto version = object.take_required_data<std::string>("version");
  if (version != "2.0")
  {
    throw ConfigError("not supported version '" + version + "'");
  }
  return ConfigFile{version, object.yaml};
}

}  // namespace multi_data_monitor
