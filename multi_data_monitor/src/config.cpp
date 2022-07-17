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

#include "config.hpp"
#include "errors.hpp"
#include <ament_index_cpp/get_package_prefix.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <yaml-cpp/yaml.h>
#include <string>
#include <filesystem>

#include <iostream>
using namespace std;

namespace multi_data_monitor
{

YAML::Node LoadFile(const std::string & package, const std::string & source)
{
  try
  {
    auto path = std::filesystem::path();
    if (!package.empty())
    {
      path.append(ament_index_cpp::get_package_share_directory(package));
    }
    path.append(source);

    if (std::filesystem::exists(path))
    {
      return YAML::LoadFile(path);
    }
    throw ConfigError::LoadFile("file not found: " + path.string());
  }
  catch(const ament_index_cpp::PackageNotFoundError & error)
  {
    throw ConfigError::LoadFile("package not found: " + package);
  }
  catch(YAML::Exception & error)
  {
    throw ConfigError::LoadFile(error.what());
  }
}

ConfigFile::ConfigFile(const std::string & package, const std::string & path)
{
  const auto yaml = LoadFile(package, path);
  cout << yaml << endl;

  for (const auto & monitor : yaml["monitors"])
  {

  }
}

}  // namespace multi_data_monitor
