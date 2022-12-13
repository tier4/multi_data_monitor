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

#include "errors.hpp"

namespace multi_data_monitor
{

SystemError::SystemError(const std::string & message) : BaseError("Failed to load config (" + message + ").")
{
}

ConfigError::ConfigError(const std::string & message) : BaseError("Failed to parse config (" + message + ").")
{
}

LogicError::LogicError(const std::string & message) : BaseError(message)
{
}

RuntimeError::RuntimeError(const std::string & message) : BaseError(message)
{
}

}  // namespace multi_data_monitor
