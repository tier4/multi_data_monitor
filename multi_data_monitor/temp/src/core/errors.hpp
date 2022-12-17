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

#ifndef CORE__ERRORS_HPP_
#define CORE__ERRORS_HPP_

#include <stdexcept>
#include <string>

namespace multi_data_monitor
{

struct BaseError : public std::runtime_error
{
  using std::runtime_error::runtime_error;
};

struct SystemError : public BaseError
{
  explicit SystemError(const std::string & message);
};

struct ConfigError : public BaseError
{
  explicit ConfigError(const std::string & message);
};

struct LogicError : public BaseError
{
  explicit LogicError(const std::string & message);
};

struct RuntimeError : public BaseError
{
  explicit RuntimeError(const std::string & message);
};

}  // namespace multi_data_monitor

#endif  // CORE__ERRORS_HPP_