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

#ifndef MULTI_DATA_MONITOR__ERRORS_HPP_
#define MULTI_DATA_MONITOR__ERRORS_HPP_

#include <fmt/format.h>
#include <exception>
#include <string>

namespace multi_data_monitor
{

struct Exception : public std::exception
{
  using std::exception::exception;
  std::string message_;
  const char * what() const noexcept override { return message_.c_str(); }
};

struct ConfigObjectError : public Exception
{
  void set_message(const std::string & field, const std::string & track, const std::string & message)
  {
    message_ = fmt::format("object field '{0}' {2} [{1}]", field, track, message);
  }
};

struct FieldNotFound : public ConfigObjectError
{
  FieldNotFound(const std::string & field, const std::string & track) { set_message(field, track, "is required"); }
};

struct LogicError : public std::logic_error
{
  using std::logic_error::logic_error;
};

struct RuntimeError : public std::runtime_error
{
  using std::runtime_error::runtime_error;
};

struct PluginError : public std::runtime_error
{
  using std::runtime_error::runtime_error;
};

struct ConfigError : public std::runtime_error
{
  using std::runtime_error::runtime_error;

  static ConfigError LabelConflict(const std::string & label, const std::string & scope)
  {
    return ConfigError(scope + " label '" + label + "' is not unique");
  }
  static ConfigError LabelNotFound(const std::string & label, const std::string & scope)
  {
    return ConfigError(scope + " label '" + label + "' is not found");
  }
};

struct FilePathError : public ConfigError
{
  using ConfigError::ConfigError;
};

struct LabelCirculation : public ConfigError
{
  using ConfigError::ConfigError;
};

struct GraphCirculation : public ConfigError
{
  using ConfigError::ConfigError;
};

struct GraphIsNotTree : public ConfigError
{
  using ConfigError::ConfigError;
};

}  // namespace multi_data_monitor

#endif  // MULTI_DATA_MONITOR__ERRORS_HPP_
