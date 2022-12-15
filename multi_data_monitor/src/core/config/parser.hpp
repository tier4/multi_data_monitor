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

#ifndef CORE__CONFIG__PARSER_HPP_
#define CORE__CONFIG__PARSER_HPP_

#include "common/exceptions.hpp"
#include "types.hpp"
#include <memory>
#include <string>

namespace multi_data_monitor
{

class ConfigLoader
{
public:
  ConfigFile operator()(const std::string & input);

private:
  ConfigFile output_;
};

class ConstructSubscription
{
public:
  StreamList operator()(const ConfigFile & input);

private:
  void parse_subscription(YAML::Node topic);
  StreamList output_;
};

class ConstructStream
{
public:
  StreamList operator()(const ConfigFile & input);

private:
  StreamLink parse_stream_link(YAML::Node yaml);
  StreamLink parse_stream_yaml(YAML::Node yaml);
  StreamLink parse_stream_dict(YAML::Node yaml);
  StreamList output_;
};

class CheckSpecialClass
{
public:
  StreamList operator()(const StreamList & input);
};

class InterfaceHandler
{
public:
  StreamList operator()(const StreamList & input);

private:
  void handle_stream(const StreamLink & stream);
  StreamList output_;
};

class ResolveConnection
{
public:
  StreamList operator()(const StreamList & input);

private:
  StreamLink resolve(const StreamLink & stream);
  StreamList output_;
};

}  // namespace multi_data_monitor

#endif  // CORE__CONFIG__PARSER_HPP_
