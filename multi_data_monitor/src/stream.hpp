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

#ifndef STREAM_HPP_
#define STREAM_HPP_

#include <yaml-cpp/yaml.h>
#include <vector>

namespace multi_data_monitor
{

struct Stream
{
public:
  virtual ~Stream() = default;
  virtual void Callback(const YAML::Node & yaml) = 0;
  virtual void Register(Stream * stream);
};

struct OutputStream : public Stream
{
public:
  void Callback(const YAML::Node & yaml) override;
  void Register(Stream * stream) override;

protected:
  std::vector<Stream *> streams_;
};

class WidgetStream : public Stream
{
public:
  void Callback(const YAML::Node & yaml) override;

private:
  // widget
};

class FilterStream : public OutputStream
{
public:
  void Callback(const YAML::Node & yaml) override;

private:
  // rules
};

}  // namespace multi_data_monitor

#endif  // STREAM_HPP_
