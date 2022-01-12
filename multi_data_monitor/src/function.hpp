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

#ifndef FUNCTION_HPP_
#define FUNCTION_HPP_

#include "style.hpp"
#include <yaml-cpp/yaml.h>

#include <optional>  // switch function

struct FunctionResult final
{
  YAML::Node value;
  StyleDefinition style;
};

class BaseFunction
{
public:
  virtual ~BaseFunction() = default;
  virtual FunctionResult Apply(const FunctionResult & base) const = 0;

protected:
  static FunctionResult Apply(const FunctionResult & base, const FunctionResult & input);
};

class FunctionRules final
{
public:
  void Load(const YAML::Node & yaml);
  FunctionResult Apply(const FunctionResult & base) const;

private:
  std::vector<std::unique_ptr<BaseFunction>> functions_;
};

class SwitchFunction : public BaseFunction
{
public:
  SwitchFunction(const YAML::Node & yaml);
  FunctionResult Apply(const FunctionResult & base) const override;

private:
  std::map<std::string, FunctionResult> mapping_;
  std::optional<FunctionResult> default_;
};

class PrecisionFunction : public BaseFunction
{
public:
  PrecisionFunction(const YAML::Node & yaml);
  FunctionResult Apply(const FunctionResult & base) const override;

private:
  int precision_;
};

class UnitsFunction : public BaseFunction
{
public:
  UnitsFunction(const YAML::Node & yaml);
  FunctionResult Apply(const FunctionResult & base) const override;

private:
  double coefficient_;
};

class LinesFunction : public BaseFunction
{
public:
  LinesFunction(const YAML::Node & yaml);
  FunctionResult Apply(const FunctionResult & base) const override;

private:
  int lines_;
};

#endif  // FUNCTION_HPP_
