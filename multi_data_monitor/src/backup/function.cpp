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

#include "function.hpp"
#include "style.hpp"

// precision
#include <sstream>
#include <iomanip>

// units
#include <cmath>

// debug
#include <iostream>

// lines
#include <algorithm>


std::unique_ptr<BaseFunction> CreateFunction(const YAML::Node & rule)
{
  const auto func = rule["func"].as<std::string>();
  if (func == "switch")
  {
    return std::make_unique<SwitchFunction>(rule);
  }
  if (func == "precision")
  {
    return std::make_unique<PrecisionFunction>(rule);
  }
  if (func == "units")
  {
    return std::make_unique<UnitsFunction>(rule);
  }
  if (func == "lines")
  {
    return std::make_unique<LinesFunction>(rule);
  }
  std::cout << "unknown function" << std::endl;
  return nullptr;
}

void FunctionRules::Load(const YAML::Node & rules)
{
  // TODO: check reload
  if (!rules) { return; }

  for (const auto & rule : rules)
  {
    functions_.push_back(CreateFunction(rule));
  }
}

FunctionResult FunctionRules::Apply(const FunctionResult & base) const
{
  FunctionResult result = base;
  for (const auto & function : functions_)
  {
    result = function->Apply(result);
  }
  return result;
}

FunctionResult BaseFunction::Apply(const FunctionResult & base, const FunctionResult & input)
{
  return FunctionResult{input.value ? input.value : base.value, base.style.Merge(input.style)};
}

SwitchFunction::SwitchFunction(const YAML::Node & yaml)
{
  for (const auto & node : yaml["mapping"])
  {
    const auto key = node.first.as<std::string>();
    mapping_.insert(std::make_pair(key, FunctionResult{node.second["value"], node.second["style"]}));
  }
  if (yaml["default"])
  {
    default_ = FunctionResult{yaml["default"]["value"], yaml["default"]["style"]};
  }
}

FunctionResult SwitchFunction::Apply(const FunctionResult & base) const
{
  const auto iter = mapping_.find(base.value.as<std::string>());
  if (iter != mapping_.end())
  {
    return BaseFunction::Apply(base, iter->second);
  }
  return default_ ? BaseFunction::Apply(base, default_.value()) : base;
}

PrecisionFunction::PrecisionFunction(const YAML::Node & yaml)
{
  precision_ = yaml["args"].as<int>();
}

FunctionResult PrecisionFunction::Apply(const FunctionResult & base) const
{
  std::stringstream ss;
  ss << std::fixed << std::setprecision(precision_) << base.value.as<double>();
  return FunctionResult{YAML::Node(ss.str()), base.style};
}

UnitsFunction::UnitsFunction(const YAML::Node & yaml)
{
  const auto type = yaml["args"].as<std::string>();

  if (type == "mps_to_kph") { coefficient_ = 1.0 * 3.6; return; }
  if (type == "kph_to_mps") { coefficient_ = 1.0 / 3.6; return; }
  if (type == "deg_to_rad") { coefficient_ = M_PI / 180.0; return; }
  if (type == "rad_to_deg") { coefficient_ = 180.0 / M_PI; return; }
  std::cout << "unknown units" << std::endl;

  coefficient_ = 1.0;
}

FunctionResult UnitsFunction::Apply(const FunctionResult & base) const
{
  double value = coefficient_ * base.value.as<double>();
  return FunctionResult{YAML::Node(value), base.style};
}

LinesFunction::LinesFunction(const YAML::Node & yaml)
{
  lines_ = yaml["args"].as<int>();
}

FunctionResult LinesFunction::Apply(const FunctionResult & base) const
{
  const auto value = base.value.as<std::string>();
  const auto count = std::count(value.begin(), value.end(), '\n');
  const auto lines = std::string(std::max(0L, lines_ - count - 1), '\n');
  return FunctionResult{YAML::Node(value + lines), base.style};
}
