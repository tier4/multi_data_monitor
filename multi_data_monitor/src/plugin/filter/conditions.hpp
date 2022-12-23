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

#ifndef PLUGIN__FILTER__CONDITIONS_HPP_
#define PLUGIN__FILTER__CONDITIONS_HPP_

#include <yaml-cpp/yaml.h>
#include <functional>
#include <string>
#include <variant>
#include <vector>

//
#include <iostream>

namespace multi_data_monitor::conditions
{

template <class T>
using TermType = std::function<bool(T)>;
template <class T>
using ExprType = std::vector<TermType<T>>;

using TextType = std::string;
using SintType = int64_t;
using UintType = uint64_t;
using RealType = double;
using TextExpr = ExprType<TextType>;
using SintExpr = ExprType<SintType>;
using UintExpr = ExprType<UintType>;
using RealExpr = ExprType<RealType>;
using Expr = std::variant<TextExpr, SintExpr, UintExpr, RealExpr>;

template <class T, template <class> class F>
void make_term(ExprType<T> & expr, const YAML::Node & yaml, const std::string & op)
{
  if (yaml[op])
  {
    const auto operand = yaml[op].as<T>();
    expr.push_back(std::bind(F<T>(), std::placeholders::_1, operand));
    std::cout << op << " " << operand << std::endl;
  }
}

template <class T>
void make_expr(ExprType<T> & expr, const YAML::Node & yaml)
{
  // clang-format off
  make_term<T, std::equal_to     >(expr, yaml, "eq");
  make_term<T, std::not_equal_to >(expr, yaml, "ne");
  make_term<T, std::less         >(expr, yaml, "lt");
  make_term<T, std::less_equal   >(expr, yaml, "le");
  make_term<T, std::greater      >(expr, yaml, "gt");
  make_term<T, std::greater_equal>(expr, yaml, "ge");
  // clang-format on
}

template <class T>
bool eval_expr(const ExprType<T> & expr, const T & value)
{
  for (const auto & expr : expr)
  {
    if (!expr(value)) return false;
  }
  return true;
}

Expr init_expr(const YAML::Node & yaml)
{
  const auto type = yaml["type"].as<std::string>("undefined");
  if (type == "text") return TextExpr();
  if (type == "uint") return UintExpr();
  if (type == "sint") return SintExpr();
  if (type == "real") return RealExpr();
  // TODO(Takagi, Isamu): error handling
  std::cerr << "SetIf: unknown type: " << type << std::endl;
  return TextExpr();
};

class MakeExpr
{
public:
  explicit MakeExpr(const YAML::Node & yaml) : yaml_(yaml) {}
  void operator()(TextExpr & expr) { return make_expr(expr, yaml_); }
  void operator()(SintExpr & expr) { return make_expr(expr, yaml_); }
  void operator()(UintExpr & expr) { return make_expr(expr, yaml_); }
  void operator()(RealExpr & expr) { return make_expr(expr, yaml_); }

private:
  YAML::Node yaml_;
};

class EvalExpr
{
public:
  explicit EvalExpr(const YAML::Node & yaml) : yaml_(yaml) {}
  bool operator()(const TextExpr & expr) { return eval_expr(expr, yaml_.as<TextType>()); }
  bool operator()(const SintExpr & expr) { return eval_expr(expr, yaml_.as<SintType>()); }
  bool operator()(const UintExpr & expr) { return eval_expr(expr, yaml_.as<UintType>()); }
  bool operator()(const RealExpr & expr) { return eval_expr(expr, yaml_.as<RealType>()); }

private:
  YAML::Node yaml_;
};

class Condition
{
public:
  explicit Condition(const YAML::Node & yaml);
  bool eval(const YAML::Node & yaml) const;

private:
  Expr expr_;
};

Condition::Condition(const YAML::Node & yaml)
{
  expr_ = init_expr(yaml);
  std::visit(MakeExpr(yaml), expr_);
}

bool Condition::eval(const YAML::Node & yaml) const
{
  return std::visit(EvalExpr(yaml), expr_);
}

}  // namespace multi_data_monitor::conditions

#endif  // PLUGIN__FILTER__CONDITIONS_HPP_
