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

#include <multi_data_monitor/filter.hpp>
#include <functional>
#include <variant>

//
#include <iostream>

namespace multi_data_monitor
{

struct SetIf : public BasicFilter
{
public:
  void setup(YAML::Node yaml) override;
  Packet apply(const Packet & packet) override;

private:
  template <class T>
  using ExprList = std::vector<std::function<bool(T)>>;
  template <class T, template <class> class F>
  static void init_expr_func(ExprList<T> & expr_list, YAML::Node & yaml, const std::string & op);
  template <class T>
  static void init_expr_list(ExprList<T> & expr_list, YAML::Node & yaml);
  template <class T>
  static bool eval_expr_list(ExprList<T> & expr_list, const T & value);

  using TextType = std::string;
  using SintType = int64_t;
  using UintType = uint64_t;
  using RealType = double;
  using TextExpr = ExprList<TextType>;
  using SintExpr = ExprList<SintType>;
  using UintExpr = ExprList<UintType>;
  using RealExpr = ExprList<RealType>;
  using Expr = std::variant<TextExpr, SintExpr, UintExpr, RealExpr>;
  struct InitExpr;
  struct EvalExpr;

  struct PacketAction
  {
    YAML::Node value;
    std::unordered_map<std::string, std::string> attrs;
  };

  Expr expr_;
  PacketAction action_;
};

template <class T, template <class> class F>
void SetIf::init_expr_func(ExprList<T> & expr_list, YAML::Node & yaml, const std::string & op)
{
  if (yaml[op])
  {
    const auto operand = yaml[op].as<T>();
    expr_list.push_back(std::bind(F<T>(), std::placeholders::_1, operand));
    std::cout << op << " " << operand << std::endl;
  }
}

template <class T>
void SetIf::init_expr_list(ExprList<T> & expr_list, YAML::Node & yaml)
{
  // clang-format off
  init_expr_func<T, std::equal_to     >(expr_list, yaml, "eq");
  init_expr_func<T, std::not_equal_to >(expr_list, yaml, "ne");
  init_expr_func<T, std::less         >(expr_list, yaml, "lt");
  init_expr_func<T, std::less_equal   >(expr_list, yaml, "le");
  init_expr_func<T, std::greater      >(expr_list, yaml, "gt");
  init_expr_func<T, std::greater_equal>(expr_list, yaml, "ge");
  // clang-format on
}

template <class T>
bool SetIf::eval_expr_list(ExprList<T> & expr_list, const T & value)
{
  for (const auto & expr : expr_list)
  {
    if (!expr(value)) return false;
  }
  return true;
}

struct SetIf::InitExpr
{
public:
  explicit InitExpr(YAML::Node yaml) : yaml_(yaml) {}
  void operator()(TextExpr & expr) { return init_expr_list(expr, yaml_); }
  void operator()(SintExpr & expr) { return init_expr_list(expr, yaml_); }
  void operator()(UintExpr & expr) { return init_expr_list(expr, yaml_); }
  void operator()(RealExpr & expr) { return init_expr_list(expr, yaml_); }

private:
  YAML::Node yaml_;
};

struct SetIf::EvalExpr
{
public:
  explicit EvalExpr(YAML::Node yaml) : yaml_(yaml) {}
  bool operator()(TextExpr & expr) { return eval_expr_list(expr, yaml_.as<TextType>()); }
  bool operator()(SintExpr & expr) { return eval_expr_list(expr, yaml_.as<SintType>()); }
  bool operator()(UintExpr & expr) { return eval_expr_list(expr, yaml_.as<UintType>()); }
  bool operator()(RealExpr & expr) { return eval_expr_list(expr, yaml_.as<RealType>()); }

private:
  YAML::Node yaml_;
};

void SetIf::setup(YAML::Node yaml)
{
  const auto get_expr_type = [](const std::string & type) -> Expr
  {
    if (type == "text") return TextExpr();
    if (type == "uint") return UintExpr();
    if (type == "sint") return SintExpr();
    if (type == "real") return RealExpr();
    // TODO(Takagi, Isamu): error handling
    std::cerr << "SetIf: unknown type: " << type << std::endl;
    return TextExpr();
  };

  const auto get_packet_action = [](const YAML::Node & yaml)
  {
    PacketAction action;
    if (yaml["value"])
    {
      action.value.reset(yaml["value"]);
    }
    if (yaml["attrs"])
    {
      for (const auto & pair : yaml["attrs"])
      {
        const auto name = pair.first.as<std::string>();
        const auto attr = pair.second.as<std::string>();
        action.attrs[name] = attr;
      }
    }
    return action;
  };

  action_ = get_packet_action(yaml);
  expr_ = get_expr_type(yaml["type"].as<std::string>("string"));
  std::visit(InitExpr(yaml), expr_);
}

Packet SetIf::apply(const Packet & packet)
{
  bool result = std::visit(EvalExpr(packet.value), expr_);
  if (result)
  {
    YAML::Node value = action_.value ? action_.value : packet.value;
    Packet::Attrs attrs = action_.attrs;
    for (const auto & pair : packet.attrs)
    {
      attrs.insert(pair);
    }
    return {value, attrs};
  }
  return packet;
}

}  // namespace multi_data_monitor

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(multi_data_monitor::SetIf, multi_data_monitor::BasicFilter)
