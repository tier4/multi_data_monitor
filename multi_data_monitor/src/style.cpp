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

#include "style.hpp"

#include <iostream>

StyleDefinition::StyleDefinition()
{
  font_size = 0;
  font_color = "";
  back_color = "";
}

StyleDefinition::StyleDefinition(const YAML::Node & yaml) : StyleDefinition()
{
  if (!yaml) { return; }
  font_size = yaml["font-size"].as<int>(0);
  font_color = yaml["font-color"].as<std::string>("");
  back_color = yaml["back-color"].as<std::string>("");
}

std::string StyleDefinition::GetStyleSheet() const
{
  std::string style_sheet;

  if (font_size != 0)
  {
    style_sheet += "font-size: " + std::to_string(font_size) + "px;";
  }
  if (font_color != "")
  {
    style_sheet += "color: " + font_color + ";";
  }
  if (back_color != "")
  {
    style_sheet += "background-color: " + back_color + ";";
  }
  return style_sheet;
}

StyleDefinition StyleDefinition::Merge(const StyleDefinition & input) const
{
  StyleDefinition style = *this;

  if (input.font_size != 0)
  {
    style.font_size = input.font_size;
  }
  if (input.font_color != "")
  {
    style.font_color = input.font_color;
  }
  if (input.back_color != "")
  {
    style.back_color = input.back_color;
  }
  return style;
}
