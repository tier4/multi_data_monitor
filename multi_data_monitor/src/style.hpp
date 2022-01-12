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

#ifndef STYLE_HPP_
#define STYLE_HPP_

#include <yaml-cpp/yaml.h>
#include <string>

class StyleDefinition
{
public:
  StyleDefinition();
  StyleDefinition(const YAML::Node & yaml);
  std::string GetStyleSheet() const;
  StyleDefinition Merge(const StyleDefinition & input) const;

private:
  int font_size;
  std::string font_color;
  std::string back_color;
};

class StyleReference
{

};

class StyleVariant
{

};

class StyleDictionary
{

};

#endif  // STYLE_HPP_
