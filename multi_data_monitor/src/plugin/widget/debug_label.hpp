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

#ifndef PLUGIN__WIDGET__DEBUG_LABEL_HPP_
#define PLUGIN__WIDGET__DEBUG_LABEL_HPP_

#include <QLabel>
#include <iostream>

namespace multi_data_monitor
{

class DebugLabel : public QLabel
{
  Q_OBJECT

public:
  static inline int counter = 0;
  int count;

  explicit DebugLabel(const char * text) : QLabel(QString(text)), count(++counter)
  {
    std::cerr << "debug label created: " << count << std::endl;
  }

  ~DebugLabel() { std::cerr << "debug label removed: " << count << std::endl; }
};

}  // namespace multi_data_monitor

#endif  // PLUGIN__WIDGET__DEBUG_LABEL_HPP_
