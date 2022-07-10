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

#include "generic_type_support/generic_type_support.hpp"
#include "impl/library.hpp"
#include "impl/message.hpp"
#include "impl/convert.hpp"

#include <yaml-cpp/yaml.h>
#include <iostream>

int main()
{
  using namespace generic_type_support;
  using namespace std;

  {
    generic_type_support::GenericMessage message("std_msgs/msg/Header");
    cout << message.GetTypeName() << endl;

    const auto field1 = message.GetField("frame_id");
    const auto field2 = message.GetField("stamp.sec");

    cout << "END" << endl;
  }
}
