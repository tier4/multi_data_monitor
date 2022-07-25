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

#include <generic_type_support/generic_type_support.hpp>
#include <iostream>

int main()
{
  using GenericMessage = generic_type_support::GenericMessage;
  using FieldError = generic_type_support::FieldError;

  try
  {
    const auto message_ = std::make_shared<GenericMessage>("std_msgs/msg/Header");
    const auto access_ = message_->GetAccess("stamp.msec");
  }
  catch (const FieldError & error)
  {
    std::cout << "ERROR: " << error.what() << std::endl;
  }
}
