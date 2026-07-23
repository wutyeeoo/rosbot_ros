// Copyright 2024 Husarion sp. z o.o.
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

// Modifications Copyright (c) 2026 [Wut Yee Oo]

#ifndef ROSBOT_HARDWARE_INTERFACES_ROBOT_SYSTEM_GPIO_TYPES_HPP_
#define ROSBOT_HARDWARE_INTERFACES_ROBOT_SYSTEM_GPIO_TYPES_HPP_

#include <map>
#include <string>

#include <gpiod.hpp>

namespace rosbot_hardware_interfaces
{

/**
 * @brief Enumeration representing available GPIO pins in the Panther system.
 */
enum class GPIOPin {
  E_STOP_RESET,
  WATCHDOG
};

/**
 * @brief Mapping of GPIO pins to their respective names.
 */
const std::map<GPIOPin, std::string> pin_names_{
  {GPIOPin::WATCHDOG, "WATCHDOG"},
  {GPIOPin::E_STOP_RESET, "E_STOP_RESET"},
};

/**
 * @brief Structure containing information related to GPIO pins such as pin configuration,
 * direction, value, etc. This information is required during the initialization process.
 */
struct GPIOInfo
{
  GPIOPin pin;
  gpiod::line::direction direction;
  bool active_low = false;
  gpiod::line::value init_value = gpiod::line::value::INACTIVE;
  gpiod::line::value value = gpiod::line::value::INACTIVE;
  gpiod::line::offset offset = -1;
};

}  // namespace rosbot_hardware_interfaces

#endif  // ROSBOT_HARDWARE_INTERFACES_ROBOT_SYSTEM_GPIO_TYPES_HPP_
