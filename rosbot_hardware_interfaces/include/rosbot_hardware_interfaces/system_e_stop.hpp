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

#ifndef ROSBOT_HARDWARE_INTERFACES_ROBOT_SYSTEM_SYSTEM_E_STOP_HPP_
#define ROSBOT_HARDWARE_INTERFACES_ROBOT_SYSTEM_SYSTEM_E_STOP_HPP_

#include <atomic>
#include <memory>
#include <mutex>

#include "rosbot_hardware_interfaces/gpio_controller.hpp"

namespace rosbot_hardware_interfaces
{

/**
 * @class EStopInterface
 * @brief Abstract base class defining the interface for emergency stop detailed implementations.
 */
class EStopInterface
{
public:
  EStopInterface() {}

  virtual ~EStopInterface() = default;

  virtual bool ReadEStopState() = 0;
  virtual void TriggerEStop() = 0;
  virtual void ResetEStop() = 0;
};

/**
 * @class EStop
 * @brief Implements the emergency stop interface.
 */
class EStop : public EStopInterface
{
public:
  EStop(
    std::shared_ptr<GPIOControllerInterface> gpio_controller,
    std::function<bool()> zero_velocity_check)
  : EStopInterface(),
    gpio_controller_(gpio_controller),
    ZeroVelocityCheck(zero_velocity_check) {};

  virtual ~EStop() override = default;

  bool ReadEStopState() override;

  void TriggerEStop() override;

  void ResetEStop() override;

protected:
  std::shared_ptr<GPIOControllerInterface> gpio_controller_;
  std::function<bool()> ZeroVelocityCheck;

  std::mutex e_stop_manipulation_mtx_;
  std::atomic_bool e_stop_triggered_ = true;
  std::atomic_bool last_e_stop_ = false;
};

}  // namespace rosbot_hardware_interfaces

#endif  // ROSBOT_UGV_HARDWARE_INTERFACES_ROBOT_SYSTEM_SYSTEM_E_STOP_HPP_
