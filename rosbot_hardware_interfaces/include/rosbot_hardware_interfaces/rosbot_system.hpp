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

#ifndef ROSBOT_HARDWARE_INTERFACES__ROSBOT_SYSTEM_HPP_
#define ROSBOT_HARDWARE_INTERFACES__ROSBOT_SYSTEM_HPP_
#include <any>
#include <functional>
#include <memory>
#include <thread>
#include <map>
#include <unordered_map>

#include "rosbot_hardware_interfaces/visibility_control.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "hardware_interface/handle.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"

#include "rosbot_hardware_interfaces/gpio_controller.hpp"
#include "rosbot_hardware_interfaces/system_e_stop.hpp"

#include "realtime_tools/realtime_buffer.hpp"
#include "realtime_tools/realtime_publisher.hpp"
#include "realtime_tools/realtime_thread_safe_box.hpp"

#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"

#include <std_msgs/msg/bool.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <std_srvs/srv/trigger.hpp>

namespace rosbot_hardware_interfaces {
using return_type = hardware_interface::return_type;
using CallbackReturn =
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
using StateInterface = hardware_interface::StateInterface;
using CommandInterface = hardware_interface::CommandInterface;

using JointState = sensor_msgs::msg::JointState;
using Float32MultiArray = std_msgs::msg::Float32MultiArray;
using BoolMsg = std_msgs::msg::Bool;
using SetBoolSrv = std_srvs::srv::SetBool;
using TriggerSrv = std_srvs::srv::Trigger;

/**
 * @brief A wrapper class for ROS services that simplifies the creation and management of a service
 * server.
 *
 * @tparam SrvT The type of the service message, derived from a .srv file by the ROS build system.
 * @tparam CallbackT The type of the callback function to be invoked on service request. Currently
 * supported callback function signatures include void() and void(bool).
 */
template <typename SrvT, typename CallbackT>
class ROSServiceWrapper
{
public:
  using SrvSharedPtr = typename rclcpp::Service<SrvT>::SharedPtr;
  using SrvRequestConstPtr = typename SrvT::Request::ConstSharedPtr;
  using SrvResponsePtr = typename SrvT::Response::SharedPtr;
  ROSServiceWrapper(const CallbackT & callback) : callback_(callback) {}

  void RegisterService(
    const rclcpp::Node::SharedPtr node, const std::string & service_name,
    rclcpp::CallbackGroup::SharedPtr group = nullptr,
    const rclcpp::QoS & qos = rclcpp::ServicesQoS());

private:
  void CallbackWrapper(SrvRequestConstPtr request, SrvResponsePtr response);
  void ProccessCallback(SrvRequestConstPtr request);

  SrvSharedPtr service_;
  CallbackT callback_;
};

class RosbotSystem : public hardware_interface::SystemInterface {
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(RosbotSystem)

  ROSBOT_HARDWARE_INTERFACES_PUBLIC
  CallbackReturn
  on_init(const hardware_interface::HardwareInfo &hardware_info) override;

  ROSBOT_HARDWARE_INTERFACES_PUBLIC
  CallbackReturn
  on_configure(const rclcpp_lifecycle::State &previous_state) override;

  ROSBOT_HARDWARE_INTERFACES_PUBLIC
  CallbackReturn
  on_cleanup(const rclcpp_lifecycle::State &previous_state) override;

  ROSBOT_HARDWARE_INTERFACES_PUBLIC
  CallbackReturn
  on_activate(const rclcpp_lifecycle::State &previous_state) override;

  ROSBOT_HARDWARE_INTERFACES_PUBLIC
  CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State &previous_state) override;

  ROSBOT_HARDWARE_INTERFACES_PUBLIC
  CallbackReturn
  on_shutdown(const rclcpp_lifecycle::State &previous_state) override;

  ROSBOT_HARDWARE_INTERFACES_PUBLIC
  CallbackReturn
  on_error(const rclcpp_lifecycle::State &previous_state) override;

  ROSBOT_HARDWARE_INTERFACES_PUBLIC
  std::vector<StateInterface> export_state_interfaces() override;

  ROSBOT_HARDWARE_INTERFACES_PUBLIC
  std::vector<CommandInterface> export_command_interfaces() override;

  ROSBOT_HARDWARE_INTERFACES_PUBLIC
  return_type read(const rclcpp::Time &time,
                   const rclcpp::Duration &period) override;

  ROSBOT_HARDWARE_INTERFACES_PUBLIC
  return_type write(const rclcpp::Time &time,
                    const rclcpp::Duration &period) override;

protected:
  template <class SrvT, class CallbackT>
  inline void AddService(
    const std::string & service_name, const CallbackT & callback, const unsigned group_id = 0,
    rclcpp::CallbackGroupType callback_group_type = rclcpp::CallbackGroupType::MutuallyExclusive,
    const rclcpp::QoS & qos = rclcpp::ServicesQoS())
  {
    rclcpp::CallbackGroup::SharedPtr callback_group = GetOrCreateNodeCallbackGroup(
      group_id, callback_group_type);

    auto wrapper = std::make_shared<ROSServiceWrapper<SrvT, CallbackT>>(callback);
    wrapper->RegisterService(node_, service_name, callback_group, qos);
    service_wrappers_storage_.push_back(wrapper);
  }

  virtual void ConfigureGPIOController();  // virtual for mocking
  virtual void ConfigureEStop();  // virtual for mocking
  void ResetEStop();

  void UpdateEStopState();

  void PublishEStopStateMsg(const bool e_stop);

  void PublishEStopStateIfChanged(const bool e_stop);
  bool AreVelocityCommandsNearZero();

  void cleanup_node();

  realtime_tools::RealtimeThreadSafeBox<std::shared_ptr<JointState>>
      received_motor_state_msg_ptr_{nullptr};

  std::shared_ptr<rclcpp::Publisher<Float32MultiArray>>
      motor_command_publisher_ = nullptr;

  std::shared_ptr<realtime_tools::RealtimePublisher<Float32MultiArray>>
      realtime_motor_command_publisher_ = nullptr;
      
  rclcpp::Subscription<JointState>::SharedPtr motor_state_subscriber_ = nullptr;

  rclcpp::Publisher<BoolMsg>::SharedPtr e_stop_state_publisher_;
  std::unique_ptr<realtime_tools::RealtimePublisher<BoolMsg>> realtime_e_stop_state_publisher_;

  std::map<std::string, double> vel_commands_;
  std::map<std::string, double> pos_state_;
  std::map<std::string, double> vel_state_;

  std::shared_ptr<GPIOControllerInterface> gpio_controller_;
  std::shared_ptr<EStopInterface> e_stop_;

  bool subscriber_is_active_ = false;

  std::shared_ptr<rclcpp::Node> node_;

  void motor_state_cb(const std::shared_ptr<JointState> msg);
  rclcpp::executors::MultiThreadedExecutor executor_;
  std::unique_ptr<std::thread> executor_thread_;

  std::vector<std::string> velocity_command_joint_order_;

  uint connection_check_period_ms_;
  uint connection_timeout_ms_;

  std::vector<std::any> service_wrappers_storage_;

  rclcpp::CallbackGroup::SharedPtr GetOrCreateNodeCallbackGroup(
    const unsigned group_id, rclcpp::CallbackGroupType callback_group_type);

  std::unordered_map<unsigned, rclcpp::CallbackGroup::SharedPtr> callback_groups_;

};

} // namespace rosbot_hardware_interfaces

#endif // ROSBOT_HARDWARE_INTERFACES__ROSBOT_SYSTEM_HPP_
