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

#include "rosbot_hardware_interfaces/rosbot_system.hpp"

#include <algorithm>
#include <string>
#include <vector>
#include <memory>
#include <string>
#include <thread>

// #include <diagnostic_updater/diagnostic_status_wrapper.hpp>
// #include <hardware_interface/types/hardware_interface_type_values.hpp>
#include "rclcpp/logging.hpp"

#include "hardware_interface/types/hardware_interface_type_values.hpp"

namespace rosbot_hardware_interfaces {

template class ROSServiceWrapper<std_srvs::srv::SetBool, std::function<void(bool)>>;
template class ROSServiceWrapper<std_srvs::srv::Trigger, std::function<void()>>;

// template <typename SrvT, typename CallbackT>
// void ROSServiceWrapper<SrvT, CallbackT>::RegisterService(
//   const rclcpp::Node::SharedPtr node, const std::string & service_name,
//   rclcpp::CallbackGroup::SharedPtr group, const rclcpp::QoS & qos)
// {
//   service_ = node->create_service<SrvT>(
//     service_name, std::bind(&ROSServiceWrapper<SrvT, CallbackT>::CallbackWrapper, this, _1, _2),
//     qos, group);
// }

template <typename SrvT, typename CallbackT>
void ROSServiceWrapper<SrvT, CallbackT>::CallbackWrapper(
  SrvRequestConstPtr request, SrvResponsePtr response)
{
  try {
    ProccessCallback(request);
    response->success = true;
  } catch (const std::exception & err) {
    response->success = false;
    response->message = err.what();

    RCLCPP_WARN_STREAM(
      rclcpp::get_logger("RosbotSystem"),
      "An exception occurred while handling the request: " << err.what());
  }
}

template <>
void ROSServiceWrapper<std_srvs::srv::SetBool, std::function<void(bool)>>::ProccessCallback(
  SrvRequestConstPtr request)
{
  callback_(request->data);
}

template <>
void ROSServiceWrapper<std_srvs::srv::Trigger, std::function<void()>>::ProccessCallback(
  SrvRequestConstPtr /* request */)
{
  callback_();
}


CallbackReturn RosbotSystem::on_init(const hardware_interface::HardwareInfo &hardware_info) {
  RCLCPP_INFO(rclcpp::get_logger("RosbotSystem"), "Initializing");

  if (hardware_interface::SystemInterface::on_init(hardware_info) !=
      CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }

  for (const hardware_interface::ComponentInfo &joint : info_.joints) {
    if (joint.command_interfaces.size() != 1) {
      RCLCPP_FATAL(rclcpp::get_logger("RosbotSystem"),
                   "Joint '%s' has %zu command interfaces found. 1 expected.",
                   joint.name.c_str(), joint.command_interfaces.size());
      return CallbackReturn::ERROR;
    }

    if (joint.command_interfaces[0].name !=
        hardware_interface::HW_IF_VELOCITY) {
      RCLCPP_FATAL(
          rclcpp::get_logger("RosbotSystem"),
          "Joint '%s' have %s command interfaces found. '%s' expected.",
          joint.name.c_str(), joint.command_interfaces[0].name.c_str(),
          hardware_interface::HW_IF_VELOCITY);
      return CallbackReturn::ERROR;
    }

    if (joint.state_interfaces.size() != 2) {
      RCLCPP_FATAL(rclcpp::get_logger("RosbotSystem"),
                   "Joint '%s' has %zu state interface. 2 expected.",
                   joint.name.c_str(), joint.state_interfaces.size());
      return CallbackReturn::ERROR;
    }

    if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION) {
      RCLCPP_FATAL(
          rclcpp::get_logger("RosbotSystem"),
          "Joint '%s' have '%s' as first state interface. '%s' expected.",
          joint.name.c_str(), joint.state_interfaces[0].name.c_str(),
          hardware_interface::HW_IF_POSITION);
      return CallbackReturn::ERROR;
    }

    if (joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY) {
      RCLCPP_FATAL(
          rclcpp::get_logger("RosbotSystem"),
          "Joint '%s' have '%s' as second state interface. '%s' expected.",
          joint.name.c_str(), joint.state_interfaces[1].name.c_str(),
          hardware_interface::HW_IF_VELOCITY);
      return CallbackReturn::ERROR;
    }
  }

  for (auto &j : info_.joints) {
    RCLCPP_INFO(rclcpp::get_logger("RosbotSystem"), "Joint '%s' found",
                j.name.c_str());

    pos_state_[j.name] = 0.0;
    vel_state_[j.name] = 0.0;
    vel_commands_[j.name] = 0.0;
  }

  connection_timeout_ms_ =
      std::stoul(info_.hardware_parameters["connection_timeout_ms"]);
  connection_check_period_ms_ =
      std::stoul(info_.hardware_parameters["connection_check_period_ms"]);

  std::string velocity_command_joint_order_raw =
      info_.hardware_parameters["velocity_command_joint_order"];
  // remove whitespaces
  velocity_command_joint_order_raw.erase(
      std::remove_if(
          velocity_command_joint_order_raw.begin(),
          velocity_command_joint_order_raw.end(),
          [](char c) { return std::isspace(static_cast<unsigned char>(c)); }),
      velocity_command_joint_order_raw.end());
  std::stringstream velocity_command_joint_order_stream(
      velocity_command_joint_order_raw);
  std::string joint_name;
  while (getline(velocity_command_joint_order_stream, joint_name, ',')) {
    velocity_command_joint_order_.push_back(joint_name);
  }

  if (velocity_command_joint_order_.size() != info_.joints.size()) {
    RCLCPP_FATAL(rclcpp::get_logger("RosbotSystem"),
                 "Joint order size is invalid");
    return CallbackReturn::ERROR;
  }

  for (auto &j : info_.joints) {
    if (std::find(velocity_command_joint_order_.begin(),
                  velocity_command_joint_order_.end(),
                  j.name) == velocity_command_joint_order_.end()) {
      RCLCPP_FATAL(rclcpp::get_logger("RosbotSystem"),
                   "Joint '%s' missing from velocity command joint order",
                   j.name.c_str());
      return CallbackReturn::ERROR;
    }
  }

  node_ = std::make_shared<rclcpp::Node>("rosbot_system_node");
  executor_.add_node(node_);
  executor_thread_ = std::make_unique<std::thread>(
      std::bind(&rclcpp::executors::MultiThreadedExecutor::spin, &executor_));

  return CallbackReturn::SUCCESS;
}

CallbackReturn RosbotSystem::on_configure(const rclcpp_lifecycle::State &) {
  RCLCPP_INFO(rclcpp::get_logger("RosbotSystem"), "Configuring");
  try {
    ConfigureGPIOController();
    ConfigureEStop();
  } catch (const std::runtime_error & e) {
    RCLCPP_ERROR_STREAM(
      rclcpp::get_logger("RosbotSystem"), "Failed to initialize E-Stop controllers. Error: " << e.what());
    return CallbackReturn::ERROR;
  }

  return CallbackReturn::SUCCESS;
}

CallbackReturn RosbotSystem::on_cleanup(const rclcpp_lifecycle::State &) {
  RCLCPP_INFO(rclcpp::get_logger("RosbotSystem"), "Cleaning up");

  cleanup_node();
  return CallbackReturn::SUCCESS;
}

CallbackReturn RosbotSystem::on_activate(const rclcpp_lifecycle::State &) {
  RCLCPP_INFO(rclcpp::get_logger("RosbotSystem"), "Activating");

  for (const auto &x : pos_state_) {
    pos_state_[x.first] = 0.0;
    vel_state_[x.first] = 0.0;
    vel_commands_[x.first] = 0.0;
  }

  motor_command_publisher_ = node_->create_publisher<Float32MultiArray>(
      "~/motors_cmd", rclcpp::SensorDataQoS());
  realtime_motor_command_publisher_ =
      std::make_shared<realtime_tools::RealtimePublisher<Float32MultiArray>>(
          motor_command_publisher_);

  e_stop_state_publisher_ = node_->create_publisher<BoolMsg>(
    "hardware/e_stop", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());
  realtime_e_stop_state_publisher_ =
    std::make_unique<realtime_tools::RealtimePublisher<BoolMsg>>(e_stop_state_publisher_);

  AddService<TriggerSrv, std::function<void()>>(
    "hardware/e_stop_trigger", std::bind(&EStopInterface::TriggerEStop, e_stop_), 1,
    rclcpp::CallbackGroupType::MutuallyExclusive);

  auto e_stop_reset_qos = rclcpp::ServicesQoS();
  e_stop_reset_qos.keep_last(1);
  AddService<TriggerSrv, std::function<void( )>>(
    "hardware/e_stop_reset", std::bind(&RosbotSystem::ResetEStop, this), 2,
    rclcpp::CallbackGroupType::MutuallyExclusive, e_stop_reset_qos);
  PublishEStopStateMsg(e_stop_->ReadEStopState());

  motor_state_subscriber_ = node_->create_subscription<JointState>(
      "~/motors_response", rclcpp::SensorDataQoS(),
      std::bind(&RosbotSystem::motor_state_cb, this, std::placeholders::_1));

  std::shared_ptr<JointState> motor_state;
  for (uint wait_time = 0; wait_time <= connection_timeout_ms_;
       wait_time += connection_check_period_ms_) {
    if (!rclcpp::ok()) {
      RCLCPP_WARN(rclcpp::get_logger("RosbotSystem"),
                  "ROS shutdown signal detected while waiting for motor "
                  "feedback messages.");
      return CallbackReturn::ERROR;
    }

    RCLCPP_WARN_SKIPFIRST_THROTTLE(
        rclcpp::get_logger("RosbotSystem"), *node_->get_clock(), 5000,
        "Feedback message from motors wasn't received yet");
    received_motor_state_msg_ptr_.get(
        [&](const auto &msg) { motor_state = msg; });

    if (motor_state) {
      RCLCPP_DEBUG(node_->get_logger(),
                   "Subscriber and publisher are now active.");
      return CallbackReturn::SUCCESS;
    }

    rclcpp::sleep_for(std::chrono::milliseconds(connection_check_period_ms_));
  }

  RCLCPP_FATAL(node_->get_logger(), "Activation failed, timeout reached while "
                                    "waiting for feedback from motors");
  return CallbackReturn::ERROR;
}

CallbackReturn RosbotSystem::on_deactivate(const rclcpp_lifecycle::State &) {
  RCLCPP_INFO(rclcpp::get_logger("RosbotSystem"), "Deactivating");
  try {
    e_stop_->TriggerEStop();
  } catch (const std::runtime_error & e) {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("RosbotSystem"), "Shutdown failed: " << e.what());
    return CallbackReturn::ERROR;
  }
  received_motor_state_msg_ptr_.set(nullptr);
  return CallbackReturn::SUCCESS;
}

CallbackReturn RosbotSystem::on_shutdown(const rclcpp_lifecycle::State &) {
  RCLCPP_INFO(rclcpp::get_logger("RosbotSystem"), "Shutting down");
  try {
    e_stop_->TriggerEStop();
  } catch (const std::runtime_error & e) {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("RosbotSystem"), "Shutdown failed: " << e.what());
    return CallbackReturn::ERROR;
  }

  cleanup_node();
  return CallbackReturn::SUCCESS;
}

CallbackReturn RosbotSystem::on_error(const rclcpp_lifecycle::State &) {
  RCLCPP_INFO(rclcpp::get_logger("RosbotSystem"), "Handling error");
  try {
    e_stop_->TriggerEStop();
  } catch (const std::runtime_error & e) {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("RosbotSystem"), "Shutdown failed: " << e.what());
    return CallbackReturn::ERROR;
  }
  cleanup_node();
  return CallbackReturn::SUCCESS;
}

std::vector<StateInterface> RosbotSystem::export_state_interfaces() {
  std::vector<StateInterface> state_interfaces;
  for (auto i = 0u; i < info_.joints.size(); i++) {
    state_interfaces.emplace_back(
        StateInterface(info_.joints[i].name, hardware_interface::HW_IF_POSITION,
                       &pos_state_[info_.joints[i].name]));
    state_interfaces.emplace_back(
        StateInterface(info_.joints[i].name, hardware_interface::HW_IF_VELOCITY,
                       &vel_state_[info_.joints[i].name]));
  }

  return state_interfaces;
}

std::vector<CommandInterface> RosbotSystem::export_command_interfaces() {
  std::vector<CommandInterface> command_interfaces;
  for (auto i = 0u; i < info_.joints.size(); i++) {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        info_.joints[i].name, hardware_interface::HW_IF_VELOCITY,
        &vel_commands_[info_.joints[i].name]));
  }

  return command_interfaces;
}

void RosbotSystem::motor_state_cb(const std::shared_ptr<JointState> msg) {
  RCLCPP_DEBUG(node_->get_logger(), "Received motors response");
  received_motor_state_msg_ptr_.set(
      [&](auto &msg_ref) { msg_ref = std::move(msg); });
}

return_type RosbotSystem::read(const rclcpp::Time &, const rclcpp::Duration &) {
  std::shared_ptr<JointState> motor_state;
  received_motor_state_msg_ptr_.get(
      [&](const auto &msg) { motor_state = msg; });

  RCLCPP_DEBUG(rclcpp::get_logger("RosbotSystem"), "Reading motors state");

  if (!motor_state) {
    RCLCPP_ERROR(rclcpp::get_logger("RosbotSystem"),
                 "Feedback message from motors wasn't received");
    return return_type::ERROR;
  }

  for (auto i = 0u; i < motor_state->name.size(); i++) {
    if (pos_state_.find(motor_state->name[i]) == pos_state_.end() ||
        vel_state_.find(motor_state->name[i]) == vel_state_.end()) {
      RCLCPP_ERROR(rclcpp::get_logger("RosbotSystem"),
                   "Position or velocity feedback not found for joint %s",
                   motor_state->name[i].c_str());
      return return_type::ERROR;
    }

    pos_state_[motor_state->name[i]] = motor_state->position[i];
    vel_state_[motor_state->name[i]] = motor_state->velocity[i];

    RCLCPP_DEBUG(rclcpp::get_logger("RosbotSystem"),
                 "Position feedback: %f, velocity feedback: %f",
                 pos_state_[motor_state->name[i]],
                 vel_state_[motor_state->name[i]]);
  }

  UpdateEStopState();

  return return_type::OK;
}

return_type RosbotSystem::write(const rclcpp::Time &,
                                const rclcpp::Duration &) {

  if (realtime_motor_command_publisher_->trylock()) {
    auto &motor_command = realtime_motor_command_publisher_->msg_;
    motor_command.data.clear();

    RCLCPP_DEBUG(rclcpp::get_logger("RosbotSystem"),
                 "Wrtiting motors cmd message");

    for (auto const &joint : velocity_command_joint_order_) {
      motor_command.data.push_back(vel_commands_[joint]);
    }

    realtime_motor_command_publisher_->unlockAndPublish();
  }

  return return_type::OK;
}

void RosbotSystem::ConfigureGPIOController()
{
  gpio_controller_ = GPIOControllerFactory::CreateGPIOController();
  gpio_controller_->Start();

  RCLCPP_INFO(rclcpp::get_logger("RosbotSystem"), "Successfully configured GPIO controller.");
}

void RosbotSystem::ConfigureEStop()
{
  // if (!gpio_controller_ || !roboteq_error_filter_ || !robot_driver_ || !robot_driver_write_mtx_) {
  //   throw std::runtime_error("Failed to configure E-Stop, make sure to setup entities first.");
  // }

  if (!gpio_controller_) {
    throw std::runtime_error("Failed to configure E-Stop, make sure to setup entities first.");
  }

  e_stop_ = std::make_shared<EStop>(
    gpio_controller_,
    std::bind(&RosbotSystem::AreVelocityCommandsNearZero, this));

  RCLCPP_INFO(rclcpp::get_logger("RosbotSystem"), "Successfully configured E-Stop");
}

void RosbotSystem::ResetEStop()
{
  const auto lifecycle_state = this->get_lifecycle_state().id();

  if (lifecycle_state != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
    throw std::runtime_error(
      "Can't reset E-Stop when the hardware interface is not in ACTIVE state.");
  }

  e_stop_->ResetEStop();
}

void RosbotSystem::UpdateEStopState()
{
  // if (robot_driver_->CommunicationError()) {
  //   e_stop_->TriggerEStop();
  // }

  const bool e_stop = e_stop_->ReadEStopState();
  PublishEStopStateIfChanged(e_stop);
}

bool RosbotSystem::AreVelocityCommandsNearZero()
{
  for (const auto & cmd : vel_commands_) {
    if (std::abs(cmd.second) > std::numeric_limits<double>::epsilon()) {
      return false;
    }
  }
  return true;
}
void RosbotSystem::PublishEStopStateMsg(const bool e_stop)
{
  realtime_e_stop_state_publisher_->msg_.data = e_stop;
  if (realtime_e_stop_state_publisher_->trylock()) {
    realtime_e_stop_state_publisher_->unlockAndPublish();
  }
}

void RosbotSystem::PublishEStopStateIfChanged(const bool e_stop)
{
  if (realtime_e_stop_state_publisher_->msg_.data != e_stop) {
    PublishEStopStateMsg(e_stop);
  }
}

void RosbotSystem::cleanup_node(){
  gpio_controller_.reset();
  e_stop_.reset();
}

rclcpp::CallbackGroup::SharedPtr RosbotSystem::GetOrCreateNodeCallbackGroup(
  const unsigned group_id, rclcpp::CallbackGroupType callback_group_type)
{
  if (group_id == 0) {
    if (callback_group_type == rclcpp::CallbackGroupType::Reentrant) {
      throw std::runtime_error(
        "Node callback group with id 0 (default group) cannot be of "
        "rclcpp::CallbackGroupType::Reentrant type.");
    }
    return nullptr;  // default node callback group
  }

  const auto search = callback_groups_.find(group_id);
  if (search != callback_groups_.end()) {
    if (search->second->type() != callback_group_type) {
      throw std::runtime_error("Requested node callback group has incorrect type.");
    }
    return search->second;
  }

  auto callback_group = node_->create_callback_group(callback_group_type);
  callback_groups_[group_id] = callback_group;
  return callback_group;
}

} // namespace rosbot_hardware_interfaces

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(rosbot_hardware_interfaces::RosbotSystem,
                       hardware_interface::SystemInterface)
