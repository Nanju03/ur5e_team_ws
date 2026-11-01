// Copyright (c) 2022 PickNik, Inc.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the {copyright_holder} nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <set>
#include <stdexcept>
#include <vector>

#include <dh_gripper_driver/default_driver_factory.hpp>
#include <dh_gripper_driver/hardware_interface.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <rclcpp/rclcpp.hpp>
#include <serial/serial.h>

namespace dh_gripper_driver
{
namespace
{
const auto kLogger = rclcpp::get_logger("DHGripperHardwareInterface");
constexpr uint16_t kMaxPosition = 1000;
constexpr uint8_t kMinForce = 20;
constexpr uint8_t kMaxForce = 100;
constexpr auto kCommsPeriod = std::chrono::milliseconds(10);

double multiplier(
  const hardware_interface::HardwareInfo & info, const std::string & name,
  double fallback)
{
  const auto found = info.hardware_parameters.find(name);
  const double value = found ==
    info.hardware_parameters.end() ? fallback : std::stod(found->second);
  if (!std::isfinite(value) || value < 0. || value > 1.) {
    throw std::invalid_argument(name + " must be finite and in 0..1");
  }
  return value;
}
uint8_t force_percent(double normalized)
{
  return static_cast<uint8_t>(std::lround(
           kMinForce + (kMaxForce - kMinForce) *
           std::clamp(normalized, 0., 1.)));
}
}  // namespace

DHGripperHardwareInterface::DHGripperHardwareInterface()
: driver_factory_(std::make_unique<DefaultDriverFactory>()) {}
DHGripperHardwareInterface::DHGripperHardwareInterface(std::unique_ptr<DriverFactory> factory)
: driver_factory_(std::move(factory)) {}
DHGripperHardwareInterface::~DHGripperHardwareInterface()
{
  communication_thread_is_running_.store(false);
  if (communication_thread_.joinable()) {communication_thread_.join();}
  if (driver_) {
    try {
      driver_->disconnect();
    } catch (const std::exception &) {
    }
  }
}

hardware_interface::CallbackReturn DHGripperHardwareInterface::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }
  try {
    gripper_closed_pos_ = std::stod(info_.hardware_parameters.at("gripper_closed_position"));
    if (!std::isfinite(gripper_closed_pos_) || gripper_closed_pos_ <= 0.) {
      throw std::invalid_argument("gripper_closed_position must be finite and positive");
    }
    gripper_speed_ = multiplier(info_, "gripper_speed_multiplier", 1.);
    gripper_force_ = multiplier(info_, "gripper_force_multiplier", .5);
    const auto timeout = info_.hardware_parameters.find("communication_timeout");
    if (timeout != info_.hardware_parameters.end()) {
      communication_timeout_ = std::stod(timeout->second);
    }
    if (!std::isfinite(communication_timeout_) || communication_timeout_ <= 0.) {
      throw std::invalid_argument("communication_timeout must be finite and positive");
    }
    if (info_.joints.size() != 1) {
      throw std::invalid_argument("The AG95 has one independent joint");
    }
    const auto & joint = info_.joints.front();
    std::set<std::string> commands, states;
    const std::set<std::string> supported_commands{
      "position", "set_gripper_max_effort", "set_gripper_max_velocity"};
    const std::set<std::string> supported_states{
      "position", "velocity", "grip_state", "raw_position", "force_percent", "communication_age",
      "sample_sequence"};
    for (const auto & interface : joint.command_interfaces) {
      if (!supported_commands.count(interface.name) || !commands.insert(interface.name).second) {
        throw std::invalid_argument(
                "Unsupported/duplicate gripper command interface: " + interface.name);
      }
    }
    for (const auto & interface : joint.state_interfaces) {
      if (!supported_states.count(interface.name) || !states.insert(interface.name).second) {
        throw std::invalid_argument(
                "Unsupported/duplicate gripper state interface: " + interface.name);
      }
    }
    if (!commands.count("position") || !states.count("position") || !states.count("velocity")) {
      throw std::invalid_argument(
              "AG95 requires position command and position/velocity state interfaces");
    }
    gripper_position_ = gripper_velocity_ = gripper_position_command_ = NO_NEW_CMD_;
    reactivate_gripper_cmd_ = NO_NEW_CMD_;
    queue_command(1000, force_percent(gripper_force_));
    driver_ = driver_factory_->create(info_);
    if (!driver_) {throw std::invalid_argument("The AG95 driver factory returned no driver");}
  } catch (const std::exception & error) {
    RCLCPP_ERROR(kLogger, "AG95 initialization failed: %s", error.what());
    return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn DHGripperHardwareInterface::on_configure(
  const rclcpp_lifecycle::State &)
{
  try {
    if (!driver_->connect()) {
      throw std::runtime_error("AG95 serial connection could not be opened");
    }
  } catch (const std::exception & error) {
    RCLCPP_ERROR(kLogger, "AG95 configure failed: %s", error.what());
    return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> DHGripperHardwareInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> interfaces;
  const auto & joint = info_.joints.front();
  for (const auto & declared : joint.state_interfaces) {
    double * value = nullptr;
    if (declared.name == "position") {
      value = &gripper_position_;
    } else if (declared.name == "velocity") {
      value = &gripper_velocity_;
    } else if (declared.name == "grip_state") {
      value = &grip_state_;
    } else if (declared.name == "raw_position") {
      value = &raw_position_;
    } else if (declared.name == "force_percent") {
      value = &force_percent_;
    } else if (declared.name == "communication_age") {
      value = &communication_age_;
    } else if (declared.name == "sample_sequence") {value = &sample_sequence_;}
    if (value) {interfaces.emplace_back(joint.name, declared.name, value);}
  }
  return interfaces;
}

std::vector<hardware_interface::CommandInterface> DHGripperHardwareInterface::
export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> interfaces;
  const auto & joint = info_.joints.front();
  for (const auto & declared : joint.command_interfaces) {
    double * value = declared.name == "position" ? &gripper_position_command_ :
      declared.name == "set_gripper_max_effort" ? &gripper_force_ : &gripper_speed_;
    interfaces.emplace_back(joint.name, declared.name, value);
  }
  // Retain the original explicit reactivation API. No reactivation is automatic.
  interfaces.emplace_back("reactivate_gripper", "reactivate_gripper_cmd", &reactivate_gripper_cmd_);
  interfaces.emplace_back(
    "reactivate_gripper", "reactivate_gripper_response",
    &reactivate_gripper_response_);
  return interfaces;
}

void DHGripperHardwareInterface::store_feedback(
  const Driver::Feedback & feedback,
  std::chrono::steady_clock::time_point acquired)
{
  if (feedback.initialization != 1 || feedback.raw_position > 1000 || feedback.grip_state > 3 ||
    feedback.force_percent < 20 || feedback.force_percent > 100)
  {
    throw std::runtime_error("AG95 feedback is not initialized or is out of range");
  }
  const auto now = std::chrono::steady_clock::now();
  if (acquired > now ||
    std::chrono::duration<double>(now - acquired).count() > communication_timeout_)
  {
    throw std::runtime_error("AG95 polling cycle exceeded its freshness timeout");
  }
  std::lock_guard<std::mutex> lock(feedback_mutex_);
  const double dt = std::chrono::duration<double>(acquired - feedback_received_).count();
  feedback_velocity_ = feedback_valid_ && dt > 0. && dt <= communication_timeout_ ?
    gripper_closed_pos_ * (static_cast<double>(feedback_.raw_position) - feedback.raw_position) /
    1000. / dt : NO_NEW_CMD_;
  feedback_ = feedback;
  feedback_received_ = acquired;
  feedback_valid_ = true;
  ++feedback_sequence_;
}

hardware_interface::CallbackReturn DHGripperHardwareInterface::on_activate(
  const rclcpp_lifecycle::State &)
{
  if (communication_thread_.joinable()) {return CallbackReturn::ERROR;}
  try {
    driver_->activate();  // Explicit lifecycle activation can initialize/move fingers.
    driver_->set_force(force_percent(gripper_force_));
    const auto acquired = std::chrono::steady_clock::now();
    const auto feedback = driver_->get_feedback();
    store_feedback(feedback, acquired);
    // Hold the actual measured position until a controller sends a new target.
    gripper_position_command_ = gripper_closed_pos_ * (1000 - feedback.raw_position) / 1000.;
    queue_command(feedback.raw_position, force_percent(gripper_force_));
    gripper_current_state_.store(feedback.raw_position);
    read(rclcpp::Time(0), rclcpp::Duration(0, 0));
    communication_thread_is_running_.store(true);
    communication_thread_ = std::thread(
      [this, initial_target = feedback.raw_position] {
        background_task(initial_target);
      });
  } catch (const std::exception & error) {
    RCLCPP_ERROR(kLogger, "AG95 activate failed: %s", error.what());
    return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn DHGripperHardwareInterface::on_deactivate(
  const rclcpp_lifecycle::State &)
{
  communication_thread_is_running_.store(false);
  if (communication_thread_.joinable()) {communication_thread_.join();}
  try {
    driver_->deactivate();
  } catch (const std::exception & error) {
    RCLCPP_ERROR(kLogger, "AG95 deactivate failed: %s", error.what());
    return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}

hardware_interface::return_type DHGripperHardwareInterface::read(
  const rclcpp::Time &,
  const rclcpp::Duration &)
{
  {
    std::lock_guard<std::mutex> lock(feedback_mutex_);
    communication_age_ = feedback_valid_ ?
      std::chrono::duration<double>(std::chrono::steady_clock::now() - feedback_received_).count() :
      std::numeric_limits<double>::infinity();
    sample_sequence_ = static_cast<double>(feedback_sequence_);
    if (feedback_valid_) {
      raw_position_ = feedback_.raw_position;
      grip_state_ = feedback_.grip_state;
      force_percent_ = feedback_.force_percent;
      gripper_position_ = gripper_closed_pos_ * (1000 - feedback_.raw_position) / 1000.;
      gripper_velocity_ = communication_age_ <=
        communication_timeout_ ? feedback_velocity_ : NO_NEW_CMD_;
    } else {gripper_velocity_ = NO_NEW_CMD_;}
  }
  if (std::isfinite(reactivate_gripper_cmd_)) {
    reactivate_gripper_async_cmd_.store(true);
    reactivate_gripper_cmd_ = NO_NEW_CMD_;
  }
  const auto response = reactivate_gripper_async_response_.exchange(std::nullopt);
  if (response.has_value()) {reactivate_gripper_response_ = response.value() ? 1. : 0.;}
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type DHGripperHardwareInterface::write(
  const rclcpp::Time &,
  const rclcpp::Duration &)
{
  if (!std::isfinite(gripper_position_command_) || !std::isfinite(gripper_force_)) {
    return hardware_interface::return_type::ERROR;
  }
  const double target = std::clamp(gripper_position_command_, 0., gripper_closed_pos_);
  queue_command(
    static_cast<uint16_t>(std::lround(1000. * (1. - target / gripper_closed_pos_))),
    force_percent(gripper_force_));
  return hardware_interface::return_type::OK;
}

void DHGripperHardwareInterface::background_task(uint16_t initial_target)
{
  uint16_t last_target = initial_target;
  bool error_reported = false;
  while (communication_thread_is_running_.load()) {
    try {
      if (reactivate_gripper_async_cmd_.exchange(false)) {
        if (!communication_thread_is_running_.load()) {break;}
        driver_->activate();
        reactivate_gripper_async_response_.store(true);
      }
      // Read successfully before issuing any new target. Every field in the
      // exported snapshot comes from this one fully validated polling cycle.
      const auto acquired = std::chrono::steady_clock::now();
      const auto feedback = driver_->get_feedback();
      if (!communication_thread_is_running_.load()) {break;}
      if (feedback.initialization != 1) {throw std::runtime_error("AG95 is not initialized");}
      if (std::chrono::duration<double>(std::chrono::steady_clock::now() - acquired).count() >
        communication_timeout_)
      {
        throw std::runtime_error("AG95 polling cycle exceeded its freshness timeout");
      }
      gripper_current_state_.store(feedback.raw_position);
      const auto command = pending_command();
      if (feedback.force_percent != command.force) {
        if (!communication_thread_is_running_.load()) {break;}
        driver_->set_force(command.force);
        // Wait for a subsequent read-back to confirm force before movement.
      } else {
        if (command.position != last_target) {
          if (!communication_thread_is_running_.load()) {break;}
          driver_->set_gripper_position(command.position);
          last_target = command.position;
        }
      }
      store_feedback(feedback, acquired);
      error_reported = false;
    } catch (const std::exception & error) {
      // Keep the polling thread alive. Do not advance age/sequence on failure;
      // stale velocity becomes NaN and consumers must check age and sequence.
      if (!error_reported) {RCLCPP_ERROR(kLogger, "AG95 communication failed: %s", error.what());}
      error_reported = true;
      reactivate_gripper_async_response_.store(false);
    }
    std::this_thread::sleep_for(kCommsPeriod);
  }
}
}  // namespace dh_gripper_driver

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  dh_gripper_driver::DHGripperHardwareInterface,
  hardware_interface::SystemInterface)
