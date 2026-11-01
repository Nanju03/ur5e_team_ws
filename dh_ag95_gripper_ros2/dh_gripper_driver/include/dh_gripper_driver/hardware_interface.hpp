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

#pragma once

#include <atomic>
#include <chrono>
#include <limits>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include <dh_gripper_driver/visibility_control.hpp>

#include <dh_gripper_driver/driver.hpp>
#include <dh_gripper_driver/driver_factory.hpp>

#include <hardware_interface/handle.hpp>
#include <hardware_interface/hardware_info.hpp>
#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>

#include <rclcpp/macros.hpp>
#include <rclcpp/rclcpp.hpp>

namespace dh_gripper_driver
{
class DHGripperHardwareInterface : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(DHGripperHardwareInterface)

  /**
   * Default constructor.
   */
  DH_DRIVER_PUBLIC
  DHGripperHardwareInterface();

  DH_DRIVER_PUBLIC
  ~DHGripperHardwareInterface();

  /**
   * Constructor with a driver factory. This method is used for testing.
   * @param driver_factory The driver that interact with the hardware.
   */
  explicit DHGripperHardwareInterface(std::unique_ptr<DriverFactory> driver_factory);

  /**
   * Initialization of the hardware interface from data parsed from the
   * robot's URDF.
   * @param hardware_info Structure with data from URDF.
   * @returns CallbackReturn::SUCCESS if required data are provided and can be
   * parsed or CallbackReturn::ERROR if any error happens or data are missing.
   */
  DH_DRIVER_PUBLIC
  CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;

  /**
   * Connect to the hardware.
   * @param previous_state The previous state.
   * @returns CallbackReturn::SUCCESS if required data are provided and can be
   * parsed or CallbackReturn::ERROR if any error happens or data are missing.
   */
  DH_DRIVER_PUBLIC
  CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;

  /**
   * This method exposes position and velocity of joints for reading.
   */
  DH_DRIVER_PUBLIC
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  /**
   * This method exposes the joints targets for writing.
   */
  DH_DRIVER_PUBLIC
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  /**
   * This method is invoked when the hardware is connected.
   * @param previous_state Unconfigured, Inactive, Active or Finalized.
   * @returns CallbackReturn::SUCCESS or CallbackReturn::ERROR.
   */
  DH_DRIVER_PUBLIC
  CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;

  /**
   * This method is invoked when the hardware is disconnected.
   * @param previous_state Unconfigured, Inactive, Active or Finalized.
   * @returns CallbackReturn::SUCCESS or CallbackReturn::ERROR.
   */
  DH_DRIVER_PUBLIC
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

  /**
   * Read data from the hardware.
   */
  DH_DRIVER_PUBLIC
  hardware_interface::return_type read(
    const rclcpp::Time & time,
    const rclcpp::Duration & period) override;

  /**
   * Write data to hardware.
   */
  DH_DRIVER_PUBLIC
  hardware_interface::return_type write(
    const rclcpp::Time & time,
    const rclcpp::Duration & period) override;

protected:
  // Interface to send binary data to the hardware using the serial port.
  std::unique_ptr<Driver> driver_;

  // Factory to create the driver during the initialization step.
  std::unique_ptr<DriverFactory> driver_factory_;

  // We use a thread to read/write to the driver so that we dont block the hardware_interface read/write.
  std::thread communication_thread_;
  std::atomic<bool> communication_thread_is_running_{false};
  void background_task(uint16_t initial_target);
  void store_feedback(
    const Driver::Feedback & feedback,
    std::chrono::steady_clock::time_point acquired = std::chrono::steady_clock::now());
  std::mutex feedback_mutex_;
  Driver::Feedback feedback_;
  std::chrono::steady_clock::time_point feedback_received_{};
  bool feedback_valid_ = false;
  uint64_t feedback_sequence_ = 0;
  double feedback_velocity_ = std::numeric_limits<double>::quiet_NaN();
  double communication_timeout_ = .5;
  double grip_state_ = std::numeric_limits<double>::quiet_NaN();
  double raw_position_ = std::numeric_limits<double>::quiet_NaN();
  double force_percent_ = std::numeric_limits<double>::quiet_NaN();
  double communication_age_ = std::numeric_limits<double>::infinity();
  double sample_sequence_ = 0.;

  double gripper_closed_pos_ = 0.0;

  static constexpr double NO_NEW_CMD_ = std::numeric_limits<double>::quiet_NaN();

  double gripper_position_ = 0.0;
  double gripper_velocity_ = 0.0;
  double gripper_position_command_ = 0.0;

  struct CommandSnapshot { uint16_t position; uint8_t force; };
  CommandSnapshot pending_command() const
  {
    const auto packed = write_request_.load();
    return {static_cast<uint16_t>(packed & 0xffffU), static_cast<uint8_t>(packed >> 16)};
  }
  void queue_command(uint16_t position, uint8_t force)
  {
    write_request_.store(static_cast<uint32_t>(position) | (static_cast<uint32_t>(force) << 16));
  }
  // Position and force belong to one controller update and must be read together.
  std::atomic<uint32_t> write_request_{1000U | (20U << 16)};
  std::atomic<uint16_t> write_speed_{0};
  std::atomic<uint16_t> gripper_current_state_{1000};

  double reactivate_gripper_cmd_ = 0.0;
  std::atomic<bool> reactivate_gripper_async_cmd_{false};
  double reactivate_gripper_response_ = 0.0;
  double gripper_force_ = 0.0;
  double gripper_speed_ = 0.0;
  std::atomic<std::optional<bool>> reactivate_gripper_async_response_{std::nullopt};
};

}  // namespace dh_gripper_driver
