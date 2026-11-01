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

#include <serial/serial.h>

#include <chrono>
#include <iostream>
#include <stdexcept>
#include <thread>

#include "dh_gripper_driver/data_utils.hpp"
#include "dh_gripper_driver/default_driver.hpp"
#include <dh_gripper_driver/crc_utils.hpp>
#include <dh_gripper_driver/driver_exception.hpp>

#include <rclcpp/rclcpp.hpp>

namespace dh_gripper_driver
{
const auto kLogger = rclcpp::get_logger("DefaultDriver");

constexpr uint8_t kReadFunctionCode = 0x03;
constexpr uint16_t kFirstOutputRegister = 0x0200;
constexpr int kReadResponseSize = 7;

constexpr uint8_t kWriteFunctionCode = 0x06;
constexpr uint16_t kActionRequestRegister = 0x0100;
constexpr int kWriteResponseSize = 8;

// If the gripper connection is not stable we may want to try sending the command again.
constexpr auto kMaxRetries = 5;

DefaultDriver::DefaultDriver(std::unique_ptr<Serial> serial)
: serial_{std::move(serial)}
{
}

std::vector<uint8_t> DefaultDriver::send(
  const std::vector<uint8_t> & request,
  size_t response_size) const
{
  std::vector<uint8_t> response;
  response.reserve(response_size);

  int retry_count = 0;
  while (retry_count < kMaxRetries) {
    try {
      serial_->write(request);
      response = serial_->read(response_size);
      if (response.size() != response_size) {
        throw DriverException("Modbus response has an unexpected length");
      }
      if (response[0] != request[0] || response[1] != request[1]) {
        throw DriverException("Modbus response has an unexpected slave/function");
      }
      const std::vector<uint8_t> payload(response.begin(), response.end() - 2);
      const auto crc = crc_utils::compute_crc(payload);
      if (response[response.size() - 2] != data_utils::get_msb(crc) ||
        response.back() != data_utils::get_lsb(crc))
      {
        throw DriverException("Modbus response CRC mismatch");
      }
      if (request[1] == kReadFunctionCode && response[2] != 2) {
        throw DriverException("Modbus register response must contain two data bytes");
      }
      if (request[1] == kWriteFunctionCode && response != request) {
        throw DriverException("Modbus write response does not echo the request");
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
      return response;
    } catch (const serial::IOException & e) {
      RCLCPP_WARN(
        kLogger, "Resending the command because the previous attempt (%d of %d) failed: %s",
        retry_count + 1,
        kMaxRetries, e.what());
      retry_count++;
    }
  }

  if (retry_count == kMaxRetries) {
    RCLCPP_ERROR(kLogger, "Reached maximum retries. Operation failed.");
    throw DriverException("Serial retry limit reached");
  }

  return response;
}

bool DefaultDriver::connect()
{
  serial_->open();
  return serial_->is_open();
}

void DefaultDriver::disconnect()
{
  serial_->close();
}

void DefaultDriver::set_slave_address(uint8_t slave_address)
{
  if (slave_address < 1 || slave_address > 247) {
    throw DriverException("Modbus slave address must be in 1..247");
  }
  slave_address_ = slave_address;
}

void DefaultDriver::activate()
{
  RCLCPP_INFO(kLogger, "Activate...");

  // set rACT to 1, clear all other registers.
  const auto request = create_write_command(kActionRequestRegister, {0x0001});
  auto response = send(request, kWriteResponseSize);
  if (response.empty()) {
    throw DriverException("Failed to activate the gripper.");
  }

  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(10);
  while (true) {
    const auto initialization = read_register(kFirstOutputRegister);
    if (initialization > 2) {throw DriverException("Invalid initialization state");}
    if (initialization == 1) {activation_status_ = ActivationStatus::ACTIVE; return;}
    if (std::chrono::steady_clock::now() >= deadline) {
      throw DriverException("Gripper initialization did not finish within ten seconds");
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
}

void DefaultDriver::deactivate()
{
  // RCLCPP_INFO(kLogger, "Deactivate...");

  // const auto request = create_write_command(kActionRequestRegister, { 0x0000, 0x0000, 0x0000, 0x0000 });
  // auto response = send(request, kWriteResponseSize);
  // if (response.empty())
  // {
  //   throw DriverException{ "Failed to deactivate the gripper." };
  // }
}

void DefaultDriver::set_gripper_position(uint16_t pos)
{
  if (pos > 1000) {throw DriverException("Gripper position must be in 0..1000");}
  send(create_write_command(kActionRequestRegister + 3, {pos}), kWriteResponseSize);
}

uint16_t DefaultDriver::get_gripper_position()
{
  return get_feedback().raw_position;
}

Driver::Feedback DefaultDriver::get_feedback()
{
  update_status();
  return feedback_;
}

bool DefaultDriver::gripper_is_moving()
{
  return get_feedback().grip_state == 0;
}

void DefaultDriver::set_speed(uint8_t speed)
{
  // The published AG Modbus manual documents no independent speed register.
  // Retain this legacy setting without inventing a write to another model's register.
  commanded_gripper_speed_ = speed;
}

void DefaultDriver::set_force(uint8_t force)
{
  if (force < 20 || force > 100) {
    throw DriverException("Gripper force setting must be in 20..100 percent");
  }
  send(create_write_command(kActionRequestRegister + 1, {force}), kWriteResponseSize);
  commanded_gripper_force_ = force;
}

std::vector<uint8_t> DefaultDriver::create_read_command(uint16_t address, uint8_t num_registers)
{
  (void)num_registers;  // This driver reads exactly one documented register per request.
  std::vector<uint8_t> request = {slave_address_,
    kReadFunctionCode,
    data_utils::get_msb(address),
    data_utils::get_lsb(address),
    0x00,
    0x01};
  auto crc = crc_utils::compute_crc(request);
  request.push_back(data_utils::get_msb(crc));
  request.push_back(data_utils::get_lsb(crc));

  return request;
}

std::vector<uint8_t> DefaultDriver::create_write_command(
  uint16_t address,
  const std::vector<uint16_t> & data)
{
  if (data.size() != 1) {throw DriverException("Function 06 writes exactly one register");}

  std::vector<uint8_t> request = {slave_address_,
    kWriteFunctionCode,
    data_utils::get_msb(address),
    data_utils::get_lsb(address),
    data_utils::get_msb(data[0]),
    data_utils::get_lsb(data[0])};

  auto crc = crc_utils::compute_crc(request);
  request.push_back(data_utils::get_msb(crc));
  request.push_back(data_utils::get_lsb(crc));


  return request;
}

uint16_t DefaultDriver::read_register(uint16_t address)
{
  const auto response = send(create_read_command(address, 1), kReadResponseSize);
  const uint16_t value = (static_cast<uint16_t>(response[3]) << 8) | response[4];
  return value;
}

void DefaultDriver::update_status()
{
  // AG Modbus manual V2.5, pp.14-16. A force setting is not a force sensor.
  // Commit the sample only when every response and register value is valid.
  Feedback sample;
  sample.initialization = read_register(kFirstOutputRegister);
  sample.grip_state = read_register(kFirstOutputRegister + 1);
  sample.raw_position = read_register(kFirstOutputRegister + 2);
  sample.force_percent = read_register(kActionRequestRegister + 1);
  if (sample.initialization > 2 || sample.grip_state > 3 || sample.raw_position > 1000 ||
    sample.force_percent < 20 || sample.force_percent > 100)
  {
    throw DriverException("Gripper feedback register value is outside its documented range");
  }
  feedback_ = sample;
  gripper_position_ = sample.raw_position;
  activation_status_ = sample.initialization ==
    1 ? ActivationStatus::ACTIVE : ActivationStatus::RESET;
}
}  // namespace dh_gripper_driver
