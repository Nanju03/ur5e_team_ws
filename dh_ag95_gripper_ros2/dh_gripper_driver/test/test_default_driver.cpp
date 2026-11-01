// SPDX-License-Identifier: BSD-3-Clause
// These tests use a memory-only Serial implementation; no device is opened.
#include <gtest/gtest.h>

#include <array>
#include <chrono>
#include <cstdint>
#include <deque>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include <dh_gripper_driver/default_driver.hpp>
#include <dh_gripper_driver/serial.hpp>

namespace
{
using Bytes = std::vector<uint8_t>;

// Independent bitwise Modbus CRC, deliberately not the driver's lookup table.
Bytes with_crc(Bytes payload)
{
  uint16_t crc = 0xffff;
  for (const uint8_t byte : payload) {
    crc ^= byte;
    for (int bit = 0; bit < 8; ++bit) {
      crc = static_cast<uint16_t>((crc >> 1) ^ ((crc & 1) ? 0xa001 : 0));
    }
  }
  payload.push_back(static_cast<uint8_t>(crc & 0xff));
  payload.push_back(static_cast<uint8_t>(crc >> 8));
  return payload;
}

Bytes read_reply(uint16_t value)
{
  return with_crc({1, 3, 2, static_cast<uint8_t>(value >> 8), static_cast<uint8_t>(value & 0xff)});
}

const Bytes kReadInitialization{0x01, 0x03, 0x02, 0x00, 0x00, 0x01, 0x85, 0xb2};
const Bytes kReadGripState{0x01, 0x03, 0x02, 0x01, 0x00, 0x01, 0xd4, 0x72};
const Bytes kReadPosition{0x01, 0x03, 0x02, 0x02, 0x00, 0x01, 0x24, 0x72};
const Bytes kReadForce{0x01, 0x03, 0x01, 0x01, 0x00, 0x01, 0xd4, 0x36};
const Bytes kWriteForce30{0x01, 0x06, 0x01, 0x01, 0x00, 0x1e, 0x59, 0xfe};
const Bytes kWritePosition500{0x01, 0x06, 0x01, 0x03, 0x01, 0xf4, 0x78, 0x21};

class MemorySerial final : public dh_gripper_driver::Serial
{
public:
  std::deque<Bytes> replies;
  std::vector<Bytes> requests;
  std::vector<size_t> requested_read_sizes;

  void open() override {open_ = true;}
  void close() override {open_ = false;}
  bool is_open() const override {return open_;}
  Bytes read(size_t size) override
  {
    requested_read_sizes.push_back(size);
    if (replies.empty()) {
      throw std::runtime_error("The memory serial has no queued reply");
    }
    auto reply = replies.front();
    replies.pop_front();
    return reply;
  }
  void write(const Bytes & request) override
  {
    if (!open_) {
      throw std::runtime_error("The memory serial is closed");
    }
    requests.push_back(request);
  }
  void set_port(const std::string & port) override {port_ = port;}
  std::string get_port() const override {return port_;}
  void set_timeout(std::chrono::milliseconds timeout) override {timeout_ = timeout;}
  std::chrono::milliseconds get_timeout() const override {return timeout_;}
  void set_baudrate(uint32_t baudrate) override {baudrate_ = baudrate;}
  uint32_t get_baudrate() const override {return baudrate_;}

private:
  bool open_ = false;
  std::string port_ = "memory-only";
  std::chrono::milliseconds timeout_{1};
  uint32_t baudrate_ = 115200;
};

class DefaultDriverTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    auto serial = std::make_unique<MemorySerial>();
    serial_ = serial.get();
    driver_ = std::make_unique<dh_gripper_driver::DefaultDriver>(std::move(serial));
    driver_->set_slave_address(1);
    ASSERT_TRUE(driver_->connect());
  }

  void feedback(
    uint16_t initialization = 1, uint16_t state = 2,
    uint16_t position = 228, uint16_t force = 30)
  {
    serial_->replies.push_back(read_reply(initialization));
    serial_->replies.push_back(read_reply(state));
    serial_->replies.push_back(read_reply(position));
    serial_->replies.push_back(read_reply(force));
  }

  MemorySerial * serial_ = nullptr;
  std::unique_ptr<dh_gripper_driver::DefaultDriver> driver_;
};

TEST_F(DefaultDriverTest, FeedbackUsesOnlyDocumentedReadsAndReportsMeasuredValues)
{
  feedback();
  const auto sample = driver_->get_feedback();
  EXPECT_EQ(sample.initialization, 1);
  EXPECT_EQ(sample.grip_state, 2);
  EXPECT_EQ(sample.raw_position, 228);
  EXPECT_EQ(sample.force_percent, 30);
  EXPECT_EQ(
    serial_->requests,
    (std::vector<Bytes>{kReadInitialization, kReadGripState, kReadPosition, kReadForce}));
  EXPECT_TRUE(serial_->replies.empty());
  for (const auto & request : serial_->requests) {
    ASSERT_GE(request.size(), 2U);
    EXPECT_EQ(request[1], 0x03);
  }
}

TEST_F(DefaultDriverTest, PositionWriteDoesNotSilentlyOverrideTheForceSetting)
{
  serial_->replies.push_back(kWritePosition500);
  driver_->set_gripper_position(500);
  ASSERT_EQ(serial_->requests.size(), 1U);
  EXPECT_EQ(serial_->requests.front(), kWritePosition500);
  EXPECT_EQ(serial_->requested_read_sizes, (std::vector<size_t>{8}));
}

TEST_F(DefaultDriverTest, ForceIsWrittenBeforePositionUsingTheRequestedPercent)
{
  serial_->replies.push_back(kWriteForce30);
  serial_->replies.push_back(kWritePosition500);
  driver_->set_force(30);
  driver_->set_gripper_position(500);
  EXPECT_EQ(serial_->requests, (std::vector<Bytes>{kWriteForce30, kWritePosition500}));
}

TEST_F(DefaultDriverTest, GripperMovingFlagUsesTheDocumentedGripState)
{
  for (uint16_t state = 0; state <= 3; ++state) {
    feedback(1, state);
    EXPECT_EQ(driver_->gripper_is_moving(), state == 0) << "state=" << state;
  }
}

TEST_F(DefaultDriverTest, LegacyPositionGetterReturnsRawSensorPosition)
{
  feedback(1, 1, 731, 20);
  EXPECT_EQ(driver_->get_gripper_position(), 731);
}

TEST_F(DefaultDriverTest, FeedbackAcceptsAllDocumentedInitializationStates)
{
  for (uint16_t state = 0; state <= 2; ++state) {
    feedback(state);
    EXPECT_EQ(driver_->get_feedback().initialization, state);
  }
}

TEST_F(DefaultDriverTest, InvalidForceCommandsAreRejectedBeforeSerialWrites)
{
  for (const uint8_t value : std::array<uint8_t, 4>{0, 19, 101, 255}) {
    EXPECT_ANY_THROW(driver_->set_force(value)) << "force=" << static_cast<int>(value);
  }
  EXPECT_TRUE(serial_->requests.empty());
}

TEST_F(DefaultDriverTest, DocumentedForceAndPositionEndpointsAreAccepted)
{
  for (const uint8_t force : std::array<uint8_t, 2>{20, 100}) {
    const auto echo = with_crc({1, 6, 1, 1, 0, force});
    serial_->replies.push_back(echo);
    EXPECT_NO_THROW(driver_->set_force(force));
    EXPECT_EQ(serial_->requests.back(), echo);
  }
  for (const uint16_t position : std::array<uint16_t, 2>{0, 1000}) {
    const auto echo = with_crc(
      {1, 6, 1, 3, static_cast<uint8_t>(position >> 8),
        static_cast<uint8_t>(position & 0xff)});
    serial_->replies.push_back(echo);
    EXPECT_NO_THROW(driver_->set_gripper_position(position));
    EXPECT_EQ(serial_->requests.back(), echo);
  }
}

TEST_F(DefaultDriverTest, DocumentedSensorPositionAndForceEndpointsAreAccepted)
{
  feedback(1, 1, 0, 20);
  const auto closed = driver_->get_feedback();
  EXPECT_EQ(closed.raw_position, 0);
  EXPECT_EQ(closed.force_percent, 20);
  feedback(1, 1, 1000, 100);
  const auto open = driver_->get_feedback();
  EXPECT_EQ(open.raw_position, 1000);
  EXPECT_EQ(open.force_percent, 100);
}

TEST_F(DefaultDriverTest, InvalidPositionCommandsAreRejectedBeforeSerialWrites)
{
  EXPECT_ANY_THROW(driver_->set_gripper_position(1001));
  EXPECT_ANY_THROW(driver_->set_gripper_position(65535));
  EXPECT_TRUE(serial_->requests.empty());
}

class MalformedReadTest : public DefaultDriverTest, public ::testing::WithParamInterface<Bytes>
{
};

TEST_P(MalformedReadTest, InvalidResponseCannotBecomeFeedback)
{
  serial_->replies.push_back(GetParam());
  EXPECT_ANY_THROW(driver_->get_feedback());
}

INSTANTIATE_TEST_SUITE_P(
  ReadValidation, MalformedReadTest,
  ::testing::Values(
    Bytes{}, Bytes{1, 3},
    // Complete-looking replies with wrong CRC, slave, function, byte count,
    // Modbus exception, or surplus bytes, respectively.
    Bytes{1, 3, 2, 0, 1, 0, 0},
    with_crc({2, 3, 2, 0, 1}),
    with_crc({1, 4, 2, 0, 1}),
    with_crc({1, 3, 1, 0, 1}),
    with_crc({1, 0x83, 2}),
    with_crc({1, 3, 2, 0, 1, 0})));

class InvalidFeedbackValueTest : public DefaultDriverTest,
  public ::testing::WithParamInterface<std::array<uint16_t, 4>>
{
};

TEST_P(InvalidFeedbackValueTest, OutOfRangeRegistersCannotBecomeFeedback)
{
  const auto values = GetParam();
  feedback(values[0], values[1], values[2], values[3]);
  EXPECT_ANY_THROW(driver_->get_feedback());
}

INSTANTIATE_TEST_SUITE_P(
  RegisterRanges, InvalidFeedbackValueTest,
  ::testing::Values(
    std::array<uint16_t, 4>{3, 2, 228, 30},
    std::array<uint16_t, 4>{1, 4, 228, 30},
    std::array<uint16_t, 4>{1, 2, 1001, 30},
    std::array<uint16_t, 4>{1, 2, 228, 19},
    std::array<uint16_t, 4>{1, 2, 228, 101}));

class MalformedWriteTest : public DefaultDriverTest, public ::testing::WithParamInterface<Bytes>
{
};

TEST_P(MalformedWriteTest, PositionRequiresAValidatedMatchingWriteEcho)
{
  serial_->replies.push_back(GetParam());
  EXPECT_ANY_THROW(driver_->set_gripper_position(500));
}

INSTANTIATE_TEST_SUITE_P(
  WriteValidation, MalformedWriteTest,
  ::testing::Values(
    Bytes{}, Bytes{1, 6, 1},
    Bytes{1, 6, 1, 3, 1, 0xf4, 0, 0},
    with_crc({2, 6, 1, 3, 1, 0xf4}),
    with_crc({1, 3, 1, 3, 1, 0xf4}),
    with_crc({1, 6, 1, 1, 1, 0xf4}),
    with_crc({1, 6, 1, 3, 1, 0xf5}),
    with_crc({1, 6, 1, 3, 1, 0xf4, 0})));

TEST_F(DefaultDriverTest, ForceRequiresAValidatedMatchingWriteEcho)
{
  serial_->replies.push_back(with_crc({1, 6, 1, 1, 0, 31}));
  EXPECT_ANY_THROW(driver_->set_force(30));
}

TEST_F(DefaultDriverTest, FeedbackRequestCrcFixturesMatchTheIndependentImplementation)
{
  EXPECT_EQ(kReadInitialization, with_crc({1, 3, 2, 0, 0, 1}));
  EXPECT_EQ(kReadGripState, with_crc({1, 3, 2, 1, 0, 1}));
  EXPECT_EQ(kReadPosition, with_crc({1, 3, 2, 2, 0, 1}));
  EXPECT_EQ(kReadForce, with_crc({1, 3, 1, 1, 0, 1}));
  EXPECT_EQ(kWriteForce30, with_crc({1, 6, 1, 1, 0, 30}));
  EXPECT_EQ(kWritePosition500, with_crc({1, 6, 1, 3, 1, 0xf4}));
}
}  // namespace
