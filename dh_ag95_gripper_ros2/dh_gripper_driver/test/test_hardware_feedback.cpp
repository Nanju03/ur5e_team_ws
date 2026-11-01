#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <limits>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

#include <dh_gripper_driver/driver.hpp>
#include <dh_gripper_driver/driver_factory.hpp>
#include <dh_gripper_driver/hardware_interface.hpp>

namespace
{
using dh_gripper_driver::Driver;
using dh_gripper_driver::DriverFactory;
using dh_gripper_driver::DHGripperHardwareInterface;
using CallbackReturn = hardware_interface::CallbackReturn;
using ReturnType = hardware_interface::return_type;
using Clock = std::chrono::steady_clock;

struct DriverCalls
{
  unsigned create = 0;
  unsigned connect = 0;
  unsigned disconnect = 0;
  unsigned activate = 0;
  unsigned deactivate = 0;
  unsigned read = 0;
  unsigned position = 0;
  unsigned force = 0;
  unsigned speed = 0;
  std::function<Driver::Feedback()> feedback_callback;
  std::function<void(uint16_t)> position_callback;
  std::function<void(uint8_t)> force_callback;
  std::vector<std::string> events;
  std::vector<uint16_t> positions;
  std::vector<uint8_t> forces;
};

// No port, ROS node, background thread, or operating-system device is involved.
class FakeDriver final : public Driver
{
public:
  explicit FakeDriver(std::shared_ptr<DriverCalls> calls)
  : calls_(std::move(calls)) {}
  void set_slave_address(uint8_t) override {}
  bool connect() override {++calls_->connect; return true;}
  void disconnect() override {++calls_->disconnect;}
  void activate() override {++calls_->activate;}
  void deactivate() override {++calls_->deactivate;}
  void set_gripper_position(uint16_t position) override
  {
    ++calls_->position;
    calls_->positions.push_back(position);
    calls_->events.push_back("position");
    if (calls_->position_callback) {calls_->position_callback(position);}
  }
  uint16_t get_gripper_position() override {++calls_->read; return 700;}
  Feedback get_feedback() override
  {
    ++calls_->read;
    calls_->events.push_back("read");
    return calls_->feedback_callback ? calls_->feedback_callback() : Feedback{1, 2, 700, 40};
  }
  bool gripper_is_moving() override {++calls_->read; return false;}
  void set_speed(uint8_t) override {++calls_->speed;}
  void set_force(uint8_t force) override
  {
    ++calls_->force;
    calls_->forces.push_back(force);
    calls_->events.push_back("force");
    if (calls_->force_callback) {calls_->force_callback(force);}
  }

private:
  std::shared_ptr<DriverCalls> calls_;
};

class FakeDriverFactory final : public DriverFactory
{
public:
  explicit FakeDriverFactory(std::shared_ptr<DriverCalls> calls)
  : calls_(std::move(calls)) {}
  std::unique_ptr<Driver> create(const hardware_interface::HardwareInfo &) const override
  {
    ++calls_->create;
    return std::make_unique<FakeDriver>(calls_);
  }

private:
  std::shared_ptr<DriverCalls> calls_;
};

class TestHardware final : public DHGripperHardwareInterface
{
public:
  explicit TestHardware(const std::shared_ptr<DriverCalls> & calls)
  : DHGripperHardwareInterface(std::make_unique<FakeDriverFactory>(calls)) {}

  void receive(const Driver::Feedback & feedback, Clock::time_point acquired = Clock::now())
  {
    store_feedback(feedback, acquired);
  }
  void queue(uint16_t position, uint8_t force) {queue_command(position, force);}
  void stop_worker() {communication_thread_is_running_.store(false);}
  void run_worker(uint16_t initial_target)
  {
    communication_thread_is_running_.store(true);
    background_task(initial_target);  // Synchronous: never starts a thread.
  }
  void set_feedback_age(double seconds)
  {
    std::lock_guard<std::mutex> lock(feedback_mutex_);
    feedback_received_ = Clock::now() -
      std::chrono::duration_cast<Clock::duration>(std::chrono::duration<double>(seconds));
  }
  Clock::time_point received_time()
  {
    std::lock_guard<std::mutex> lock(feedback_mutex_);
    return feedback_received_;
  }
  uint64_t sequence()
  {
    std::lock_guard<std::mutex> lock(feedback_mutex_);
    return feedback_sequence_;
  }
  uint16_t queued_position() const {return pending_command().position;}
  uint8_t queued_force() const {return pending_command().force;}
};

hardware_interface::InterfaceInfo interface(const std::string & name)
{
  hardware_interface::InterfaceInfo value;
  value.name = name;
  return value;
}

hardware_interface::HardwareInfo hardware_info(bool extended = true)
{
  hardware_interface::HardwareInfo info;
  info.name = "memory_only_ag95";
  info.type = "system";
  info.hardware_parameters = {
    {"gripper_closed_position", "0.93"},
    {"gripper_force_multiplier", "0.25"},
    {"gripper_speed_multiplier", "0.5"},
    {"communication_timeout", "0.5"}};
  hardware_interface::ComponentInfo joint;
  joint.name = "left_outer_knuckle_joint";
  joint.type = "joint";
  joint.state_interfaces = {interface("position"), interface("velocity")};
  if (extended) {
    for (const auto & name :
      {"grip_state", "raw_position", "force_percent", "communication_age", "sample_sequence"})
    {
      joint.state_interfaces.push_back(interface(name));
    }
  }
  joint.command_interfaces = {
    interface("position"), interface("set_gripper_max_effort"),
    interface("set_gripper_max_velocity")};
  info.joints.push_back(joint);
  return info;
}

class HardwareFeedbackTest : public ::testing::Test
{
protected:
  HardwareFeedbackTest()
  : calls_(std::make_shared<DriverCalls>()), hardware_(calls_) {}
  void initialize(hardware_interface::HardwareInfo info = hardware_info())
  {
    ASSERT_EQ(hardware_.on_init(info), CallbackReturn::SUCCESS);
    states_ = hardware_.export_state_interfaces();
    commands_ = hardware_.export_command_interfaces();
  }
  double state(const std::string & name) const
  {
    const auto found = std::find_if(
      states_.begin(), states_.end(), [&](const auto & handle) {
        return handle.get_interface_name() == name;
      });
    if (found == states_.end()) {throw std::logic_error("Missing state interface: " + name);}
    return found->get_value();
  }
  void command(const std::string & name, double value)
  {
    const auto found = std::find_if(
      commands_.begin(), commands_.end(), [&](const auto & handle) {
        return handle.get_interface_name() == name;
      });
    if (found == commands_.end()) {throw std::logic_error("Missing command interface: " + name);}
    found->set_value(value);
  }
  double command_value(const std::string & name) const
  {
    const auto found = std::find_if(
      commands_.begin(), commands_.end(), [&](const auto & handle) {
        return handle.get_interface_name() == name;
      });
    if (found == commands_.end()) {throw std::logic_error("Missing command interface: " + name);}
    return found->get_value();
  }
  ReturnType read() {return hardware_.read(rclcpp::Time(0), rclcpp::Duration(0, 0));}
  ReturnType write() {return hardware_.write(rclcpp::Time(0), rclcpp::Duration(0, 0));}

  std::shared_ptr<DriverCalls> calls_;
  TestHardware hardware_;
  std::vector<hardware_interface::StateInterface> states_;
  std::vector<hardware_interface::CommandInterface> commands_;
};

TEST_F(HardwareFeedbackTest, LegacyPositionAndVelocityStateDeclarationsRemainSupported)
{
  auto info = hardware_info(false);
  info.joints.front().command_interfaces = {interface("position")};
  initialize(info);
  ASSERT_EQ(states_.size(), 2U);
  EXPECT_EQ(states_[0].get_name(), "left_outer_knuckle_joint/position");
  EXPECT_EQ(states_[1].get_name(), "left_outer_knuckle_joint/velocity");
  EXPECT_EQ(commands_.size(), 3U);  // Position and the two legacy reactivation handles.
}

TEST_F(HardwareFeedbackTest, ExtendedDeclarationsExportSevenLiveStateHandles)
{
  initialize();
  ASSERT_EQ(states_.size(), 7U);
  for (const auto & handle : states_) {
    EXPECT_EQ(handle.get_prefix_name(), "left_outer_knuckle_joint");
  }
  EXPECT_TRUE(std::isnan(state("position")));
  EXPECT_TRUE(std::isnan(state("velocity")));
  EXPECT_EQ(state("sample_sequence"), 0.);
  EXPECT_TRUE(std::isinf(state("communication_age")));
}

TEST_F(HardwareFeedbackTest, InitialCommandsUseParameterValuesRatherThanMapEntryCounts)
{
  initialize();
  EXPECT_DOUBLE_EQ(command_value("set_gripper_max_effort"), .25);
  EXPECT_DOUBLE_EQ(command_value("set_gripper_max_velocity"), .5);
  EXPECT_EQ(hardware_.queued_force(), 40);
}

TEST_F(HardwareFeedbackTest, HalfForceAndQuarterSpeedParametersAreAlsoPreserved)
{
  auto info = hardware_info();
  info.hardware_parameters["gripper_force_multiplier"] = "0.5";
  info.hardware_parameters["gripper_speed_multiplier"] = "0.25";
  initialize(info);
  EXPECT_DOUBLE_EQ(command_value("set_gripper_max_effort"), .5);
  EXPECT_DOUBLE_EQ(command_value("set_gripper_max_velocity"), .25);
  EXPECT_EQ(hardware_.queued_force(), 60);
}

TEST_F(HardwareFeedbackTest, ReadyFeedbackMapsActualRawPositionAndExportsOneFreshSample)
{
  initialize();
  hardware_.receive({1, 2, 228, 40});
  ASSERT_EQ(read(), ReturnType::OK);
  EXPECT_NEAR(state("position"), .93 * .772, 1.e-12);
  EXPECT_DOUBLE_EQ(state("raw_position"), 228.);
  EXPECT_DOUBLE_EQ(state("grip_state"), 2.);
  EXPECT_DOUBLE_EQ(state("force_percent"), 40.);
  EXPECT_DOUBLE_EQ(state("sample_sequence"), 1.);
  EXPECT_GE(state("communication_age"), 0.);
  EXPECT_LT(state("communication_age"), .5);
  EXPECT_TRUE(std::isnan(state("velocity")));  // One position is not a velocity measurement.
}

TEST_F(HardwareFeedbackTest, VelocityUsesActualRawPositionChangeAndSteadySampleInterval)
{
  initialize();
  hardware_.receive({1, 0, 800, 40});
  hardware_.set_feedback_age(.1);
  const auto previous_time = hardware_.received_time();
  hardware_.receive({1, 2, 700, 40});
  const double interval =
    std::chrono::duration<double>(hardware_.received_time() - previous_time).count();
  ASSERT_EQ(read(), ReturnType::OK);
  EXPECT_GT(state("velocity"), 0.);
  EXPECT_NEAR(state("velocity"), .93 * .1 / interval, 1.e-10);
  EXPECT_DOUBLE_EQ(state("sample_sequence"), 2.);
}

TEST_F(HardwareFeedbackTest, StaleFeedbackKeepsLastPositionAndSequenceButInvalidatesVelocity)
{
  initialize();
  hardware_.receive({1, 0, 800, 40});
  hardware_.set_feedback_age(.1);
  hardware_.receive({1, 2, 700, 40});
  ASSERT_EQ(read(), ReturnType::OK);
  ASSERT_TRUE(std::isfinite(state("velocity")));
  hardware_.set_feedback_age(1.);
  ASSERT_EQ(read(), ReturnType::OK);
  EXPECT_NEAR(state("position"), .279, 1.e-12);
  EXPECT_DOUBLE_EQ(state("raw_position"), 700.);
  EXPECT_DOUBLE_EQ(state("sample_sequence"), 2.);
  EXPECT_GE(state("communication_age"), 1.);
  EXPECT_TRUE(std::isnan(state("velocity")));
}

TEST_F(HardwareFeedbackTest, FirstSampleAfterStaleIntervalDoesNotInventAnAverageVelocity)
{
  initialize();
  hardware_.receive({1, 0, 800, 40});
  hardware_.set_feedback_age(1.);
  hardware_.receive({1, 2, 700, 40});
  ASSERT_EQ(read(), ReturnType::OK);
  EXPECT_TRUE(std::isnan(state("velocity")));
  EXPECT_DOUBLE_EQ(state("sample_sequence"), 2.);
  EXPECT_LT(state("communication_age"), .5);
}

TEST_F(HardwareFeedbackTest, RepeatedReadDoesNotPretendCachedDataIsANewSensorSample)
{
  initialize();
  hardware_.receive({1, 1, 500, 60});
  ASSERT_EQ(read(), ReturnType::OK);
  hardware_.set_feedback_age(.2);
  ASSERT_EQ(read(), ReturnType::OK);
  EXPECT_DOUBLE_EQ(state("sample_sequence"), 1.);
  EXPECT_GE(state("communication_age"), .2);
  EXPECT_NEAR(state("position"), .465, 1.e-12);
}

TEST_F(HardwareFeedbackTest, AcquisitionTimeIsPreservedRatherThanReplacingItWithPublicationTime)
{
  initialize();
  const auto acquired = Clock::now() - std::chrono::milliseconds(200);
  hardware_.receive({1, 2, 700, 40}, acquired);
  EXPECT_EQ(hardware_.received_time(), acquired);
  ASSERT_EQ(read(), ReturnType::OK);
  EXPECT_GE(state("communication_age"), .2);
  EXPECT_LT(state("communication_age"), .5);
  EXPECT_DOUBLE_EQ(state("sample_sequence"), 1.);
}

TEST_F(HardwareFeedbackTest, StaleAcquisitionCannotRefreshOrReplaceTheLastSensorSample)
{
  initialize();
  hardware_.receive({1, 2, 700, 40});
  const auto previous_time = hardware_.received_time();
  const auto stale = Clock::now() - std::chrono::seconds(1);
  EXPECT_THROW(hardware_.receive({1, 1, 1000, 60}, stale), std::exception);
  EXPECT_EQ(hardware_.received_time(), previous_time);
  EXPECT_EQ(hardware_.sequence(), 1U);
  ASSERT_EQ(read(), ReturnType::OK);
  EXPECT_DOUBLE_EQ(state("raw_position"), 700.);
  EXPECT_DOUBLE_EQ(state("force_percent"), 40.);
}

class InvalidFeedbackTest : public HardwareFeedbackTest,
  public ::testing::WithParamInterface<Driver::Feedback> {};

TEST_P(InvalidFeedbackTest, DoesNotReplaceTheLastValidatedSampleOrRefreshItsTime)
{
  initialize();
  hardware_.receive({1, 2, 700, 40});
  hardware_.set_feedback_age(.2);
  const auto previous_time = hardware_.received_time();
  const auto previous_sequence = hardware_.sequence();
  EXPECT_THROW(hardware_.receive(GetParam()), std::exception);
  EXPECT_EQ(hardware_.received_time(), previous_time);
  EXPECT_EQ(hardware_.sequence(), previous_sequence);
  ASSERT_EQ(read(), ReturnType::OK);
  EXPECT_DOUBLE_EQ(state("sample_sequence"), 1.);
  EXPECT_DOUBLE_EQ(state("raw_position"), 700.);
  EXPECT_DOUBLE_EQ(state("grip_state"), 2.);
  EXPECT_DOUBLE_EQ(state("force_percent"), 40.);
  EXPECT_GE(state("communication_age"), .2);
}

INSTANTIATE_TEST_SUITE_P(
  NotReadyOrOutsideDocumentedRange, InvalidFeedbackTest, ::testing::Values(
    Driver::Feedback{0, 2, 700, 40}, Driver::Feedback{2, 2, 700, 40},
    Driver::Feedback{3, 2, 700, 40},
    Driver::Feedback{1, 4, 700, 40}, Driver::Feedback{1, 2, 1001, 40},
    Driver::Feedback{1, 2, 700, 19}, Driver::Feedback{1, 2, 700, 101}));

TEST_F(HardwareFeedbackTest, PositionCommandsClampToPhysicalJointRangeAndInvertRawMapping)
{
  initialize();
  command("position", -.1);
  ASSERT_EQ(write(), ReturnType::OK);
  EXPECT_EQ(hardware_.queued_position(), 1000);
  command("position", .465);
  ASSERT_EQ(write(), ReturnType::OK);
  EXPECT_EQ(hardware_.queued_position(), 500);
  command("position", 1.5);
  ASSERT_EQ(write(), ReturnType::OK);
  EXPECT_EQ(hardware_.queued_position(), 0);
}

TEST_F(HardwareFeedbackTest, NormalizedForceCommandMapsToDocumentedTwentyToHundredPercentRange)
{
  initialize();
  command("position", .465);
  for (const auto & setting : std::vector<std::pair<double, uint8_t>>{
      {-1., 20}, {0., 20}, {.25, 40}, {.5, 60}, {1., 100}, {2., 100}})
  {
    command("set_gripper_max_effort", setting.first);
    ASSERT_EQ(write(), ReturnType::OK);
    EXPECT_EQ(hardware_.queued_force(), setting.second);
  }
}

TEST_F(HardwareFeedbackTest, NonfinitePositionOrForceRejectsWholeCommandWithoutChangingQueuedValues)
{
  initialize();
  const double nan = std::numeric_limits<double>::quiet_NaN();
  const double inf = std::numeric_limits<double>::infinity();
  command("position", .465);
  command("set_gripper_max_effort", .25);
  ASSERT_EQ(write(), ReturnType::OK);
  for (double invalid : {nan, inf, -inf}) {
    command("position", invalid);
    EXPECT_EQ(write(), ReturnType::ERROR);
    EXPECT_EQ(hardware_.queued_position(), 500);
    EXPECT_EQ(hardware_.queued_force(), 40);
    command("position", .1);
    command("set_gripper_max_effort", invalid);
    EXPECT_EQ(write(), ReturnType::ERROR);
    EXPECT_EQ(hardware_.queued_position(), 500);
    EXPECT_EQ(hardware_.queued_force(), 40);
    command("set_gripper_max_effort", .25);
  }
}

TEST_F(HardwareFeedbackTest, OfflineInitializationReadAndWriteNeverCallTheDeviceDriver)
{
  initialize();
  hardware_.receive({1, 2, 700, 40});
  ASSERT_EQ(read(), ReturnType::OK);
  command("position", .465);
  ASSERT_EQ(write(), ReturnType::OK);
  EXPECT_EQ(calls_->create, 1U);
  EXPECT_EQ(calls_->connect, 0U);
  EXPECT_EQ(calls_->activate, 0U);
  EXPECT_EQ(calls_->deactivate, 0U);
  EXPECT_EQ(calls_->read, 0U);
  EXPECT_EQ(calls_->position, 0U);
  EXPECT_EQ(calls_->force, 0U);
  EXPECT_EQ(calls_->speed, 0U);
}

TEST_F(HardwareFeedbackTest, FirstQueuedTargetBeforeWorkerStartsIsNotMistakenForAnAlreadySentTarget)
{
  initialize();
  hardware_.queue(400, 40);
  calls_->feedback_callback = [this] {
      if (calls_->read >= 3) {
        hardware_.stop_worker();                       // Bounded failure path.
      }
      return Driver::Feedback{1, 2, 700, 40};
    };
  calls_->position_callback = [this](uint16_t) {hardware_.stop_worker();};
  hardware_.run_worker(700);  // Actual measured initial position, not the queued target.
  ASSERT_EQ(calls_->positions.size(), 1U);
  EXPECT_EQ(calls_->positions.front(), 400);
  EXPECT_EQ(calls_->force, 0U);
}

TEST_F(HardwareFeedbackTest, PositionWaitsForALaterSensorReadConfirmingTheRequestedForce)
{
  initialize();
  hardware_.queue(400, 40);
  calls_->feedback_callback = [this] {
      if (calls_->read >= 3) {hardware_.stop_worker();}
      return Driver::Feedback{1, 2, 700, static_cast<uint16_t>(calls_->read == 1 ? 60 : 40)};
    };
  calls_->position_callback = [this](uint16_t) {hardware_.stop_worker();};
  hardware_.run_worker(700);
  EXPECT_EQ(calls_->events, (std::vector<std::string>{"read", "force", "read", "position"}));
  ASSERT_EQ(calls_->forces.size(), 1U);
  EXPECT_EQ(calls_->forces.front(), 40);
  ASSERT_EQ(calls_->positions.size(), 1U);
  EXPECT_EQ(calls_->positions.front(), 400);
}

TEST_F(HardwareFeedbackTest, ForceEchoMismatchNeverPermitsAPositionWrite)
{
  initialize();
  hardware_.queue(400, 40);
  calls_->feedback_callback = [this] {
      if (calls_->read >= 3) {hardware_.stop_worker();}
      return Driver::Feedback{1, 2, 700, 60};
    };
  hardware_.run_worker(700);
  EXPECT_EQ(calls_->position, 0U);
  ASSERT_FALSE(calls_->forces.empty());
  for (auto force : calls_->forces) {
    EXPECT_EQ(force, 40);
  }
}

TEST_F(HardwareFeedbackTest, CommandChangedDuringAReadUsesTheMatchingForceAndPositionPair)
{
  initialize();
  hardware_.queue(700, 60);
  calls_->feedback_callback = [this] {
      if (calls_->read == 1) {hardware_.queue(400, 40);}
      if (calls_->read >= 4) {hardware_.stop_worker();}
      return Driver::Feedback{1, 2, 700, static_cast<uint16_t>(calls_->force == 0 ? 60 : 40)};
    };
  calls_->position_callback = [this](uint16_t) {
      EXPECT_GE(calls_->read, 2U);
      ASSERT_FALSE(calls_->forces.empty());
      EXPECT_EQ(calls_->forces.back(), 40);
      hardware_.stop_worker();
    };
  hardware_.run_worker(700);
  ASSERT_EQ(calls_->positions.size(), 1U);
  EXPECT_EQ(calls_->positions.front(), 400);
}

TEST_F(HardwareFeedbackTest, StopRequestedDuringReadPreventsBothForceAndPositionWrites)
{
  initialize();
  hardware_.queue(400, 40);
  calls_->feedback_callback = [this] {
      hardware_.stop_worker();
      return Driver::Feedback{1, 2, 700, 60}; // Would otherwise request a force write.
    };
  hardware_.run_worker(700);
  EXPECT_EQ(calls_->read, 1U);
  EXPECT_EQ(calls_->force, 0U);
  EXPECT_EQ(calls_->position, 0U);
}

TEST_F(HardwareFeedbackTest, StopRequestedDuringAForceMatchedReadPreventsPositionWrite)
{
  initialize();
  hardware_.queue(400, 40);
  calls_->feedback_callback = [this] {
      hardware_.stop_worker();
      return Driver::Feedback{1, 2, 700, 40};
    };
  hardware_.run_worker(700);
  EXPECT_EQ(calls_->read, 1U);
  EXPECT_EQ(calls_->force, 0U);
  EXPECT_EQ(calls_->position, 0U);
}

class InvalidClosedPositionTest : public HardwareFeedbackTest,
  public ::testing::WithParamInterface<const char *> {};

TEST_P(InvalidClosedPositionTest, RejectsBeforeCreatingDriver)
{
  auto info = hardware_info();
  info.hardware_parameters["gripper_closed_position"] = GetParam();
  EXPECT_EQ(hardware_.on_init(info), CallbackReturn::ERROR);
  EXPECT_EQ(calls_->create, 0U);
}

INSTANTIATE_TEST_SUITE_P(
  NonpositiveOrNonfinite, InvalidClosedPositionTest,
  ::testing::Values("-0.93", "0", "nan", "inf", "-inf"));

TEST_F(HardwareFeedbackTest, UnsupportedCommandInterfaceIsRejected)
{
  auto info = hardware_info();
  info.joints.front().command_interfaces.push_back(interface("effort"));
  EXPECT_EQ(hardware_.on_init(info), CallbackReturn::ERROR);
  EXPECT_EQ(calls_->create, 0U);
}

TEST_F(HardwareFeedbackTest, UnsupportedStateInterfaceIsRejected)
{
  auto info = hardware_info();
  info.joints.front().state_interfaces.push_back(interface("motor_current"));
  EXPECT_EQ(hardware_.on_init(info), CallbackReturn::ERROR);
  EXPECT_EQ(calls_->create, 0U);
}

TEST_F(HardwareFeedbackTest, DuplicateStateOrCommandInterfacesAreRejected)
{
  auto info = hardware_info();
  info.joints.front().state_interfaces.push_back(interface("position"));
  EXPECT_EQ(hardware_.on_init(info), CallbackReturn::ERROR);
  info = hardware_info();
  info.joints.front().command_interfaces.push_back(interface("position"));
  EXPECT_EQ(hardware_.on_init(info), CallbackReturn::ERROR);
  EXPECT_EQ(calls_->create, 0U);
}

TEST_F(HardwareFeedbackTest, RequiredPositionCommandAndVelocityStateCannotBeOmitted)
{
  auto info = hardware_info();
  info.joints.front().command_interfaces = {interface("set_gripper_max_effort")};
  EXPECT_EQ(hardware_.on_init(info), CallbackReturn::ERROR);
  info = hardware_info();
  info.joints.front().state_interfaces = {interface("position")};
  EXPECT_EQ(hardware_.on_init(info), CallbackReturn::ERROR);
  EXPECT_EQ(calls_->create, 0U);
}
}  // namespace
