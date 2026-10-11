#include "sil.hpp"

#include <atomic>
#include <array>
#include <chrono>
#include <condition_variable>
#include <cstddef>
#include <deque>
#include <mutex>
#include <thread>

namespace raceup::sil {
namespace {

SilSystem* active_system = nullptr;

constexpr auto kCyclePeriod = std::chrono::milliseconds(10);
constexpr std::size_t kQueueCapacity = 64U;

constexpr std::size_t sensor_index(const SensorId id) {
  return static_cast<std::size_t>(id);
}

constexpr std::size_t output_index(const OutputId id) {
  return static_cast<std::size_t>(id);
}

}  // namespace

struct SilSystem::Impl {
  struct State {
    // All simulator-owned state is protected by mutex. The two queues have
    // deliberately different meanings: ebs_input is consumed by the EBS;
    // emitted is the monitor queue consumed by receive_frame().
    std::array<std::uint16_t, sensor_index(SensorId::Count)> sensors{};
    std::array<std::uint16_t, output_index(OutputId::Count)> outputs{};
    std::deque<Frame> ebs_input;
    std::deque<Frame> emitted;
    std::mutex mutex;
    std::condition_variable emitted_available;
  } state;

  std::atomic<bool> stop_requested{false};
  std::atomic<bool> running{false};
  std::thread smu_thread;
  std::thread ebs_thread;
};

SilSystem::SilSystem() : impl_(std::make_unique<Impl>()) {}

SilSystem::~SilSystem() { stop(); }

bool SilSystem::start() {
  if (impl_->running.exchange(true)) {
    return true;
  }

  active_system = this;
  impl_->stop_requested.store(false);
  smu::reset();
  ebs::reset();

  impl_->smu_thread = std::thread([this] {
    auto next = std::chrono::steady_clock::now();
    while (!impl_->stop_requested.load()) {
      next += kCyclePeriod;
      smu::run_cycle();
      std::this_thread::sleep_until(next);
    }
  });

  impl_->ebs_thread = std::thread([this] {
    auto next = std::chrono::steady_clock::now();
    while (!impl_->stop_requested.load()) {
      next += kCyclePeriod;
      ebs::run_cycle();
      std::this_thread::sleep_until(next);
    }
  });

  return true;
}

void SilSystem::stop() {
  if (!impl_->running.exchange(false)) {
    return;
  }

  impl_->stop_requested.store(true);
  if (impl_->smu_thread.joinable()) {
    impl_->smu_thread.join();
  }
  if (impl_->ebs_thread.joinable()) {
    impl_->ebs_thread.join();
  }
  if (active_system == this) {
    active_system = nullptr;
  }
}

bool SilSystem::running() const { return impl_->running.load(); }

void SilSystem::set_sensor(const SensorId id, const std::uint16_t value) {
  // TODO(candidate): protect state.sensors with state.mutex and store value.
  // SensorId::Count is not a valid sensor and does not need to be accepted.
  if (id >= SensorId::Count) {
    return;
  }
  std::lock_guard<std::mutex> lock(impl_->state.mutex);
  impl_->state.sensors[sensor_index(id)] = value;

}

std::uint16_t SilSystem::output(const OutputId id) const {
  // TODO(candidate): protect state.outputs with state.mutex and return the
  // requested value.
  std::lock_guard<std::mutex> lock(impl_->state.mutex);
  return impl_->state.outputs[output_index(id)];
}

void SilSystem::send_frame(const Frame& frame) {
  // TODO(candidate): protect state.ebs_input with state.mutex and enqueue the
  // frame. If the queue reaches kQueueCapacity, discard the oldest input
  // before adding the new one.
  
  std::lock_guard<std::mutex> lock(impl_->state.mutex);
  // To guarantee strong exception safety and avoid failures on memory allocation (std::bad_alloc)
  // the new frame is inserted before enforcing the capacity limit.
  // So, if memory allocation fails, the queue is left completely unmodified and no frames are lost.
  
  impl_->state.ebs_input.push_back(frame);
  
  if (impl_->state.ebs_input.size() > kQueueCapacity) {
    impl_->state.ebs_input.pop_front();
  }

}

bool SilSystem::receive_frame(Frame& frame,
                              const std::chrono::milliseconds timeout) {
  // TODO(candidate): acquire state.mutex with a std::unique_lock and wait on
  // state.emitted_available until state.emitted is non-empty or timeout
  // expires. On success, move the oldest frame to frame.
  std::unique_lock<std::mutex> lock(impl_->state.mutex);
}

std::uint16_t SilSystem::read_sensor(const SensorId id) const {
  // TODO(candidate): protect state.sensors with state.mutex and return the
  // requested value.
  (void)id;
  return 0U;
}

bool SilSystem::try_read(const Ecu ecu, Frame& frame) {
  // TODO(candidate): protect state.ebs_input with state.mutex. Only Ecu::Ebs
  // has an input queue in this exercise. If it is empty, return false;
  // otherwise pop its oldest frame.
  (void)ecu;
  (void)frame;
  return false;
}

void SilSystem::write(const Ecu ecu, const Frame& frame) {
  // TODO(candidate): protect both queues with state.mutex. Every ECU frame
  // goes to state.emitted. A frame written by the SMU must also be copied to
  // state.ebs_input. Keep both queues bounded by kQueueCapacity, then notify
  // waiting test code through state.emitted_available.
  (void)ecu;
  (void)frame;
}

void SilSystem::write_output(const Ecu /*ecu*/, const OutputId id,
                             const std::uint16_t value) {
  // TODO(candidate): protect state.outputs with state.mutex and store value.
  (void)id;
  (void)value;
}

namespace ecu_io {

std::uint16_t read_sensor(const SensorId id) {
  return active_system == nullptr ? 0U : active_system->read_sensor(id);
}

bool try_read(const Ecu ecu, Frame& frame) {
  return active_system != nullptr && active_system->try_read(ecu, frame);
}

void write(const Ecu ecu, const Frame& frame) {
  if (active_system != nullptr) {
    active_system->write(ecu, frame);
  }
}

void write_output(const Ecu ecu, const OutputId id, const std::uint16_t value) {
  if (active_system != nullptr) {
    active_system->write_output(ecu, id, value);
  }
}

}  // namespace ecu_io

}  // namespace raceup::sil
