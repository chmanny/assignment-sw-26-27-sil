#include "sil.hpp"

#include <chrono>
#include <cstdint>
#include <cstdio>
#include <format>
#include <thread>
#include <utility>

namespace {

using raceup::sil::Frame;
using raceup::sil::OutputId;
using raceup::sil::SensorId;
using raceup::sil::SilSystem;

constexpr auto kWait = std::chrono::milliseconds(300);

struct TestContext {
  int failures = 0;

  template <typename... Args>
  void expect(const bool condition, std::format_string<Args...> fmt,
              Args&&... args) {
    const auto message = std::format(fmt, std::forward<Args>(args)...);
    if (condition) {
      std::printf("[PASS] %s\n", message.c_str());
    } else {
      std::printf("[FAIL] %s\n", message.c_str());
      ++failures;
    }
  }
};

template <typename Predicate>
bool wait_for_matching_frame(SilSystem& system, const std::uint32_t id,
                             Predicate predicate, Frame& result) {
  const auto deadline = std::chrono::steady_clock::now() + kWait;
  while (std::chrono::steady_clock::now() < deadline) {
    Frame frame{};
    if (system.receive_frame(frame, std::chrono::milliseconds(20)) &&
        frame.id == id && predicate(frame)) {
      result = frame;
      return true;
    }
  }
  return false;
}

bool wait_for_frame(SilSystem& system, const std::uint32_t id, Frame& result) {
  return wait_for_matching_frame(
      system, id, [](const Frame&) { return true; }, result);
}

Frame vehicle_status(const bool manual, const bool ready_to_drive,
                     const std::uint8_t speed) {
  Frame frame = raceup::sil::make_frame(raceup::sil::can_id::VehicleStatus, 3U);
  frame.data[0] = static_cast<std::uint8_t>(manual);
  frame.data[1] = static_cast<std::uint8_t>(ready_to_drive);
  frame.data[2] = speed;
  return frame;
}

Frame brake_pressure(const std::uint8_t front, const std::uint8_t rear,
                     const std::uint8_t pneumatic_left,
                     const std::uint8_t pneumatic_right) {
  Frame frame = raceup::sil::make_frame(raceup::sil::can_id::BrakePressure, 4U);
  frame.data[0] = front;
  frame.data[1] = rear;
  frame.data[2] = pneumatic_left;
  frame.data[3] = pneumatic_right;
  return frame;
}

Frame ebs_command(const bool brake_request, const bool mission_finished) {
  Frame frame = raceup::sil::make_frame(raceup::sil::can_id::EbsCommand, 2U);
  frame.data[0] = static_cast<std::uint8_t>(brake_request);
  frame.data[1] = static_cast<std::uint8_t>(mission_finished);
  return frame;
}

struct EbsInputs {
  bool manual = false;
  bool ready_to_drive = true;
  std::uint8_t speed = 0U;
  std::uint8_t front_brake = 40U;
  std::uint8_t rear_brake = 40U;
  std::uint8_t pneumatic_left = 8U;
  std::uint8_t pneumatic_right = 8U;
  bool brake_request = false;
  bool mission_finished = false;
};

template <typename Publisher, typename Predicate>
bool wait_for_ebs_status_with_publisher(
    SilSystem& system, Publisher publish_inputs, Predicate predicate,
    Frame& result,
    const std::chrono::milliseconds timeout = std::chrono::milliseconds(800),
    const std::chrono::milliseconds refresh_interval =
        std::chrono::milliseconds(40)) {
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  auto next_refresh = std::chrono::steady_clock::now();
  constexpr auto kPollTimeout = std::chrono::milliseconds(10);

  while (std::chrono::steady_clock::now() < deadline) {
    const auto now = std::chrono::steady_clock::now();
    if (now >= next_refresh) {
      publish_inputs();
      next_refresh = now + refresh_interval;
    }

    Frame frame{};
    if (system.receive_frame(frame, kPollTimeout) &&
        frame.id == raceup::sil::can_id::EbsStatus && predicate(frame)) {
      result = frame;
      return true;
    }
  }
  return false;
}

template <typename Predicate>
bool wait_for_ebs_status_with_inputs(
    SilSystem& system, const EbsInputs& inputs, Predicate predicate,
    Frame& result,
    const std::chrono::milliseconds timeout = std::chrono::milliseconds(800),
    const std::chrono::milliseconds refresh_interval =
        std::chrono::milliseconds(40)) {
  return wait_for_ebs_status_with_publisher(
      system,
      [&system, &inputs] {
        system.send_frame(
            vehicle_status(inputs.manual, inputs.ready_to_drive, inputs.speed));
        system.send_frame(brake_pressure(inputs.front_brake, inputs.rear_brake,
                                         inputs.pneumatic_left,
                                         inputs.pneumatic_right));
        system.send_frame(
            ebs_command(inputs.brake_request, inputs.mission_finished));
      },
      predicate, result, timeout, refresh_interval);
}

template <typename Predicate>
bool wait_for_ebs_status_with_refresh(
    SilSystem& system, const std::uint8_t left, const std::uint8_t right,
    Predicate predicate, Frame& result,
    const std::chrono::milliseconds timeout = std::chrono::milliseconds(800),
    const std::chrono::milliseconds refresh_interval =
        std::chrono::milliseconds(40)) {
  EbsInputs inputs{};
  inputs.pneumatic_left = left;
  inputs.pneumatic_right = right;
  return wait_for_ebs_status_with_inputs(system, inputs, predicate, result,
                                         timeout, refresh_interval);
}


void test_lifecycle(TestContext& context) {
  SilSystem system;
  context.expect(system.start(), "SIL starts");
  context.expect(system.running(), "SIL reports running");
  system.stop();
  context.expect(!system.running(), "SIL stops and joins its threads");
}

void configure_nominal_inputs(SilSystem& system) {
  system.set_sensor(SensorId::SmuApps1Raw, 3500U);
  system.set_sensor(SensorId::SmuApps2Raw, 3000U);
  system.set_sensor(SensorId::SmuBrakeRaw, 2500U);
  system.set_sensor(SensorId::EbsValvesPowered, 1U);
  system.set_sensor(SensorId::EbsAsmsActive, 1U);
  system.set_sensor(SensorId::EbsWatchdogHealthy, 1U);
}

void test_smu_nominal(TestContext& context) {
  SilSystem system;
  configure_nominal_inputs(system);
  const bool started = system.start();
  context.expect(started, "SIL starts for the SMU nominal test");
  if (!started) {
    return;
  }

  Frame frame{};
  const bool received = wait_for_frame(
      system, raceup::sil::can_id::SmuSense, frame);
  context.expect(received, "SMU publishes a nominal sense frame");
  if (received) {
    context.expect(frame.dlc == 3U, "SMU sense frame has DLC 3");
    context.expect(frame.data[0] == 50U,
                   "SMU publishes exactly 50 percent throttle");
    context.expect(frame.data[1] == 50U,
                   "SMU publishes exactly 50 percent brake");
    context.expect(frame.data[2] == 0U, "SMU reports no APP fault");
    context.expect(system.output(OutputId::SmuThrottlePercent) == 50U,
                   "SMU throttle output is 50 percent");
    context.expect(system.output(OutputId::SmuBrakePercent) == 50U,
                   "SMU brake output is 50 percent");
  }

  system.stop();
}

void test_smu_track_short_and_recovery(TestContext& context) {
  SilSystem system;
  configure_nominal_inputs(system);
  const bool started = system.start();
  context.expect(started, "SIL starts for the SMU track-short test");
  if (!started) {
    return;
  }

  Frame nominal{};
  const bool nominal_received = wait_for_matching_frame(
      system, raceup::sil::can_id::SmuSense,
      [](const Frame& candidate) {
        return candidate.dlc == 3U && candidate.data[0] == 50U &&
               candidate.data[1] == 50U && candidate.data[2] == 0U;
      },
      nominal);
  context.expect(nominal_received,
                 "SMU reaches the nominal state before the live fault");
  if (!nominal_received) {
    system.stop();
    return;
  }

  system.set_sensor(SensorId::SmuApps2Raw, 3490U);

  Frame fault{};
  const bool fault_received = wait_for_matching_frame(
      system, raceup::sil::can_id::SmuSense,
      [](const Frame& candidate) {
        return candidate.dlc == 3U && candidate.data[0] == 0U &&
               candidate.data[1] == 50U && candidate.data[2] == 1U;
      },
      fault);
  context.expect(fault_received,
                 "SMU reports a track-short fault and inhibits throttle");
  if (!fault_received) {
    system.stop();
    return;
  }
  context.expect(system.output(OutputId::SmuThrottlePercent) == 0U,
                 "SMU throttle output is inhibited");
  context.expect(system.output(OutputId::SmuBrakePercent) == 50U,
                 "SMU brake output remains unchanged during the APP fault");

  system.set_sensor(SensorId::SmuApps1Raw, 3250U);
  system.set_sensor(SensorId::SmuApps2Raw, 2750U);
  Frame recovered{};
  const bool recovered_received = wait_for_matching_frame(
      system, raceup::sil::can_id::SmuSense,
      [](const Frame& candidate) {
        return candidate.dlc == 3U && candidate.data[0] == 75U &&
               candidate.data[1] == 50U && candidate.data[2] == 0U;
      },
      recovered);
  context.expect(recovered_received,
                 "SMU clears the fault and follows the restored pedals");
  if (recovered_received) {
    context.expect(system.output(OutputId::SmuThrottlePercent) == 75U,
                   "SMU throttle output recovers to 75 percent");
  }

  system.stop();
}

void test_smu_implausibility_and_recovery(TestContext& context) {
  SilSystem system;
  configure_nominal_inputs(system);
  const bool started = system.start();
  context.expect(started, "SIL starts for the SMU implausibility test");
  if (!started) {
    return;
  }

  Frame nominal{};
  const bool nominal_received = wait_for_matching_frame(
      system, raceup::sil::can_id::SmuSense,
      [](const Frame& candidate) {
        return candidate.dlc == 3U && candidate.data[0] == 50U &&
               candidate.data[1] == 50U && candidate.data[2] == 0U;
      },
      nominal);
  context.expect(nominal_received,
                 "SMU reaches the nominal state before the plausibility fault");
  if (!nominal_received) {
    system.stop();
    return;
  }

  system.set_sensor(SensorId::SmuApps2Raw, 2800U);

  Frame fault{};
  const bool fault_received = wait_for_matching_frame(
      system, raceup::sil::can_id::SmuSense,
      [](const Frame& candidate) {
        return candidate.dlc == 3U && candidate.data[0] == 0U &&
               candidate.data[1] == 50U && candidate.data[2] == 2U;
      },
      fault);
  context.expect(fault_received,
                 "SMU reports an implausibility fault and inhibits throttle");
  if (!fault_received) {
    system.stop();
    return;
  }
  context.expect(system.output(OutputId::SmuThrottlePercent) == 0U,
                 "SMU throttle output is inhibited on implausibility");
  context.expect(
      system.output(OutputId::SmuBrakePercent) == 50U,
      "SMU brake output remains unchanged during the plausibility fault");

  system.set_sensor(SensorId::SmuApps1Raw, 3250U);
  system.set_sensor(SensorId::SmuApps2Raw, 2750U);
  Frame recovered{};
  const bool recovered_received = wait_for_matching_frame(
      system, raceup::sil::can_id::SmuSense,
      [](const Frame& candidate) {
        return candidate.dlc == 3U && candidate.data[0] == 75U &&
               candidate.data[1] == 50U && candidate.data[2] == 0U;
      },
      recovered);
  context.expect(recovered_received,
                 "SMU clears the plausibility fault and follows restored pedals");
  if (recovered_received) {
    context.expect(system.output(OutputId::SmuThrottlePercent) == 75U,
                   "SMU throttle output recovers to 75 percent");
  }

  system.stop();
}

void test_ebs_nominal_and_stale_input(TestContext& context) {
  SilSystem system;
  configure_nominal_inputs(system);
  const bool started = system.start();
  context.expect(started, "SIL starts for EBS nominal and stale input test");
  if (!started) {
    return;
  }

  // 1. Establish and verify exact nominal state
  Frame nominal{};
  const bool nominal_received = wait_for_ebs_status_with_inputs(
      system, {},
      [](const Frame& f) {
        return f.dlc == 5U && f.data[0] == 1U && f.data[1] == 0U &&
               f.data[2] == 0U && f.data[3] == 0U && f.data[4] == 0U;
      },
      nominal);
  context.expect(nominal_received,
                 "EBS reaches nominal state with fresh inputs");
  if (nominal_received) {
    context.expect(system.output(OutputId::EbsAsNodeClosed) == 1U,
                   "EBS closes ASNode in nominal state");
    context.expect(system.output(OutputId::EbsValve1Engaged) == 0U,
                   "EBS valve 1 is disengaged in nominal state");
    context.expect(system.output(OutputId::EbsValve2Engaged) == 0U,
                   "EBS valve 2 is disengaged in nominal state");
    context.expect(system.output(OutputId::EbsFaultCode) == 0U,
                   "EBS fault code is 0 in nominal state");
  } else {
    system.stop();
    return;
  }

  // 2. Stop refreshing inputs: after 25 cycles (250 ms), inputs become stale.
  // Use an explicit 800 ms deadline instead of wait_for_matching_frame (which uses the global
  // kWait = 300 ms constant) because the stale threshold itself is ~250 ms and 300 ms can be
  // insufficient under system load. 800 ms provides reliable margin without hanging.
  Frame stale_frame{};
  bool stale_received = false;
  {
    const auto stale_deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(800);
    while (std::chrono::steady_clock::now() < stale_deadline) {
      Frame f{};
      if (system.receive_frame(f, std::chrono::milliseconds(20)) &&
          f.id == raceup::sil::can_id::EbsStatus &&
          f.dlc == 5U && f.data[0] == 0U && f.data[1] == 1U &&
          f.data[2] == 1U && f.data[3] == 1U) {
        stale_frame = f;
        stale_received = true;
        break;
      }
    }
  }
  context.expect(
      stale_received,
      "EBS enters safe state with communication fault 1 when inputs become stale");
  if (stale_received) {
    context.expect(system.output(OutputId::EbsAsNodeClosed) == 0U,
                   "EBS opens ASNode after inputs become stale");
    context.expect(system.output(OutputId::EbsValve1Engaged) == 1U,
                   "EBS engages valve 1 on stale input fault");
    context.expect(system.output(OutputId::EbsValve2Engaged) == 1U,
                   "EBS engages valve 2 on stale input fault");
    context.expect(system.output(OutputId::EbsFaultCode) == 1U,
                   "EBS output fault code is 1 (communication)");
  }

  system.stop();
}

void test_ebs_requires_ready_to_drive(TestContext& context) {
  SilSystem system;
  configure_nominal_inputs(system);
  const bool started = system.start();
  context.expect(started, "SIL starts for EBS ready-to-drive requirement test");
  if (!started) {
    return;
  }

  // Phase 1: manual = true, ready_to_drive = true -> establish complete nominal state first.
  // This drains the startup communication-fault frames before testing the readiness transition,
  // ensuring the subsequent safe-state observation is caused by ready_to_drive = false and not
  // by a startup race where VehicleStatus had not yet been received.
  EbsInputs nominal_inputs{};
  nominal_inputs.manual = true;
  nominal_inputs.ready_to_drive = true;

  Frame nominal_frame{};
  const bool nominal_received = wait_for_ebs_status_with_inputs(
      system, nominal_inputs,
      [](const Frame& f) {
        return f.dlc == 5U && f.data[0] == 1U && f.data[1] == 0U &&
               f.data[2] == 0U && f.data[3] == 0U && f.data[4] == 0U;
      },
      nominal_frame);
  context.expect(nominal_received,
                 "EBS reaches nominal state with manual = true, ready_to_drive = true");
  if (!nominal_received) {
    system.stop();
    return;
  }
  context.expect(system.output(OutputId::EbsAsNodeClosed) == 1U,
                 "ASNode closed in initial nominal state");
  context.expect(system.output(OutputId::EbsValve1Engaged) == 0U,
                 "Valve 1 disengaged in initial nominal state");
  context.expect(system.output(OutputId::EbsValve2Engaged) == 0U,
                 "Valve 2 disengaged in initial nominal state");
  context.expect(system.output(OutputId::EbsFaultCode) == 0U,
                 "Fault code 0 in initial nominal state");

  // Phase 2: keep manual = true, change ready_to_drive to false -> safe state with fault 1
  EbsInputs not_ready_inputs{};
  not_ready_inputs.manual = true;
  not_ready_inputs.ready_to_drive = false;

  Frame not_ready_frame{};
  const bool not_ready_received = wait_for_ebs_status_with_inputs(
      system, not_ready_inputs,
      [](const Frame& f) {
        return f.dlc == 5U && f.data[0] == 0U && f.data[1] == 1U &&
               f.data[2] == 1U && f.data[3] == 1U;
      },
      not_ready_frame);
  context.expect(
      not_ready_received,
      "EBS enters safe state with fault 1 when ready-to-drive becomes inactive (manual remains true)");
  if (not_ready_received) {
    context.expect(system.output(OutputId::EbsAsNodeClosed) == 0U,
                   "ASNode opens when ready-to-drive is false");
    context.expect(system.output(OutputId::EbsValve1Engaged) == 1U,
                   "Valve 1 engages when ready-to-drive is false");
    context.expect(system.output(OutputId::EbsValve2Engaged) == 1U,
                   "Valve 2 engages when ready-to-drive is false");
    context.expect(system.output(OutputId::EbsFaultCode) == 1U,
                   "Fault code is 1 (communication) when ready-to-drive is false");
  } else {
    system.stop();
    return;
  }

  // Phase 3: restore ready_to_drive = true -> recovery to nominal
  EbsInputs ready_inputs{};
  ready_inputs.manual = true;
  ready_inputs.ready_to_drive = true;

  Frame recovered_frame{};
  const bool recovered_received = wait_for_ebs_status_with_inputs(
      system, ready_inputs,
      [](const Frame& f) {
        return f.dlc == 5U && f.data[0] == 1U && f.data[1] == 0U &&
               f.data[2] == 0U && f.data[3] == 0U && f.data[4] == 0U;
      },
      recovered_frame);
  context.expect(recovered_received,
                 "EBS returns to nominal state after ready-to-drive is restored");
  if (recovered_received) {
    context.expect(system.output(OutputId::EbsAsNodeClosed) == 1U,
                   "ASNode closes after readiness restored");
    context.expect(system.output(OutputId::EbsValve1Engaged) == 0U,
                   "Valve 1 disengages after readiness restored");
    context.expect(system.output(OutputId::EbsValve2Engaged) == 0U,
                   "Valve 2 disengages after readiness restored");
    context.expect(system.output(OutputId::EbsFaultCode) == 0U,
                   "Fault code is cleared after readiness restored");
  }

  system.stop();
}

void test_ebs_brake_request_and_mission_finished(TestContext& context) {
  SilSystem system;
  configure_nominal_inputs(system);
  const bool started = system.start();
  context.expect(started, "SIL starts for EBS brake request and mission finished test");
  if (!started) {
    return;
  }

  // 1. Establish nominal baseline
  Frame nominal{};
  const bool nominal_received = wait_for_ebs_status_with_inputs(
      system, {},
      [](const Frame& f) {
        return f.dlc == 5U && f.data[0] == 1U && f.data[3] == 0U;
      },
      nominal);
  context.expect(nominal_received, "EBS reaches nominal baseline before command changes");
  if (!nominal_received) {
    system.stop();
    return;
  }

  // 2. Live brake request -> safe state with fault 6 (BrakeRequest)
  EbsInputs brake_req_inputs{};
  brake_req_inputs.brake_request = true;

  Frame brake_req_frame{};
  const bool brake_req_received = wait_for_ebs_status_with_inputs(
      system, brake_req_inputs,
      [](const Frame& f) {
        return f.dlc == 5U && f.data[0] == 0U && f.data[1] == 1U &&
               f.data[2] == 1U && f.data[3] == 6U;
      },
      brake_req_frame);
  context.expect(brake_req_received,
                 "EBS opens ASNode and engages valves on live brake request (fault 6)");
  if (brake_req_received) {
    context.expect(system.output(OutputId::EbsAsNodeClosed) == 0U,
                   "ASNode open on brake request");
    context.expect(system.output(OutputId::EbsValve1Engaged) == 1U,
                   "Valve 1 engaged on brake request");
    context.expect(system.output(OutputId::EbsValve2Engaged) == 1U,
                   "Valve 2 engaged on brake request");
    context.expect(system.output(OutputId::EbsFaultCode) == 6U,
                   "Fault code is 6 (brake request)");
  } else {
    system.stop();
    return;
  }

  // 3. Recovery from brake request -> nominal (complete predicate, guard for mission-finished phase)
  Frame recovered_brake{};
  const bool rec_brake_received = wait_for_ebs_status_with_inputs(
      system, {},
      [](const Frame& f) {
        return f.dlc == 5U && f.data[0] == 1U && f.data[1] == 0U &&
               f.data[2] == 0U && f.data[3] == 0U && f.data[4] == 0U;
      },
      recovered_brake);
  context.expect(rec_brake_received,
                 "EBS returns to complete nominal state after brake request is cleared");
  if (!rec_brake_received) {
    // Cannot enter the mission-finished phase without a clean nominal baseline
    system.stop();
    return;
  }
  context.expect(system.output(OutputId::EbsAsNodeClosed) == 1U,
                 "ASNode closed after brake-request recovery");
  context.expect(system.output(OutputId::EbsValve1Engaged) == 0U,
                 "Valve 1 disengaged after brake-request recovery");
  context.expect(system.output(OutputId::EbsValve2Engaged) == 0U,
                 "Valve 2 disengaged after brake-request recovery");
  context.expect(system.output(OutputId::EbsFaultCode) == 0U,
                 "Fault code is 0 after brake-request recovery");

  // 4. Live mission finished -> safe state with fault 6 (BrakeRequest)
  EbsInputs finish_inputs{};
  finish_inputs.mission_finished = true;

  Frame finish_frame{};
  const bool finish_received = wait_for_ebs_status_with_inputs(
      system, finish_inputs,
      [](const Frame& f) {
        return f.dlc == 5U && f.data[0] == 0U && f.data[1] == 1U &&
               f.data[2] == 1U && f.data[3] == 6U;
      },
      finish_frame);
  context.expect(finish_received,
                 "EBS opens ASNode and engages valves on mission finished (fault 6)");
  if (finish_received) {
    context.expect(system.output(OutputId::EbsAsNodeClosed) == 0U,
                   "ASNode open on mission finished");
    context.expect(system.output(OutputId::EbsValve1Engaged) == 1U,
                   "Valve 1 engaged on mission finished");
    context.expect(system.output(OutputId::EbsValve2Engaged) == 1U,
                   "Valve 2 engaged on mission finished");
    context.expect(system.output(OutputId::EbsFaultCode) == 6U,
                   "Fault code is 6 (mission finished)");
  } else {
    system.stop();
    return;
  }

  // 5. Recovery from mission finished -> complete nominal state
  Frame recovered_finish{};
  const bool rec_finish_received = wait_for_ebs_status_with_inputs(
      system, {},
      [](const Frame& f) {
        return f.dlc == 5U && f.data[0] == 1U && f.data[1] == 0U &&
               f.data[2] == 0U && f.data[3] == 0U && f.data[4] == 0U;
      },
      recovered_finish);
  context.expect(rec_finish_received,
                 "EBS returns to complete nominal state after mission finished is cleared");
  if (rec_finish_received) {
    context.expect(system.output(OutputId::EbsAsNodeClosed) == 1U,
                   "ASNode closed after mission-finished recovery");
    context.expect(system.output(OutputId::EbsValve1Engaged) == 0U,
                   "Valve 1 disengaged after mission-finished recovery");
    context.expect(system.output(OutputId::EbsValve2Engaged) == 0U,
                   "Valve 2 disengaged after mission-finished recovery");
    context.expect(system.output(OutputId::EbsFaultCode) == 0U,
                   "Fault code is 0 after mission-finished recovery");
  }

  system.stop();
}

void test_ebs_missing_critical_inputs(TestContext& context) {
  struct MissingCase {
    const char* name;
    bool send_vehicle;
    bool send_pressure;
    bool send_command;
  };

  const MissingCase cases[] = {
      {"VehicleStatus missing from startup", false, true, true},
      {"BrakePressure missing from startup", true, false, true},
      {"EbsCommand missing from startup", true, true, false},
  };

  for (const auto& test_case : cases) {
    SilSystem system;
    configure_nominal_inputs(system);
    const bool started = system.start();
    context.expect(started, "SIL starts for missing input test: {}", test_case.name);
    if (!started) {
      continue;
    }

    // Observe for 500 ms while refreshing only the available inputs.
    // A correct EBS must never close ASNode (data[0] == 1) when a critical frame is absent,
    // because the missing stream makes all inputs stale within 250 ms (25 cycles x 10 ms).
    // We require at least one safe frame (fault 1, ASNode open) and zero nominal frames.
    // A single-frame early-exit would be fooled by the startup communication-fault frame
    // that appears before the EBS has processed the missing-input condition for the first time.
    constexpr auto kObserveWindow = std::chrono::milliseconds(500);
    constexpr auto kRefreshInterval = std::chrono::milliseconds(40);
    constexpr auto kPollTimeout = std::chrono::milliseconds(10);

    bool any_safe_seen = false;
    bool any_nominal_seen = false;
    const auto obs_deadline = std::chrono::steady_clock::now() + kObserveWindow;
    auto next_refresh = std::chrono::steady_clock::now();

    while (std::chrono::steady_clock::now() < obs_deadline) {
      const auto now = std::chrono::steady_clock::now();
      if (now >= next_refresh) {
        if (test_case.send_vehicle) {
          system.send_frame(vehicle_status(false, true, 0U));
        }
        if (test_case.send_pressure) {
          system.send_frame(brake_pressure(40U, 40U, 8U, 8U));
        }
        if (test_case.send_command) {
          system.send_frame(ebs_command(false, false));
        }
        next_refresh = now + kRefreshInterval;
      }

      Frame f{};
      if (system.receive_frame(f, kPollTimeout) &&
          f.id == raceup::sil::can_id::EbsStatus && f.dlc == 5U) {
        // Safe state: ASNode open (0), both valves engaged (1), fault 1
        if (f.data[0] == 0U && f.data[1] == 1U && f.data[2] == 1U && f.data[3] == 1U) {
          any_safe_seen = true;
        }
        // Nominal: ASNode closed (1) -- must never occur when a stream is absent
        if (f.data[0] == 1U) {
          any_nominal_seen = true;
        }
      }
    }

    context.expect(
        any_safe_seen,
        "[{}]: EBS produced at least one safe-state frame (fault 1) during 500 ms observation",
        test_case.name);
    context.expect(
        !any_nominal_seen,
        "[{}]: EBS never closed ASNode while a critical input stream was absent",
        test_case.name);

    if (any_safe_seen) {
      context.expect(system.output(OutputId::EbsAsNodeClosed) == 0U,
                     "[{}]: ASNode is open at end of observation", test_case.name);
      context.expect(system.output(OutputId::EbsValve1Engaged) == 1U,
                     "[{}]: Valve 1 is engaged at end of observation", test_case.name);
      context.expect(system.output(OutputId::EbsValve2Engaged) == 1U,
                     "[{}]: Valve 2 is engaged at end of observation", test_case.name);
      context.expect(system.output(OutputId::EbsFaultCode) == 1U,
                     "[{}]: Fault code is 1 (communication) at end of observation",
                     test_case.name);
    }

    system.stop();
  }
}

void test_ebs_individual_input_staleness(TestContext& context) {
  struct StalenessCase {
    const char* name;
    bool keep_vehicle;
    bool keep_pressure;
    bool keep_command;
  };

  const StalenessCase cases[] = {
      {"VehicleStatus stream stopped", false, true, true},
      {"BrakePressure stream stopped", true, false, true},
      {"EbsCommand stream stopped", true, true, false},
  };

  for (const auto& test_case : cases) {
    SilSystem system;
    configure_nominal_inputs(system);
    const bool started = system.start();
    context.expect(started, "SIL starts for staleness test: {}", test_case.name);
    if (!started) {
      continue;
    }

    // 1. Establish nominal baseline with all 3 streams active
    Frame nominal{};
    const bool nominal_received = wait_for_ebs_status_with_inputs(
        system, {},
        [](const Frame& f) {
          return f.dlc == 5U && f.data[0] == 1U && f.data[3] == 0U;
        },
        nominal);
    context.expect(nominal_received,
                   "[{} - nominal] EBS reaches nominal baseline", test_case.name);
    if (!nominal_received) {
      system.stop();
      continue;
    }

    // 2. Stop one stream while refreshing the other two -> safe state with fault 1
    Frame stale_frame{};
    const bool stale_received = wait_for_ebs_status_with_publisher(
        system,
        [&system, &test_case] {
          if (test_case.keep_vehicle) {
            system.send_frame(vehicle_status(false, true, 0U));
          }
          if (test_case.keep_pressure) {
            system.send_frame(brake_pressure(40U, 40U, 8U, 8U));
          }
          if (test_case.keep_command) {
            system.send_frame(ebs_command(false, false));
          }
        },
        [](const Frame& f) {
          return f.dlc == 5U && f.data[0] == 0U && f.data[1] == 1U &&
                 f.data[2] == 1U && f.data[3] == 1U;
        },
        stale_frame, std::chrono::milliseconds(800));

    context.expect(
        stale_received,
        "[{} - stale]: EBS returns to safe state with fault 1 when stream stops",
        test_case.name);
    if (stale_received) {
      context.expect(system.output(OutputId::EbsAsNodeClosed) == 0U,
                     "[{} - stale]: ASNode opens", test_case.name);
      context.expect(system.output(OutputId::EbsValve1Engaged) == 1U,
                     "[{} - stale]: Valve 1 engages", test_case.name);
      context.expect(system.output(OutputId::EbsValve2Engaged) == 1U,
                     "[{} - stale]: Valve 2 engages", test_case.name);
      context.expect(system.output(OutputId::EbsFaultCode) == 1U,
                     "[{} - stale]: Fault code is 1", test_case.name);
    } else {
      system.stop();
      continue;
    }

    // 3. Resume sending all 3 streams -> recovery to nominal
    Frame recovered{};
    const bool recovered_received = wait_for_ebs_status_with_inputs(
        system, {},
        [](const Frame& f) {
          return f.dlc == 5U && f.data[0] == 1U && f.data[3] == 0U;
        },
        recovered);
    context.expect(
        recovered_received,
        "[{} - recovery]: EBS recovers to nominal after stream is restored",
        test_case.name);
    if (recovered_received) {
      context.expect(system.output(OutputId::EbsAsNodeClosed) == 1U,
                     "[{} - recovery]: ASNode closes", test_case.name);
      context.expect(system.output(OutputId::EbsValve1Engaged) == 0U,
                     "[{} - recovery]: Valve 1 disengages", test_case.name);
      context.expect(system.output(OutputId::EbsValve2Engaged) == 0U,
                     "[{} - recovery]: Valve 2 disengages", test_case.name);
      context.expect(system.output(OutputId::EbsFaultCode) == 0U,
                     "[{} - recovery]: Fault code is 0", test_case.name);
    }

    system.stop();
  }
}

void test_ebs_invalid_pneumatic_pressure(TestContext& context) {
  struct PneumaticCase {
    const char* name;
    std::uint8_t left;
    std::uint8_t right;
  };

  const PneumaticCase cases[] = {
      {"Left pneumatic at lower boundary", 7U, 8U},
      {"Left pneumatic at upper boundary", 10U, 8U},
      {"Right pneumatic at lower boundary", 8U, 7U},
      {"Right pneumatic at upper boundary", 8U, 10U},
  };

  for (const auto& test_case : cases) {
    SilSystem system;
    configure_nominal_inputs(system);
    const bool started = system.start();
    context.expect(started, "SIL starts for {}", test_case.name);
    if (!started) {
      continue;
    }

    // Phase 1: Nominal inputs (8U, 8U) -> verify nominal
    Frame nominal{};
    const bool nominal_received = wait_for_ebs_status_with_refresh(
        system, 8U, 8U,
        [](const Frame& f) {
          return f.dlc == 5U && f.data[0] == 1U && f.data[1] == 0U &&
                 f.data[2] == 0U && f.data[3] == 0U && f.data[4] == 0U;
        },
        nominal);
    context.expect(nominal_received,
                  "[{} - nominal] EBS publishes nominal status frame",
                  test_case.name);
    if (nominal_received) {
      context.expect(system.output(OutputId::EbsAsNodeClosed) == 1U,
                    "[{} - nominal] EBS closes ASNode in nominal state",
                    test_case.name);
      context.expect(system.output(OutputId::EbsValve1Engaged) == 0U,
                    "[{} - nominal] EBS valve 1 is disengaged in nominal state",
                    test_case.name);
      context.expect(system.output(OutputId::EbsValve2Engaged) == 0U,
                    "[{} - nominal] EBS valve 2 is disengaged in nominal state",
                    test_case.name);
      context.expect(system.output(OutputId::EbsFaultCode) == 0U,
                    "[{} - nominal] EBS reports no fault in nominal state",
                    test_case.name);
    } else {
      system.stop();
      continue;
    }

    // Phase 2: Fault pressure (test_case.left, test_case.right) -> verify safe state with fault 3
    Frame fault{};
    const bool fault_received = wait_for_ebs_status_with_refresh(
        system, test_case.left, test_case.right,
        [](const Frame& f) {
          return f.dlc == 5U && f.data[0] == 0U && f.data[1] == 1U &&
                 f.data[2] == 1U && f.data[3] == 3U && f.data[4] == 0U;
        },
        fault);
    context.expect(
        fault_received,
        "[{} - fault] EBS status reports fault code 3 and enters safe state",
        test_case.name);
    if (fault_received) {
      context.expect(system.output(OutputId::EbsAsNodeClosed) == 0U,
                    "[{} - fault] EBS opens ASNode on pneumatic fault",
                    test_case.name);
      context.expect(system.output(OutputId::EbsValve1Engaged) == 1U,
                    "[{} - fault] EBS engages valve 1 on pneumatic fault",
                    test_case.name);
      context.expect(system.output(OutputId::EbsValve2Engaged) == 1U,
                    "[{} - fault] EBS engages valve 2 on pneumatic fault",
                    test_case.name);
      context.expect(
          system.output(OutputId::EbsFaultCode) == 3U,
          "[{} - fault] EBS output fault code is 3 (pneumatic pressure)",
          test_case.name);
    } else {
      system.stop();
      continue;
    }

    // Phase 3: Recovered pressures (8U, 8U) -> verify return to nominal
    Frame recovered{};
    const bool recovered_received = wait_for_ebs_status_with_refresh(
        system, 8U, 8U,
        [](const Frame& f) {
          return f.dlc == 5U && f.data[0] == 1U && f.data[1] == 0U &&
                 f.data[2] == 0U && f.data[3] == 0U && f.data[4] == 0U;
        },
        recovered);
    context.expect(
        recovered_received,
        "[{} - recovery] EBS status returns to nominal after pressure recovery",
        test_case.name);
    if (recovered_received) {
      context.expect(system.output(OutputId::EbsAsNodeClosed) == 1U,
                    "[{} - recovery] EBS closes ASNode after recovery",
                    test_case.name);
      context.expect(
          system.output(OutputId::EbsValve1Engaged) == 0U,
          "[{} - recovery] EBS valve 1 is disengaged after recovery",
          test_case.name);
      context.expect(
          system.output(OutputId::EbsValve2Engaged) == 0U,
          "[{} - recovery] EBS valve 2 is disengaged after recovery",
          test_case.name);
      context.expect(system.output(OutputId::EbsFaultCode) == 0U,
                    "[{} - recovery] EBS reports no fault after recovery",
                    test_case.name);
    }

    system.stop();
  }
}

void test_ebs_hydraulic_pressure_and_coherency(TestContext& context) {
  struct HydraulicCase {
    const char* name;
    std::uint8_t front;
    std::uint8_t rear;
    std::uint8_t left_pneu;
    std::uint8_t right_pneu;
  };

  const HydraulicCase cases[] = {
      {"Front hydraulic lower boundary (15 bar)", 15U, 40U, 8U, 8U},
      {"Front hydraulic upper boundary (99 bar)", 99U, 40U, 8U, 8U},
      {"Rear hydraulic lower boundary (15 bar)", 40U, 15U, 8U, 8U},
      {"Rear hydraulic upper boundary (99 bar)", 40U, 99U, 8U, 8U},
      {"Front coherency violation (hydraulic 16 * 10 <= pneumatic 9 * 18)", 16U, 40U, 9U, 8U},
      {"Rear coherency violation (hydraulic 16 * 10 <= pneumatic 9 * 18)", 40U, 16U, 8U, 9U},
  };

  for (const auto& test_case : cases) {
    SilSystem system;
    configure_nominal_inputs(system);
    const bool started = system.start();
    context.expect(started, "SIL starts for hydraulic test: {}", test_case.name);
    if (!started) {
      continue;
    }

    // Phase 1: Establish nominal baseline (front 40, rear 40, pneu 8, 8)
    Frame nominal{};
    const bool nominal_received = wait_for_ebs_status_with_inputs(
        system, {},
        [](const Frame& f) {
          return f.dlc == 5U && f.data[0] == 1U && f.data[3] == 0U;
        },
        nominal);
    context.expect(nominal_received, "[{} - nominal] EBS reaches nominal state",
                   test_case.name);
    if (!nominal_received) {
      system.stop();
      continue;
    }

    // Phase 2: Inject fault pressures -> safe state with fault code 4 (BrakePressure)
    EbsInputs fault_inputs{};
    fault_inputs.front_brake = test_case.front;
    fault_inputs.rear_brake = test_case.rear;
    fault_inputs.pneumatic_left = test_case.left_pneu;
    fault_inputs.pneumatic_right = test_case.right_pneu;

    Frame fault{};
    const bool fault_received = wait_for_ebs_status_with_inputs(
        system, fault_inputs,
        [](const Frame& f) {
          return f.dlc == 5U && f.data[0] == 0U && f.data[1] == 1U &&
                 f.data[2] == 1U && f.data[3] == 4U && f.data[4] == 0U;
        },
        fault);
    context.expect(
        fault_received,
        "[{} - fault] EBS enters safe state with fault code 4 (brake pressure)",
        test_case.name);
    if (fault_received) {
      context.expect(system.output(OutputId::EbsAsNodeClosed) == 0U,
                     "[{} - fault] ASNode opens", test_case.name);
      context.expect(system.output(OutputId::EbsValve1Engaged) == 1U,
                     "[{} - fault] Valve 1 engages", test_case.name);
      context.expect(system.output(OutputId::EbsValve2Engaged) == 1U,
                     "[{} - fault] Valve 2 engages", test_case.name);
      context.expect(system.output(OutputId::EbsFaultCode) == 4U,
                     "[{} - fault] Fault code is 4 (BrakePressure)", test_case.name);
    } else {
      system.stop();
      continue;
    }

    // Phase 3: Recovery back to nominal inputs
    Frame recovered{};
    const bool recovered_received = wait_for_ebs_status_with_inputs(
        system, {},
        [](const Frame& f) {
          return f.dlc == 5U && f.data[0] == 1U && f.data[3] == 0U;
        },
        recovered);
    context.expect(
        recovered_received,
        "[{} - recovery] EBS recovers to nominal state", test_case.name);
    if (recovered_received) {
      context.expect(system.output(OutputId::EbsAsNodeClosed) == 1U,
                     "[{} - recovery] ASNode closes", test_case.name);
      context.expect(system.output(OutputId::EbsValve1Engaged) == 0U,
                     "[{} - recovery] Valve 1 disengages", test_case.name);
      context.expect(system.output(OutputId::EbsValve2Engaged) == 0U,
                     "[{} - recovery] Valve 2 disengages", test_case.name);
      context.expect(system.output(OutputId::EbsFaultCode) == 0U,
                     "[{} - recovery] Fault code is cleared", test_case.name);
    }

    system.stop();
  }
}

void test_ebs_nominal_maintained_with_refresh(TestContext& context) {
  SilSystem system;
  configure_nominal_inputs(system);
  const bool started = system.start();
  context.expect(started, "SIL starts for sustained nominal test");
  if (!started) {
    return;
  }

  // 1. Establish nominal baseline before verifying sustained nominal state
  Frame nominal{};
  const bool nominal_reached = wait_for_ebs_status_with_inputs(
      system, {},
      [](const Frame& f) {
        return f.dlc == 5U && f.data[0] == 1U && f.data[3] == 0U;
      },
      nominal);
  context.expect(nominal_reached, "EBS reaches initial nominal state");
  if (!nominal_reached) {
    system.stop();
    return;
  }

  // 2. Run for 400 ms (> 250 ms stale threshold) refreshing every 40 ms.
  // Verify that EBS status stays continuously nominal throughout without glitching into safe state.
  std::uint32_t status_count = 0U;
  bool any_fault_seen = false;
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(400);
  auto next_refresh = std::chrono::steady_clock::now();
  constexpr auto kRefreshInterval = std::chrono::milliseconds(40);
  constexpr auto kPollTimeout = std::chrono::milliseconds(10);

  while (std::chrono::steady_clock::now() < deadline) {
    const auto now = std::chrono::steady_clock::now();
    if (now >= next_refresh) {
      system.send_frame(vehicle_status(false, true, 0U));
      system.send_frame(brake_pressure(40U, 40U, 8U, 8U));
      system.send_frame(ebs_command(false, false));
      next_refresh = now + kRefreshInterval;
    }

    Frame frame{};
    if (system.receive_frame(frame, kPollTimeout) &&
        frame.id == raceup::sil::can_id::EbsStatus) {
      ++status_count;
      if (frame.data[0] != 1U || frame.data[1] != 0U || frame.data[2] != 0U ||
          frame.data[3] != 0U) {
        any_fault_seen = true;
      }
    }
  }

  context.expect(status_count >= 15U,
                 "Received {} EBS status frames during sustained 400ms period",
                 status_count);
  context.expect(!any_fault_seen,
                 "EBS maintained continuous nominal state without any fault drop");
  context.expect(system.output(OutputId::EbsAsNodeClosed) == 1U,
                 "ASNode remains closed after 400ms sustained refresh");
  context.expect(system.output(OutputId::EbsValve1Engaged) == 0U &&
                     system.output(OutputId::EbsValve2Engaged) == 0U,
                 "Both valves remain disengaged after 400ms sustained refresh");
  context.expect(system.output(OutputId::EbsFaultCode) == 0U,
                 "Fault code remains 0 after 400ms sustained refresh");

  system.stop();
}

void test_ebs_sensor_faults(TestContext& context) {
  struct SensorCase {
    const char* name;
    SensorId sensor;
    std::uint8_t expected_fault;
  };

  const SensorCase cases[] = {
      {"ASMS inactive", SensorId::EbsAsmsActive, 2U},
      {"Watchdog unhealthy", SensorId::EbsWatchdogHealthy, 5U},
      {"Valves unpowered", SensorId::EbsValvesPowered, 5U},
  };

  for (const auto& test_case : cases) {
    SilSystem system;
    configure_nominal_inputs(system);
    const bool started = system.start();
    context.expect(started, "SIL starts for EBS sensor test: {}", test_case.name);
    if (!started) {
      continue;
    }

    // 1. Establish nominal state
    Frame nominal{};
    const bool nominal_received = wait_for_ebs_status_with_refresh(
        system, 8U, 8U,
        [](const Frame& f) {
          return f.dlc == 5U && f.data[0] == 1U && f.data[1] == 0U &&
                 f.data[2] == 0U && f.data[3] == 0U && f.data[4] == 0U;
        },
        nominal);
    context.expect(nominal_received,
                   "[{} - nominal] EBS reaches nominal state", test_case.name);
    if (!nominal_received) {
      system.stop();
      continue;
    }

    // 2. Trigger sensor fault by deactivating the sensor
    system.set_sensor(test_case.sensor, 0U);
    Frame fault{};
    const bool fault_received = wait_for_ebs_status_with_refresh(
        system, 8U, 8U,
        [expected_fault = test_case.expected_fault](const Frame& f) {
          return f.dlc == 5U && f.data[0] == 0U && f.data[1] == 1U &&
                 f.data[2] == 1U && f.data[3] == expected_fault && f.data[4] == 0U;
        },
        fault);
    context.expect(fault_received,
                   "[{} - fault] EBS enters safe state with fault code {}",
                   test_case.name, test_case.expected_fault);
    if (fault_received) {
      context.expect(system.output(OutputId::EbsAsNodeClosed) == 0U,
                     "[{} - fault] ASNode opens on sensor fault", test_case.name);
      context.expect(system.output(OutputId::EbsValve1Engaged) == 1U,
                     "[{} - fault] Valve 1 engages on sensor fault", test_case.name);
      context.expect(system.output(OutputId::EbsValve2Engaged) == 1U,
                     "[{} - fault] Valve 2 engages on sensor fault", test_case.name);
      context.expect(system.output(OutputId::EbsFaultCode) == test_case.expected_fault,
                     "[{} - fault] Fault code matches expected", test_case.name);
    } else {
      system.stop();
      continue;
    }

    // 3. Restore sensor to 1 and verify recovery to nominal
    system.set_sensor(test_case.sensor, 1U);
    Frame recovered{};
    const bool recovered_received = wait_for_ebs_status_with_refresh(
        system, 8U, 8U,
        [](const Frame& f) {
          return f.dlc == 5U && f.data[0] == 1U && f.data[1] == 0U &&
                 f.data[2] == 0U && f.data[3] == 0U && f.data[4] == 0U;
        },
        recovered);
    context.expect(recovered_received,
                   "[{} - recovery] EBS recovers to nominal state", test_case.name);
    if (recovered_received) {
      context.expect(system.output(OutputId::EbsAsNodeClosed) == 1U,
                     "[{} - recovery] ASNode closes after recovery", test_case.name);
      context.expect(system.output(OutputId::EbsValve1Engaged) == 0U,
                     "[{} - recovery] Valve 1 disengages after recovery", test_case.name);
      context.expect(system.output(OutputId::EbsValve2Engaged) == 0U,
                     "[{} - recovery] Valve 2 disengages after recovery", test_case.name);
      context.expect(system.output(OutputId::EbsFaultCode) == 0U,
                     "[{} - recovery] Fault code is cleared after recovery", test_case.name);
    }

    system.stop();
  }
}

void test_receive_frame_timeout(TestContext& context) {
  SilSystem system;
  Frame frame{};

  // 1. Unstarted system with empty queue must time out without hanging
  const auto start_time = std::chrono::steady_clock::now();
  const bool received_empty = system.receive_frame(frame, std::chrono::milliseconds(50));
  const auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::steady_clock::now() - start_time);
  context.expect(!received_empty, "receive_frame returns false when no frames are available");
  context.expect(elapsed >= std::chrono::milliseconds(30) && elapsed < std::chrono::milliseconds(500),
                 "receive_frame times out within expected duration without hanging");

  // 2. Stopped system must return false promptly without blocking for the full timeout.
  // Guard with the start() return value: if start() fails, skip this section rather than
  // calling stop() on an unstarted system and misinterpreting the subsequent receive_frame result.
  const bool part2_started = system.start();
  context.expect(part2_started, "SIL starts for stopped-system timeout section");
  if (part2_started) {
    system.stop();
    // Drain frames emitted while running using a bounded deadline and iteration limit
    const auto drain_deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(200);
    std::size_t drain_count = 0;
    while (std::chrono::steady_clock::now() < drain_deadline && drain_count < 100 &&
           system.receive_frame(frame, std::chrono::milliseconds(5))) {
      ++drain_count;
    }
    context.expect(!system.receive_frame(frame, std::chrono::milliseconds(5)),
                   "Emitted queue is fully drained after system stop");

    const auto stopped_start = std::chrono::steady_clock::now();
    const bool received_stopped = system.receive_frame(frame, std::chrono::milliseconds(500));
    const auto stopped_elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now() - stopped_start);
    context.expect(!received_stopped, "receive_frame returns false when system is stopped");
    context.expect(stopped_elapsed < std::chrono::milliseconds(200),
                   "receive_frame returns promptly on stopped system without waiting for full timeout");
  }
}

void test_bus_routing_smu_to_ebs(TestContext& context) {
  SilSystem system;
  configure_nominal_inputs(system);
  const bool started = system.start();
  context.expect(started, "SIL starts for SMU-to-EBS bus routing test");
  if (!started) {
    return;
  }

  // Write frames through ecu_io::write with Ecu::Smu during wait.
  // In a working simulator, SilSystem::write(Ecu::Smu, frame) copies the frame to ebs_input.
  // If this routing branch is omitted, EBS never receives the frames and remains in safe state.
  Frame status{};
  const bool status_received = wait_for_ebs_status_with_publisher(
      system,
      [] {
        raceup::sil::ecu_io::write(raceup::sil::Ecu::Smu, vehicle_status(false, true, 3U));
        raceup::sil::ecu_io::write(raceup::sil::Ecu::Smu, brake_pressure(40U, 40U, 8U, 8U));
        raceup::sil::ecu_io::write(raceup::sil::Ecu::Smu, ebs_command(false, false));
      },
      [](const Frame& f) {
        return f.dlc == 5U && f.data[0] == 1U && f.data[1] == 0U &&
               f.data[2] == 0U && f.data[3] == 0U && f.data[4] == 3U;
      },
      status);

  context.expect(status_received,
                 "EBS receives frames routed from SMU and enters nominal with speed 3");
  if (status_received) {
    context.expect(system.output(OutputId::EbsAsNodeClosed) == 1U,
                   "ASNode is closed when EBS receives frames from SMU routing");
    context.expect(system.output(OutputId::EbsValve1Engaged) == 0U &&
                       system.output(OutputId::EbsValve2Engaged) == 0U,
                   "Both valves released when EBS receives frames from SMU routing");
    context.expect(system.output(OutputId::EbsFaultCode) == 0U,
                   "Fault code is 0 when EBS receives frames from SMU routing");
  }

  // In addition, frames written by SMU must be observable in the monitor queue
  raceup::sil::ecu_io::write(raceup::sil::Ecu::Smu,
                             vehicle_status(false, true, 3U));
  Frame vehicle_status_frame{};
  const bool vehicle_status_observed = wait_for_matching_frame(
      system, raceup::sil::can_id::VehicleStatus,
      [](const Frame& f) { return f.dlc == 3U && f.data[2] == 3U; },
      vehicle_status_frame);
  context.expect(vehicle_status_observed,
                 "VehicleStatus frame written by SMU is observable in monitor queue");

  system.stop();
}

void test_bus_and_external_delivery(TestContext& context) {
  SilSystem system;
  configure_nominal_inputs(system);
  const bool started = system.start();
  context.expect(started, "SIL starts for bus and external delivery test");
  if (!started) {
    return;
  }

  // 1. SMU frames are observable through receive_frame()
  Frame smu_frame{};
  const bool smu_observed = wait_for_matching_frame(
      system, raceup::sil::can_id::SmuSense,
      [](const Frame& f) { return f.dlc == 3U && f.data[0] == 50U; }, smu_frame);
  context.expect(smu_observed, "SMU frames are observable through receive_frame");

  // 2. EBS frames are observable through receive_frame()
  Frame ebs_frame{};
  const bool ebs_observed = wait_for_matching_frame(
      system, raceup::sil::can_id::EbsStatus,
      [](const Frame& f) { return f.dlc == 5U; }, ebs_frame);
  context.expect(ebs_observed, "EBS status frames are observable through receive_frame");

  // 3. Frames delivered to EBS input side via send_frame are processed concurrently
  // while SMU is actively publishing SmuSense frames into the bus
  system.send_frame(vehicle_status(false, true, 0U));
  system.send_frame(brake_pressure(40U, 40U, 8U, 8U));
  system.send_frame(ebs_command(false, false));

  Frame nominal_status{};
  const bool nominal_reached = wait_for_matching_frame(
      system, raceup::sil::can_id::EbsStatus,
      [](const Frame& f) {
        return f.dlc == 5U && f.data[0] == 1U && f.data[3] == 0U;
      },
      nominal_status);
  context.expect(nominal_reached, "EBS input queue delivers frames and processes them concurrently with SMU");
  context.expect(system.output(OutputId::EbsAsNodeClosed) == 1U,
                 "EBS closes ASNode with valid frames delivered to input queue");

  system.stop();
}

void test_queue_buffer_overflow(TestContext& context) {
  SilSystem system;
  configure_nominal_inputs(system);
  const bool started = system.start();
  context.expect(started, "SIL starts for queue buffer overflow test");
  if (!started) {
    return;
  }

  // This test exercises the monitor (emitted) queue, not the EBS input queue.
  // The monitor queue holds frames produced by both ECUs and is observed via receive_frame().
  // Its capacity is 64 frames; when full, the oldest frame is discarded to make room for the newest.
  // Queue contents survive system.stop(), allowing us to drain and inspect them afterwards.

  // Confirm phase 1 (50% throttle) started by waiting for an initial frame
  Frame initial_frame{};
  const bool phase1_started = wait_for_matching_frame(
      system, raceup::sil::can_id::SmuSense,
      [](const Frame& f) { return f.dlc == 3U && f.data[0] == 50U; },
      initial_frame);
  context.expect(phase1_started, "Phase 1 confirmed started with 50% throttle frame");
  if (!phase1_started) {
    system.stop();
    return;
  }

  // Phase 1 overload: let SMU and EBS emit ~80 frames with nominal inputs
  std::this_thread::sleep_for(std::chrono::milliseconds(400));

  // Switch to 75% throttle and confirm phase 2 started before measuring overload retention
  system.set_sensor(SensorId::SmuApps1Raw, 3250U);
  system.set_sensor(SensorId::SmuApps2Raw, 2750U);
  Frame phase2_frame{};
  const bool phase2_started = wait_for_matching_frame(
      system, raceup::sil::can_id::SmuSense,
      [](const Frame& f) { return f.dlc == 3U && f.data[0] == 75U; },
      phase2_frame);
  context.expect(phase2_started, "Phase 2 confirmed started with 75% throttle frame");
  if (!phase2_started) {
    system.stop();
    return;
  }

  // Phase 2 overload: let SMU and EBS emit another ~80 frames
  std::this_thread::sleep_for(std::chrono::milliseconds(400));

  // Stop the system cleanly so no further frames are produced
  system.stop();

  // Read buffered frames from the monitor queue, capping drain loop at 65 reads.
  // Over ~160 frames were produced, but capacity is capped at 64.
  // Exactly 64 frames must be retained, and the oldest (50% throttle) must be discarded.
  std::uint32_t count = 0U;
  bool any_old_50_pct = false;
  bool found_new_75_pct = false;
  Frame frame{};
  while (count < 65U && system.receive_frame(frame, std::chrono::milliseconds(5))) {
    ++count;
    if (frame.id == raceup::sil::can_id::SmuSense) {
      if (frame.data[0] == 50U) {
        any_old_50_pct = true;
      }
      if (frame.data[0] == 75U) {
        found_new_75_pct = true;
      }
    }
  }

  context.expect(count == 64U, "Emitted monitor queue enforces 64-element capacity limit under overload");
  context.expect(!any_old_50_pct, "Oldest frames (50% throttle) were discarded from monitor queue");
  context.expect(found_new_75_pct, "Newest frames (75% throttle) survived in monitor queue");
}

void test_continuous_publishing(TestContext& context) {
  SilSystem system;
  configure_nominal_inputs(system);
  const bool started = system.start();
  context.expect(started, "SIL starts for continuous publishing test");
  if (!started) {
    return;
  }

  // Divide the 300 ms observation period into 3 independent windows of 100 ms each.
  // Each window must contain at least one SMU frame and at least one EBS frame.
  // A single-window total count can be fooled by an initial burst followed by silence;
  // per-window checks require that both ECUs publish continuously throughout the period.
  constexpr int kWindows = 3;
  constexpr int kWindowMs = 100;

  for (int w = 0; w < kWindows; ++w) {
    std::uint32_t smu_w = 0U;
    std::uint32_t ebs_w = 0U;
    const auto wend = std::chrono::steady_clock::now() + std::chrono::milliseconds(kWindowMs);
    while (std::chrono::steady_clock::now() < wend) {
      Frame frame{};
      if (system.receive_frame(frame, std::chrono::milliseconds(15))) {
        if (frame.id == raceup::sil::can_id::SmuSense) {
          ++smu_w;
        } else if (frame.id == raceup::sil::can_id::EbsStatus) {
          ++ebs_w;
        }
      }
    }
    context.expect(smu_w >= 1U,
                   "SMU published at least one frame in 100 ms window {} (got {})", w, smu_w);
    context.expect(ebs_w >= 1U,
                   "EBS published at least one frame in 100 ms window {} (got {})", w, ebs_w);
  }

  system.stop();
}

}  // namespace

int main() {
  TestContext context;
  test_lifecycle(context);
  test_smu_nominal(context);
  test_smu_track_short_and_recovery(context);
  test_smu_implausibility_and_recovery(context);
  test_ebs_nominal_and_stale_input(context);
  test_ebs_requires_ready_to_drive(context);
  test_ebs_brake_request_and_mission_finished(context);
  test_ebs_missing_critical_inputs(context);
  test_ebs_individual_input_staleness(context);
  test_ebs_invalid_pneumatic_pressure(context);
  test_ebs_hydraulic_pressure_and_coherency(context);
  test_ebs_nominal_maintained_with_refresh(context);
  test_ebs_sensor_faults(context);
  test_receive_frame_timeout(context);
  test_bus_routing_smu_to_ebs(context);
  test_bus_and_external_delivery(context);
  test_queue_buffer_overflow(context);
  test_continuous_publishing(context);
  if (context.failures != 0) {
    std::printf("%d test assertion(s) failed\n", context.failures);
    return 1;
  }

  std::printf("all SIL tests passed\n");
  return 0;
}
