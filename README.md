# Task 1 — POSIX Software In the Loop

This repository is the starter package for the RaceUP Software Department
selection assignment.

The goal is to build a small host-based Software In the Loop (SIL) environment
for two vehicle ECUs:

- the Sensor Management Unit (SMU);
- the Emergency Brake System (EBS).

The supplied ECU code contains the safety logic. It accesses sensors, the
communication bus, and actuator outputs through the fixed interface in
`given/ecu_api.hpp`. The candidate must implement the POSIX simulation around
that interface, extend the tests, and provide a working CI workflow.

No knowledge of the actual vehicle hardware is required. The assignment does
not require FreeRTOS, STM32 code, SocketCAN, multiple processes, or a complete
vehicle model.

## Candidate deliverables

Work in a personal public fork of this assignment repository. Submit the URL
of your public fork together with your solution so that the implementation,
tests, CI workflow, and documentation can be reviewed.

Implement in your fork:

1. The missing thread-safe simulator and local bus in `src/sil.cpp`.
2. Additional black-box tests in `tests/sil_tests.cpp`.
3. A CI workflow in `.github/workflows/ci.yml` that builds and runs the tests.
4. Any small lifecycle or test-harness changes needed for a reliable solution.
5. A concise explanation of the main design choices in
   [`docs/ThoughtProcess.md`](docs/ThoughtProcess.md).

The public tests are only a starting point. Add tests for the safety behavior
listed below. At least three focused tests must be added, and at least one must
change a sensor value or inject a bus frame while the SIL is already running
and verify the resulting state transition.

Please use [`docs/ThoughtProcess.md`](docs/ThoughtProcess.md) to explain the synchronization strategy,
communication routing, test approach, and relevant trade-offs. The document
does not need to be long; it should make the reasoning behind the solution
easy to review.

Do not modify the safety logic in `given/smu.cpp` or `given/ebs.cpp`. Those files
are the application under test. The evaluator may compile the solution with
private mutated versions of either ECU to check whether the candidate tests
detect realistic regressions.

## Scope and constraints

The SIL must:

- run the SMU and EBS concurrently using POSIX threads;
- run entirely on one Linux machine;
- use local in-process communication; no hardware, privileges, network
  interfaces, or external services are required;
- synchronize all access to simulator state;
- use deadlines and timed condition-variable waits instead of infinite waits
  or busy-waiting;
- stop and join all ECU threads cleanly.

The supplied ECU modules contain process-global state. Running more than one
`SilSystem` instance concurrently is outside the scope of this assignment;
tests should use one running SIL instance at a time.

Keep the implementation small. It is acceptable for the simulator to remain
in one source file.

## Build and test

Requirements:

- CMake 3.25 or newer;
- a C++20 compiler;
- Ninja.

Run the complete local workflow with:

```bash
cmake --workflow --preset sil
```

Build artifacts are written to `.build/`. The starter is expected to compile,
but its tests will not all pass until the TODOs in `src/sil.cpp` are completed.

CI is part of the assignment. The repository contains a minimal starting
workflow, but the candidate owns the workflow in the submission. It must run
on pushes and pull requests, perform a clean configure/build/test operation,
and return a non-zero status when the tests fail. It must not depend on local
build artifacts, hardware, secrets, or manual steps.


### Running via Docker/Podman
To ensure a reproducible build environment, the entire build process and test suite can be run inside a Docker or Podman container. 

Run the following command from the root of the repository:

Using Podman:
```bash
podman run --rm -v "$PWD":/workspace -w /workspace ubuntu:24.04 bash -c "apt-get update && apt-get install -y cmake ninja-build g++ && cmake --workflow --preset sil"
```

>[!NOTE]
>If you are using a RHEL-based distribution (Fedora, CentOS) and have SELinux enabled, you may need to [adjust the volume mount options](https://docs.docker.com/engine/storage/bind-mounts/#configure-the-selinux-label) to avoid permission issues when running the container, by adding `:z` or `:Z` to the volume mount, like so:
>```bash
>podman run --rm -v "$PWD":/workspace:z -w /workspace ubuntu:24.04 bash -c "apt-get update && apt-get install -y cmake ninja-build g++ && cmake --workflow --preset sil"
>```
>Alternatively, you can run the container with elevated privileges using `--privileged` or temporarily disable SELinux using `sudo setenforce 0`, but this is generally discouraged for security reasons.

Using Docker:
```bash
docker run --rm -v "$PWD":/workspace -w /workspace ubuntu:24.04 bash -c "apt-get update && apt-get install -y cmake ninja-build g++ && cmake --workflow --preset sil"
```

#### Running with ThreadSanitizer
To run the test suite with ThreadSanitizer enabled inside Podman/Docker:

```bash
podman run --rm -v "$PWD":/workspace:z -w /workspace ubuntu:24.04 bash -c "apt-get update && apt-get install -y cmake ninja-build g++ && CXXFLAGS='-fsanitize=thread -g -fno-omit-frame-pointer' LDFLAGS='-fsanitize=thread' TSAN_OPTIONS='second_deadlock_stack=1' cmake --workflow --preset sil"
```

>[!NOTE]
>On Linux kernel 6.5+ hosts, ThreadSanitizer may abort on startup with `FATAL: ThreadSanitizer: unexpected memory mapping` due to ASLR entropy. You can resolve this either by running with `setarch x86_64 -R` or by temporarily adjusting `mmap_rnd_bits` on the host.


## Simulator contract

The fixed platform contract is in `given/ecu_api.hpp`:

```cpp
uint16_t ecu_io::read_sensor(SensorId id);
bool ecu_io::try_read(Ecu ecu, Frame& frame);
void ecu_io::write(Ecu ecu, const Frame& frame);
void ecu_io::write_output(Ecu ecu, OutputId id, uint16_t value);
```

The test-facing API is `SilSystem` in `src/sil.hpp`:

- `set_sensor` changes a simulated sensor value;
- `output` returns the latest simulated actuator value;
- `send_frame` injects a frame into the EBS input queue;
- `receive_frame` reads the next frame emitted by either ECU and returns false
  on timeout.

The simulator state contains sensor values, actuator values, an EBS input
queue, an emitted-frame monitor queue, a mutex, and a condition variable.
Both queues are bounded. If a queue is full, discard its oldest frame before
adding a new one.

The SMU does not consume bus input. The EBS is the only ECU with an input
queue. Every frame written by the SMU must be observable through
`receive_frame` and also made available to the EBS input queue. Every frame
written by the EBS must be observable through `receive_frame`.

Because the queues are bounded, frames may be discarded under overload. Tests
must assert the behavior of frames they intentionally inject or wait for, not
assume that an unbounded history is retained.

## SMU behavior

The SMU reads inverted raw 12-bit ADC values:

| Sensor | Zero | Full |
|---|---:|---:|
| APP1 | 4000 | 3000 |
| APP2 | 3500 | 2500 |
| Brake pedal | 4000 | 1000 |

where "APP" stands for "Accelerator Pedal Position". 

A lower raw value represents a higher pedal position. The supplied logic
normalizes each value to a percentage and clamps values outside its calibration
range to the nearest endpoint.

The SMU must:

- detect an APP track short when the two raw APP readings differ by fewer than
  50 counts;
- detect an APP plausibility fault when normalized APP values differ by more
  than 10 percentage points;
- set throttle to zero whenever an APP fault is detected;
- publish the normalized brake and throttle outputs;
- publish a `SMU_SENSE` frame every cycle.

`SMU_SENSE` has ID `0x120`, DLC 3:

```text
byte 0: throttle percentage
byte 1: brake percentage
byte 2: SMU fault code, zero when no fault is active
```

## EBS behavior

The EBS receives these frames:

### `VEHICLE_STATUS` — ID `0x300`, DLC 3

```text
byte 0: manual mission flag
byte 1: ready-to-drive flag
byte 2: vehicle speed in decimetres per second
```

### `BRAKE_PRESSURE` — ID `0x301`, DLC 4

```text
byte 0: front hydraulic pressure in bar
byte 1: rear hydraulic pressure in bar
byte 2: left pneumatic pressure in bar
byte 3: right pneumatic pressure in bar
```

### `EBS_COMMAND` — ID `0x302`, DLC 2

```text
byte 0: brake request flag
byte 1: mission finished flag
```

The EBS also reads these simulated sensors:

- valve power;
- ASMS activation;
- watchdog health.

Critical input frames become stale after 250 ms. The EBS runs every 10 ms, so
nominal-path tests must refresh the input frames periodically when they run for
longer than the timeout.

The EBS can close the ASNode only when all of the following are true:

- all critical input frames are fresh;
- ASMS is active;
- the valves are powered;
- the watchdog is healthy;
- ready-to-drive is active;
- pneumatic pressures are strictly between 7 and 10 bar;
- front and rear hydraulic pressures are strictly between 15 and 99 bar;
- hydraulic pressure satisfies the supplied 1.8 coherency rule;
- there is no brake request or finished mission;
- vehicle speed is at or below 5 decimetres per second.

Manual mission does not bypass the ready-to-drive requirement.

When the EBS is safe, it opens the ASNode (a relé in the car) and engages both valves. A fault
caused by invalid or stale inputs has a non-zero fault code. A vehicle moving
above the stationary-speed threshold also keeps the safe actuator outputs, but
the supplied logic does not assign a separate speed fault code.

When the EBS is nominal, it closes the ASNode, releases both valves, and sets
the fault code to zero.

`EBS_STATUS` has ID `0x220`, DLC 5:

```text
byte 0: ASNode closed, logical boolean
byte 1: valve 1 engaged, logical boolean
byte 2: valve 2 engaged, logical boolean
byte 3: EBS fault code
byte 4: vehicle speed in decimetres per second
```

The simulator exposes logical actuator values. The candidate does not need to
model the electrical active-low polarity of the real valve outputs.

## Required tests

The candidate tests should cover observable behavior, not private ECU state.
At least the following cases are expected.

### SMU

- nominal APP and brake values produce the expected frame and outputs;
- an APP track short inhibits throttle;
- an APP plausibility disagreement inhibits throttle;
- recovery from a live APP fault restores throttle when the inputs are valid;
- periodic SMU frames continue to be emitted.

### EBS

- missing critical inputs result in the safe state;
- ready-to-drive must be required even when manual mission is set;
- valid, periodically refreshed inputs allow the nominal state;
- a stopped input stream becomes stale and returns the EBS to the safe state;
- ASMS, watchdog, and valve-power faults result in safe outputs;
- invalid pressure values result in safe outputs;
- a brake request or finished mission engages both valves and opens the ASNode;
- an `EBS_STATUS` frame is emitted with the expected contents.

### Integration and infrastructure

- SMU and EBS run concurrently;
- SMU frames are observable and are delivered to the EBS input side;
- `receive_frame` times out without hanging;
- all threads stop and join cleanly;
- the CI workflow builds and runs the complete test suite.

Tests must use deadlines rather than infinite waits. A failed assertion must
make the test executable and CTest return a non-zero status.

## Evaluation

The submission is evaluated on:

- correct and race-free simulator state and bus behavior;
- safe behavior for invalid, missing, and stale inputs;
- meaningful black-box tests;
- clean thread lifecycle management;
- reproducible CMake/CTest execution;
- a working CI workflow;
- concise documentation and maintainable code.
