# Software In the Loop (SIL) — Architectural Design & Thought Process

>[!WARNING]
>The original file `docs/ToughtProcess.md` was renamed to `docs/ThoughtProcess.md` to fix the spelling typos.

## 1. Synchronization and Concurrency Strategy

The SIL environment coordinates two independent vehicle ECUs executing in distinct POSIX threads alongside an external test suite. Thread safety is achieved through the following design decisions:

- All shared simulator state is encapsulated within `SilSystem::Impl::State` and protected by a single `std::mutex`. State accessors (`set_sensor`, `output`, `read_sensor`, `write_output`) employ `std::lock_guard` for scoped RAII acquisition and index bound checks are used to prevent out-of-bounds undefined behavior.
- The `receive_frame` API uses `std::unique_lock` with `std::condition_variable::wait_for` bound to a state predicate (`!emitted.empty() || stop_requested`) in order to be immune to spurious wakeups.
- During `SilSystem::stop()`, `stop_requested` is atomically flagged and the condition variable is broadcasted outside any lock scope (`notify_all()`), unblocking waiting threads immediately without causing thread re-contention on the mutex. Similarly, `write()` notifies `emitted_available` after releasing the state lock.
- In `start()`, thread construction is wrapped in a `try-catch` block, so if an OS resource failure occurs (e.g. `std::system_error`), `stop()` is invoked to join any partially created workers. In addition, `start()` detects and rejects concurrent activations of multiple `SilSystem` instances.



## 2. Bounded queue management
Given that the SIL environment is designed to simulate real-time ECU behavior, it is critical to manage the flow of CAN frames through bounded queues. The following design principles are applied:

- Frames written by the SMU are pushed to the test monitor queue and simultaneously duplicated into the EBS input queue. Frames emitted by the EBS are delivered exclusively to the monitor queue, preventing self-loopback into the EBS input stage. External frames injected via `send_frame` enter the EBS input queue directly.
- Both the EBS input queue and the monitor queue are strictly bounded by `kQueueCapacity = 64`, so under high bus load, the oldest frames are discarded to guarantee bounded memory consumption.
- To avoid unrecoverable state corruption upon memory allocation failure (`std::bad_alloc`), new frames are inserted into queues **before** popping older elements; in `write()`, where a frame is routed across two distinct queues, a `try-catch` mechanism undoes the first push (`emitted.pop_back()`) if the second push fails, ensuring strong exception guarantees across both data structures.
- **Flight Recorder Lifecycle:** Queue clearing and actuator output resets are performed in `start()` rather than `stop()`. When the system stops, the queue buffer is preserved in memory as a flight recorder, enabling deterministic post-mortem inspection by test harnesses. When `start()` is subsequently invoked, all queues and outputs are reset under the mutex, preventing stale frame leakage across test runs.

## 3. Test Architecture
Safety-critical ECU testing requires validating complete state transitions rather than static fault snapshots. So, every fault test follows a 3-phase cycle: Nominal baseline $\to$ Fault injection $\to$ Recovery.
1. during normal operation, assert ASNode closed (1), valves released (0), and fault code 0
2. a fault is injected, asserting ASNode open (0), valves engaged (1), and specific fault code matching the failure mode
3. nominal inputs are restored, and the ECU is expected to de-latch cleanly and resume nominal operation.


Tests specifically target boundary edge-cases to guarantee detection of private ECU logic mutations, as well as to ensure that malformed inputs are rejected and do not cause silent failures.

The EBS enforces a 250 ms (25 cycles) staleness timeout on critical CAN streams. The custom `wait_for_ebs_status_with_publisher` helper function maintains a 40 ms periodic transmission schedule, replicating continuous vehicle CAN broadcasting and allowing assertions over arbitrary observation windows without tripping artificial communication faults.

To ensure that test failures are reported with precise context, the `TestContext::expect` method utilizes `std::format_string<Args...>` and variadic templates. This approach ensures compile-time format validation (and type safety) and reduces the code duplication.



## 4. Additional SIL Design Considerations

Because the provided ECU implementations maintain process-global state, concurrent SIL instances are unsupported by design. Therefore, the `start()` function includes instance collision detection to enforce serialized execution, and the `stop()` function ensures that all threads are joined and resources are released before returning. This design guarantees that each SIL instance operates in isolation, preventing interference between tests and ensuring reproducibility of results.

The bounded monitor queue is verified deterministically after `stop()` has joined and terminated all background worker threads. Under the flight recorder design, queue buffers remain intact following `stop()`, so no active ECU threads can race with the test harness during queue drainage. This enables an exact, race-free assertion (`count == 64U`) that strictly confirms the 64-element capacity cap, guarantees the drop-oldest eviction policy, and prevents any test flakiness under high CI load.