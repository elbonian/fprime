# MagneticDetumble Component Integration Tests

This document outlines integration test cases for the `Svc.MagneticDetumble` component within the `Ref` application. These tests are designed to be executed using the F' Ground Data System (GDS) and Python scripting (`fprime-gds`).

## 1. Test Setup Overview

### Application Configuration:

*   The `Ref` application topology must include an instance of the `Svc.MagneticDetumble` component (e.g., named `magneticDetumble`).
*   **Standard Connections:**
    *   `magneticDetumble.cmdIn` connected to `cmdDisp.compCmdStat`
    *   `cmdDisp.compCmdReg` connected to `magneticDetumble.cmdRegOut`
    *   `cmdDisp.compCmdResponse` connected to `magneticDetumble.cmdResponseOut`
    *   `magneticDetumble.eventOut` connected to `eventLogger.logRecv`
    *   `magneticDetumble.textEventOut` connected to `textLogger.textLogRecv` (or `activeLogger.logRecv`)
    *   `magneticDetumble.tlmOut` connected to `tlmSend.tlmRecv`
    *   `magneticDetumble.prmGetOut` connected to `prmDb.getPrm`
    *   `prmDb.setPrm` connected to `magneticDetumble.prmSetIn`
    *   `magneticDetumble.timeCaller` connected to `timeReceiver.timeCaller` (e.g., `posixTime.timeCaller`)
*   **Scheduler Connection:**
    *   `magneticDetumble.schedIn` connected to a rate group output, e.g., `rateGroup1.RateGroupMemberOut[X]`.
*   **Specific Port Connections (with Mocks):**
    *   `magneticDetumble.angularVelocityIn` connected to `mockIMU.angularVelocityOut`.
    *   `magneticDetumble.magFieldIn` connected to `mockMagnetometer.magFieldOut`.
    *   `magneticDetumble.magnetorquerCmdOut` connected to `mockMagnetorquerDriver.dipoleCmdIn`.

### Tools:

*   **F' GDS:** For commanding, telemetry visualization, event logging, and sequence execution.
*   **Python Scripting (`fprime-gds`):** To automate test sequences, send commands, monitor telemetry and events, and interact with mock components.

## 2. Mock Components/Simulators Needed

The following mock components need to be developed and integrated into the `Ref` application's test topology:

*   ### MockIMU:
    *   **Purpose:** Simulates an Inertial Measurement Unit providing angular velocity.
    *   **FPP Definition (Conceptual):**
        ```fpp
        active component MockIMU {
          output port angularVelocityOut: Fw.Math.Vector3
          command SET_ANGULAR_VELOCITY(omega: Fw.Math.Vector3)
          telemetry ANGULAR_VELOCITY_SET: Fw.Math.Vector3
        }
        ```
    *   **Functionality:**
        *   Stores an internal `Fw.Math.Vector3` for angular velocity.
        *   Upon receiving `SET_ANGULAR_VELOCITY` command, updates its internal value and emits `ANGULAR_VELOCITY_SET` telemetry.
        *   When its `angularVelocityOut` port is invoked (if it's a polled port, or if it's driven by a scheduler to actively send), it sends the currently stored angular velocity. (For simplicity, can be a get port called by `MagneticDetumble` if `MagneticDetumble`'s input ports were changed to `get` style). *Assuming `MagneticDetumble` uses `sync Fw.Success` input ports, `MockIMU` would need to be called by `MagneticDetumble` and would return the stored value.*
        *   Alternatively, if `MagneticDetumble` input ports are `async`, `MockIMU` would have a `schedIn` port and actively call `MagneticDetumble.angularVelocityIn`. (The current `MagneticDetumble` uses `sync Fw.Success` input ports, so the former applies). For the purpose of these tests, we assume `magneticDetumble.angularVelocityIn` is a synchronous input port that will call out to a `get` port on the `MockIMU`.

*   ### MockMagnetometer:
    *   **Purpose:** Simulates a magnetometer providing the magnetic field vector.
    *   **FPP Definition (Conceptual):**
        ```fpp
        active component MockMagnetometer {
          output port magFieldOut: Fw.Math.Vector3
          command SET_MAG_FIELD(b_vector: Fw.Math.Vector3)
          telemetry MAG_FIELD_SET: Fw.Math.Vector3
        }
        ```
    *   **Functionality:**
        *   Similar to `MockIMU`, stores an internal `Fw.Math.Vector3` for the magnetic field.
        *   `SET_MAG_FIELD` command updates the value and emits telemetry.
        *   Provides the stored magnetic field when its `magFieldOut` port is invoked by `MagneticDetumble.magFieldIn`.

*   ### MockMagnetorquerDriver:
    *   **Purpose:** Simulates the interface to magnetorquers, receiving and logging/telemetrying dipole commands.
    *   **FPP Definition (Conceptual):**
        ```fpp
        active component MockMagnetorquerDriver {
          input port dipoleCmdIn: Fw.Math.Vector3
          event DIPOLE_CMD_RECV(dipole: Fw.Math.Vector3)
          telemetry LAST_DIPOLE_CMD: Fw.Math.Vector3
        }
        ```
    *   **Functionality:**
        *   When `dipoleCmdIn` is invoked by `MagneticDetumble.magnetorquerCmdOut`, it:
            *   Emits a `DIPOLE_CMD_RECV` event with the received dipole vector.
            *   Updates and emits `LAST_DIPOLE_CMD` telemetry with the received dipole.

## 3. Test Cases

### TC001: Set Mode to MANUAL and Command Dipole

*   **Objective:** Verify MANUAL mode operation and direct dipole commanding.
*   **Steps:**
    1.  Send `magneticDetumble.SET_MODE` command with `mode = Fw.On.OFF (MANUAL)`.
    2.  **Verify:**
        *   `magneticDetumble.MODE` telemetry channel updates to `Fw.On.OFF`.
        *   `magneticDetumble.MODE_CHANGED` event is emitted with `mode = Fw.On.OFF`.
    3.  Define a test dipole vector, e.g., `D_test = (0.1, -0.1, 0.05)`.
    4.  Send `magneticDetumble.SET_DIPOLE` command with `dipole = D_test`.
    5.  **Verify:**
        *   `magneticDetumble.DIPOLE` telemetry channel updates to `D_test`.
        *   `magneticDetumble.DIPOLE_SET` event is emitted with `dipole = D_test`.
    6.  Trigger `magneticDetumble.SCHED_IN` port (e.g., by ensuring the connected rate group runs).
    7.  **Verify:**
        *   `mockMagnetorquerDriver` emits `DIPOLE_CMD_RECV` event with `dipole = D_test`.
        *   `mockMagnetorquerDriver.LAST_DIPOLE_CMD` telemetry updates to `D_test`.
        *   `magneticDetumble.DIPOLE` telemetry remains `D_test`.

### TC002: Set Mode to AUTO and Observe B-dot Control

*   **Objective:** Verify AUTO mode B-dot control law operation.
*   **Setup:**
    *   Set `MockIMU` angular velocity to a non-zero value, e.g., `omega_test = (0.1, 0.0, -0.1)` rad/s.
*   **Steps:**
    1.  Send `magneticDetumble.SET_MODE` command with `mode = Fw.On.ON (AUTO)`.
    2.  **Verify:**
        *   `magneticDetumble.MODE` telemetry channel updates to `Fw.On.ON`.
        *   `magneticDetumble.MODE_CHANGED` event is emitted with `mode = Fw.On.ON`.
    3.  **Iteration 1:**
        *   Command `MockMagnetometer.SET_MAG_FIELD` with `B1 = (10000, 20000, -5000)` nT.
        *   Trigger `magneticDetumble.SCHED_IN`.
        *   **Verify:** `magneticDetumble.MAG_FIELD` telemetry updates to `B1`. `magneticDetumble.OMEGA` updates to `omega_test`.
        *   (No dipole command expected yet, or zero, as it's the first run for B-dot).
    4.  **Iteration 2:**
        *   Wait for a duration `dt` (consistent with `SCHED_IN` period).
        *   Command `MockMagnetometer.SET_MAG_FIELD` with `B2 = (12000, 18000, -4000)` nT.
        *   Trigger `magneticDetumble.SCHED_IN`.
        *   **Verify:** `magneticDetumble.MAG_FIELD` telemetry updates to `B2`.
        *   Observe `magneticDetumble.DIPOLE` telemetry and `mockMagnetorquerDriver.LAST_DIPOLE_CMD`. Let this be `M2`.
        *   Calculate expected `B_dot_approx = (B2 - B1) / dt`.
        *   Verify `M2` is approximately proportional to `-B_dot_approx / ||B2||` (scaled by control gain). Polarity should oppose `B_dot_approx`.
    5.  **Iteration 3 (optional):**
        *   Command `MockMagnetometer.SET_MAG_FIELD` with `B3 = (13000, 15000, -3000)` nT.
        *   Trigger `magneticDetumble.SCHED_IN`.
        *   Verify updated dipole `M3` based on `(B3 - B2) / dt`.
    6.  Throughout AUTO mode iterations, verify no `CONTROL_ITERATION_ERROR` events.

### TC003: AUTO Mode - Zero B-Field Scenario

*   **Objective:** Verify safe behavior when magnetic field magnitude is too low for B-dot.
*   **Steps:**
    1.  Send `magneticDetumble.SET_MODE` command with `mode = Fw.On.ON (AUTO)`.
    2.  Verify mode change as in TC002.
    3.  Command `MockMagnetometer.SET_MAG_FIELD` with `B_zero = (0.001, -0.002, 0.0005)` nT (a very small magnitude).
    4.  Trigger `magneticDetumble.SCHED_IN` (may need two calls if first run logic is active).
    5.  **Verify:**
        *   `magneticDetumble.MAG_FIELD` telemetry updates to `B_zero`.
        *   `magneticDetumble.DIPOLE` telemetry is zero or very close to zero.
        *   `mockMagnetorquerDriver.LAST_DIPOLE_CMD` is zero or very close to zero.
        *   `magneticDetumble.CONTROL_ITERATION_ERROR` event is emitted, with a status indicating low B-field magnitude (e.g., "Magnetic field magnitude too small").

### TC004: Parameter Change - Control Gain

*   **Objective:** Verify that changes to `CONTROL_GAIN` parameter affect the dipole calculation in AUTO mode.
*   **Setup:**
    *   Ensure `MagneticDetumble.CONTROL_GAIN` parameter (`ID: 0` or as per dictionary) is set to an initial value `K1` (e.g., via `prmDb.PARAMETER_LOAD` or default).
    *   Set mode to AUTO.
    *   Use `MockIMU` and `MockMagnetometer` to provide consistent, non-trivial `omega` and changing `B` vectors (`B_initial1`, `B_initial2`) for B-dot calculation.
*   **Steps:**
    1.  Trigger `magneticDetumble.SCHED_IN` twice (to establish `previous_B` and then calculate a dipole).
    2.  Observe the commanded dipole `M_K1` via `mockMagnetorquerDriver.LAST_DIPOLE_CMD`.
    3.  Define a new gain `K2 = 2 * K1`.
    4.  Send command to `prmDb` to set `MagneticDetumble.CONTROL_GAIN` to `K2`. (e.g., `prmDb.PRM_SAVE` might be needed, or ensure `prmSetIn` on `MagneticDetumble` is called and handled).
        *   *Note: The component's `paramUpdated` method would need to re-read the gain if it's to take effect immediately without re-init. Assume for this test that parameters pushed via `prmSetIn` are handled dynamically or `init` is called again if that's the design for parameter updates to take effect.*
        *   Alternatively, if the component only reads gain at `init`, this test would involve re-initializing the component, which is more complex for an integration test. Assume dynamic update for now.
    5.  Re-send the same magnetic field sequence (`B_initial1`, then `B_initial2`) via `MockMagnetometer` to ensure the same `B_dot` and `||B||`.
    6.  Trigger `magneticDetumble.SCHED_IN` twice again.
    7.  Observe the new commanded dipole `M_K2` via `mockMagnetorquerDriver.LAST_DIPOLE_CMD`.
    8.  **Verify:** `M_K2` is approximately `2 * M_K1` (or `(K2/K1) * M_K1`).

This markdown document should serve as a good starting point for developing executable integration tests once the `Ref` application build is resolved and mock components are available.
