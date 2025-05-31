# Software Design Description: MagneticDetumble Component

## 1. Introduction

The `MagneticDetumble` component is an F' active component responsible for managing a spacecraft's magnetic torquers (magnetorquers) to help with detumbling and potentially other attitude control maneuvers. It provides an interface to command specific magnetic dipole moments and can also operate autonomously using a B-dot (B-dot) control algorithm to reduce spacecraft angular rates.

This component is typically located in the `Drv` layer of an F' application.

## 2. Purpose and Scope

**Purpose:**
- To provide a software interface for controlling magnetorquer hardware.
- To implement a B-dot algorithm for autonomous detumbling.
- To generate telemetry regarding its operational state, commanded dipoles, and sensor inputs.
- To generate events for significant occurrences, warnings, and errors.

**Scope:**
- Receiving commands to set a 3-axis magnetic dipole.
- Receiving commands to enable/disable B-dot mode and configure its gain.
- Receiving inputs for the spacecraft's magnetic field vector and angular velocity.
- Periodically executing the B-dot algorithm when enabled.
- Calculating the required magnetic dipole based on the B-dot algorithm.
- Abstractly commanding underlying hardware (e.g., coil drivers) to achieve the target dipole. This component does *not* directly interface with hardware registers; it outputs current commands per axis.
- Reporting status, telemetry, and events.

**Out of Scope:**
- Direct hardware register manipulation for coil control (delegated to lower-level drivers via the `hwControlOut` port).
- Complex attitude determination or control algorithms beyond B-dot (these would reside in a dedicated ACS component).
- Magnetic field model calculations.

## 3. Component Interface

The interface of the `MagneticDetumble` component is defined in `Drv/MagneticDetumble/MagneticDetumble.fpp`.

### 3.1. Ports

#### 3.1.1. Input Ports

-   **`setDipoleCmdIn(x: F32, y: F32, z: F32)` (async):**
    Receives a direct command to set the desired magnetic dipole moment for each of the X, Y, and Z spacecraft axes.
-   **`magFieldIn(x: F32, y: F32, z: F32)` (async):**
    Receives the current magnetic field vector components, typically from a magnetometer.
-   **`angularVelocityIn(x: F32, y: F32, z: F32)` (async):**
    Receives the current spacecraft angular velocity components, typically from a gyroscope or attitude estimation system.
-   **`runScheduleIn(context: U32)` (sync):**
    A scheduled input used to trigger periodic operations, primarily the B-dot algorithm calculation when enabled.

#### 3.1.2. Output Ports

-   **`statusOut: Fw.Success`:**
    Reports the success or failure status of asynchronous operations initiated via input ports (e.g., `setDipoleCmdIn`).
-   **`hwControlOut(axis: U8, current: F32)`:**
    Outputs the calculated current command for a specific magnetorquer coil.
    -   `axis`: 0 for X, 1 for Y, 2 for Z.
    -   `current`: The current (e.g., in Amperes) to be applied to the coil. This port abstracts the actual hardware interface.

### 3.2. Commands

-   **`MAG_DETUMBLE_SET_DIPOLE(x: F32, y: F32, z: F32)` (Opcode `0x01`):**
    Commands the component to achieve a specific magnetic dipole moment (Mx, My, Mz). If B-dot mode is active, this command will effectively override the B-dot calculation for the current cycle.
-   **`MAG_DETUMBLE_ENABLE_BDOT(enable: bool)` (Opcode `0x02`):**
    Enables (`true`) or disables (`false`) the internal B-dot control algorithm. When disabling, coil currents are typically commanded to zero.
-   **`MAG_DETUMBLE_SET_BDOT_GAIN(gain: F32)` (Opcode `0x03`):**
    Sets the control gain (`k`) for the B-dot algorithm (`m = -k * B_dot`). The gain must be non-negative.

### 3.3. Telemetry

-   **`MagDetumble_DipoleCmdX, Y, Z` (F32):** The current target magnetic dipole moment for each axis, whether set by direct command or calculated by B-dot.
-   **`MagDetumble_MagFieldX, Y, Z` (F32):** The last received magnetic field vector components.
-   **`MagDetumble_AngularVelocityX, Y, Z` (F32):** The last received angular velocity components.
-   **`MagDetumble_BdotGain` (F32):** The current gain value for the B-dot controller.
-   **`MagDetumble_BdotEnabled` (bool):** The current status of the B-dot mode (true if enabled, false if disabled).
-   **`MagDetumble_CoilCurrentX, Y, Z` (F32):** The last commanded current to each of the X, Y, and Z coils.

### 3.4. Events

-   **`MAG_DETUMBLE_DIPOLE_CMD_RECEIVED(x, y, z: F32)` (ID `0x01`, Activity High):**
    Generated when a `MAG_DETUMBLE_SET_DIPOLE` command or `setDipoleCmdIn` port call is processed.
-   **`MAG_DETUMBLE_BDOT_STATUS_CHANGED(enabled: bool)` (ID `0x02`, Activity High):**
    Generated when the B-dot mode is enabled or disabled via command.
-   **`MAG_DETUMBLE_BDOT_GAIN_SET(gain: F32)` (ID `0x03`, Activity High):**
    Generated when the B-dot gain is successfully updated.
-   **`MAG_DETUMBLE_BDOT_UPDATE(dipoleX, dipoleY, dipoleZ: F32)` (ID `0x04`, Activity High):**
    Generated each cycle the B-dot algorithm runs and calculates a new target dipole.
-   **`MAG_DETUMBLE_HW_CMD_SENT(axis: U8, current: F32)` (ID `0x05`, Activity Low):**
    Generated when a current command is sent to a coil via `hwControlOut`.
-   **`MAG_DETUMBLE_INVALID_MAG_FIELD` (ID `0x06`, Warning High):**
    Generated if B-dot is active and current magnetic field data is considered stale or has not been received.
-   **`MAG_DETUMBLE_INVALID_ANG_VEL` (ID `0x07`, Warning High):**
    Generated if B-dot is active and angular velocity data is considered stale or has not been received. (Note: B-dot can operate without angular velocity, but this event flags missing data if the port is connected).
-   **`MAG_DETUMBLE_ZERO_MAG_FIELD` (ID `0x08`, Warning High):**
    Generated if the magnitude of the magnetic field vector is near zero, rendering B-dot control potentially ineffective. Coils are typically commanded to zero in this case.
-   **`MAG_DETUMBLE_COIL_SATURATION(axis: U8, requested_current: F32, actual_current: F32)` (ID `0x09`, Warning Low):**
    Generated if the requested current for a coil exceeds its maximum limit, and the commanded current is saturated.
-   **`MAG_DETUMBLE_INVALID_BDOT_GAIN(gain: F32)` (ID `0x0A`, Warning Low):**
    Generated if a `MAG_DETUMBLE_SET_BDOT_GAIN` command provides an invalid gain value (e.g., negative).

## 4. Component Behavior

### 4.1. Initialization

Upon initialization, the component:
- Sets the default B-dot gain (e.g., 1.0).
- Disables B-dot mode.
- Initializes sensor data received flags to `false`.
- Initializes commanded dipole and coil currents to zero.

### 4.2. Direct Dipole Commanded Mode

- When a dipole is commanded via `MAG_DETUMBLE_SET_DIPOLE` or `setDipoleCmdIn`:
    - The component stores the new target dipole (Mx, My, Mz).
    - It emits the `MagDetumble_DipoleCmdX,Y,Z` telemetry.
    - It logs the `MAG_DETUMBLE_DIPOLE_CMD_RECEIVED` event.
    - It calls the `commandCoils` helper method to translate this dipole into coil current commands.
    - If invoked via `setDipoleCmdIn`, it reports `Fw::Success::SUCCESS` via `statusOut`.

### 4.3. B-dot Autonomous Mode

- Enabled/disabled by the `MAG_DETUMBLE_ENABLE_BDOT` command.
- Gain configured by the `MAG_DETUMBLE_SET_BDOT_GAIN` command.
- When the `runScheduleIn` port is called and B-dot is enabled:
    1.  **Sensor Data Check:**
        - Verifies that `magFieldIn` data is available and not stale (based on `m_magFieldTimeTag` and `SENSOR_DATA_STALE_THRESHOLD_SECONDS`). If stale, logs `MAG_DETUMBLE_INVALID_MAG_FIELD` and aborts B-dot for the cycle.
        - Verifies that `angularVelocityIn` data is available (if it's expected to be used or monitored). If stale, logs `MAG_DETUMBLE_INVALID_ANG_VEL`. (Current B-dot uses only B and B_dot from B).
    2.  **B_dot Calculation:**
        - Requires a previous magnetic field reading (`m_prevMagFieldX,Y,Z` and `m_prevMagFieldTimeTag`).
        - If no previous reading, the current reading is stored as "previous" for the next cycle.
        - Calculates `dt = m_magFieldTimeTag - m_prevMagFieldTimeTag`. If `dt <= 0`, aborts calculation for the cycle.
        - Calculates `B_dot = (current_B - previous_B) / dt`.
    3.  **Magnetic Field Strength Check:**
        - Calculates the magnitude of the current magnetic field `B`.
        - If `B` is near zero (below a threshold), logs `MAG_DETUMBLE_ZERO_MAG_FIELD`, commands zero dipole, and updates previous B for the next cycle.
    4.  **Dipole Calculation:**
        - Target dipole `m = -gain * B_dot` (vector components).
    5.  **Output:**
        - The calculated dipole (m_x, m_y, m_z) is stored and telemetered via `MagDetumble_DipoleCmdX,Y,Z`.
        - The `MAG_DETUMBLE_BDOT_UPDATE` event is logged.
        - The `commandCoils` helper is called with this target dipole.
    6.  **State Update:** The current magnetic field reading (`m_magFieldX,Y,Z`, `m_magFieldTimeTag`) becomes the "previous" reading for the next cycle.

### 4.4. Coil Commanding (`commandCoils` helper)

- This internal method takes a target 3-axis dipole (Mx, My, Mz).
- **Dipole to Current Mapping:** It converts this dipole into individual current commands for each coil (X, Y, Z).
    - *Simplification:* Currently assumes a direct 1:1 mapping (e.g., `current_X = dipole_X`), which implies that factors like number of coil turns and area are either 1 or absorbed into the B-dot gain or the definition of the commanded dipole. A more realistic model would use `current = dipole / (N * A * cos(theta))` or similar, considering coil properties and orientation.
- **Saturation:** Checks if the calculated current for any coil exceeds `MAX_COIL_CURRENT`.
    - If so, the current for that coil is clamped to `+/- MAX_COIL_CURRENT`.
    - The `MAG_DETUMBLE_COIL_SATURATION` event is logged for each saturated coil.
- **Output:** Sends the (possibly saturated) current commands to the `hwControlOut` port for each axis.
- Emits `MagDetumble_CoilCurrentX,Y,Z` telemetry.
- Logs `MAG_DETUMBLE_HW_CMD_SENT` events.

## 5. Component State Variables

- `m_dipoleCmdX, Y, Z`: Current target dipole.
- `m_magFieldX, Y, Z`: Last received magnetic field vector.
- `m_magFieldTimeTag`: Timestamp of the last magnetic field reading.
- `m_magFieldReceived`: Flag indicating if magnetic field data has ever been received.
- `m_angularVelocityX, Y, Z`: Last received angular velocity vector.
- `m_angularVelocityTimeTag`: Timestamp of the last angular velocity reading.
- `m_angularVelocityReceived`: Flag indicating if angular velocity data has ever been received.
- `m_bdotEnabled`: Boolean status of B-dot mode.
- `m_bdotGain`: Gain for the B-dot algorithm.
- `m_prevMagFieldX, Y, Z`: Magnetic field vector from the previous B-dot cycle.
- `m_prevMagFieldTimeTag`: Timestamp of the previous magnetic field reading.
- `m_prevMagFieldReceived`: Flag indicating if a previous B-dot magnetic field reading is available.
- `MAX_COIL_CURRENT`: Static constant defining coil saturation limit.
- `SENSOR_DATA_STALE_THRESHOLD_SECONDS`: Static constant for sensor data timeout.

## 6. Dependencies

- `Fw::Time` for time-tagging and calculating `dt`.
- Standard F' active component ports (`CmdDisp`, `CmdStatus`, `CmdReg`, `Log`, `LogText`, `Time`, `Tlm`, `PingIn`, `PingOut`).
- `Fw::Success` type.
- Math functions (e.g., `sqrt`, `fabs`) from `cmath`.

## 7. Unit Test Considerations

Unit tests are located in `Drv/MagneticDetumble/test/ut`. They cover:
- Command handling for all commands.
- Input port handling.
- Nominal B-dot algorithm execution.
- B-dot behavior with stale sensor data.
- B-dot behavior with zero magnetic field.
- Coil current saturation logic.
- Correct emission of events and telemetry.
- Correct invocation of output ports (`hwControlOut`, `statusOut`).
