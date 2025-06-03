# Software Design Description: MagneticDetumble Component

## 1. Introduction

The `MagneticDetumble` component is an F´ active component responsible for calculating the magnetic dipole moment required to detumble a spacecraft. It implements a B-dot control law based on the paper "Magnetic Detumbling of a Rigid Spacecraft" by Avanzini and Giulietti (DOI: 10.2514/1.53074).

The component receives angular velocity and magnetic field vectors as inputs and computes a commanded magnetic dipole moment. This dipole, when applied by spacecraft magnetorquers, is intended to reduce the spacecraft's rotational kinetic energy.

## 2. Requirements

| ID | Description | Verification Method |
|---|---|---|
| MD-001 | The component shall accept the spacecraft's current angular velocity vector. | Test |
| MD-002 | The component shall accept the current local magnetic field vector. | Test |
| MD-003 | The component shall calculate a commanded magnetic dipole moment based on the B-dot algorithm. | Test |
| MD-004 | The component shall output the commanded magnetic dipole moment. | Test |
| MD-005 | The component shall calculate the control gain `k!` based on configurable parameters. | Test |
| MD-006 | The component shall telemeter its inputs (angular velocity, magnetic field), calculated gain, and output (commanded dipole). | Test, Inspection |
| MD-007 | The component shall emit an event upon successful initialization. | Test |
| MD-008 | The component shall emit an event if the magnetic field magnitude is too low for reliable control. | Test |
| MD-009 | The component shall emit an event if the commanded dipole moment is saturated. | Test |

## 3. Design

### 3.1. Component Overview

`MagneticDetumble` is an active F´ component. It uses a scheduler input (`schedIn`) to periodically perform its calculations. On each scheduled cycle, it attempts to read the latest angular velocity and magnetic field data from its input ports. If new data is available for both, it computes the B-dot control law and outputs the resulting magnetic dipole command.

### 3.2. Control Law

The core control law implemented is derived from Eq. (14) of the reference paper:
`m = -(k! / |b|) * (b_hat x w)`
where:
- `m` is the commanded magnetic dipole moment.
- `k!` is the control gain.
- `b` is the magnetic field vector.
- `|b|` is the magnitude of the magnetic field vector.
- `b_hat` is the unit vector of the magnetic field (`b / |b|`).
- `w` is the angular velocity vector of the spacecraft.

The implementation uses an equivalent form to reduce divisions:
`m = -k! / |b|^2 * (b x w)`

The control gain `k!` is calculated based on Eq. (30) from the paper:
`k! = 2 * orbit_rate * (1 + sin(beta_m)) * J_min`
where:
- `orbit_rate` is the spacecraft's orbital rate in rad/s.
- `beta_m` is the inclination of the spacecraft orbit relative to the geomagnetic equatorial plane (geomagnetic inclination) in radians.
- `J_min` is the minimum principal moment of inertia of the spacecraft in kg*m^2.

### 3.3. Internal Parameters

The component utilizes the following internal parameters for its calculations. In the current version, these are hardcoded during construction but are designed to be configurable in future versions (e.g., via parameters or commands).

-   **Orbit Rate (`m_orbitRate`):** Defaults to `0.0011f` rad/s (approx. 90-minute orbit).
-   **Geomagnetic Inclination (`m_geomagneticInclination`):** Defaults to `PI/4` radians (45 degrees).
-   **Minimum Inertia (`m_minInertia`):** Defaults to `0.33f` kg*m^2.
-   **Dipole Saturation Limit (`m_dipoleSaturationLimit`):** Defaults to `2.0f` A*m^2. This is the maximum magnitude for the commanded dipole moment.

### 3.4. State
The component maintains the last received angular velocity and magnetic field vectors, along with flags indicating if new data has arrived for each.

## 4. Component Interface

### 4.1. Ports

#### 4.1.1. Input Ports

1.  **`angularVelocityIn`**:
    -   Type: `Svc.AngVel` (which uses `Svc.Vector3`)
    -   Direction: Input
    -   Kind: Asynchronous
    -   Max Connections: 1
    -   Function: Receives the current angular velocity vector of the spacecraft. `Svc.Vector3` contains three F32 members: `x`, `y`, `z`.
    -   Units: rad/s

2.  **`magneticFieldIn`**:
    -   Type: `Svc.MagField` (which uses `Svc.Vector3`)
    -   Direction: Input
    -   Kind: Asynchronous
    -   Max Connections: 1
    -   Function: Receives the current local magnetic field vector measured by the spacecraft.
    -   Units: Tesla (T)

3.  **`schedIn`**:
    -   Type: `Fw.Sched`
    -   Direction: Input
    -   Kind: Synchronous
    -   Max Connections: 1
    -   Function: Scheduler input to trigger the periodic execution of the control law calculation.

#### 4.1.2. Output Ports

1.  **`dipoleRequestOut`**:
    -   Type: `Svc.MagDipoleCmd` (which uses `Svc.Vector3`)
    -   Direction: Output
    -   Kind: Asynchronous
    -   Max Connections: 1 (typically connected to a magnetorquer driver or actuator interface)
    -   Function: Outputs the calculated commanded magnetic dipole moment.
    -   Units: A*m^2 (Ampere * meter^2)

### 4.2. Telemetry

1.  **`angularVelocity` (ID: 0)**
    -   Type: `Svc.Vector3`
    -   Description: The last angular velocity vector received and used in the control law calculation.
    -   Format: `(%f, %f, %f) rad/s`

2.  **`magneticField` (ID: 1)**
    -   Type: `Svc.Vector3`
    -   Description: The last magnetic field vector received and used in the control law calculation.
    -   Format: `(%f, %f, %f) T`

3.  **`controlGain` (ID: 2)**
    -   Type: `F32`
    -   Description: The calculated control gain `k!` currently being used by the component.
    -   Format: `%f`

4.  **`commandedDipole` (ID: 3)**
    -   Type: `Svc.Vector3`
    -   Description: The last commanded magnetic dipole moment calculated by the component. This value is after any saturation clamping.
    -   Format: `(%f, %f, %f) A*m^2`

### 4.3. Events

1.  **`DetumbleInitialized` (ID: 0)**
    -   Severity: `ACTIVITY_HI`
    -   Format String: `"MagneticDetumble component initialized with gain k! = %f"`
    -   Arguments:
        -   `gain` (F32): The initial calculated control gain.
    -   Description: Emitted once the component has been initialized.

2.  **`MagFieldNearZero` (ID: 1)**
    -   Severity: `WARNING_LO`
    -   Format String: `"Magnetic field magnitude (%f T) is near zero. Commanding zero dipole."`
    -   Arguments:
        -   `magnitude` (F32): The measured magnitude of the magnetic field.
    -   Description: Emitted if the magnitude of the input magnetic field vector is too small for reliable control calculation, resulting in a zero dipole command.

3.  **`DipoleSaturated` (ID: 2)**
    -   Severity: `WARNING_LO`
    -   Format String: `"Commanded dipole moment (%f, %f, %f A*m^2) exceeds saturation limit of %f A*m^2. Outputting clamped value."`
    -   Arguments:
        -   `commanded_x` (F32): The x-component of the originally calculated dipole.
        -   `commanded_y` (F32): The y-component of the originally calculated dipole.
        -   `commanded_z` (F32): The z-component of the originally calculated dipole.
        -   `limit` (F32): The saturation limit for the dipole moment magnitude.
    -   Description: Emitted if the calculated magnetic dipole moment's magnitude exceeds the configured saturation limit. The output dipole is clamped to this limit.

4.  **`ParameterUpdate` (ID: 3)**
    -   Severity: `ACTIVITY_HI`
    -   Format String: `"Control parameter %s updated to %f. Recalculated gain k! = %f"`
    -   Arguments:
        -   `parameterName` (Fw::LogStringArg, size 40): Name of the parameter that was updated.
        -   `value` (F32): The new value of the parameter.
        -   `newGain` (F32): The newly calculated control gain `k!` after the parameter update.
    -   Description: Emitted when a configurable control parameter is updated and the gain is recalculated. (Note: Parameter update mechanism is not yet implemented in the first version).


## 5. Component Dependencies
- `Fw::Time` (for `schedIn` port)
- `Fw::Logger` (for event logging)
- `Svc::Vector3` (as part of port types `Svc.AngVel`, `Svc.MagField`, `Svc.MagDipoleCmd`)

## 6. Assumptions
- The input angular velocity vector is provided in the spacecraft body frame.
- The input magnetic field vector is provided in the spacecraft body frame.
- The output magnetic dipole moment is expected in the spacecraft body frame.
- The component operates on F32 floating-point precision.

## 7. Future Work / Enhancements
-   Make internal parameters (`m_orbitRate`, `m_geomagneticInclination`, `m_minInertia`, `m_dipoleSaturationLimit`) configurable via F´ parameters.
-   Add commands to enable/disable the detumbling control calculation.
-   Add a command to manually set the control gain `k!`.
-   More sophisticated handling of sensor data validity or staleness.
