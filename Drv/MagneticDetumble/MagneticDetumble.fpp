module Drv {

  @ Component for magnetic detumbling using magnetorquers
  active component MagneticDetumble {

    include "Fw/Time/TimePortAi.xml"
    include "Fw/Com/ComPortAi.xml"
    include "Fw/Cmd/CmdResponsePortAi.xml"
    include "Fw/Cmd/CmdPortAi.xml"
    include "Fw/Log/LogPortAi.xml"
    include "Fw/Log/LogTextPortAi.xml"
    include "Fw/Tlm/TlmPortAi.xml"
    include "Fw/Buffer/BufferSendPortAi.xml"
    include "Fw/Success/Success.fpp" // For Fw::Success

    @ Input port for commanding a specific magnetic dipole
    async input port setDipoleCmdIn(
      x: F32 @< X-axis dipole component
      y: F32 @< Y-axis dipole component
      z: F32 @< Z-axis dipole component
    )

    @ Input port for receiving the current magnetic field vector
    async input port magFieldIn(
      x: F32 @< X-axis magnetic field component
      y: F32 @< Y-axis magnetic field component
      z: F32 @< Z-axis magnetic field component
    )

    @ Input port for receiving spacecraft angular velocity
    async input port angularVelocityIn(
      x: F32 @< X-axis angular velocity
      y: F32 @< Y-axis angular velocity
      z: F32 @< Z-axis angular velocity
    )

    @ Input port for scheduled execution (e.g., for B-dot calculation)
    sync input port runScheduleIn(
      context: U32 @< The call context
    )

    @ Output port for command execution status
    output port statusOut: Fw.Success

    @ Output port for commanding coil currents (abstracted hardware control)
    output port hwControlOut(
      axis: U8 @< Axis identifier (0=X, 1=Y, 2=Z)
      current: F32 @< Current to apply to the coil
    )

    @ Command to set the target magnetic dipole directly
    command MAG_DETUMBLE_SET_DIPOLE(
      x: F32 @< X-axis dipole component
      y: F32 @< Y-axis dipole component
      z: F32 @< Z-axis dipole component
    ) "Set target magnetic dipole"
    opcode 0x01
    priority 100

    @ Command to enable/disable internal B-dot control mode
    command MAG_DETUMBLE_ENABLE_BDOT(
      enable: bool @< True to enable B-dot, false to disable
    ) "Enable or disable B-dot control mode"
    opcode 0x02
    priority 100

    @ Command to set the gain for the B-dot controller
    command MAG_DETUMBLE_SET_BDOT_GAIN(
      gain: F32 @< B-dot controller gain
    ) "Set B-dot controller gain"
    opcode 0x03
    priority 100

    telemetry MagDetumble_DipoleCmdX: F32 format "%.3f" @ Commanded X-axis dipole
    telemetry MagDetumble_DipoleCmdY: F32 format "%.3f" @ Commanded Y-axis dipole
    telemetry MagDetumble_DipoleCmdZ: F32 format "%.3f" @ Commanded Z-axis dipole

    telemetry MagDetumble_MagFieldX: F32 format "%.3f" @ Measured X-axis magnetic field
    telemetry MagDetumble_MagFieldY: F32 format "%.3f" @ Measured Y-axis magnetic field
    telemetry MagDetumble_MagFieldZ: F32 format "%.3f" @ Measured Z-axis magnetic field

    telemetry MagDetumble_AngularVelocityX: F32 format "%.3f" @ Measured X-axis angular velocity
    telemetry MagDetumble_AngularVelocityY: F32 format "%.3f" @ Measured Y-axis angular velocity
    telemetry MagDetumble_AngularVelocityZ: F32 format "%.3f" @ Measured Z-axis angular velocity

    telemetry MagDetumble_BdotGain: F32 format "%.3f" @ B-dot controller gain
    telemetry MagDetumble_BdotEnabled: bool @ B-dot mode status

    telemetry MagDetumble_CoilCurrentX: F32 format "%.3f" @ Commanded X-axis coil current
    telemetry MagDetumble_CoilCurrentY: F32 format "%.3f" @ Commanded Y-axis coil current
    telemetry MagDetumble_CoilCurrentZ: F32 format "%.3f" @ Commanded Z-axis coil current

    event MAG_DETUMBLE_DIPOLE_CMD_RECEIVED(
      x: F32 @< Commanded X-axis dipole
      y: F32 @< Commanded Y-axis dipole
      z: F32 @< Commanded Z-axis dipole
    ) severity activity high format "Direct dipole command received: (%.3f, %.3f, %.3f)" id 0x01

    event MAG_DETUMBLE_BDOT_STATUS_CHANGED(
      enabled: bool @< New B-dot status
    ) severity activity high format "B-dot mode status changed to: %d" id 0x02

    event MAG_DETUMBLE_BDOT_GAIN_SET(
      gain: F32 @< New B-dot gain
    ) severity activity high format "B-dot gain set to: %.3f" id 0x03

    event MAG_DETUMBLE_BDOT_UPDATE(
      dipoleX: F32 @< Calculated X-axis dipole by B-dot
      dipoleY: F32 @< Calculated Y-axis dipole by B-dot
      dipoleZ: F32 @< Calculated Z-axis dipole by B-dot
    ) severity activity high format "B-dot update. Calculated dipole: (%.3f, %.3f, %.3f)" id 0x04

    event MAG_DETUMBLE_HW_CMD_SENT(
      axis: U8 @< Axis commanded
      current: F32 @< Current commanded
    ) severity activity low format "Hardware command sent for axis %d with current %.3f" id 0x05

    event MAG_DETUMBLE_INVALID_MAG_FIELD
      severity warning high format "Magnetic field data is stale or missing for B-dot calculation." id 0x06

    event MAG_DETUMBLE_INVALID_ANG_VEL
      severity warning high format "Angular velocity data is stale or missing for B-dot calculation." id 0x07

    event MAG_DETUMBLE_ZERO_MAG_FIELD
      severity warning high format "Magnetic field vector magnitude is near zero; B-dot control may be ineffective." id 0x08

    event MAG_DETUMBLE_COIL_SATURATION(
        axis: U8 @< Axis commanded
        requested_current: F32 @< Requested current
        actual_current: F32 @< Actual current set (due to saturation)
    ) severity warning low format "Coil saturation on axis %d. Requested %.3f, actual %.3f" id 0x09

    event MAG_DETUMBLE_INVALID_BDOT_GAIN(
      gain: F32 @< The invalid gain value provided
    ) severity warning low format "Invalid B-dot gain provided: %.3f. Gain must be non-negative." id 0x0A

    internal port PingIn(
        key: U32 @< The ping key
    )

    output port PingOut(
        key: U32 @< The ping key
    )

    input port CmdDisp(
        opCode: U32 @< The command opcode
        cmdSeq: U32 @< The command sequence number
        args: Fw.CmdArgBuffer @< The command arguments
    )

    output port CmdReg(
        opCode: U32 @< The command opcode
    )

    output port CmdStatus(
        opCode: U32 @< The command opcode
        cmdSeq: U32 @< The command sequence number
        response: Fw.CmdResponse @< The command response
    )

    output port Log(
        id: FwEventIdType @< The event ID
        timeTag: Fw.Time @< The time
        severity: Fw.LogSeverity @< The severity
        args: Fw.LogBuffer @< The serialized arguments
    )

    output port LogText(
        id: FwEventIdType @< The event ID
        timeTag: Fw.Time @< The time
        severity: Fw.LogSeverity @< The severity
        text: Fw.TextLogString @< The event string
    )

    output port Time(
        time: Fw.Time @< The U32 cmd argument
    )

    output port Tlm(
        id: FwChanIdType @< The channel ID
        timeTag: Fw.Time @< The time
        val: Fw.TlmBuffer @< The channel value
    )

  }
}
