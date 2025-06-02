module Svc {

  passive component MagneticDetumble {

    imports {
      Fw.On
      Fw.Math.Vector3
      Fw.Time
    } # Corrected: Added missing closing brace

    commands {
      SET_MODE(mode: Fw.On.Enabled) \
        opcode 0x00 \
        priority 100 \
        help "Sets the detumble mode (AUTO/MANUAL)."

      SET_DIPOLE(dipole: Fw.Math.Vector3) \
        opcode 0x01 \
        priority 100 \
        help "Sets the desired magnetic dipole in Am^2 (for manual mode)."
    }

    telemetry {
      MODE: Fw.On.Enabled \
        id 0x00 \
        update always \
        help "Current detumble mode."

      DIPOLE: Fw.Math.Vector3 \
        id 0x01 \
        update always \
        help "Current magnetic dipole in Am^2."

      OMEGA: Fw.Math.Vector3 \
        id 0x02 \
        update always \
        help "Current angular velocity in rad/s."

      MAG_FIELD: Fw.Math.Vector3 \
        id 0x03 \
        update always \
        help "Current magnetic field vector in Tesla."
    }

    events {
      MODE_CHANGED(mode: Fw.On.Enabled) \
        id 0x00 \
        severity activity_hi \
        format "Detumble mode changed to {}" \
        help "Indicates that the detumble mode has changed."

      DIPOLE_SET(dipole: Fw.Math.Vector3) \
        id 0x01 \
        severity activity_hi \
        format "Magnetic dipole set to {}" \
        help "Indicates that the magnetic dipole has been set."

      CONTROL_ITERATION_ERROR(status: I32) \
        id 0x02 \
        severity warning_hi \
        format "Control iteration error with status {}" \
        help "Indicates an error in the control iteration."
    }

    parameters {
      CONTROL_LAW_GAIN: F64 \
        id 0x00 \
        default 1.0 \
        help "The gain for the B-dot control law."
    }

    ports {
      # Input ports
      port angularVelocityIn: [1] Fw.Math.Vector3Read \
        kind sync_input \
        help "Port for angular velocity input"

      port magFieldIn: [1] Fw.Math.Vector3Read \
        kind sync_input \
        help "Port for magnetic field vector input"

      port schedIn: [1] Svc.Sched \
        kind sync_input \
        help "Scheduler input port"

      port cmdIn: [1] Fw.Cmd \
        kind sync_input \
        help "Command input port"

      port prmSetIn: [1] Fw.PrmSet \
        kind sync_input \
        help "Parameter set input port"

      # Output ports
      port magnetorquerCmdOut: [1] Fw.Math.Vector3Write \
        kind output \
        help "Port for commanding magnetorquers"

      port cmdRegOut: [1] Fw.CmdReg \
        kind output \
        help "Command registration output port"

      port cmdResponseOut: [1] Fw.CmdResponse \
        kind output \
        help "Command response output port"

      port eventOut: [1] Fw.Log \
        kind output \
        help "Event output port"

      port textEventOut: [1] Fw.LogText \
        kind output \
        help "Text event output port"

      port tlmOut: [1] Fw.Tlm \
        kind output \
        help "Telemetry output port"

      port prmGetOut: [1] Fw.PrmGet \
        kind output \
        help "Parameter get output port"

      port timeCaller: [1] Fw.Time \
        kind output \
        help "Time caller output port"
    }
  }
}
