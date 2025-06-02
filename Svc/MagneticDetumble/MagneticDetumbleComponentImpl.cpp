#include <Svc/MagneticDetumble/MagneticDetumbleComponentImpl.hpp>
#include <Fw/Types/BasicTypes.hpp>
#include <Fw/Types/Assert.hpp>
#include <cmath> // For sqrt, fabs

// ISF generated code includes
#include <Fw/Com/ComBuffer.hpp> // For ComBuffer (needed for port calls)
#include <Fw/Types/SerialBuffer.hpp> // For serial buffer (needed for port calls)


namespace Svc {

  // ----------------------------------------------------------------------
  // Construction, initialization, and destruction
  // ----------------------------------------------------------------------

  MagneticDetumbleComponentImpl ::
    MagneticDetumbleComponentImpl(
        const char *const compName
    ) : MagneticDetumbleComponentBase(compName),
        m_currentMode(Fw::On::OFF), // Default to OFF, will be set by param or command
        m_manualDipole(0.0f, 0.0f, 0.0f),
        m_previousB(0.0f, 0.0f, 0.0f),
        m_previousTime(), // Default constructor for Fw::Time
        m_controlGain(0.0), // Will be loaded from parameter
        m_firstRun(true)
  {

  }

  void MagneticDetumbleComponentImpl ::
    init(
        const NATIVE_INT_TYPE instance
    )
  {
    MagneticDetumbleComponentBase::init(instance);

    // Load parameters
    // If paramValid is NO_VALID, the default value is used.
    // If it is DEFAULT, the value is the default.
    // If it is VALID, then the new value is used.
    Fw::ParamValid gainValid = Fw::ParamValid::DEFAULT;
    this->m_controlGain = this->paramGet_CONTROL_GAIN(gainValid);
    if (gainValid == Fw::ParamValid::INVALID) {
        this->log_WARNING_HI_PARAM("CONTROL_GAIN parameter load failed, using default: %f", this->m_controlGain);
    } else {
        this->log_ACTIVITY_LO_PARAM("CONTROL_GAIN parameter loaded: %f", this->m_controlGain);
    }
    
    Fw::ParamValid initialModeValid = Fw::ParamValid::DEFAULT;
    Fw::On initialMode = this->paramGet_INITIAL_MODE(initialModeValid);
    if (initialModeValid == Fw::ParamValid::INVALID) {
        this->log_WARNING_HI_PARAM("INITIAL_MODE parameter load failed, defaulting to AUTO");
        this->m_currentMode = Fw::On::ON; // ON typically means AUTO for this component
    } else {
        this->m_currentMode = initialMode;
        this->log_ACTIVITY_LO_PARAM("INITIAL_MODE parameter loaded: %s", this->m_currentMode == Fw::On::ON ? "AUTO" : "MANUAL");
    }
    
    // After loading params, update telemetry for initial state
    this->tlmWrite_MODE(this->m_currentMode);
    this->tlmWrite_DIPOLE(this->m_manualDipole); // Initial manual dipole is zero
  }

  MagneticDetumbleComponentImpl ::
    ~MagneticDetumbleComponentImpl()
  {

  }

  // ----------------------------------------------------------------------
  // Command handler implementations
  // ----------------------------------------------------------------------

  void MagneticDetumbleComponentImpl ::
    SET_MODE_cmdHandler(
        FwOpcodeType opCode,
        U32 cmdSeq,
        Fw::On::t mode
    )
  {
    this->m_currentMode = mode;
    this->log_ACTIVITY_HI_MODE_CHANGED(mode);
    this->tlmWrite_MODE(mode);
    this->cmdResponse_out(opCode, cmdSeq, Fw::CmdResponse::OK);
  }

  void MagneticDetumbleComponentImpl ::
    SET_DIPOLE_cmdHandler(
        FwOpcodeType opCode,
        U32 cmdSeq,
        const Fw::Math::Vector3 &dipole
    )
  {
    if (this->m_currentMode == Fw::On::OFF) { // Assuming OFF means MANUAL
      this->m_manualDipole = dipole;
      this->log_ACTIVITY_HI_DIPOLE_SET(dipole);
      this->tlmWrite_DIPOLE(dipole);
      this->cmdResponse_out(opCode, cmdSeq, Fw::CmdResponse::OK);
    } else { // AUTO mode
      this->log_WARNING_HI_COMMAND("SET_DIPOLE command rejected in AUTO mode");
      this->cmdResponse_out(opCode, cmdSeq, Fw::CmdResponse::VALIDATION_ERROR);
    }
  }

  // ----------------------------------------------------------------------
  // Handler implementations for user-defined typed input ports
  // ----------------------------------------------------------------------

  void MagneticDetumbleComponentImpl ::
    SCHED_IN_handler(
        const NATIVE_INT_TYPE portNum,
        NATIVE_UINT_TYPE context
    )
  {
    // Read Angular Velocity
    Fw::Math::Vector3 omega(0.0f, 0.0f, 0.0f);
    if (this->isConnected_angularVelocityIn_InputPort(0)) {
      Fw::Success status = this->angularVelocityIn_InputPort[0]->invoke(omega);
      if (status != Fw::Success::SUCCESS) {
          this->log_WARNING_HI_CONTROL_ITERATION_ERROR("Failed to get angular velocity");
          // Decide if we should proceed or return early
      }
    } else {
      this->log_WARNING_LO_CONTROL_ITERATION_ERROR("Angular velocity port not connected");
      // Decide if we should proceed or return early. For detumble, often we can proceed without omega if not available.
    }
    this->tlmWrite_OMEGA(omega);

    // Read Magnetic Field
    Fw::Math::Vector3 current_B(0.0f, 0.0f, 0.0f);
    if (this->isConnected_magFieldIn_InputPort(0)) {
      Fw::Success status = this->magFieldIn_InputPort[0]->invoke(current_B);
       if (status != Fw::Success::SUCCESS) {
          this->log_WARNING_HI_CONTROL_ITERATION_ERROR("Failed to get magnetic field");
          return; // Cannot proceed without magnetic field
      }
    } else {
      this->log_WARNING_HI_CONTROL_ITERATION_ERROR("Magnetic field port not connected");
      return; // Cannot proceed without magnetic field
    }
    this->tlmWrite_MAG_FIELD(current_B);

    Fw::Math::Vector3 dipoleToCommand(0.0f, 0.0f, 0.0f);

    if (this->m_currentMode == Fw::On::ON) { // AUTO mode
      Fw::Time currentTime;
      if (this->isConnected_timeCaller_OutputPort(0)) {
         currentTime = this->timeCaller_OutputPort[0]->invoke();
      } else {
         this->log_WARNING_HI_CONTROL_ITERATION_ERROR("Time port not connected");
         // Use a default or skip B_dot if time is critical and not available
         // For simplicity here, we'll proceed, but B_dot will be inaccurate
         // A real implementation might need a more robust fallback or error state
      }

      if (this->m_firstRun) {
        this->m_previousB = current_B;
        this->m_previousTime = currentTime;
        this->m_firstRun = false;
        this->log_ACTIVITY_LO_CONTROL_ITERATION("First run, storing initial B field and time.");
        // Don't command dipole on first run as B_dot is not available yet
      } else {
        Fw::Math::Vector3 b_dot(0.0f, 0.0f, 0.0f);
        F64 dt_secs = 0.0;

        if (currentTime.getSeconds() == this->m_previousTime.getSeconds() && currentTime.getUSeconds() == this->m_previousTime.getUSeconds()) {
            // dt is zero, B_dot cannot be calculated. This might happen if SCHED_IN is called too rapidly
            // or if time resolution is insufficient or time port doesn't update quickly enough.
            this->log_WARNING_HI_CONTROL_ITERATION_ERROR("Time delta is zero, cannot calculate B_dot.");
            // Keep previous dipole or set to zero. For now, don't update dipole.
            dipoleToCommand = this->tlmRecv_DIPOLE(); // Use last commanded dipole
        } else {
            // Calculate dt in seconds (Fw::Time stores seconds and microseconds)
            dt_secs = static_cast<F64>(currentTime.getSeconds() - this->m_previousTime.getSeconds()) +
                      static_cast<F64>(currentTime.getUSeconds() - this->m_previousTime.getUSeconds()) / 1000000.0;

            if (dt_secs <= 0) { // Should not happen if time is moving forward
                this->log_WARNING_HI_CONTROL_ITERATION_ERROR("Time delta is zero or negative, cannot calculate B_dot accurately.");
                dipoleToCommand = this->tlmRecv_DIPOLE(); 
            } else {
                // B_dot = (current_B - previous_B) / dt
                b_dot.set(
                    (current_B.getX() - this->m_previousB.getX()) / dt_secs,
                    (current_B.getY() - this->m_previousB.getY()) / dt_secs,
                    (current_B.getZ() - this->m_previousB.getZ()) / dt_secs
                );

                F32 b_magnitude = current_B.magnitude();
                const F32 epsilon = 1e-6f; // Threshold for B-field magnitude to avoid division by zero/small number

                if (b_magnitude < epsilon) {
                  this->log_WARNING_HI_CONTROL_ITERATION_ERROR("Magnetic field magnitude too small, cannot calculate dipole.");
                  // Keep previous dipole or set to zero. For now, set to zero to be safe.
                  dipoleToCommand.set(0.0f, 0.0f, 0.0f);
                } else {
                  // m = -K * B_dot / ||B||  (F´ vectors are column vectors, so direct scalar mult and div is fine)
                  // Ensure B_dot is also scaled by -m_controlGain
                  dipoleToCommand = b_dot * (-this->m_controlGain / b_magnitude);
                  this->log_ACTIVITY_LO_CONTROL_ITERATION("Calculated dipole in AUTO mode.");
                }
            }
        }
        // Store current B and time for next iteration
        this->m_previousB = current_B;
        this->m_previousTime = currentTime;
      }
    } else { // MANUAL mode (m_currentMode == Fw::On::OFF)
      dipoleToCommand = this->m_manualDipole;
      this->log_ACTIVITY_LO_CONTROL_ITERATION("Using manual dipole.");
    }

    // Send dipole command to magnetorquers
    if (this->isConnected_magnetorquerCmdOut_OutputPort(0)) {
      // Assuming magnetorquerCmdOut_OutputPort takes Fw::Math::Vector3 directly
      this->magnetorquerCmdOut_OutputPort[0]->invoke(dipoleToCommand);
    } else {
      this->log_WARNING_LO_CONTROL_ITERATION_ERROR("Magnetorquer command port not connected.");
    }

    // Report current mode and commanded dipole telemetry
    this->tlmWrite_MODE(this->m_currentMode);
    this->tlmWrite_DIPOLE(dipoleToCommand);
  }

} // end namespace Svc
