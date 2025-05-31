#include <Drv/MagneticDetumble/MagneticDetumbleImpl.hpp>
#include <Fw/Types/BasicTypes.hpp>
#include <Fw/Types/Assert.hpp>
#include <cmath> // For fabs, sqrt

// Maybe include a platform specific header for time if needed for Fw::Time subtractions
// For example, Os/Posix/File.hpp or similar if it provides time utilities,
// but Fw::Time should be sufficient for time differences.

namespace Drv {

  // ----------------------------------------------------------------------
  // Construction, initialization, and destruction
  // ----------------------------------------------------------------------

  MagneticDetumbleImpl::MagneticDetumbleImpl(const char* const compName) :
    MagneticDetumbleComponentBase(compName),
    m_dipoleCmdX(0.0f),
    m_dipoleCmdY(0.0f),
    m_dipoleCmdZ(0.0f),
    m_magFieldX(0.0f),
    m_magFieldY(0.0f),
    m_magFieldZ(0.0f),
    m_magFieldReceived(false),
    m_angularVelocityX(0.0f),
    m_angularVelocityY(0.0f),
    m_angularVelocityZ(0.0f),
    m_angularVelocityReceived(false),
    m_bdotEnabled(false),
    m_bdotGain(1.0f), // Default gain
    m_prevMagFieldX(0.0f),
    m_prevMagFieldY(0.0f),
    m_prevMagFieldZ(0.0f),
    m_prevMagFieldReceived(false)
  {
    // Initialize time tags to zero or a specific epoch if available
    // Fw::Time() constructor initializes to zero (TB_NONE, 0 seconds, 0 useconds)
  }

  void MagneticDetumbleImpl::init(const NATIVE_INT_TYPE instance) {
    MagneticDetumbleComponentBase::init(instance);
  }

  MagneticDetumbleImpl::~MagneticDetumbleImpl() {}

  // ----------------------------------------------------------------------
  // Handler implementations for user-defined typed input ports
  // ----------------------------------------------------------------------

  void MagneticDetumbleImpl::setDipoleCmdIn_handler(
      const NATIVE_INT_TYPE portNum,
      F32 x, F32 y, F32 z
  ) {
    // If B-dot is enabled, direct dipole commands might be ignored or logged as an override.
    // For now, allow direct command to override B-dot's target for one cycle.
    // Or, we could choose to disable B-dot if a direct command comes in.
    // Current choice: direct command takes precedence.
    if (this->m_bdotEnabled) {
        this->log_ACTIVITY_HI_MAG_DETUMBLE_DIPOLE_CMD_RECEIVED(x,y,z);
        // this->log_WARNING_LO_MAG_DETUMBLE_BDOT_OVERRIDDEN(); // Example event if we add it
    }

    this->m_dipoleCmdX = x;
    this->m_dipoleCmdY = y;
    this->m_dipoleCmdZ = z;

    this->tlmWrite_MagDetumble_DipoleCmdX(m_dipoleCmdX);
    this->tlmWrite_MagDetumble_DipoleCmdY(m_dipoleCmdY);
    this->tlmWrite_MagDetumble_DipoleCmdZ(m_dipoleCmdZ);

    this->log_ACTIVITY_HI_MAG_DETUMBLE_DIPOLE_CMD_RECEIVED(x, y, z);

    // Command coils directly based on this input
    this->commandCoils(m_dipoleCmdX, m_dipoleCmdY, m_dipoleCmdZ);

    if (this->isConnected_statusOut_OutputPort(0)) {
        this->statusOut_out(0, Fw::Success::SUCCESS);
    }
  }

  void MagneticDetumbleImpl::magFieldIn_handler(
      const NATIVE_INT_TYPE portNum,
      F32 x, F32 y, F32 z
  ) {
    this->m_magFieldX = x;
    this->m_magFieldY = y;
    this->m_magFieldZ = z;
    this->m_magFieldTimeTag = this->getTime();
    this->m_magFieldReceived = true;

    this->tlmWrite_MagDetumble_MagFieldX(x);
    this->tlmWrite_MagDetumble_MagFieldY(y);
    this->tlmWrite_MagDetumble_MagFieldZ(z);
  }

  void MagneticDetumbleImpl::angularVelocityIn_handler(
      const NATIVE_INT_TYPE portNum,
      F32 x, F32 y, F32 z
  ) {
    this->m_angularVelocityX = x;
    this->m_angularVelocityY = y;
    this->m_angularVelocityZ = z;
    this->m_angularVelocityTimeTag = this->getTime();
    this->m_angularVelocityReceived = true;

    this->tlmWrite_MagDetumble_AngularVelocityX(x);
    this->tlmWrite_MagDetumble_AngularVelocityY(y);
    this->tlmWrite_MagDetumble_AngularVelocityZ(z);
  }

  void MagneticDetumbleImpl::runScheduleIn_handler(
      const NATIVE_INT_TYPE portNum,
      U32 context
  ) {
    if (!this->m_bdotEnabled) {
      // B-dot is not active, potentially decay commanded dipole or do nothing.
      // For now, if B-dot is off, only direct commands change the dipole.
      // Optionally, zero out coils if no commands for a while.
      return;
    }

    Fw::Time currentTime = this->getTime();

    // Check if sensor data is fresh
    if (!this->m_magFieldReceived) {
        this->log_WARNING_HI_MAG_DETUMBLE_INVALID_MAG_FIELD();
        return;
    }
    if (!this->m_angularVelocityReceived) { // B-dot doesn't strictly need this, but often used in practice or for B-field rate estimation
        this->log_WARNING_HI_MAG_DETUMBLE_INVALID_ANG_VEL();
        // Depending on B-dot variant, may or may not return.
        // Classical B-dot only needs B and B_dot.
    }

    Fw::Time magTimeDiff = Fw::Time::sub(currentTime, this->m_magFieldTimeTag);
    if (magTimeDiff.getSeconds() >= SENSOR_DATA_STALE_THRESHOLD_SECONDS) {
        this->log_WARNING_HI_MAG_DETUMBLE_INVALID_MAG_FIELD();
        return;
    }

    // Check if previous magnetic field reading exists for B_dot calculation
    if (!this->m_prevMagFieldReceived) {
        // Store current as previous for next cycle
        this->m_prevMagFieldX = this->m_magFieldX;
        this->m_prevMagFieldY = this->m_magFieldY;
        this->m_prevMagFieldZ = this->m_magFieldZ;
        this->m_prevMagFieldTimeTag = this->m_magFieldTimeTag; // Use the timestamp of the current reading
        this->m_prevMagFieldReceived = true;
        //this->log_ACTIVITY_LO_MAG_DETUMBLE_BDOT_FIRST_CYCLE(); // Example event
        return;
    }

    Fw::Time bdotTimeDelta_fw = Fw::Time::sub(this->m_magFieldTimeTag, this->m_prevMagFieldTimeTag);
    F64 dt = static_cast<F64>(bdotTimeDelta_fw.getSeconds()) + static_cast<F64>(bdotTimeDelta_fw.getUSeconds()) / 1000000.0;

    if (dt <= 0.0) { // Avoid division by zero or negative time delta
        // Update previous B-field and time for the next iteration
        this->m_prevMagFieldX = this->m_magFieldX;
        this->m_prevMagFieldY = this->m_magFieldY;
        this->m_prevMagFieldZ = this->m_magFieldZ;
        this->m_prevMagFieldTimeTag = this->m_magFieldTimeTag;
        //this->log_WARNING_LO_MAG_DETUMBLE_BDOT_DT_INVALID(dt); // Example event
        return;
    }

    // Calculate B_dot
    F32 bdotX = (this->m_magFieldX - this->m_prevMagFieldX) / static_cast<F32>(dt);
    F32 bdotY = (this->m_magFieldY - this->m_prevMagFieldY) / static_cast<F32>(dt);
    F32 bdotZ = (this->m_magFieldZ - this->m_prevMagFieldZ) / static_cast<F32>(dt);

    // Check if magnetic field is too weak
    F32 bMag = sqrt(this->m_magFieldX * this->m_magFieldX +
                    this->m_magFieldY * this->m_magFieldY +
                    this->m_magFieldZ * this->m_magFieldZ);
    if (bMag < 1e-7) { // Threshold for near-zero magnetic field (example value)
        this->log_WARNING_HI_MAG_DETUMBLE_ZERO_MAG_FIELD();
        // Command zero dipole if field is unusable
        this->commandCoils(0.0f, 0.0f, 0.0f);

        // Update previous B-field for next cycle
        this->m_prevMagFieldX = this->m_magFieldX;
        this->m_prevMagFieldY = this->m_magFieldY;
        this->m_prevMagFieldZ = this->m_magFieldZ;
        this->m_prevMagFieldTimeTag = this->m_magFieldTimeTag;
        return;
    }

    // B-dot control law: m = -k * B_dot (vector form)
    // We need to be careful with coordinate frames. Assuming B_dot and m are in the spacecraft body frame.
    F32 targetDipoleX = -this->m_bdotGain * bdotX;
    F32 targetDipoleY = -this->m_bdotGain * bdotY;
    F32 targetDipoleZ = -this->m_bdotGain * bdotZ;

    this->m_dipoleCmdX = targetDipoleX;
    this->m_dipoleCmdY = targetDipoleY;
    this->m_dipoleCmdZ = targetDipoleZ;

    this->tlmWrite_MagDetumble_DipoleCmdX(m_dipoleCmdX);
    this->tlmWrite_MagDetumble_DipoleCmdY(m_dipoleCmdY);
    this->tlmWrite_MagDetumble_DipoleCmdZ(m_dipoleCmdZ);

    this->log_ACTIVITY_HI_MAG_DETUMBLE_BDOT_UPDATE(targetDipoleX, targetDipoleY, targetDipoleZ);
    this->commandCoils(targetDipoleX, targetDipoleY, targetDipoleZ);

    // Update previous B-field for next cycle
    this->m_prevMagFieldX = this->m_magFieldX;
    this->m_prevMagFieldY = this->m_magFieldY;
    this->m_prevMagFieldZ = this->m_magFieldZ;
    this->m_prevMagFieldTimeTag = this->m_magFieldTimeTag; // Use the timestamp of the current reading
  }


  // ----------------------------------------------------------------------
  // Command handler implementations
  // ----------------------------------------------------------------------

  void MagneticDetumbleImpl::MAG_DETUMBLE_SET_DIPOLE_cmdHandler(
      const FwOpcodeType opCode,
      const U32 cmdSeq,
      F32 x, F32 y, F32 z
  ) {
    // If B-dot is enabled, this command will override it for one cycle via setDipoleCmdIn.
    // Or, we could disable B-dot mode here.
    // For simplicity, this command directly sets the dipole, B-dot mode is separate.
    // However, the `setDipoleCmdIn` is the primary way to set a dipole if not using B-dot.
    // This command can be seen as a specific way to invoke that port's logic.

    this->m_dipoleCmdX = x;
    this->m_dipoleCmdY = y;
    this->m_dipoleCmdZ = z;

    this->tlmWrite_MagDetumble_DipoleCmdX(m_dipoleCmdX);
    this->tlmWrite_MagDetumble_DipoleCmdY(m_dipoleCmdY);
    this->tlmWrite_MagDetumble_DipoleCmdZ(m_dipoleCmdZ);

    this->log_ACTIVITY_HI_MAG_DETUMBLE_DIPOLE_CMD_RECEIVED(x, y, z);
    this->commandCoils(m_dipoleCmdX, m_dipoleCmdY, m_dipoleCmdZ);
    this->cmdResponse_out(opCode, cmdSeq, Fw::CmdResponse::OK);
  }

  void MagneticDetumbleImpl::MAG_DETUMBLE_ENABLE_BDOT_cmdHandler(
      const FwOpcodeType opCode,
      const U32 cmdSeq,
      bool enable
  ) {
    this->m_bdotEnabled = enable;
    this->tlmWrite_MagDetumble_BdotEnabled(this->m_bdotEnabled);
    this->log_ACTIVITY_HI_MAG_DETUMBLE_BDOT_STATUS_CHANGED(this->m_bdotEnabled);

    // Reset previous B-field data when B-dot is toggled to ensure fresh B_dot calculation
    if (this->m_bdotEnabled) {
        this->m_prevMagFieldReceived = false;
        // Also reset current sensor data flags so it waits for fresh data
        this->m_magFieldReceived = false;
        this->m_angularVelocityReceived = false;
    } else {
        // If disabling B-dot, maybe command zero dipole?
        this->commandCoils(0.0f, 0.0f, 0.0f);
        this->m_dipoleCmdX = 0.0f;
        this->m_dipoleCmdY = 0.0f;
        this->m_dipoleCmdZ = 0.0f;
        this->tlmWrite_MagDetumble_DipoleCmdX(m_dipoleCmdX);
        this->tlmWrite_MagDetumble_DipoleCmdY(m_dipoleCmdY);
        this->tlmWrite_MagDetumble_DipoleCmdZ(m_dipoleCmdZ);
    }

    this->cmdResponse_out(opCode, cmdSeq, Fw::CmdResponse::OK);
  }

  void MagneticDetumbleImpl::MAG_DETUMBLE_SET_BDOT_GAIN_cmdHandler(
      const FwOpcodeType opCode,
      const U32 cmdSeq,
      F32 gain
  ) {
    if (gain < 0) {
      this->log_WARNING_LOW_MAG_DETUMBLE_INVALID_BDOT_GAIN(gain);
      this->cmdResponse_out(opCode, cmdSeq, Fw::CmdResponse::VALIDATION_ERROR);
      return;
    }
    this->m_bdotGain = gain;
    this->tlmWrite_MagDetumble_BdotGain(this->m_bdotGain);
    this->log_ACTIVITY_HI_MAG_DETUMBLE_BDOT_GAIN_SET(this->m_bdotGain);
    this->cmdResponse_out(opCode, cmdSeq, Fw::CmdResponse::OK);
  }

  // ----------------------------------------------------------------------
  // Helper methods
  // ----------------------------------------------------------------------

  void MagneticDetumbleImpl::commandCoils(F32 targetDipoleX, F32 targetDipoleY, F32 targetDipoleZ) {
    // Simplified: Assume dipole is directly proportional to current and axes are aligned.
    // And that each coil produces dipole only along its axis.
    // M = N * I * A (Magnetic dipole moment = Num_turns * Current * Area_of_coil)
    // So, I = M / (N * A). Let k_i = 1 / (N_i * A_i) be the per-axis coefficient.
    // For simplicity, assume k_x = k_y = k_z = 1.0 for now.
    // This means targetDipoleX is directly the current for X coil, etc.
    // This is a placeholder for actual physical mapping.

    F32 currentX = targetDipoleX;
    F32 currentY = targetDipoleY;
    F32 currentZ = targetDipoleZ;

    // Apply saturation
    F32 actualCurrentX = currentX;
    F32 actualCurrentY = currentY;
    F32 actualCurrentZ = currentZ;
    bool saturated = false;

    if (fabs(actualCurrentX) > MAX_COIL_CURRENT) {
        actualCurrentX = (actualCurrentX > 0) ? MAX_COIL_CURRENT : -MAX_COIL_CURRENT;
        this->log_WARNING_LOW_MAG_DETUMBLE_COIL_SATURATION(0, currentX, actualCurrentX);
        saturated = true;
    }
    if (fabs(actualCurrentY) > MAX_COIL_CURRENT) {
        actualCurrentY = (actualCurrentY > 0) ? MAX_COIL_CURRENT : -MAX_COIL_CURRENT;
        this->log_WARNING_LOW_MAG_DETUMBLE_COIL_SATURATION(1, currentY, actualCurrentY);
        saturated = true;
    }
    if (fabs(actualCurrentZ) > MAX_COIL_CURRENT) {
        actualCurrentZ = (actualCurrentZ > 0) ? MAX_COIL_CURRENT : -MAX_COIL_CURRENT;
        this->log_WARNING_LOW_MAG_DETUMBLE_COIL_SATURATION(2, currentZ, actualCurrentZ);
        saturated = true;
    }

    // Send commands to hardware if connected
    if (this->isConnected_hwControlOut_OutputPort(0)) {
      this->hwControlOut_out(0, 0, actualCurrentX); // Axis 0 for X
      this->log_ACTIVITY_LOW_MAG_DETUMBLE_HW_CMD_SENT(0, actualCurrentX);
      this->tlmWrite_MagDetumble_CoilCurrentX(actualCurrentX);

      this->hwControlOut_out(0, 1, actualCurrentY); // Axis 1 for Y
      this->log_ACTIVITY_LOW_MAG_DETUMBLE_HW_CMD_SENT(1, actualCurrentY);
      this->tlmWrite_MagDetumble_CoilCurrentY(actualCurrentY);

      this->hwControlOut_out(0, 2, actualCurrentZ); // Axis 2 for Z
      this->log_ACTIVITY_LOW_MAG_DETUMBLE_HW_CMD_SENT(2, actualCurrentZ);
      this->tlmWrite_MagDetumble_CoilCurrentZ(actualCurrentZ);
    }
  }

} // end namespace Drv
