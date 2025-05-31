#ifndef Drv_MagneticDetumble_Impl_HPP
#define Drv_MagneticDetumble_Impl_HPP

#include "Drv/MagneticDetumble/MagneticDetumbleComponentAc.hpp"
#include "Fw/Time/Time.hpp"

namespace Drv {

  class MagneticDetumbleImpl : public MagneticDetumbleComponentBase {

    public:

      // ----------------------------------------------------------------------
      // Construction, initialization, and destruction
      // ----------------------------------------------------------------------

      //! Construct object MagneticDetumble
      MagneticDetumbleImpl(
          const char* const compName //!< The component name
      );

      //! Initialize object MagneticDetumble
      void init(
          const NATIVE_INT_TYPE instance = 0 //!< The instance number
      );

      //! Destroy object MagneticDetumble
      ~MagneticDetumbleImpl();

    PRIVATE:

      // ----------------------------------------------------------------------
      // Handler implementations for user-defined typed input ports
      // ----------------------------------------------------------------------

      //! Handler implementation for setDipoleCmdIn
      void setDipoleCmdIn_handler(
          const NATIVE_INT_TYPE portNum, //!< The port number
          F32 x, //!< X-axis dipole component
          F32 y, //!< Y-axis dipole component
          F32 z //!< Z-axis dipole component
      );

      //! Handler implementation for magFieldIn
      void magFieldIn_handler(
          const NATIVE_INT_TYPE portNum, //!< The port number
          F32 x, //!< X-axis magnetic field component
          F32 y, //!< Y-axis magnetic field component
          F32 z //!< Z-axis magnetic field component
      );

      //! Handler implementation for angularVelocityIn
      void angularVelocityIn_handler(
          const NATIVE_INT_TYPE portNum, //!< The port number
          F32 x, //!< X-axis angular velocity
          F32 y, //!< Y-axis angular velocity
          F32 z //!< Z-axis angular velocity
      );

      //! Handler implementation for runScheduleIn
      void runScheduleIn_handler(
          const NATIVE_INT_TYPE portNum, //!< The port number
          U32 context //!< The call context
      );

    PRIVATE:

      // ----------------------------------------------------------------------
      // Command handler implementations
      // ----------------------------------------------------------------------

      //! Implementation for MAG_DETUMBLE_SET_DIPOLE command handler
      void MAG_DETUMBLE_SET_DIPOLE_cmdHandler(
          const FwOpcodeType opCode, //!< The opcode
          const U32 cmdSeq, //!< The command sequence number
          F32 x, //!< X-axis dipole component
          F32 y, //!< Y-axis dipole component
          F32 z //!< Z-axis dipole component
      );

      //! Implementation for MAG_DETUMBLE_ENABLE_BDOT command handler
      void MAG_DETUMBLE_ENABLE_BDOT_cmdHandler(
          const FwOpcodeType opCode, //!< The opcode
          const U32 cmdSeq, //!< The command sequence number
          bool enable //!< True to enable B-dot, false to disable
      );

      //! Implementation for MAG_DETUMBLE_SET_BDOT_GAIN command handler
      void MAG_DETUMBLE_SET_BDOT_GAIN_cmdHandler(
          const FwOpcodeType opCode, //!< The opcode
          const U32 cmdSeq, //!< The command sequence number
          F32 gain //!< B-dot controller gain
      );

    PRIVATE:

      // ----------------------------------------------------------------------
      // Helper methods
      // ----------------------------------------------------------------------

      //! Calculate and command coil currents based on the target dipole
      void commandCoils(F32 targetDipoleX, F32 targetDipoleY, F32 targetDipoleZ);

      // ----------------------------------------------------------------------
      // Member variables
      // ----------------------------------------------------------------------

      // Commanded dipole (either directly or from B-dot)
      F32 m_dipoleCmdX;
      F32 m_dipoleCmdY;
      F32 m_dipoleCmdZ;

      // Magnetic field
      F32 m_magFieldX;
      F32 m_magFieldY;
      F32 m_magFieldZ;
      Fw::Time m_magFieldTimeTag;
      bool m_magFieldReceived;


      // Angular velocity
      F32 m_angularVelocityX;
      F32 m_angularVelocityY;
      F32 m_angularVelocityZ;
      Fw::Time m_angularVelocityTimeTag;
      bool m_angularVelocityReceived;

      // B-dot algorithm state
      bool m_bdotEnabled;
      F32 m_bdotGain;
      F32 m_prevMagFieldX;
      F32 m_prevMagFieldY;
      F32 m_prevMagFieldZ;
      Fw::Time m_prevMagFieldTimeTag;
      bool m_prevMagFieldReceived;

      // Maximum coil current (example, could be configurable)
      static constexpr F32 MAX_COIL_CURRENT = 1.0f; // Amperes
      // Time threshold to consider sensor data stale (e.g., 2 seconds)
      static constexpr U32 SENSOR_DATA_STALE_THRESHOLD_SECONDS = 2;

  };

} // end namespace Drv

#endif
