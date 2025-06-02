#ifndef SVC_MAGNETICDETUMBLE_COMPONENTIMPL_HPP
#define SVC_MAGNETICDETUMBLE_COMPONENTIMPL_HPP

#include "Svc/MagneticDetumble/MagneticDetumbleComponentAc.hpp"
#include "Fw/Types/On.hpp"
#include "Fw/Math/Vector3.hpp"
#include "Fw/Time/Time.hpp"

namespace Svc {

  class MagneticDetumbleComponentImpl : public MagneticDetumbleComponentBase {

    public:

      // ----------------------------------------------------------------------
      // Construction, initialization, and destruction
      // ----------------------------------------------------------------------

      //! Construct object MagneticDetumble
      MagneticDetumbleComponentImpl(
          const char *const compName //!< The component name
      );

      //! Initialize object MagneticDetumble
      void init(
          const NATIVE_INT_TYPE instance = 0 //!< The instance number
      );

      //! Destroy object MagneticDetumble
      ~MagneticDetumbleComponentImpl();

    PRIVATE:

      // ----------------------------------------------------------------------
      // Handler implementations for user-defined typed input ports
      // ----------------------------------------------------------------------

      //! Handler implementation for SCHED_IN
      void SCHED_IN_handler(
          const NATIVE_INT_TYPE portNum, //!< The port number
          NATIVE_UINT_TYPE context //!< The call order
      );

      // ----------------------------------------------------------------------
      // Command handler implementations
      // ----------------------------------------------------------------------

      //! Implementation for SET_MODE command handler
      void SET_MODE_cmdHandler(
          FwOpcodeType opCode, //!< The opcode
          U32 cmdSeq, //!< The command sequence number
          Fw::On::t mode //!< The desired mode (AUTO or MANUAL)
      );

      //! Implementation for SET_DIPOLE command handler
      void SET_DIPOLE_cmdHandler(
          FwOpcodeType opCode, //!< The opcode
          U32 cmdSeq, //!< The command sequence number
          const Fw::Math::Vector3 &dipole //!< The desired dipole vector for MANUAL mode
      );

    PRIVATE:

      // ----------------------------------------------------------------------
      // Helper methods
      // ----------------------------------------------------------------------

      // No explicit helper for B_dot calculation in header, will be part of SCHED_IN_handler or local to .cpp
      // Fw::Math::Vector3 calculate_B_dot(
      //     const Fw::Math::Vector3 &current_B,
      //     Fw::Time currentTime
      // );

      // ----------------------------------------------------------------------
      // Member variables
      // ----------------------------------------------------------------------

      //! Current operational mode of the detumble component
      Fw::On::t m_currentMode;

      //! Manually set dipole moment (used in MANUAL mode)
      Fw::Math::Vector3 m_manualDipole;

      //! Previous magnetic field vector reading
      Fw::Math::Vector3 m_previousB;

      //! Time of the previous magnetic field vector reading
      Fw::Time m_previousTime;

      //! Control gain for the B-dot algorithm, loaded from parameter
      F64 m_controlGain;

      //! Flag to indicate if this is the first run (for B_dot calculation)
      bool m_firstRun;

  };

} // end namespace Svc

#endif
