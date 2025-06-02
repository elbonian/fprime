#ifndef SVC_MAGNETICDETUMBLE_TESTER_HPP
#define SVC_MAGNETICDETUMBLE_TESTER_HPP

#include "Svc/MagneticDetumble/MagneticDetumbleComponentImpl.hpp"
#include "Svc/MagneticDetumble/MagneticDetumbleGTestBase.hpp"
#include "Fw/Com/ComBuffer.hpp"
#include "Fw/Types/On.hpp"
#include "Fw/Math/Vector3.hpp"
#include "Fw/Time/Time.hpp"

namespace Svc {

  class MagneticDetumbleTester : public MagneticDetumbleGTestBase {
    public:
      // ----------------------------------------------------------------------
      // Construction and destruction
      // ----------------------------------------------------------------------

      //! Construct object MagneticDetumbleTester
      MagneticDetumbleTester();

      //! Destroy object MagneticDetumbleTester
      ~MagneticDetumbleTester();

    public:
      // ----------------------------------------------------------------------
      // Tests
      // ----------------------------------------------------------------------
      //! Test parameter loading and initial telemetry
      void test_initialization();
      //! Test SET_MODE command to AUTO
      void test_set_mode_auto();
      //! Test SET_MODE command to MANUAL
      void test_set_mode_manual();
      //! Test SET_DIPOLE command in MANUAL mode
      void test_set_dipole_manual_ok();
      //! Test SET_DIPOLE command in AUTO mode (expect error)
      void test_set_dipole_auto_error();
      //! Test SCHED_IN handler in MANUAL mode
      void test_sched_in_manual_mode();
      //! Test SCHED_IN handler in AUTO mode (nominal B-dot calculation)
      void test_sched_in_auto_mode_nominal();
      //! Test SCHED_IN handler in AUTO mode when B-field magnitude is zero/too small
      void test_sched_in_auto_mode_b_zero();
      //! Test SCHED_IN handler in AUTO mode when dt is zero or negative
      void test_sched_in_auto_mode_dt_error();
      //! Test behavior when angularVelocityIn port is not connected
      void test_port_not_connected_angular_velocity();
      //! Test behavior when magFieldIn port is not connected
      void test_port_not_connected_mag_field();
       //! Test behavior when magnetorquerCmdOut port is not connected
      void test_port_not_connected_magnetorquer_cmd();


    private:
      // ----------------------------------------------------------------------
      // Handlers for typed from ports
      // ----------------------------------------------------------------------

      //! Handler for from_magnetorquerCmdOut
      void from_magnetorquerCmdOut_handler(
          const NATIVE_INT_TYPE portNum, //!< The port number
          const Fw::Math::Vector3& dipoleCmd //!< The dipole command vector
      );

      //! Handler for from_timeCaller
      //  Provides time to the component under test
      void from_timeCaller_handler(
          const NATIVE_INT_TYPE portNum, //!< The port number
          Fw::Time& time //!< The U32 cmd argument
      );


    private:
      // ----------------------------------------------------------------------
      // Helper methods
      // ----------------------------------------------------------------------

      //! Connect ports
      void connectPorts();

      //! Initialize component parameters for testing
      voidinitComponents();


      // ----------------------------------------------------------------------
      // Variables
      // ----------------------------------------------------------------------

      //! The component under test
      Svc::MagneticDetumbleComponentImpl component;

      //! Last received dipole command from magnetorquerCmdOut
      Fw::Math::Vector3 m_lastDipoleCmd;
      //! Count of magnetorquerCmdOut invocations
      U32 m_magnetorquerCmdCount;

      //! Current time to be returned by timeCaller
      Fw::Time m_currentTime;
  };

} // end namespace Svc

#endif
