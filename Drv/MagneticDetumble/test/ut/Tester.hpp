#ifndef Drv_MagneticDetumble_Tester_HPP
#define Drv_MagneticDetumble_Tester_HPP

#include "Drv/MagneticDetumble/MagneticDetumbleGTestBase.hpp"
#include "Drv/MagneticDetumble/MagneticDetumbleImpl.hpp"

namespace Drv {

  class Tester : public MagneticDetumbleGTestBase {

    public:
      // ----------------------------------------------------------------------
      // Construction and destruction
      // ----------------------------------------------------------------------

      //! Construct object Tester
      Tester();

      //! Destroy object Tester
      ~Tester();

    public:
      // ----------------------------------------------------------------------
      // Tests
      // ----------------------------------------------------------------------

      //! Test direct dipole command
      void test_directDipoleCommand();

      //! Test B-dot enable and gain commands
      void test_bdotConfigurationCommands();

      //! Test B-dot algorithm execution (nominal case)
      void test_bdotAlgorithm_nominal();

      //! Test B-dot with stale magnetic field data
      void test_bdotAlgorithm_staleMagField();

      //! Test B-dot with stale angular velocity data (if it affects logic)
      void test_bdotAlgorithm_staleAngVel();

      //! Test B-dot with zero magnetic field
      void test_bdotAlgorithm_zeroMagField();

      //! Test B-dot with invalid time delta
      void test_bdotAlgorithm_invalidDt();

      //! Test coil saturation logic
      void test_coilSaturation();

      //! Test input port for direct dipole setting
      void test_setDipoleCmdInPort();


    PRIVATE:
      // ----------------------------------------------------------------------
      // Handlers for typed from ports
      // ----------------------------------------------------------------------

      //! Handler for from_hwControlOut
      void from_hwControlOut_handler(
          const NATIVE_INT_TYPE portNum, //!< The port number
          U8 axis, //!< Axis identifier (0=X, 1=Y, 2=Z)
          F32 current //!< Current to apply to the coil
      );

      //! Handler for from_statusOut
      void from_statusOut_handler(
          const NATIVE_INT_TYPE portNum, //!< The port number
          Fw::Success success //!< The command execution status
      );

    PRIVATE:
      // ----------------------------------------------------------------------
      // Helper methods
      // ----------------------------------------------------------------------

      //! Connect ports
      void connectPorts();

      //! Initialize components
      void initComponents();

      //! Set up a default state for the component before each test
      void setupTest();

      //! Helper to advance time for the component
      void advanceTime(U32 seconds, U32 useconds = 0);


    PRIVATE:
      // ----------------------------------------------------------------------
      // Variables
      // ----------------------------------------------------------------------

      //! The component under test
      MagneticDetumbleImpl component;

      // Variables to store outputs from the component
      U8 m_hwControl_axis[3]; // Store axis for up to 3 calls (X,Y,Z)
      F32 m_hwControl_current[3]; // Store current for up to 3 calls
      U32 m_hwControl_invocations;

      Fw::Success m_statusOut_success;
      U32 m_statusOut_invocations;

      // Default time for testing
      Fw::Time m_testTime;

  };

} // end namespace Drv

#endif
