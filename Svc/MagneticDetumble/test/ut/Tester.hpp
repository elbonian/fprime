#ifndef SVC_MAGNETICDETUMBLE_TESTER_HPP
#define SVC_MAGNETICDETUMBLE_TESTER_HPP

#include "Svc/MagneticDetumble/MagneticDetumbleGTestBase.hpp"
#include "Svc/MagneticDetumble/MagneticDetumbleComponentImpl.hpp"

namespace Svc {

  class Tester :
    public MagneticDetumbleGTestBase
  {

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
      //! Test initialization
      void testInitialization();
      //! Test nominal dipole calculation
      void testNominalCalculation();
      //! Test zero magnetic field
      void testZeroMagField();
      //! Test saturation
      void testSaturation();
      //! Test gain calculation (indirectly via initialization or a dedicated method if params were settable)
      void testGainCalculation();


    private:
      // ----------------------------------------------------------------------
      // Handlers for typed from ports
      // ----------------------------------------------------------------------

      //! Handler for from_dipoleRequestOut
      void from_dipoleRequestOut_handler(
          const NATIVE_INT_TYPE portNum, /*!< The port number*/
          Svc::Vector3 &val /*!< The magnetic dipole vector*/
      ) override;

    private:
      // ----------------------------------------------------------------------
      // Helper methods
      // ----------------------------------------------------------------------

      //! Connect ports
      void connectPorts();

      //! Initialize components
      void initComponents();

    private:
      // ----------------------------------------------------------------------
      // Variables
      // ----------------------------------------------------------------------

      //! The component under test
      MagneticDetumbleComponentImpl component;

      //! Last commanded dipole
      Svc::Vector3 m_lastDipoleCmd;
      //! Count of dipole commands received
      U32 m_dipoleCmdCount;

  };

} // end namespace Svc

#endif
