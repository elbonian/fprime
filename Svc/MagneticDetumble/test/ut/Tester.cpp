#include "Tester.hpp"
#include <Fw/Test/UnitTest.hpp>
#include <cmath> // For fabs

// Value for nearly_equal comparisons
const F32 TEST_EPSILON = 1e-5f;

namespace Svc {

  // ----------------------------------------------------------------------
  // Construction and destruction
  // ----------------------------------------------------------------------

  Tester ::
    Tester() :
      MagneticDetumbleGTestBase("Tester", 10), // Max history depth
      component("MagneticDetumble"),
      m_lastDipoleCmd{0.0f, 0.0f, 0.0f},
      m_dipoleCmdCount(0)
  {
    this->initComponents();
    this->connectPorts();
  }

  Tester ::
    ~Tester()
  {

  }

  // ----------------------------------------------------------------------
  // Tests
  // ----------------------------------------------------------------------

  void Tester ::
    testInitialization()
  {
    // Call init on the component
    this->component.init(0);

    // Check for initialization event
    ASSERT_EVENTS_SIZE(1);
    ASSERT_EVENTS_DetumbleInitialized_SIZE(1);
    // Check gain telemetry (assuming default params from component Impl)
    // k! = 2 * orbit_rate * (1 + sin(beta_m)) * J_min
    // k! = 2 * 0.0011 * (1 + sin(PI/4)) * 0.33
    // k! = 2 * 0.0011 * (1 + 0.70710678) * 0.33
    // k! = 0.0022 * 1.70710678 * 0.33 = 0.001239...
    F32 expected_gain = 2.0f * 0.0011f * (1.0f + std::sin(static_cast<F32>(M_PI / 4.0))) * 0.33f;
    ASSERT_EVENTS_DetumbleInitialized(0, expected_gain);
    ASSERT_TLM_SIZE(1);
    ASSERT_TLM_controlGain_SIZE(1);
    ASSERT_TLM_controlGain(0, expected_gain, TEST_EPSILON);
  }

  void Tester ::
    testGainCalculation() // This is essentially covered by testInitialization for now
  {
    this->component.init(0); // Re-init to ensure gain is calculated
    F32 expected_gain = 2.0f * component.m_orbitRate * (1.0f + std::sin(component.m_geomagneticInclination)) * component.m_minInertia;
    ASSERT_TLM_controlGain_SIZE(1);
    ASSERT_TLM_controlGain(0, expected_gain, TEST_EPSILON);
    ASSERT_EVENTS_DetumbleInitialized_SIZE(1);
    ASSERT_EVENTS_DetumbleInitialized(0, expected_gain);
  }


  void Tester ::
    testNominalCalculation()
  {
    this->component.init(0);
    this->clearHistory();

    Svc::Vector3 angVel = {0.1f, -0.05f, 0.02f};
    Svc::Vector3 magField = {20000e-9f, 10000e-9f, -50000e-9f}; // In Teslas

    // Manually calculate expected dipole
    // m = -k!/|b|^2 * (b x w)
    F32 k_gain = component.m_controlGainK;
    F32 b_mag_sq = (magField.getx()*magField.getx() + magField.gety()*magField.gety() + magField.getz()*magField.getz());

    Svc::Vector3 b_cross_w;
    b_cross_w.setx(magField.gety() * angVel.getz() - magField.getz() * angVel.gety());
    b_cross_w.sety(magField.getz() * angVel.getx() - magField.getx() * angVel.getz());
    b_cross_w.setz(magField.getx() * angVel.gety() - magField.gety() * angVel.getx());

    Svc::Vector3 expected_dipole;
    expected_dipole.setx(-k_gain / b_mag_sq * b_cross_w.getx());
    expected_dipole.sety(-k_gain / b_mag_sq * b_cross_w.gety());
    expected_dipole.setz(-k_gain / b_mag_sq * b_cross_w.getz());

    // Invoke port handlers
    this->invoke_to_angularVelocityIn(0, angVel);
    ASSERT_TLM_angularVelocity_SIZE(1);
    ASSERT_TLM_angularVelocity(0, angVel);

    this->invoke_to_magneticFieldIn(0, magField);
    ASSERT_TLM_magneticField_SIZE(1);
    ASSERT_TLM_magneticField(0, magField);

    this->invoke_to_schedIn(0, 0); // Invoke scheduler

    ASSERT_FROM_PORT_HISTORY_SIZE(1);
    ASSERT_EQ(this->m_dipoleCmdCount, 1);
    ASSERT_NEAR(this->m_lastDipoleCmd.getx(), expected_dipole.getx(), TEST_EPSILON);
    ASSERT_NEAR(this->m_lastDipoleCmd.gety(), expected_dipole.gety(), TEST_EPSILON);
    ASSERT_NEAR(this->m_lastDipoleCmd.getz(), expected_dipole.getz(), TEST_EPSILON);

    ASSERT_TLM_commandedDipole_SIZE(1);
    ASSERT_TLM_commandedDipole(0, expected_dipole, TEST_EPSILON); // Check telemetry also
    ASSERT_EVENTS_SIZE(0); // No warning events expected
  }

  void Tester ::
    testZeroMagField()
  {
    this->component.init(0);
    this->clearHistory();

    Svc::Vector3 angVel = {0.1f, 0.1f, 0.1f};
    Svc::Vector3 magField = {0.0f, 0.0f, 0.0f};

    this->invoke_to_angularVelocityIn(0, angVel);
    this->invoke_to_magneticFieldIn(0, magField);
    this->invoke_to_schedIn(0, 0);

    ASSERT_FROM_PORT_HISTORY_SIZE(1);
    ASSERT_EQ(this->m_dipoleCmdCount, 1);
    ASSERT_NEAR(this->m_lastDipoleCmd.getx(), 0.0f, TEST_EPSILON);
    ASSERT_NEAR(this->m_lastDipoleCmd.gety(), 0.0f, TEST_EPSILON);
    ASSERT_NEAR(this->m_lastDipoleCmd.getz(), 0.0f, TEST_EPSILON);

    ASSERT_EVENTS_SIZE(1);
    ASSERT_EVENTS_MagFieldNearZero_SIZE(1);
    ASSERT_EVENTS_MagFieldNearZero(0, 0.0f, TEST_EPSILON);

    ASSERT_TLM_commandedDipole_SIZE(1);
    ASSERT_TLM_commandedDipole(0, Svc::Vector3(0.0f, 0.0f, 0.0f), TEST_EPSILON);
  }

  void Tester ::
    testSaturation()
  {
    this->component.init(0);
    // Temporarily set a very small gain to force saturation easily
    // or craft inputs carefully. Let's try crafting inputs.
    // We know m_dipoleSaturationLimit is 2.0f
    // m = -k!/|b|^2 * (b x w). We need |m| > 2.0
    // k! is approx 0.001239.
    // Let |b|^2 be small, e.g., (1e-5)^2 = 1e-10
    // Let |b x w| be large. e.g. if b = (1e-5,0,0), w = (0,1,0), then b x w = (0,0,1e-5)
    // |m| = 0.001239 / 1e-10 * 1e-5 = 0.001239 * 1e5 = 123.9. This will saturate.
    this->clearHistory();

    Svc::Vector3 angVel = {0.0f, 10.0f, 0.0f}; // Large omega component
    Svc::Vector3 magField = {1e-5f, 0.0f, 0.0f}; // Small B field, orthogonal to omega

    this->invoke_to_angularVelocityIn(0, angVel);
    this->invoke_to_magneticFieldIn(0, magField);
    this->invoke_to_schedIn(0, 0);

    ASSERT_FROM_PORT_HISTORY_SIZE(1);
    ASSERT_EQ(this->m_dipoleCmdCount, 1);

    // Check that the magnitude of m_lastDipoleCmd is near m_dipoleSaturationLimit (2.0)
    F32 dipole_mag = std::sqrt(
        m_lastDipoleCmd.getx() * m_lastDipoleCmd.getx() +
        m_lastDipoleCmd.gety() * m_lastDipoleCmd.gety() +
        m_lastDipoleCmd.getz() * m_lastDipoleCmd.getz()
    );
    ASSERT_NEAR(dipole_mag, component.m_dipoleSaturationLimit, TEST_EPSILON);

    ASSERT_EVENTS_SIZE(1);
    ASSERT_EVENTS_DipoleSaturated_SIZE(1);
    // Check that the event reports the original (unsaturated) values if possible, or at least the limit
    // The current event reports commanded_x,y,z (original) and limit.
    // We'd need to calculate the original expected value here to verify event args fully.
    // For now, just check size.
    ASSERT_TLM_commandedDipole_SIZE(1); // Telemetry should be the clamped value
    Svc::Vector3 tlm_dipole = this->tlmHistory_commandedDipole->at(0).arg;
    F32 tlm_dipole_mag = std::sqrt(tlm_dipole.getx()*tlm_dipole.getx() + tlm_dipole.gety()*tlm_dipole.gety() + tlm_dipole.getz()*tlm_dipole.getz());
    ASSERT_NEAR(tlm_dipole_mag, component.m_dipoleSaturationLimit, TEST_EPSILON);

  }


  // ----------------------------------------------------------------------
  // Handlers for typed from ports
  // ----------------------------------------------------------------------

  void Tester ::
    from_dipoleRequestOut_handler(
        const NATIVE_INT_TYPE portNum,
        Svc::Vector3 &val
    )
  {
    this->pushFromPortEntry_dipoleRequestOut(val);
    this->m_lastDipoleCmd = val;
    this->m_dipoleCmdCount++;
  }

  // ----------------------------------------------------------------------
  // Helper methods
  // ----------------------------------------------------------------------

  void Tester ::
    connectPorts()
  {
    // Connect ports
    this->connect_to_angularVelocityIn(0, this->component.get_angularVelocityIn_InputPort(0));
    this->connect_to_magneticFieldIn(0, this->component.get_magneticFieldIn_InputPort(0));
    this->connect_to_schedIn(0, this->component.get_schedIn_InputPort(0));

    this->component.set_dipoleRequestOut_OutputPort(0, this->get_from_dipoleRequestOut(0));
    this->component.set_Tlm_OutputPort(0, this->get_from_Tlm(0));
    this->component.set_Log_OutputPort(0, this->get_from_Log(0));
    this->component.set_LogText_OutputPort(0,this->get_from_LogText(0)); // If text logging is used

  }

  void Tester ::
    initComponents()
  {
    this->init(); // GTestBase init
    this->component.init(0); // Component under test init
     // Explicitly clear history after component init's event/tlm
    this->clearHistory();

  }

} // end namespace Svc
