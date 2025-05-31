#include "Tester.hpp"
#include "Fw/Test/UnitTest.hpp"
#include <Fw/Types/Assert.hpp>
#include <Os/Log.hpp> // For Os::Log used in some test setups
#include <iostream> // For debugging test failures if needed

// Set a default time for the Fw::Time object
#define TEST_TIME_SECONDS 600
#define TEST_TIME_USECONDS 0


namespace Drv {

  // ----------------------------------------------------------------------
  // Construction and destruction
  // ----------------------------------------------------------------------

  Tester::Tester() :
    MagneticDetumbleGTestBase("Tester", 100), // Name, max history
    component("MagneticDetumbleImpl")
  {
    this->initComponents();
    this->connectPorts();
    this->m_testTime.set(TEST_TIME_SECONDS, TEST_TIME_USECONDS); // Initialize test time
  }

  Tester::~Tester() {}

  // ----------------------------------------------------------------------
  // Helper methods
  // ----------------------------------------------------------------------

  void Tester::connectPorts() {
    // Connect input ports
    this->connect_to_setDipoleCmdIn(0, this->component.get_setDipoleCmdIn_InputPort(0));
    this->connect_to_magFieldIn(0, this->component.get_magFieldIn_InputPort(0));
    this->connect_to_angularVelocityIn(0, this->component.get_angularVelocityIn_InputPort(0));
    this->connect_to_runScheduleIn(0, this->component.get_runScheduleIn_InputPort(0));

    // Connect command ports
    this->connect_to_CmdDisp(0, this->component.get_CmdDisp_InputPort(0));

    // Connect output ports that the tester handles
    this->component.set_hwControlOut_OutputPort(0, this->get_from_hwControlOut(0));
    this->component.set_statusOut_OutputPort(0, this->get_from_statusOut(0));

    // Connect standard output ports (telemetry, events, time, cmd response) to GTestBase
    this->component.set_CmdStatus_OutputPort(0, this->get_from_CmdStatus(0));
    this->component.set_Tlm_OutputPort(0, this->get_from_Tlm(0));
    this->component.set_Log_OutputPort(0, this->get_from_Log(0));
    this->component.set_LogText_OutputPort(0, this->get_from_LogText(0));
    this->component.set_Time_OutputPort(0, this->get_from_Time(0));

    // Connect Ping ports if needed (usually handled by GTestBase or not explicitly connected for simple tests)
    this->connect_to_PingIn(0, component.get_PingIn_InputPort(0));
    this->component.set_PingOut_OutputPort(0, this->get_from_PingOut(0));
  }

  void Tester::initComponents() {
    this->init(); // Initialize GTestBase
    this->component.init(0); // Initialize component instance 0
  }

  void Tester::setupTest() {
    // Clear history
    this->clearHistory();

    // Reset component state if necessary (or rely on re-initialization for some tests)
    // For now, we re-initialize component for some tests, or manually reset members.
    // Resetting internal state of the component:
    component.m_dipoleCmdX = 0.0f;
    component.m_dipoleCmdY = 0.0f;
    component.m_dipoleCmdZ = 0.0f;
    component.m_magFieldX = 0.0f;
    component.m_magFieldY = 0.0f;
    component.m_magFieldZ = 0.0f;
    component.m_magFieldTimeTag.set(0,0);
    component.m_magFieldReceived = false;
    component.m_angularVelocityX = 0.0f;
    component.m_angularVelocityY = 0.0f;
    component.m_angularVelocityZ = 0.0f;
    component.m_angularVelocityTimeTag.set(0,0);
    component.m_angularVelocityReceived = false;
    component.m_bdotEnabled = false;
    component.m_bdotGain = 1.0f;
    component.m_prevMagFieldX = 0.0f;
    component.m_prevMagFieldY = 0.0f;
    component.m_prevMagFieldZ = 0.0f;
    component.m_prevMagFieldTimeTag.set(0,0);
    component.m_prevMagFieldReceived = false;

    // Reset tester's port history counters and stored values
    this->m_hwControl_invocations = 0;
    this->m_statusOut_invocations = 0;
    for (int i=0; i<3; ++i) {
        this->m_hwControl_axis[i] = 0;
        this->m_hwControl_current[i] = 0.0f;
    }
    this->m_statusOut_success = Fw::Success::FAILURE; // Default to failure

    // Set default time for the component
    this->m_testTime.set(TEST_TIME_SECONDS, TEST_TIME_USECONDS);
    this->setTestTime(this->m_testTime);
  }

  void Tester::advanceTime(U32 seconds, U32 useconds) {
    this->m_testTime.add(seconds, useconds);
    this->setTestTime(this->m_testTime);
  }

  // ----------------------------------------------------------------------
  // Handlers for typed from ports
  // ----------------------------------------------------------------------

  void Tester::from_hwControlOut_handler(
      const NATIVE_INT_TYPE portNum,
      U8 axis,
      F32 current
  ) {
    this->pushFromPortEntry_hwControlOut(axis, current);
    if (this->m_hwControl_invocations < 3) { // Store up to 3 axis commands
        this->m_hwControl_axis[this->m_hwControl_invocations] = axis;
        this->m_hwControl_current[this->m_hwControl_invocations] = current;
    }
    this->m_hwControl_invocations++;
  }

  void Tester::from_statusOut_handler(
      const NATIVE_INT_TYPE portNum,
      Fw::Success success
  ) {
    this->pushFromPortEntry_statusOut(success);
    this->m_statusOut_success = success;
    this->m_statusOut_invocations++;
  }

  // ----------------------------------------------------------------------
  // Tests
  // ----------------------------------------------------------------------

  void Tester::test_directDipoleCommand() {
    this->setupTest();
    const F32 x = 0.1f, y = -0.2f, z = 0.3f;

    // Send command
    this->sendCmd_MAG_DETUMBLE_SET_DIPOLE(0, 10, x, y, z);
    this->component.doDispatch(); // Dispatch message queue

    // Verify command response
    ASSERT_CMD_RESPONSE_SIZE(1);
    ASSERT_CMD_RESPONSE(0, MagneticDetumbleImpl::OPCODE_MAG_DETUMBLE_SET_DIPOLE, 10, Fw::CmdResponse::OK);

    // Verify event
    ASSERT_EVENTS_SIZE(1);
    ASSERT_EVENTS_MAG_DETUMBLE_DIPOLE_CMD_RECEIVED_SIZE(1);
    ASSERT_EVENTS_MAG_DETUMBLE_DIPOLE_CMD_RECEIVED(0, x, y, z);

    // Verify telemetry
    ASSERT_TLM_SIZE(6); // 3 for dipole, 3 for coil currents
    ASSERT_TLM_MagDetumble_DipoleCmdX(0, x);
    ASSERT_TLM_MagDetumble_DipoleCmdY(0, y);
    ASSERT_TLM_MagDetumble_DipoleCmdZ(0, z);
    ASSERT_TLM_MagDetumble_CoilCurrentX_SIZE(1);
    ASSERT_TLM_MagDetumble_CoilCurrentY_SIZE(1);
    ASSERT_TLM_MagDetumble_CoilCurrentZ_SIZE(1);
    // Check actual current values via from_hwControlOut history

    // Verify hwControlOut (called 3 times, one for each axis)
    ASSERT_FROM_PORT_HISTORY_SIZE(3); // Expect 3 calls to hwControlOut
    ASSERT_EQ(this->m_hwControl_invocations, 3);

    // Check X-axis (assuming axis 0)
    ASSERT_EQ(this->m_hwControl_axis[0], 0);
    ASSERT_FLOAT_EQ(this->m_hwControl_current[0], x); // Simplified: current = dipole
    // Check Y-axis (assuming axis 1)
    ASSERT_EQ(this->m_hwControl_axis[1], 1);
    ASSERT_FLOAT_EQ(this->m_hwControl_current[1], y);
    // Check Z-axis (assuming axis 2)
    ASSERT_EQ(this->m_hwControl_axis[2], 2);
    ASSERT_FLOAT_EQ(this->m_hwControl_current[2], z);
  }

  void Tester::test_setDipoleCmdInPort() {
    this->setupTest();
    const F32 x = 0.5f, y = 0.6f, z = -0.7f;

    // Invoke the input port
    this->invoke_to_setDipoleCmdIn(0, x, y, z);
    this->component.doDispatch(); // Dispatch message queue if port is async

    // Verify event
    ASSERT_EVENTS_SIZE(1);
    ASSERT_EVENTS_MAG_DETUMBLE_DIPOLE_CMD_RECEIVED_SIZE(1);
    ASSERT_EVENTS_MAG_DETUMBLE_DIPOLE_CMD_RECEIVED(0, x, y, z);

    // Verify telemetry
    ASSERT_TLM_SIZE(6); // 3 for dipole, 3 for coil currents
    ASSERT_TLM_MagDetumble_DipoleCmdX(0, x);
    ASSERT_TLM_MagDetumble_DipoleCmdY(0, y);
    ASSERT_TLM_MagDetumble_DipoleCmdZ(0, z);
    // Check actual current values via from_hwControlOut history

    // Verify hwControlOut
    ASSERT_FROM_PORT_HISTORY_SIZE(3);
    ASSERT_EQ(this->m_hwControl_invocations, 3);
    ASSERT_EQ(this->m_hwControl_axis[0], 0);
    ASSERT_FLOAT_EQ(this->m_hwControl_current[0], x);
    ASSERT_EQ(this->m_hwControl_axis[1], 1);
    ASSERT_FLOAT_EQ(this->m_hwControl_current[1], y);
    ASSERT_EQ(this->m_hwControl_axis[2], 2);
    ASSERT_FLOAT_EQ(this->m_hwControl_current[2], z);

    // Verify statusOut
    ASSERT_EQ(this->m_statusOut_invocations, 1);
    ASSERT_EQ(this->m_statusOut_success, Fw::Success::SUCCESS);
  }


  void Tester::test_bdotConfigurationCommands() {
    this->setupTest();

    // Test enabling B-dot
    this->sendCmd_MAG_DETUMBLE_ENABLE_BDOT(0, 11, true);
    this->component.doDispatch();
    ASSERT_CMD_RESPONSE_SIZE(1);
    ASSERT_CMD_RESPONSE(0, MagneticDetumbleImpl::OPCODE_MAG_DETUMBLE_ENABLE_BDOT, 11, Fw::CmdResponse::OK);
    ASSERT_EVENTS_MAG_DETUMBLE_BDOT_STATUS_CHANGED_SIZE(1);
    ASSERT_EVENTS_MAG_DETUMBLE_BDOT_STATUS_CHANGED(0, true);
    ASSERT_TLM_MagDetumble_BdotEnabled_SIZE(1);
    ASSERT_TLM_MagDetumble_BdotEnabled(0, true);
    ASSERT_TRUE(this->component.m_bdotEnabled);
    // Enabling B-dot should reset sensor received flags
    ASSERT_FALSE(this->component.m_magFieldReceived);
    ASSERT_FALSE(this->component.m_angularVelocityReceived);
    ASSERT_FALSE(this->component.m_prevMagFieldReceived);


    // Test setting B-dot gain
    const F32 gain = 2.5f;
    this->sendCmd_MAG_DETUMBLE_SET_BDOT_GAIN(0, 12, gain);
    this->component.doDispatch();
    ASSERT_CMD_RESPONSE_SIZE(2); // Previous + current
    ASSERT_CMD_RESPONSE(1, MagneticDetumbleImpl::OPCODE_MAG_DETUMBLE_SET_BDOT_GAIN, 12, Fw::CmdResponse::OK);
    ASSERT_EVENTS_MAG_DETUMBLE_BDOT_GAIN_SET_SIZE(1);
    ASSERT_EVENTS_MAG_DETUMBLE_BDOT_GAIN_SET(0, gain);
    ASSERT_TLM_MagDetumble_BdotGain_SIZE(1);
    ASSERT_TLM_MagDetumble_BdotGain(0, gain);
    ASSERT_FLOAT_EQ(this->component.m_bdotGain, gain);

    // Test setting invalid B-dot gain (negative)
    this->sendCmd_MAG_DETUMBLE_SET_BDOT_GAIN(0, 13, -1.0f);
    this->component.doDispatch();
    ASSERT_CMD_RESPONSE_SIZE(3);
    ASSERT_CMD_RESPONSE(2, MagneticDetumbleImpl::OPCODE_MAG_DETUMBLE_SET_BDOT_GAIN, 13, Fw::CmdResponse::VALIDATION_ERROR);
    ASSERT_EVENTS_MAG_DETUMBLE_INVALID_BDOT_GAIN_SIZE(1);
    ASSERT_EVENTS_MAG_DETUMBLE_INVALID_BDOT_GAIN(0, -1.0f);
    // Gain should remain unchanged
    ASSERT_TLM_MagDetumble_BdotGain_SIZE(1); // Still 1 from previous, no new tlm for invalid
    ASSERT_FLOAT_EQ(this->component.m_bdotGain, gain);


    // Test disabling B-dot
    this->sendCmd_MAG_DETUMBLE_ENABLE_BDOT(0, 14, false);
    this->component.doDispatch();
    ASSERT_CMD_RESPONSE_SIZE(4);
    ASSERT_CMD_RESPONSE(3, MagneticDetumbleImpl::OPCODE_MAG_DETUMBLE_ENABLE_BDOT, 14, Fw::CmdResponse::OK);
    ASSERT_EVENTS_MAG_DETUMBLE_BDOT_STATUS_CHANGED_SIZE(2);
    ASSERT_EVENTS_MAG_DETUMBLE_BDOT_STATUS_CHANGED(1, false);
    ASSERT_TLM_MagDetumble_BdotEnabled_SIZE(2);
    ASSERT_TLM_MagDetumble_BdotEnabled(1, false);
    ASSERT_FALSE(this->component.m_bdotEnabled);
    // Disabling B-dot should command zero current and zero dipole telemetry
    ASSERT_FROM_PORT_HISTORY_SIZE(3); // 3 calls from enable, 3 from disable = 6 total
    ASSERT_EQ(this->m_hwControl_invocations, 3*2); // X,Y,Z for disabling
    ASSERT_FLOAT_EQ(this->m_hwControl_current[0], 0.0f); // from last set of calls
    ASSERT_FLOAT_EQ(this->m_hwControl_current[1], 0.0f);
    ASSERT_FLOAT_EQ(this->m_hwControl_current[2], 0.0f);
    ASSERT_TLM_MagDetumble_DipoleCmdX_SIZE(1); // Should be 1 after disable
    ASSERT_TLM_MagDetumble_DipoleCmdX(0, 0.0f);
  }

  void Tester::test_bdotAlgorithm_nominal() {
    this->setupTest();

    // Enable B-dot and set gain
    const F32 gain = 2.0f;
    this->sendCmd_MAG_DETUMBLE_ENABLE_BDOT(0, 1, true);
    this->component.doDispatch();
    this->sendCmd_MAG_DETUMBLE_SET_BDOT_GAIN(0, 2, gain);
    this->component.doDispatch();
    this->clearHistory(); // Clear command/event history from setup

    // --- First runScheduleIn: Store prevMagField ---
    this->invoke_to_magFieldIn(0, 1.0f, 2.0f, 3.0f); // B1
    this->component.doDispatch(); // Process magFieldIn
    this->clearTlm(); // Clear telemetry from magFieldIn

    this->invoke_to_runScheduleIn(0, 0); // context = 0
    this->component.doDispatch();

    ASSERT_TRUE(this->component.m_prevMagFieldReceived);
    ASSERT_FLOAT_EQ(this->component.m_prevMagFieldX, 1.0f);
    ASSERT_EVENTS_SIZE(0); // No B-dot update event yet
    ASSERT_FROM_PORT_HISTORY_SIZE(0); // No hwControlOut yet

    // --- Second runScheduleIn: Calculate B-dot ---
    this->advanceTime(1, 0); // dt = 1 second
    this->invoke_to_magFieldIn(0, 1.5f, 2.5f, 3.5f); // B2
    this->component.doDispatch(); // Process magFieldIn
    this->clearTlm();

    this->invoke_to_runScheduleIn(0, 0);
    this->component.doDispatch();

    // B_dot = (B2 - B1)/dt = (0.5, 0.5, 0.5) / 1.0 = (0.5, 0.5, 0.5)
    // Dipole_cmd = -gain * B_dot = -2.0 * (0.5, 0.5, 0.5) = (-1.0, -1.0, -1.0)
    F32 expectedDipole = -1.0f;

    ASSERT_EVENTS_MAG_DETUMBLE_BDOT_UPDATE_SIZE(1);
    ASSERT_EVENTS_MAG_DETUMBLE_BDOT_UPDATE(0, expectedDipole, expectedDipole, expectedDipole);

    ASSERT_TLM_MagDetumble_DipoleCmdX(0, expectedDipole);
    ASSERT_TLM_MagDetumble_DipoleCmdY(0, expectedDipole);
    ASSERT_TLM_MagDetumble_DipoleCmdZ(0, expectedDipole);

    ASSERT_FROM_PORT_HISTORY_SIZE(3); // X, Y, Z coil commands
    ASSERT_EQ(this->m_hwControl_invocations, 3);
    ASSERT_FLOAT_EQ(this->m_hwControl_current[0], expectedDipole); // Assuming 1:1 dipole to current
    ASSERT_FLOAT_EQ(this->m_hwControl_current[1], expectedDipole);
    ASSERT_FLOAT_EQ(this->m_hwControl_current[2], expectedDipole);

    // Verify previous mag field updated for next cycle
    ASSERT_FLOAT_EQ(this->component.m_prevMagFieldX, 1.5f);
  }

  void Tester::test_bdotAlgorithm_staleMagField() {
    this->setupTest();
    this->sendCmd_MAG_DETUMBLE_ENABLE_BDOT(0, 1, true);
    this->component.doDispatch();
    this->clearHistory();

    // Send initial mag field
    this->invoke_to_magFieldIn(0, 1.0f, 1.0f, 1.0f);
    this->component.doDispatch();
    this->invoke_to_runScheduleIn(0, 0); // Store as previous
    this->component.doDispatch();
    this->clearHistory();

    // Advance time beyond stale threshold
    this->advanceTime(MagneticDetumbleImpl::SENSOR_DATA_STALE_THRESHOLD_SECONDS + 1, 0);

    this->invoke_to_runScheduleIn(0, 0);
    this->component.doDispatch();

    ASSERT_EVENTS_MAG_DETUMBLE_INVALID_MAG_FIELD_SIZE(1);
    ASSERT_FROM_PORT_HISTORY_SIZE(0); // No coil commands
  }

  void Tester::test_bdotAlgorithm_staleAngVel() {
    // This test is relevant if ang vel is critical for B-dot.
    // In the current implementation, it's only telemetered and checked for presence,
    // but not strictly used in the B_dot = (B_curr - B_prev)/dt calculation.
    // So, this test will show the MAG_DETUMBLE_INVALID_ANG_VEL event.
    this->setupTest();
    this->sendCmd_MAG_DETUMBLE_ENABLE_BDOT(0, 1, true);
    this->component.doDispatch();
    this->clearHistory();

    // Send initial mag field (enough for B-dot to proceed past mag field checks)
    this->invoke_to_magFieldIn(0, 1.0f, 1.0f, 1.0f);
    this->component.doDispatch();
    this->invoke_to_runScheduleIn(0, 0); // Store B1
    this->component.doDispatch();

    this->advanceTime(1,0); // dt=1s
    this->invoke_to_magFieldIn(0, 2.0f, 2.0f, 2.0f); // B2
    this->component.doDispatch();
    // DO NOT send angular velocity

    this->clearHistory(); // Clear events from magFieldIn etc.
    this->invoke_to_runScheduleIn(0, 0);
    this->component.doDispatch();

    // Even with invalid ang vel, B-dot calculation proceeds but logs event
    ASSERT_EVENTS_MAG_DETUMBLE_INVALID_ANG_VEL_SIZE(1);
    ASSERT_EVENTS_MAG_DETUMBLE_BDOT_UPDATE_SIZE(1); // B-dot should still run
    ASSERT_FROM_PORT_HISTORY_SIZE(3); // Coil commands should be sent
  }

  void Tester::test_bdotAlgorithm_zeroMagField() {
    this->setupTest();
    this->sendCmd_MAG_DETUMBLE_ENABLE_BDOT(0, 1, true);
    this->component.doDispatch();
    this->clearHistory();

    // Initial non-zero field
    this->invoke_to_magFieldIn(0, 1.0f, 1.0f, 1.0f);
    this->component.doDispatch();
    this->invoke_to_runScheduleIn(0, 0); // Store B1
    this->component.doDispatch();
    this->clearHistory();

    // Next field is zero
    this->advanceTime(1,0);
    this->invoke_to_magFieldIn(0, 0.0f, 0.0f, 0.0f); // B2 is zero
    this->component.doDispatch();

    this->invoke_to_runScheduleIn(0, 0);
    this->component.doDispatch();

    ASSERT_EVENTS_MAG_DETUMBLE_ZERO_MAG_FIELD_SIZE(1);
    // Coils should be commanded to zero
    ASSERT_FROM_PORT_HISTORY_SIZE(3);
    ASSERT_EQ(this->m_hwControl_invocations, 3);
    ASSERT_FLOAT_EQ(this->m_hwControl_current[0], 0.0f);
    ASSERT_FLOAT_EQ(this->m_hwControl_current[1], 0.0f);
    ASSERT_FLOAT_EQ(this->m_hwControl_current[2], 0.0f);
  }

  void Tester::test_bdotAlgorithm_invalidDt() {
    this->setupTest();
    this->sendCmd_MAG_DETUMBLE_ENABLE_BDOT(0, 1, true);
    this->component.doDispatch();
    this->clearHistory();

    // Send B1
    this->invoke_to_magFieldIn(0, 1.0f, 1.0f, 1.0f);
    this->component.doDispatch();
    this->invoke_to_runScheduleIn(0, 0); // Stores B1
    this->component.doDispatch();
    this->clearHistory();

    // Send B2 with same timestamp as B1 (dt=0)
    // Note: this->setTestTime() in advanceTime normally prevents this.
    // We need to ensure m_magFieldTimeTag for B2 is same as m_prevMagFieldTimeTag
    this->component.m_magFieldX = 2.0f; // Manually set current B without invoking port to control time
    this->component.m_magFieldY = 2.0f;
    this->component.m_magFieldZ = 2.0f;
    this->component.m_magFieldTimeTag = this->component.m_prevMagFieldTimeTag; // Force dt=0
    this->component.m_magFieldReceived = true;

    this->invoke_to_runScheduleIn(0, 0);
    this->component.doDispatch();

    // No B-dot update, no coil commands, possibly an event if we added one for dt <= 0
    ASSERT_EVENTS_SIZE(0); // Current FPP doesn't have MAG_DETUMBLE_BDOT_DT_INVALID
    ASSERT_FROM_PORT_HISTORY_SIZE(0);
  }

  void Tester::test_coilSaturation() {
    this->setupTest();
    F32 overLimit = MagneticDetumbleImpl::MAX_COIL_CURRENT + 0.5f;

    this->invoke_to_setDipoleCmdIn(0, overLimit, -overLimit, MagneticDetumbleImpl::MAX_COIL_CURRENT / 2.0f);
    this->component.doDispatch();

    ASSERT_EVENTS_MAG_DETUMBLE_DIPOLE_CMD_RECEIVED_SIZE(1);
    ASSERT_EVENTS_MAG_DETUMBLE_COIL_SATURATION_SIZE(2); // X and Y axes saturate
    ASSERT_EVENTS_MAG_DETUMBLE_COIL_SATURATION(0, 0, overLimit, MagneticDetumbleImpl::MAX_COIL_CURRENT); // X
    ASSERT_EVENTS_MAG_DETUMBLE_COIL_SATURATION(1, 1, -overLimit, -MagneticDetumbleImpl::MAX_COIL_CURRENT); // Y

    ASSERT_FROM_PORT_HISTORY_SIZE(3);
    ASSERT_EQ(this->m_hwControl_invocations, 3);
    ASSERT_FLOAT_EQ(this->m_hwControl_current[0], MagneticDetumbleImpl::MAX_COIL_CURRENT); // X saturated
    ASSERT_FLOAT_EQ(this->m_hwControl_current[1], -MagneticDetumbleImpl::MAX_COIL_CURRENT); // Y saturated
    ASSERT_FLOAT_EQ(this->m_hwControl_current[2], MagneticDetumbleImpl::MAX_COIL_CURRENT / 2.0f); // Z not saturated
  }


} // end namespace Drv
