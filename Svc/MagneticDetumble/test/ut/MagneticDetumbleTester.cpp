#include "Svc/MagneticDetumble/test/ut/MagneticDetumbleTester.hpp"
#include "Fw/Test/UnitTest.hpp" // For REQUIRE_JSON_OBJECT_EQUALS and other test macros
#include <Os/Log.hpp> // For Os::Log

// Define a specific tolerance for floating point comparisons if needed
#define FLOAT_TOLERANCE 1e-5f

namespace Svc {

  // ----------------------------------------------------------------------
  // Construction and destruction
  // ----------------------------------------------------------------------

  MagneticDetumbleTester ::
    MagneticDetumbleTester() :
      MagneticDetumbleGTestBase("MagneticDetumbleTester", MagneticDetumbleComponentImpl::MAX_HISTORY_SIZE),
      component("MagneticDetumble")
  {
    // Initialize test variables
    this->m_lastDipoleCmd.set(0.0f, 0.0f, 0.0f);
    this->m_magnetorquerCmdCount = 0;
    this->m_currentTime.set(0, 0); // Initialize time to zero or a known start time

    // Connect ports
    this->connectPorts();

    // Initialize component for testing (call its init methods)
    this->initComponents();
  }

  MagneticDetumbleTester ::
    ~MagneticDetumbleTester()
  {

  }

  // ----------------------------------------------------------------------
  // Helper methods
  // ----------------------------------------------------------------------

  void MagneticDetumbleTester ::
    connectPorts()
  {
    // Connect to command registration, response, and sequence ports
    this->connect_to_cmdIn(0, this->component.get_cmdIn_InputPort(0));
    this->component.set_cmdRegOut_OutputPort(0, this->get_from_cmdRegOut(0));
    this->component.set_cmdResponseOut_OutputPort(0, this->get_from_cmdResponseOut(0));

    // Connect to telemetry output port
    this->component.set_tlmOut_OutputPort(0, this->get_from_tlmOut(0));

    // Connect to event output port
    this->component.set_logOut_OutputPort(0, this->get_from_logOut(0));
    this->component.set_logTextOut_OutputPort(0, this->get_from_logTextOut(0)); // If text logging is used

    // Connect to time caller port
    this->component.set_timeCaller_OutputPort(0, this->get_from_timeCaller(0));

    // Connect output ports of the component to the tester's input ports
    this->connect_to_magnetorquerCmdOut(0, this->component.get_magnetorquerCmdOut_OutputPort(0));

    // Connect input ports of the component to the tester's output ports
    // These are invoked by the tester to simulate inputs to the component
    // For SCHED_IN, angularVelocityIn, magFieldIn
    // No explicit connection needed here, they are invoked using invoke_to_<portname>
  }

  void MagneticDetumbleTester ::
    initComponents()
  {
    // Initialize the component instance
    this->component.init(0); // Instance 0

    // Setup default parameters for tests if not already handled by param DB
    // For example, explicitly set parameters if there's no external param loader for UT
    // Or rely on the defaults set in MagneticDetumbleComponentImpl::init if they are suitable
    // For now, we assume defaults or a param loader is handled elsewhere or by component's init.
    // If specific values are needed for tests, set them here using paramSet_ APIs if available
    // or by modifying the component directly if whitebox testing is acceptable.

    // Example: Set a default gain if not loaded by PrmDb for testing
    // This would typically be done via a test-specific PrmDb setup
    // For simplicity, if the component's init doesn't load it, we might need to set it manually
    // or ensure the default in the component is used.
    // We are relying on the component's `init` to load params for now.
  }


  // ----------------------------------------------------------------------
  // Handlers for typed from ports
  // ----------------------------------------------------------------------

  void MagneticDetumbleTester ::
    from_magnetorquerCmdOut_handler(
        const NATIVE_INT_TYPE portNum,
        const Fw::Math::Vector3& dipoleCmd
    )
  {
    this->m_lastDipoleCmd = dipoleCmd;
    this->m_magnetorquerCmdCount++;
    this->pushFromPortEntry_magnetorquerCmdOut(dipoleCmd);
  }

  void MagneticDetumbleTester ::
    from_timeCaller_handler(
        const NATIVE_INT_TYPE portNum,
        Fw::Time& time
    )
  {
    // This handler is called by the component under test when it needs the current time.
    // We provide the time that we've set in m_currentTime.
    time = this->m_currentTime;
    this->pushFromPortEntry_timeCaller(time); // Record that the port was called
  }


  // ----------------------------------------------------------------------
  // Tests
  // ----------------------------------------------------------------------

  void MagneticDetumbleTester ::
    test_initialization()
  {
    // Pre-conditions:initComponents() in constructor has run component.init()
    // Parameters CONTROL_GAIN and INITIAL_MODE should have been loaded.
    // Default values (from .fpp or .cpp) or explicitly set test params should be active.

    // Check if parameters were loaded (relying on component's logging in init for this)
    // Or, if we had accessors or a way to read back params, we'd use them.
    // For now, check initial telemetry which depends on these params.

    // Assuming INITIAL_MODE defaults to AUTO (Fw::On::ON) if not specified or loaded
    // And CONTROL_GAIN has a default.
    // The component's init logs parameter load status. We can check for those logs.

    // Check initial MODE telemetry
    // The component's init() should have emitted MODE telemetry.
    ASSERT_TLM_MODE_SIZE(1); // Should have been one MODE tlm write from init()
    // The value depends on INITIAL_MODE parameter. Let's assume it defaults to ON (AUTO).
    // If your .fpp has a different default, adjust this.
    // Or, if you have a test PrmDb, set INITIAL_MODE there.
    Fw::On expectedInitialMode = Fw::On::ON; // Default to AUTO if param not loaded
    // Read the parameter if possible, or use the default from component
    Fw::ParamValid initialModeValid = Fw::ParamValid::DEFAULT;
    Fw::On prmInitialMode = component.paramGet_INITIAL_MODE(initialModeValid);
    if (initialModeValid == Fw::ParamValid::VALID || initialModeValid == Fw::ParamValid::DEFAULT) {
        expectedInitialMode = prmInitialMode;
    }
    ASSERT_TLM_MODE(0, expectedInitialMode);


    // Check initial DIPOLE telemetry (should be zero)
    ASSERT_TLM_DIPOLE_SIZE(1); // Should have been one DIPOLE tlm write from init()
    Fw::Math::Vector3 expectedInitialDipole(0.0f, 0.0f, 0.0f);
    ASSERT_TLM_DIPOLE(0, expectedInitialDipole);

    // Check if CONTROL_GAIN parameter load was logged (assuming it has a default)
    // This requires inspecting logs, which GTestBase helps with.
    // Example: ASSERT_LOG_TEXT_EVENT_COUNT(1); // If specific log message is expected for param
    // ASSERT_EVENTS_CONTROL_GAIN_LOADED_SIZE(1); // If you made a specific event for this.
    // For now, we rely on the component's own logging during init.
    // The component logs "CONTROL_GAIN parameter loaded" or "CONTROL_GAIN parameter load failed".
    // We can check for one of these.
  }

  void MagneticDetumbleTester ::
    test_set_mode_auto()
  {
    this->clearHistory();
    this->sendCmd_SET_MODE(0, 0, Fw::On::ON); // cmdSeq 0, opCode 0 (doesn't matter for this test)
    ASSERT_CMD_RESPONSE_SIZE(1);
    ASSERT_CMD_RESPONSE(0, MagneticDetumbleComponentBase::OPCODE_SET_MODE, 0, Fw::CmdResponse::OK);

    ASSERT_TLM_MODE_SIZE(1);
    ASSERT_TLM_MODE(0, Fw::On::ON);

    ASSERT_EVENTS_MODE_CHANGED_SIZE(1);
    ASSERT_EVENTS_MODE_CHANGED(0, Fw::On::ON);
  }

  void MagneticDetumbleTester ::
    test_set_mode_manual()
  {
    this->clearHistory();
    this->sendCmd_SET_MODE(0, 0, Fw::On::OFF);
    ASSERT_CMD_RESPONSE_SIZE(1);
    ASSERT_CMD_RESPONSE(0, MagneticDetumbleComponentBase::OPCODE_SET_MODE, 0, Fw::CmdResponse::OK);

    ASSERT_TLM_MODE_SIZE(1);
    ASSERT_TLM_MODE(0, Fw::On::OFF);

    ASSERT_EVENTS_MODE_CHANGED_SIZE(1);
    ASSERT_EVENTS_MODE_CHANGED(0, Fw::On::OFF);
  }

  void MagneticDetumbleTester ::
    test_set_dipole_manual_ok()
  {
    // First, set mode to MANUAL
    this->sendCmd_SET_MODE(0, 0, Fw::On::OFF);
    this->clearHistory(); // Clear events/tlm from SET_MODE

    Fw::Math::Vector3 dipoleToSet(1.0f, 2.0f, 3.0f);
    this->sendCmd_SET_DIPOLE(0, 0, dipoleToSet);
    ASSERT_CMD_RESPONSE_SIZE(1);
    ASSERT_CMD_RESPONSE(0, MagneticDetumbleComponentBase::OPCODE_SET_DIPOLE, 0, Fw::CmdResponse::OK);

    ASSERT_TLM_DIPOLE_SIZE(1);
    ASSERT_TLM_DIPOLE(0, dipoleToSet);

    ASSERT_EVENTS_DIPOLE_SET_SIZE(1);
    ASSERT_EVENTS_DIPOLE_SET(0, dipoleToSet);
  }

  void MagneticDetumbleTester ::
    test_set_dipole_auto_error()
  {
    // First, set mode to AUTO
    this->sendCmd_SET_MODE(0, 0, Fw::On::ON);
    this->clearHistory();

    Fw::Math::Vector3 dipoleToSet(1.0f, 2.0f, 3.0f);
    this->sendCmd_SET_DIPOLE(0, 0, dipoleToSet);
    ASSERT_CMD_RESPONSE_SIZE(1);
    ASSERT_CMD_RESPONSE(0, MagneticDetumbleComponentBase::OPCODE_SET_DIPOLE, 0, Fw::CmdResponse::VALIDATION_ERROR);

    ASSERT_TLM_DIPOLE_SIZE(0); // No change in dipole telemetry
    ASSERT_EVENTS_DIPOLE_SET_SIZE(0); // No event
    ASSERT_EVENTS_COMMAND_ERROR_SIZE(1); // Should be a command error event from the component
    ASSERT_LOG_WARNING_HI_COMMAND_SIZE(1); // From the log in SET_DIPOLE_cmdHandler
  }


  void MagneticDetumbleTester ::
    test_sched_in_manual_mode()
  {
    // 1. Set mode to MANUAL
    this->sendCmd_SET_MODE(0, 0, Fw::On::OFF);

    // 2. Set a manual dipole
    Fw::Math::Vector3 manualDipole(0.1f, -0.2f, 0.3f);
    this->sendCmd_SET_DIPOLE(0, 1, manualDipole); // cmdSeq 1
    this->clearHistory(); // Clear telemetry and events from commands

    // 3. Simulate inputs (though not used for dipole calculation in MANUAL)
    Fw::Math::Vector3 omega(0.01f, 0.02f, 0.03f);
    Fw::Math::Vector3 magField(0.2f, 0.3f, 0.4f);

    // Provide inputs via invoke_to_<port>
    // Need to mock the input ports if they are synchronous,
    // or connect them to tester's `from_` ports if they are asynchronous.
    // For this component, angularVelocityIn and magFieldIn are sync input ports.
    // We need to set up mocks for them.
    // For simplicity in this example, we'll assume they are called and return fixed values.
    // This part of the GTestBase is usually auto-generated for invoking *to* the component.
    // The component directly calls this->angularVelocityIn_InputPort[0]->invoke(omega)
    // So, we need to connect these to something the tester provides.
    // This is typically done by connecting them to tester's `from_` ports that we then handle.
    // However, `angularVelocityIn` and `magFieldIn` are `Guarded` and `Sync` input ports.
    // The tester invokes `SCHED_IN`. The component then invokes `angularVelocityIn` and `magFieldIn`.
    // The tester needs to *provide* these implementations.
    // This is usually done by inheriting from the `*PortAi.xml` generated `InputPort` class
    // and overriding the `invoke` method.
    // The GTestBase should provide `invoke_to_angularVelocityIn` and `invoke_to_magFieldIn`
    // if these ports were *outputs* of the DUT. Since they are *inputs*, the component calls them.
    // The tester *is* the one providing the implementation for these ports.
    // Let's assume the GTestBase already provides a way to "push" data to these input ports
    // for the component to read when it calls invoke() on them.
    // If not, we'd have to add mock input port implementations.

    // The `MagneticDetumbleGTestBase` should have generated `invoke_to_angularVelocityIn` and `invoke_to_magFieldIn`
    // if these ports were asynchronous. Since they are synchronous, the component calls them.
    // The tester needs to provide implementations for `angularVelocityIn_InputPort` and `magFieldIn_InputPort`.
    // This is done by the `isConnected_angularVelocityIn_InputPort` and `isConnected_magFieldIn_InputPort`
    // and then the `from_` port handlers in the GTestBase.
    // The default GTestBase `from_port_handler` for sync input ports typically does an `ASSERT_TRUE(false)`
    // meaning you *must* override them in the tester if the component calls them.

    // Let's assume we will use `this->component.set_angularVelocityIn_InputPort` to connect to a mock.
    // For now, let's skip the detailed port mocking and focus on the SCHED_IN logic.
    // We will assume the ports return some default values or are not connected for this simplified step.
    // A more complete test would require proper mocking of these synchronous input ports.

    // We will directly call SCHED_IN. The component will try to call the input ports.
    // If not connected, it should handle it gracefully.
    // Let's assume they are connected and provide some values.
    // For now, we'll check the output based on manualDipole.

    // Invoke SCHED_IN
    this->invoke_to_SCHED_IN(0, 0); // portNum 0, context 0

    // Check telemetry
    ASSERT_TLM_MODE_SIZE(1);
    ASSERT_TLM_MODE(0, Fw::On::OFF);
    ASSERT_TLM_DIPOLE_SIZE(1);
    ASSERT_TLM_DIPOLE(0, manualDipole); // Should be the manual dipole
    // ASSERT_TLM_OMEGA_SIZE(1); // Add if omega is telemetered
    // ASSERT_TLM_MAG_FIELD_SIZE(1); // Add if mag_field is telemetered

    // Check output port
    ASSERT_FROM_PORT_HISTORY_SIZE(1); // Expect one call to magnetorquerCmdOut
    ASSERT_from_magnetorquerCmdOut_SIZE(1);
    ASSERT_from_magnetorquerCmdOut(0, manualDipole);

    // Check events (expect a generic iteration event, no error)
    ASSERT_EVENTS_CONTROL_ITERATION_SIZE(1);
    ASSERT_EVENTS_CONTROL_ITERATION_ERROR_SIZE(0);
  }

  // Placeholder for other tests to be implemented
  void MagneticDetumbleTester :: test_sched_in_auto_mode_nominal() {
      // Set mode to AUTO
      this->sendCmd_SET_MODE(0,0,Fw::On::ON);
      this->clearHistory();

      // Set initial time
      this->m_currentTime.set(10, 0); // 10 seconds, 0 microseconds

      // First SCHED_IN call (firstRun = true)
      // Provide MagField Data (Omega not used in B-dot directly)
      // This requires a way to inject data for magFieldIn_InputPort.
      // The component will call this->magFieldIn_InputPort[0]->invoke(current_B);
      // For this test, we need to ensure that when the component calls this, it gets a value.
      // This is usually done by connecting the component's input port to a tester's output port
      // and then having the tester "push" data to that port.
      // Or, more commonly for sync inputs, the tester provides an implementation of that input port.
      // The GTestBase *should* provide `connect_to_magFieldIn` which connects the component's
      // input port to the tester's `from_magFieldIn` handler. We then need to override that handler.
      // Let's assume this is setup.
      // For now, this test will be high-level.

      // TODO: Mocking or providing data for magFieldIn and timeCaller is crucial here.
      // The from_timeCaller_handler is already set up to provide m_currentTime.
      // We need a similar mechanism for magFieldIn.
      // For now, let's assume magFieldIn will return some values.

      // 1. First call to SCHED_IN (stores B1, t1)
      Fw::Math::Vector3 B1(1.0f, 2.0f, 3.0f);
      // How to make magFieldIn_InputPort return B1?
      // This requires the tester to implement the `from_magFieldIn_handler` if it's a sync call,
      // or to `invoke_to_magFieldIn` if it's an async call on the component side.
      // Since `magFieldIn` is a sync input port for the component, the component calls it.
      // The GTestBase should have:
      //  - `connect_to_magFieldIn(NATIVE_INT_TYPE portNum, Fw::InputPortBase* port)`
      //  - `virtual Fw::Success from_magFieldIn_handler(...)`
      // We need to connect it:
      //    this->component.set_magFieldIn_InputPort(0, this->get_from_magFieldIn(0)); (WRONG - this is for output ports of DUT)
      //    this->connect_to_magFieldIn(0, this->component.get_magFieldIn_InputPort(0)); (ALSO WRONG)

      // Correct way for sync input ports on DUT:
      // The component *has* an input port `magFieldIn_InputPort`.
      // The tester needs to provide an *output port* that connects to it.
      // `this->connect(this->component.get_magFieldIn_InputPort(0), this->get_to_magFieldIn(0))`
      // And then `this->invoke_to_magFieldIn(0, someValue)`.
      // This seems more for *driving* an input port.
      // The component's `magFieldIn_InputPort[0]->invoke(val)` needs a connected port to call.
      // This is where the `Port.fpp` for `Fw::Success` return type input ports comes in.
      // The tester needs to *be* the implementation of that port.
      // This is usually done by the GTestBase generating `invoke_fakeschedIn_InputPort` etc.
      // Let's assume we have `this->pushMagFieldData(B1);` which stores B1 for the component to get.

      this->log_ACTIVITY_LO_MSG("test_sched_in_auto_mode_nominal: Test needs proper sync input port mocking.");
      // For now, this test is incomplete due to the complexity of mocking sync inputs without seeing generated GTestBase.
  }
  void MagneticDetumbleTester :: test_sched_in_auto_mode_b_zero() { /* Placeholder */ }
  void MagneticDetumbleTester :: test_sched_in_auto_mode_dt_error() { /* Placeholder */ }
  void MagneticDetumbleTester :: test_port_not_connected_angular_velocity() { /* Placeholder */ }
  void MagneticDetumbleTester :: test_port_not_connected_mag_field() { /* Placeholder */ }
  void MagneticDetumbleTester :: test_port_not_connected_magnetorquer_cmd() { /* Placeholder */ }


} // end namespace Svc
