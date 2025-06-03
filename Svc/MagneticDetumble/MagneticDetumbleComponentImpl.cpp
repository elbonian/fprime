#include <Svc/MagneticDetumble/MagneticDetumbleComponentImpl.hpp>
#include <Fw/Types/BasicTypes.hpp>
#include <cmath> // For sqrt, sin
#include <Fw/Logger/Logger.hpp> // For logging
#include <Fw/Types/LogString.hpp> // For Fw::LogStringArg

// Anonymous namespace for helper functions or constants if needed
namespace {
    // Small epsilon value for float comparisons, especially for checking zero magnitude
    const F32 FLOAT_EPSILON = 1e-9f;
}


namespace Svc {

  // ----------------------------------------------------------------------
  // Construction, initialization, and destruction
  // ----------------------------------------------------------------------

  MagneticDetumbleComponentImpl ::
    MagneticDetumbleComponentImpl(
        const char *const compName
    ) : MagneticDetumbleComponentBase(compName),
        m_angularVelocity{0.0f, 0.0f, 0.0f},
        m_magneticField{0.0f, 0.0f, 0.0f},
        m_angVelReceived(false),
        m_magFieldReceived(false),
        m_orbitRate(0.0011f), // Approx 90 min orbit period (rad/s)
        m_geomagneticInclination(static_cast<F32>(M_PI / 4.0)), // 45 degrees in radians
        m_minInertia(0.33f),   // kg*m^2, from paper Table 1
        m_controlGainK(0.0f),
        m_dipoleSaturationLimit(2.0f) // Initialize saturation limit (A*m^2)
  {

  }

  void MagneticDetumbleComponentImpl ::
    init(
        const NATIVE_INT_TYPE instance
    )
  {
    MagneticDetumbleComponentBase::init(instance);
    calculateGain(); // Calculate initial gain
    tlmWrite_controlGain(this->m_controlGainK);
    this->log_ACTIVITY_HI_DetumbleInitialized(this->m_controlGainK); // Changed from Fw::Logger
  }

  MagneticDetumbleComponentImpl ::
    ~MagneticDetumbleComponentImpl()
  {

  }

  // ----------------------------------------------------------------------
  // Handler implementations for user-defined typed input ports
  // ----------------------------------------------------------------------

  void MagneticDetumbleComponentImpl ::
    angularVelocityIn_handler(
        const NATIVE_INT_TYPE portNum,
        Svc::Vector3 &val
    )
  {
    this->m_angularVelocity = val;
    this->m_angVelReceived = true;
    // Telemetry for received angular velocity
    this->tlmWrite_angularVelocity(this->m_angularVelocity);
  }

  void MagneticDetumbleComponentImpl ::
    magneticFieldIn_handler(
        const NATIVE_INT_TYPE portNum,
        Svc::Vector3 &val
    )
  {
    this->m_magneticField = val;
    this->m_magFieldReceived = true;
    // Telemetry for received magnetic field
    this->tlmWrite_magneticField(this->m_magneticField);
  }

  void MagneticDetumbleComponentImpl ::
    schedIn_handler(
        const NATIVE_INT_TYPE portNum,
        NATIVE_UINT_TYPE context
    )
  {
    // Ensure we have received both angular velocity and magnetic field data
    if (!this->m_angVelReceived || !this->m_magFieldReceived) {
      return;
    }

    Svc::Vector3 b = this->m_magneticField;
    Svc::Vector3 w = this->m_angularVelocity;
    Svc::Vector3 m_commanded;

    F32 b_mag = this->magnitude(b);


    if (b_mag < FLOAT_EPSILON) {
      this->log_WARNING_LO_MagFieldNearZero(b_mag);
      m_commanded.setx(0.0f);
      m_commanded.sety(0.0f);
      m_commanded.setz(0.0f);
    } else {
      // Alternative calculation to minimize divisions: m = -k!/|b|^2 * (b x w)
      Svc::Vector3 b_cross_w = this->crossProduct(b,w);
      F32 b_mag_sq = b_mag * b_mag;
      m_commanded = this->scale(b_cross_w, -this->m_controlGainK / b_mag_sq);

      // Check for saturation
      F32 m_mag = this->magnitude(m_commanded);
      if (m_mag > this->m_dipoleSaturationLimit) {
        Svc::Vector3 m_clamped = this->scale(m_commanded, this->m_dipoleSaturationLimit / m_mag);
        this->log_WARNING_LO_DipoleSaturated(
            m_commanded.getx(), m_commanded.gety(), m_commanded.getz(), this->m_dipoleSaturationLimit
        );
        m_commanded = m_clamped;
      }
    }

    // Telemetry for commanded dipole
    this->tlmWrite_commandedDipole(m_commanded);

    // Output the commanded dipole moment
    if (this->isConnected_dipoleRequestOut_OutputPort(0)) {
      this->dipoleRequestOut_out(0, m_commanded);
    }

  }

  // ----------------------------------------------------------------------
  // Private helper methods
  // ----------------------------------------------------------------------
  void MagneticDetumbleComponentImpl::calculateGain() {
    // k! = 2 * orbit_rate * (1 + sin(beta_m)) * J_min
    // Ensure m_geomagneticInclination is in radians for sin function
    this->m_controlGainK = 2.0f * this->m_orbitRate *
                           (1.0f + std::sin(this->m_geomagneticInclination)) *
                           this->m_minInertia;
    // Example of ParameterUpdate event (if parameters were configurable)
    // Fw::LogStringArg paramName("m_orbitRate");
    // this->log_ACTIVITY_HI_ParameterUpdate(paramName, this->m_orbitRate, this->m_controlGainK);
  }

  Svc::Vector3 MagneticDetumbleComponentImpl::crossProduct(const Svc::Vector3& v1, const Svc::Vector3& v2) {
      Svc::Vector3 result;
      result.setx(v1.gety() * v2.getz() - v1.getz() * v2.gety());
      result.sety(v1.getz() * v2.getx() - v1.getx() * v2.getz());
      result.setz(v1.getx() * v2.gety() - v1.gety() * v2.getx());
      return result;
  }

  F32 MagneticDetumbleComponentImpl::magnitude(const Svc::Vector3& v) {
      return std::sqrt(v.getx() * v.getx() + v.gety() * v.gety() + v.getz() * v.getz());
  }

  Svc::Vector3 MagneticDetumbleComponentImpl::scale(const Svc::Vector3& v, F32 s) {
      Svc::Vector3 result;
      result.setx(v.getx() * s);
      result.sety(v.gety() * s);
      result.setz(v.getz() * s);
      return result;
  }

  Svc::Vector3 MagneticDetumbleComponentImpl::normalize(const Svc::Vector3& v) {
      F32 mag = this->magnitude(v);
      if (mag < FLOAT_EPSILON) { // Avoid division by zero
          Svc::Vector3 zeroVec = {0.0f, 0.0f, 0.0f};
          return zeroVec;
      }
      return this->scale(v, 1.0f / mag);
  }


} // end namespace Svc
