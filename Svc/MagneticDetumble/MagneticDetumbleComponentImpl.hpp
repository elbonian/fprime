#ifndef SVC_MAGNETICDETUMBLE_COMPONENTIMPL_HPP
#define SVC_MAGNETICDETUMBLE_COMPONENTIMPL_HPP

#include "Svc/MagneticDetumble/MagneticDetumbleComponentAc.hpp"
#include "Svc/MagneticDetumble/MagneticDetumbleTypes.hpp" // For Svc::Vector3

namespace Svc {

  class MagneticDetumbleComponentImpl :
    public MagneticDetumbleComponentBase
  {

    public:

      // ----------------------------------------------------------------------
      // Construction, initialization, and destruction
      // ----------------------------------------------------------------------

      //! Construct object MagneticDetumble
      MagneticDetumbleComponentImpl(
          const char *const compName /*!< The component name*/
      );

      //! Initialize object MagneticDetumble
      void init(
          const NATIVE_INT_TYPE instance = 0 /*!< The instance number*/
      );

      //! Destroy object MagneticDetumble
      ~MagneticDetumbleComponentImpl();

    PRIVATE:

      // ----------------------------------------------------------------------
      // Handler implementations for user-defined typed input ports
      // ----------------------------------------------------------------------

      //! Handler implementation for schedIn
      void schedIn_handler(
          const NATIVE_INT_TYPE portNum, /*!< The port number*/
          NATIVE_UINT_TYPE context /*!< The call order*/
      );

      //! Handler implementation for angularVelocityIn
      void angularVelocityIn_handler(
          const NATIVE_INT_TYPE portNum, /*!< The port number*/
          Svc::Vector3 &val /*!< The angular velocity vector*/
      );

      //! Handler implementation for magneticFieldIn
      void magneticFieldIn_handler(
          const NATIVE_INT_TYPE portNum, /*!< The port number*/
          Svc::Vector3 &val /*!< The magnetic field vector*/
      );

    PRIVATE:

      // ----------------------------------------------------------------------
      // Private helper methods
      // ----------------------------------------------------------------------
      void calculateGain();
      Svc::Vector3 crossProduct(const Svc::Vector3& v1, const Svc::Vector3& v2);
      F32 magnitude(const Svc::Vector3& v);
      Svc::Vector3 scale(const Svc::Vector3& v, F32 s);
      Svc::Vector3 normalize(const Svc::Vector3& v);

      // ----------------------------------------------------------------------
      // Member variables
      // ----------------------------------------------------------------------
      Svc::Vector3 m_angularVelocity; //!< Last received angular velocity
      Svc::Vector3 m_magneticField;   //!< Last received magnetic field
      bool m_angVelReceived;          //!< Flag indicating angular velocity received
      bool m_magFieldReceived;        //!< Flag indicating magnetic field received

      // Parameters for gain calculation (will be made configurable later)
      F32 m_orbitRate;      //!< Orbit rate (rad/s)
      F32 m_geomagneticInclination; //!< Geomagnetic inclination beta_m (radians)
      F32 m_minInertia;     //!< Minimum principal moment of inertia J_min (kg*m^2)
      F32 m_controlGainK;   //!< Calculated control gain k!
      F32 m_dipoleSaturationLimit; //!< Saturation limit for magnetic dipole (A*m^2)
  };

} // end namespace Svc

#endif
