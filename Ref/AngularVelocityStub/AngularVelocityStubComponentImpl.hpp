#ifndef REF_ANGULARVELOCITYSTUB_COMPONENTIMPL_HPP
#define REF_ANGULARVELOCITYSTUB_COMPONENTIMPL_HPP

#include "Ref/AngularVelocityStub/AngularVelocityStubComponentAc.hpp"
#include "Svc/MagneticDetumble/MagneticDetumbleTypes.hpp"

namespace Ref {
  class AngularVelocityStubComponentImpl : public AngularVelocityStubComponentBase {
    public:
      AngularVelocityStubComponentImpl(const char* const compName);
      void init(const NATIVE_INT_TYPE instance = 0);
      ~AngularVelocityStubComponentImpl();
    PRIVATE:
      void schedIn_handler(const NATIVE_INT_TYPE portNum, NATIVE_UINT_TYPE context);
      Svc::Vector3 m_stubVelocity;
  };
}
#endif
