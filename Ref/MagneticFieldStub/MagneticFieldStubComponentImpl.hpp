#ifndef REF_MAGNETICFIELDSTUB_COMPONENTIMPL_HPP
#define REF_MAGNETICFIELDSTUB_COMPONENTIMPL_HPP

#include "Ref/MagneticFieldStub/MagneticFieldStubComponentAc.hpp"
#include "Svc/MagneticDetumble/MagneticDetumbleTypes.hpp"

namespace Ref {
  class MagneticFieldStubComponentImpl : public MagneticFieldStubComponentBase {
    public:
      MagneticFieldStubComponentImpl(const char* const compName);
      void init(const NATIVE_INT_TYPE instance = 0);
      ~MagneticFieldStubComponentImpl();
    PRIVATE:
      void schedIn_handler(const NATIVE_INT_TYPE portNum, NATIVE_UINT_TYPE context);
      Svc::Vector3 m_stubMagField;
  };
}
#endif
