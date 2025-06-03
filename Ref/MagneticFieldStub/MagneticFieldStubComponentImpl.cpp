#include "Ref/MagneticFieldStub/MagneticFieldStubComponentImpl.hpp"

namespace Ref {
  MagneticFieldStubComponentImpl::MagneticFieldStubComponentImpl(const char* const compName) : MagneticFieldStubComponentBase(compName) {
    m_stubMagField.setx(20000e-9f); m_stubMagField.sety(10000e-9f); m_stubMagField.setz(-50000e-9f);
  }
  void MagneticFieldStubComponentImpl::init(const NATIVE_INT_TYPE instance) { MagneticFieldStubComponentBase::init(instance); }
  MagneticFieldStubComponentImpl::~MagneticFieldStubComponentImpl() {}
  void MagneticFieldStubComponentImpl::schedIn_handler(const NATIVE_INT_TYPE portNum, NATIVE_UINT_TYPE context) {
    if (isConnected_vectorOut_OutputPort(0)) {
      this->vectorOut_out(0, m_stubMagField);
    }
  }
}
