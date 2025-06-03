#include "Ref/AngularVelocityStub/AngularVelocityStubComponentImpl.hpp"

namespace Ref {
  AngularVelocityStubComponentImpl::AngularVelocityStubComponentImpl(const char* const compName) : AngularVelocityStubComponentBase(compName) {
    m_stubVelocity.setx(0.01f); m_stubVelocity.sety(-0.02f); m_stubVelocity.setz(0.03f);
  }
  void AngularVelocityStubComponentImpl::init(const NATIVE_INT_TYPE instance) { AngularVelocityStubComponentBase::init(instance); }
  AngularVelocityStubComponentImpl::~AngularVelocityStubComponentImpl() {}
  void AngularVelocityStubComponentImpl::schedIn_handler(const NATIVE_INT_TYPE portNum, NATIVE_UINT_TYPE context) {
    if (isConnected_vectorOut_OutputPort(0)) {
      this->vectorOut_out(0, m_stubVelocity);
    }
  }
}
