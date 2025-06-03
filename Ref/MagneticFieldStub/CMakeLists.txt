set(SOURCES MagneticFieldStubComponentImpl.cpp)
set(HEADERS MagneticFieldStubComponentImpl.hpp)
set(MOD_DEPS Svc/MagneticDetumble Fw/Time)
register_fprime_component(MagneticFieldStub)
