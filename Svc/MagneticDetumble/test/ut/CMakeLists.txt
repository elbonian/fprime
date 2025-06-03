set(UT_SOURCES
  Tester.cpp
  MagneticDetumbleTest.cpp # Main test cases
)

set(UT_HEADERS
  Tester.hpp
)

set(TEST_MOD_DEPS "") # Add module dependencies if any are needed by the component itself

# Check for fprime_generate_headers to support older CMake versions
if(COMMAND fprime_generate_headers)
    fprime_generate_headers(MagneticDetumbleTest ${UT_HEADERS})
endif()

register_fprime_ut()
