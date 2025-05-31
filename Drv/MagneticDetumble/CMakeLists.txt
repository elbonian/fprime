# CMakeLists.txt for Drv/MagneticDetumble

set(COMPONENT_NAME MagneticDetumble)
set(COMPONENT_SOURCES
    "${CMAKE_CURRENT_LIST_DIR}/MagneticDetumbleImpl.cpp"
)

set(COMPONENT_HEADERS
    "${CMAKE_CURRENT_LIST_DIR}/MagneticDetumbleImpl.hpp"
)

# Register the component with FPrime build system
register_fprime_component()

# --- Unit Test ---
# Add unit test sources
set(UNIT_TEST_SOURCES
    "${CMAKE_CURRENT_LIST_DIR}/test/ut/Tester.cpp"
    "${CMAKE_CURRENT_LIST_DIR}/test/ut/TestMain.cpp"
)

# Register the unit test with FPrime build system
register_fprime_ut()
