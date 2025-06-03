set(SOURCES
    MagneticDetumbleComponentImpl.cpp
)

set(HEADERS
    MagneticDetumbleComponentImpl.hpp
)

set(SYS_GEN_XML_FILES
    MagneticDetumbleComponentAi.xml
)

set(MOD_DEPS
    Fw/Time # For schedIn port
    Fw/Logger # For logging events
)

set(SYS_GEN_FPP_FILES
    Svc/MagneticDetumble/Ports.fpp
    Svc/MagneticDetumble/MagneticDetumbleTypes.fpp
)

# Check for fprime_generate_headers to support older CMake versions
if(COMMAND fprime_generate_headers)
    fprime_generate_headers(MagneticDetumble ${HEADERS})
endif()

register_fprime_component(MagneticDetumble)

add_fprime_subdirectory(${CMAKE_CURRENT_LIST_DIR}/test/ut) # Add this line
