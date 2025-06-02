set(SOURCE_FILES
    MagneticDetumbleComponentImpl.cpp
)
set(FPP_FILES
    MagneticDetumble.fpp
)
# Boxcar file is typically empty for components
set(BOXCAR_FILE)

register_fprime_module()

# Unit test target
# The register_fprime_ut command will automatically find the
# test files in the test/ut subdirectory and link against
# the component library.
if (BUILD_TESTING)
    register_fprime_ut()
endif()
