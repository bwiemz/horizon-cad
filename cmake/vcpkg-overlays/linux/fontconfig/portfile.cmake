# The system's fontconfig, not one built here (the Linux presets'
# VCPKG_OVERLAY_PORTS, cmake/vcpkg-overlays/linux). A fontconfig reads the
# configuration its own version was written for, and a desktop's is its
# system's: vcpkg's, built into Qt, warned for every rule a newer
# distribution wrote (#182).
# Nothing is built or installed: Qt and HarfBuzz find the system's library
# with CMake's Find modules or pkg-config and link it shared, and a package
# takes it from the machine it runs on, as an AppImage expects to.
set(VCPKG_POLICY_EMPTY_PACKAGE enabled)

find_program(HZ_PKG_CONFIG NAMES pkg-config pkgconf)
if(HZ_PKG_CONFIG)
    execute_process(COMMAND "${HZ_PKG_CONFIG}" --exists fontconfig RESULT_VARIABLE missing)
    if(missing)
        message(FATAL_ERROR "fontconfig's development files are not installed: "
                            "apt install libfontconfig-dev, or dnf install fontconfig-devel")
    endif()
endif()
