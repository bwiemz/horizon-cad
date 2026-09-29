# The system's FreeType, not one built here (the Linux presets'
# VCPKG_OVERLAY_PORTS, cmake/vcpkg-overlays/linux). With the system's
# fontconfig (#182), which reads fonts through the FreeType beside it and is
# handed Qt's FreeType faces (FcFreeTypeQueryFace), Qt's FreeType must be the
# system's too, not a second copy.
# Nothing is built or installed: Qt and HarfBuzz find the system's library
# with CMake's Find modules or pkg-config and link it shared, and a package
# takes it from the machine it runs on, as an AppImage expects to.
set(VCPKG_POLICY_EMPTY_PACKAGE enabled)

find_program(HZ_PKG_CONFIG NAMES pkg-config pkgconf)
if(HZ_PKG_CONFIG)
    execute_process(COMMAND "${HZ_PKG_CONFIG}" --exists freetype2 RESULT_VARIABLE missing)
    if(missing)
        message(FATAL_ERROR "FreeType's development files are not installed: "
                            "apt install libfreetype-dev, or dnf install freetype-devel")
    endif()
endif()
