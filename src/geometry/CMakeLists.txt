add_library(hz_geometry STATIC
    src/Frame.cpp
    src/surfaces/Surface.cpp
    src/surfaces/AnalyticSurfaces.cpp
    src/curves/Curve2D.cpp
    src/curves/Line2D.cpp
    src/curves/Circle2D.cpp
    src/curves/Arc2D.cpp
    src/curves/Line3D.cpp
    src/curves/NurbsCurve.cpp
    src/curves/AnalyticCurves.cpp
    src/curves/Nurbs2D.cpp
    src/surfaces/NurbsSurface.cpp
    src/surfaces/SurfaceBuilder.cpp
    src/surfaces/SurfaceRecognition.cpp
    src/Triangulate2D.cpp
    src/Pcurve.cpp
)

target_include_directories(hz_geometry
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

target_link_libraries(hz_geometry
    PUBLIC Horizon::Math
)

add_library(Horizon::Geometry ALIAS hz_geometry)
