add_library(hz_drafting STATIC
    src/DraftEntity.cpp
    src/DraftDocument.cpp
    src/Layer.cpp
    src/SnapEngine.cpp
    src/DraftLine.cpp
    src/DraftCircle.cpp
    src/DraftArc.cpp
    src/DraftRectangle.cpp
    src/DraftPolyline.cpp
    src/Intersection.cpp
    src/DraftDimension.cpp
    src/DraftLinearDimension.cpp
    src/DraftRadialDimension.cpp
    src/DraftAngularDimension.cpp
    src/DraftLeader.cpp
    src/BlockTable.cpp
    src/DraftBlockRef.cpp
    src/DraftText.cpp
    src/DraftSpline.cpp
    src/DraftHatch.cpp
    src/DraftEllipse.cpp
)

target_include_directories(hz_drafting
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

target_link_libraries(hz_drafting
    PUBLIC
        Horizon::Math
        Horizon::Geometry
)

add_library(Horizon::Drafting ALIAS hz_drafting)
