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
