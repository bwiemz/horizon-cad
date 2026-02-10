add_library(hz_document STATIC
    src/Document.cpp
    src/FeatureTree.cpp
    src/UndoStack.cpp
)

target_include_directories(hz_document
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

target_link_libraries(hz_document
    PUBLIC
        Horizon::Math
        Horizon::Geometry
        Horizon::Topology
        Horizon::Modeling
        Horizon::Constraint
        Horizon::Drafting
        nlohmann_json::nlohmann_json
)

add_library(Horizon::Document ALIAS hz_document)
