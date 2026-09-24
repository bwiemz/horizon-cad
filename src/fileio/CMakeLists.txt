find_package(flatbuffers CONFIG REQUIRED)

# Generate the FlatBuffers header for the binary document container.
set(HZ_FBS_GENERATED_DIR ${CMAKE_CURRENT_BINARY_DIR}/generated)
add_custom_command(
    OUTPUT ${HZ_FBS_GENERATED_DIR}/hzbinary_generated.h
    COMMAND flatbuffers::flatc --cpp -o ${HZ_FBS_GENERATED_DIR}
            ${CMAKE_CURRENT_SOURCE_DIR}/schemas/hzbinary.fbs
    DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/schemas/hzbinary.fbs
    COMMENT "flatc: generating hzbinary_generated.h"
)
add_custom_target(hz_fileio_fbs DEPENDS ${HZ_FBS_GENERATED_DIR}/hzbinary_generated.h)

# Crash-safe file writing, a library of its own so that modules the formats
# do not belong in (pdm) can use it without linking them.
add_library(hz_atomic_file STATIC
    src/AtomicFile.cpp
)

target_include_directories(hz_atomic_file
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

add_library(Horizon::AtomicFile ALIAS hz_atomic_file)

add_library(hz_fileio STATIC
    src/NativeFormat.cpp
    src/BinaryFormat.cpp
    src/StlExporter.cpp
    src/StepFormat.cpp
    src/GltfExport.cpp
    src/StlExport.cpp
    src/DxfFormat.cpp
    src/DxfCodec.cpp
    src/DrawingExport.cpp
    src/DrawingDocumentIO.cpp
    src/DrawingDimensionRenderer.cpp
    src/GeometricToleranceRenderer.cpp
    src/BalloonRenderer.cpp
    src/BomExport.cpp
    src/TitleBlockRenderer.cpp
)

target_include_directories(hz_fileio
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
    PRIVATE
        ${HZ_FBS_GENERATED_DIR}
)

add_dependencies(hz_fileio hz_fileio_fbs)

target_link_libraries(hz_fileio
    PUBLIC
        Horizon::AtomicFile
        Horizon::Math
        Horizon::Geometry
        Horizon::Topology
        Horizon::Drafting
        Horizon::Constraint
        Horizon::Document
        Horizon::Modeling
        Horizon::Render
        nlohmann_json::nlohmann_json
    PRIVATE
        flatbuffers::flatbuffers
)

add_library(Horizon::FileIO ALIAS hz_fileio)
