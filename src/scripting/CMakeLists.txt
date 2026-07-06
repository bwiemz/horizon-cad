add_library(hz_scripting STATIC
    src/ScriptContext.cpp
    src/ScriptEngine.cpp
)

target_include_directories(hz_scripting
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

target_link_libraries(hz_scripting
    PUBLIC
        Horizon::Math
        Horizon::Modeling
        Horizon::Document
        # embed brings the Python headers/libs; PUBLIC so the interpreter
        # symbols resolve in the final executable that links this static lib.
        pybind11::embed
    PRIVATE
        Horizon::Drafting
        Horizon::FileIO
        Horizon::Simulation
        Horizon::Cam
)

add_library(Horizon::Scripting ALIAS hz_scripting)
