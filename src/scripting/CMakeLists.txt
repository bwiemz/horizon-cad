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

# pybind11 declares its types with hidden visibility; code holding them (e.g.
# ScriptEngine::Impl's py::dict) must be hidden too, or GCC warns that the
# class has greater visibility than its members. pybind11 documents
# -fvisibility=hidden for exactly this. Static library: nothing is exported.
set_target_properties(hz_scripting PROPERTIES
    CXX_VISIBILITY_PRESET hidden
    VISIBILITY_INLINES_HIDDEN ON
)

add_library(Horizon::Scripting ALIAS hz_scripting)
