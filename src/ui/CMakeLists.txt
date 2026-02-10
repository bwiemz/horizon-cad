set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

add_library(hz_ui STATIC
    include/horizon/ui/MainWindow.h
    include/horizon/ui/ViewportWidget.h
    include/horizon/ui/Tool.h
    include/horizon/ui/ToolManager.h
    include/horizon/ui/LineTool.h
    include/horizon/ui/CircleTool.h
    include/horizon/ui/SelectTool.h
    src/MainWindow.cpp
    src/ViewportWidget.cpp
    src/ToolManager.cpp
    src/LineTool.cpp
    src/CircleTool.cpp
    src/SelectTool.cpp
)

target_include_directories(hz_ui
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

target_link_libraries(hz_ui
    PUBLIC
        Horizon::Math
        Horizon::Geometry
        Horizon::Render
        Horizon::Drafting
        Horizon::Document
        Horizon::FileIO
        Qt6::Core
        Qt6::Gui
        Qt6::Widgets
        Qt6::OpenGLWidgets
        spdlog::spdlog
)

add_library(Horizon::UI ALIAS hz_ui)
