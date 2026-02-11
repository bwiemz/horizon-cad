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
    include/horizon/ui/ArcTool.h
    include/horizon/ui/RectangleTool.h
    include/horizon/ui/PolylineTool.h
    include/horizon/ui/MoveTool.h
    include/horizon/ui/OffsetTool.h
    include/horizon/ui/TrimTool.h
    include/horizon/ui/FilletTool.h
    include/horizon/ui/MirrorTool.h
    include/horizon/ui/RotateTool.h
    include/horizon/ui/ScaleTool.h
    include/horizon/ui/Clipboard.h
    include/horizon/ui/PasteTool.h
    include/horizon/ui/RectArrayDialog.h
    include/horizon/ui/PolarArrayDialog.h
    include/horizon/ui/PropertyPanel.h
    include/horizon/ui/LayerPanel.h
    include/horizon/ui/LinearDimensionTool.h
    include/horizon/ui/RadialDimensionTool.h
    include/horizon/ui/AngularDimensionTool.h
    include/horizon/ui/LeaderTool.h
    include/horizon/ui/ConstraintTool.h
    include/horizon/ui/InsertBlockTool.h
    include/horizon/ui/InsertBlockDialog.h
    src/MainWindow.cpp
    src/ViewportWidget.cpp
    src/ToolManager.cpp
    src/LineTool.cpp
    src/CircleTool.cpp
    src/SelectTool.cpp
    src/ArcTool.cpp
    src/RectangleTool.cpp
    src/PolylineTool.cpp
    src/MoveTool.cpp
    src/OffsetTool.cpp
    src/TrimTool.cpp
    src/FilletTool.cpp
    src/MirrorTool.cpp
    src/RotateTool.cpp
    src/ScaleTool.cpp
    src/Clipboard.cpp
    src/PasteTool.cpp
    src/RectArrayDialog.cpp
    src/PolarArrayDialog.cpp
    src/PropertyPanel.cpp
    src/LayerPanel.cpp
    src/LinearDimensionTool.cpp
    src/RadialDimensionTool.cpp
    src/AngularDimensionTool.cpp
    src/LeaderTool.cpp
    src/ConstraintTool.cpp
    src/InsertBlockTool.cpp
    src/InsertBlockDialog.cpp
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
        Horizon::Constraint
        Qt6::Core
        Qt6::Gui
        Qt6::Widgets
        Qt6::OpenGLWidgets
        spdlog::spdlog
)

add_library(Horizon::UI ALIAS hz_ui)
