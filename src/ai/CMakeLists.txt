# The AI assistant's core (hz::ai): a conversation in no provider's format,
# an adapter per provider wire format, the agent loop, and the CAD tools a
# model calls. No Qt: the application supplies the HTTP transport.
add_library(hz_ai STATIC
    src/AgentSession.cpp
    src/AnthropicAdapter.cpp
    src/CadAdvancedTools.cpp
    src/CadAssemblyTools.cpp
    src/CadDrawingTools.cpp
    src/CadDraftTools.cpp
    src/CadModelTools.cpp
    src/CadQueryTools.cpp
    src/CadSheetTools.cpp
    src/CadTools.cpp
    src/Conversation.cpp
    src/GeminiAdapter.cpp
    src/McpServer.cpp
    src/OpenAIAdapter.cpp
    src/ProfileBuilder.cpp
    src/Provider.cpp
    src/ServerSentEvents.cpp
    src/SystemPrompt.cpp
    src/ToolRegistry.cpp
    src/ViewImage.cpp
)

target_include_directories(hz_ai
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

# PNG images of a view, compressed with zlib (get_view_image).
find_package(ZLIB REQUIRED)

target_link_libraries(hz_ai
    PUBLIC
        nlohmann_json::nlohmann_json
    PRIVATE
        ZLIB::ZLIB
        Horizon::Geometry
        Horizon::Document
        Horizon::Drafting
        Horizon::Modeling
        Horizon::Topology
        Horizon::Math
)

add_library(Horizon::AI ALIAS hz_ai)
