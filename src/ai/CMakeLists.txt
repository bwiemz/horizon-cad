# The AI assistant's core (hz::ai): a conversation in no provider's format,
# an adapter per provider wire format, the agent loop, and the CAD tools a
# model calls. No Qt: the application supplies the HTTP transport.
add_library(hz_ai STATIC
    src/AgentSession.cpp
    src/AnthropicAdapter.cpp
    src/CadDraftTools.cpp
    src/CadModelTools.cpp
    src/CadQueryTools.cpp
    src/CadTools.cpp
    src/Conversation.cpp
    src/GeminiAdapter.cpp
    src/OpenAIAdapter.cpp
    src/ProfileBuilder.cpp
    src/Provider.cpp
    src/SystemPrompt.cpp
    src/ToolRegistry.cpp
)

target_include_directories(hz_ai
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

target_link_libraries(hz_ai
    PUBLIC
        nlohmann_json::nlohmann_json
    PRIVATE
        Horizon::Document
        Horizon::Drafting
        Horizon::Modeling
        Horizon::Topology
        Horizon::Math
)

add_library(Horizon::AI ALIAS hz_ai)
