# The AI assistant's core (hz::ai): a conversation in no provider's format,
# an adapter per provider wire format, the agent loop, and the CAD tools a
# model calls. No Qt: the application supplies the HTTP transport.
add_library(hz_ai STATIC
    src/AnthropicAdapter.cpp
    src/Conversation.cpp
    src/GeminiAdapter.cpp
    src/OpenAIAdapter.cpp
    src/Provider.cpp
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
)

add_library(Horizon::AI ALIAS hz_ai)
