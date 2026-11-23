#include "horizon/ai/Conversation.h"

#include <utility>

namespace hz::ai {

ContentBlock ContentBlock::makeText(std::string text) {
    ContentBlock block;
    block.kind = Kind::Text;
    block.text = std::move(text);
    return block;
}

ContentBlock ContentBlock::makeToolCall(std::string id, std::string name, nlohmann::json input) {
    ContentBlock block;
    block.kind = Kind::ToolCall;
    block.id = std::move(id);
    block.name = std::move(name);
    block.input = std::move(input);
    return block;
}

ContentBlock ContentBlock::makeToolResult(std::string callId, std::string name, std::string content,
                                          bool isError, std::vector<Image> images) {
    ContentBlock block;
    block.kind = Kind::ToolResult;
    block.id = std::move(callId);
    block.name = std::move(name);
    block.text = std::move(content);
    block.isError = isError;
    block.images = std::move(images);
    return block;
}

ContentBlock ContentBlock::makeOpaque(std::string provider, nlohmann::json raw) {
    ContentBlock block;
    block.kind = Kind::Opaque;
    block.provider = std::move(provider);
    block.raw = std::move(raw);
    return block;
}

std::string Message::text() const {
    std::string out;
    for (const auto& block : blocks) {
        if (block.kind != ContentBlock::Kind::Text || block.text.empty()) continue;
        if (!out.empty()) out += "\n\n";
        out += block.text;
    }
    return out;
}

std::vector<const ContentBlock*> Message::toolCalls() const {
    std::vector<const ContentBlock*> calls;
    for (const auto& block : blocks) {
        if (block.kind == ContentBlock::Kind::ToolCall) calls.push_back(&block);
    }
    return calls;
}

}  // namespace hz::ai
