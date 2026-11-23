#include "horizon/ai/ToolRegistry.h"

#include <algorithm>
#include <cctype>
#include <exception>
#include <utility>

#include "horizon/ai/Http.h"

namespace hz::ai {

HttpTransport::~HttpTransport() = default;

std::string HttpRequest::header(const std::string& name) const {
    const auto lower = [](std::string s) {
        std::transform(s.begin(), s.end(), s.begin(),
                       [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
        return s;
    };
    const std::string wanted = lower(name);
    for (const auto& [key, value] : headers) {
        if (lower(key) == wanted) return value;
    }
    return {};
}

ToolResult ToolResult::ok(const nlohmann::json& value, bool changedModel) {
    // Replace, not throw, on invalid UTF-8 (an entity's text, a layer name
    // read from a file): the model gets the rest.
    return {value.dump(-1, ' ', false, nlohmann::json::error_handler_t::replace), false,
            changedModel, changedModel ? 1 : 0};
}

ToolResult ToolResult::message(std::string text, bool changedModel) {
    return {std::move(text), false, changedModel, changedModel ? 1 : 0};
}

ToolResult ToolResult::error(std::string message) {
    return {std::move(message), true, false, 0};
}

void ToolRegistry::add(ToolSpec spec, ToolAccess access, Handler handler) {
    for (auto& entry : m_entries) {
        if (entry.spec.name == spec.name) {
            entry = {std::move(spec), access, std::move(handler)};
            return;
        }
    }
    m_entries.push_back({std::move(spec), access, std::move(handler)});
}

std::vector<ToolSpec> ToolRegistry::specs() const {
    std::vector<ToolSpec> out;
    out.reserve(m_entries.size());
    for (const auto& entry : m_entries) out.push_back(entry.spec);
    return out;
}

std::optional<ToolAccess> ToolRegistry::access(const std::string& name) const {
    const Entry* entry = find(name);
    if (!entry) return std::nullopt;
    return entry->access;
}

const ToolRegistry::Entry* ToolRegistry::find(const std::string& name) const {
    for (const auto& entry : m_entries) {
        if (entry.spec.name == name) return &entry;
    }
    return nullptr;
}

ToolResult ToolRegistry::run(const std::string& name, const nlohmann::json& input) const {
    const Entry* entry = find(name);
    if (!entry) return ToolResult::error("There is no tool named \"" + name + "\".");
    // Some models send no arguments at all for a tool that takes none.
    const nlohmann::json args = input.is_null() ? nlohmann::json::object() : input;
    if (!args.is_object()) {
        return ToolResult::error(name + ": the arguments must be a JSON object.");
    }
    // An adapter that could not parse the arguments says so here.
    if (const auto bad = args.find("_invalid_arguments"); bad != args.end()) {
        return ToolResult::error(name + ": the arguments were not valid JSON: " +
                                 (bad->is_string() ? bad->get<std::string>() : bad->dump()));
    }
    const auto& schema = entry->spec.inputSchema;
    if (const auto required = schema.find("required");
        required != schema.end() && required->is_array()) {
        for (const auto& property : *required) {
            if (!property.is_string()) continue;
            const auto key = property.get<std::string>();
            if (!args.contains(key) || args[key].is_null()) {
                std::string given;
                for (auto it = args.begin(); it != args.end(); ++it) {
                    given += (given.empty() ? "" : ", ") + it.key();
                }
                return ToolResult::error(name + ": the argument \"" + key +
                                         "\" is required (given: " +
                                         (given.empty() ? std::string("nothing") : given) + ").");
            }
        }
    }
    try {
        return entry->handler(args);
    } catch (const std::exception& e) {
        return ToolResult::error(name + " failed: " + e.what());
    } catch (...) {
        return ToolResult::error(name + " failed.");
    }
}

}  // namespace hz::ai
