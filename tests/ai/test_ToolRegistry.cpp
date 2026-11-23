#include <gtest/gtest.h>

#include <stdexcept>

#include "horizon/ai/Http.h"
#include "horizon/ai/ToolRegistry.h"

using hz::ai::ToolAccess;
using hz::ai::ToolRegistry;
using hz::ai::ToolResult;
using hz::ai::ToolSpec;
using nlohmann::json;

namespace {

ToolSpec spec(const std::string& name, json required = json::array()) {
    return {name, "test tool",
            json{{"type", "object"},
                 {"properties", {{"x", {{"type", "number"}}}}},
                 {"required", std::move(required)}}};
}

}  // namespace

TEST(ToolRegistry, RunsAHandler) {
    ToolRegistry registry;
    registry.add(spec("double"), ToolAccess::Read, [](const json& in) {
        return ToolResult::ok(json{{"y", in.value("x", 0.0) * 2}});
    });
    const ToolResult result = registry.run("double", json{{"x", 21}});
    EXPECT_FALSE(result.isError);
    EXPECT_EQ(json::parse(result.content)["y"], 42.0);
    EXPECT_EQ(registry.access("double"), ToolAccess::Read);
    ASSERT_EQ(registry.specs().size(), 1u);
}

TEST(ToolRegistry, UnknownToolIsAnError) {
    ToolRegistry registry;
    const ToolResult result = registry.run("nope", json::object());
    EXPECT_TRUE(result.isError);
    EXPECT_NE(result.content.find("nope"), std::string::npos);
    EXPECT_FALSE(registry.access("nope").has_value());
}

TEST(ToolRegistry, MissingRequiredIsAnError) {
    ToolRegistry registry;
    bool ran = false;
    registry.add(spec("needs_x", json::array({"x"})), ToolAccess::Modify, [&ran](const json&) {
        ran = true;
        return ToolResult::message("done");
    });
    const ToolResult result = registry.run("needs_x", json::object());
    EXPECT_TRUE(result.isError);
    EXPECT_NE(result.content.find("\"x\""), std::string::npos);
    EXPECT_FALSE(ran);
    // A null counts as missing.
    EXPECT_TRUE(registry.run("needs_x", json{{"x", nullptr}}).isError);
}

TEST(ToolRegistry, HandlerExceptionBecomesAnError) {
    ToolRegistry registry;
    registry.add(spec("throws"), ToolAccess::Read,
                 [](const json&) -> ToolResult { throw std::runtime_error("kaboom"); });
    const ToolResult result = registry.run("throws", json::object());
    EXPECT_TRUE(result.isError);
    EXPECT_NE(result.content.find("kaboom"), std::string::npos);

    // A nlohmann type error, as a handler reading a string as a number throws.
    registry.add(spec("types"), ToolAccess::Read, [](const json& in) {
        return ToolResult::ok(json{{"y", in.at("x").get<double>()}});
    });
    EXPECT_TRUE(registry.run("types", json{{"x", "ten"}}).isError);
}

TEST(ToolRegistry, NonObjectInputIsAnError) {
    ToolRegistry registry;
    registry.add(spec("any"), ToolAccess::Read,
                 [](const json&) { return ToolResult::message("ok"); });
    EXPECT_TRUE(registry.run("any", json::array({1, 2})).isError);
    EXPECT_TRUE(registry.run("any", json("text")).isError);
    // No arguments at all is an empty object.
    EXPECT_FALSE(registry.run("any", json()).isError);
}

TEST(ToolRegistry, InvalidArgumentsFromAnAdapterAreAnError) {
    ToolRegistry registry;
    registry.add(spec("any"), ToolAccess::Read,
                 [](const json&) { return ToolResult::message("ok"); });
    const ToolResult result = registry.run("any", json{{"_invalid_arguments", "{oops"}});
    EXPECT_TRUE(result.isError);
    EXPECT_NE(result.content.find("{oops"), std::string::npos);
}

TEST(ToolRegistry, LaterToolOfTheSameNameReplaces) {
    ToolRegistry registry;
    registry.add(spec("t"), ToolAccess::Read,
                 [](const json&) { return ToolResult::message("one"); });
    registry.add(spec("t"), ToolAccess::Modify,
                 [](const json&) { return ToolResult::message("two"); });
    EXPECT_EQ(registry.specs().size(), 1u);
    EXPECT_EQ(registry.run("t", json::object()).content, "two");
    EXPECT_EQ(registry.access("t"), ToolAccess::Modify);
}

TEST(ToolResultTest, InvalidUtf8IsReplacedNotThrown) {
    const ToolResult result = ToolResult::ok(json{{"text", std::string("a\xff"
                                                                       "b")}});
    EXPECT_FALSE(result.isError);
    EXPECT_FALSE(result.content.empty());
}

TEST(HttpRequestTest, HeaderLookupIgnoresCase) {
    hz::ai::HttpRequest request;
    request.headers = {{"Content-Type", "application/json"}, {"x-api-key", "k"}};
    EXPECT_EQ(request.header("content-type"), "application/json");
    EXPECT_EQ(request.header("X-API-KEY"), "k");
    EXPECT_EQ(request.header("missing"), "");
}
