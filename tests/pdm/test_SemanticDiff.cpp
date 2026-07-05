#include <gtest/gtest.h>

#include <string>
#include <vector>

#include "horizon/pdm/SemanticDiff.h"

using hz::pdm::ChangeKind;
using hz::pdm::diffJson;
using hz::pdm::formatChanges;
using hz::pdm::JsonChange;

namespace {
const JsonChange* find(const std::vector<JsonChange>& v, const std::string& path) {
    for (const auto& c : v) {
        if (c.path == path) return &c;
    }
    return nullptr;
}
}  // namespace

// Identical documents produce no changes.
TEST(SemanticDiffTest, IdenticalIsEmpty) {
    const std::string doc = R"({"a":1,"b":[1,2,3]})";
    EXPECT_TRUE(diffJson(doc, doc).empty());
}

// A changed scalar is reported as Modified with before/after values.
TEST(SemanticDiffTest, ModifiedScalar) {
    const auto changes = diffJson(R"({"radius":5.0})", R"({"radius":6.0})");
    ASSERT_EQ(changes.size(), 1u);
    const JsonChange* c = find(changes, "/radius");
    ASSERT_NE(c, nullptr);
    EXPECT_EQ(c->kind, ChangeKind::Modified);
    EXPECT_EQ(c->before, "5.0");
    EXPECT_EQ(c->after, "6.0");
}

// Keys present only on one side are Added / Removed.
TEST(SemanticDiffTest, AddedAndRemovedKeys) {
    const auto changes = diffJson(R"({"keep":1,"gone":2})", R"({"keep":1,"new":3})");
    ASSERT_EQ(changes.size(), 2u);

    const JsonChange* added = find(changes, "/new");
    ASSERT_NE(added, nullptr);
    EXPECT_EQ(added->kind, ChangeKind::Added);
    EXPECT_EQ(added->after, "3");

    const JsonChange* removed = find(changes, "/gone");
    ASSERT_NE(removed, nullptr);
    EXPECT_EQ(removed->kind, ChangeKind::Removed);
    EXPECT_EQ(removed->before, "2");
}

// Nested feature-tree edits are located by their full path, and array growth is
// reported positionally (an appended feature is Added at its index).
TEST(SemanticDiffTest, NestedFeatureTreeDiff) {
    const std::string before = R"({"features":[{"type":"box","w":10}]})";
    const std::string after = R"({"features":[{"type":"box","w":12},{"type":"fillet","r":2}]})";
    const auto changes = diffJson(before, after);

    const JsonChange* w = find(changes, "/features/0/w");
    ASSERT_NE(w, nullptr);
    EXPECT_EQ(w->kind, ChangeKind::Modified);
    EXPECT_EQ(w->before, "10");
    EXPECT_EQ(w->after, "12");

    const JsonChange* added = find(changes, "/features/1");
    ASSERT_NE(added, nullptr);
    EXPECT_EQ(added->kind, ChangeKind::Added);
    EXPECT_NE(added->after.find("fillet"), std::string::npos);
}

// Non-JSON inputs fall back to a single whole-document Modified change.
TEST(SemanticDiffTest, NonJsonFallback) {
    const auto changes = diffJson("not json", "also not json");
    ASSERT_EQ(changes.size(), 1u);
    EXPECT_EQ(changes[0].kind, ChangeKind::Modified);
    EXPECT_EQ(changes[0].path, "");
}

// The formatter renders each change kind with its sigil and path.
TEST(SemanticDiffTest, FormatsHumanReadable) {
    const auto changes = diffJson(R"({"a":1,"gone":2})", R"({"a":2,"new":3})");
    const std::string text = formatChanges(changes);
    EXPECT_NE(text.find("~ /a: 1 -> 2"), std::string::npos);
    EXPECT_NE(text.find("+ /new: 3"), std::string::npos);
    EXPECT_NE(text.find("- /gone: 2"), std::string::npos);
}
