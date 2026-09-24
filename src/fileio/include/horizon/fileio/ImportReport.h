#pragma once

#include <string>
#include <vector>

namespace hz::io {

/// What reading a file left out or changed, in the user's terms — shown
/// before a save could drop the missing items for good.
struct ImportReport {
    /// Items not brought in, each with why: "entity 7 (spline): …".
    std::vector<std::string> skipped;
    /// Items brought in, but not exactly as they were.
    std::vector<std::string> approximated;

    bool empty() const { return skipped.empty() && approximated.empty(); }

    /// "2 items were left out, and 1 was approximated."
    std::string summary() const {
        const auto count = [](size_t n, const char* one, const char* many) {
            return std::to_string(n) + " " + (n == 1 ? one : many);
        };
        std::string text;
        if (!skipped.empty()) {
            text = count(skipped.size(), "item was", "items were") + " left out";
        }
        if (!approximated.empty()) {
            if (!text.empty()) text += ", and ";
            text += count(approximated.size(), "item was", "items were") + " approximated";
        }
        return text.empty() ? text : text + ".";
    }
};

}  // namespace hz::io
