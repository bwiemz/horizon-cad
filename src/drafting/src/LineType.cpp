#include "horizon/drafting/LineType.h"

#include <algorithm>
#include <cctype>

namespace hz::draft {

static const char* kNames[] = {
    "ByLayer", "Continuous", "Dashed", "Dotted",
    "DashDot", "Center", "Hidden", "Phantom"};

static const char* kDxfNames[] = {
    "BYLAYER", "CONTINUOUS", "DASHED", "DOT",
    "DASHDOT", "CENTER", "HIDDEN", "PHANTOM"};

const char* lineTypeName(LineType lt) {
    int idx = static_cast<int>(lt);
    if (idx < 0 || idx > 7) return "Continuous";
    return kNames[idx];
}

const char* lineTypeName(int lt) {
    return lineTypeName(static_cast<LineType>(lt));
}

LineType lineTypeFromName(const std::string& name) {
    for (int i = 0; i <= 7; ++i) {
        if (name == kNames[i]) return static_cast<LineType>(i);
    }
    return LineType::ByLayer;
}

const char* lineTypeDxfName(LineType lt) {
    int idx = static_cast<int>(lt);
    if (idx < 0 || idx > 7) return "CONTINUOUS";
    return kDxfNames[idx];
}

LineType lineTypeFromDxfName(const std::string& dxfName) {
    // Case-insensitive comparison.
    std::string upper = dxfName;
    std::transform(upper.begin(), upper.end(), upper.begin(),
                   [](unsigned char c) { return static_cast<char>(std::toupper(c)); });
    for (int i = 0; i <= 7; ++i) {
        if (upper == kDxfNames[i]) return static_cast<LineType>(i);
    }
    return LineType::Continuous;
}

}  // namespace hz::draft
