#include "horizon/modeling/HoleCallout.h"

#include <cmath>
#include <cstdio>
#include <string>
#include <vector>

namespace hz::model {

namespace {

constexpr const char* kDiameter = "\xE2\x8C\x80";  // ⌀
constexpr const char* kTimes = "\xC3\x97";         // ×
constexpr const char* kDegree = "\xC2\xB0";        // °

std::string angle(double degrees) {
    char text[32];
    std::snprintf(text, sizeof text, "%.4g", degrees);
    return text;
}

}  // namespace

std::vector<std::string> holeCalloutLines(const HoleCallout& callout,
                                          const std::function<std::string(double)>& length) {
    std::string first;
    if (callout.count > 1) first = std::to_string(callout.count) + kTimes + " ";
    first += kDiameter + length(callout.diameter);
    switch (callout.depth) {
        case HoleCallout::Depth::Through:
            first += " THRU";
            break;
        case HoleCallout::Depth::Blind:
            first += std::string(" ") + kTimes + " " + length(callout.holeDepth) + " DEEP";
            break;
        case HoleCallout::Depth::UpToFace:
            break;
    }
    std::vector<std::string> lines{first};
    switch (callout.kind) {
        case HoleCallout::Kind::Counterbore:
            lines.push_back(std::string("CBORE ") + kDiameter + length(callout.boreDiameter) + " " +
                            kTimes + " " + length(callout.boreDepth) + " DEEP");
            break;
        case HoleCallout::Kind::Countersink:
            lines.push_back(std::string("CSK ") + kDiameter + length(callout.sinkDiameter) + " " +
                            kTimes + " " + angle(callout.sinkAngleDegrees) + kDegree);
            break;
        case HoleCallout::Kind::Simple:
            break;
    }
    return lines;
}

std::string withoutPiece(const std::string& edgeName) {
    const std::size_t at = edgeName.rfind("/piece:");
    if (at == std::string::npos) return edgeName;
    for (std::size_t i = at + 7; i < edgeName.size(); ++i) {
        if (edgeName[i] < '0' || edgeName[i] > '9') return edgeName;
    }
    return edgeName.size() > at + 7 ? edgeName.substr(0, at) : edgeName;
}

}  // namespace hz::model
