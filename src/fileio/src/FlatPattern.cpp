#include "horizon/fileio/FlatPattern.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdio>
#include <memory>
#include <string>
#include <utility>

#include "horizon/document/Document.h"
#include "horizon/document/FeatureTree.h"
#include "horizon/drafting/DraftLine.h"
#include "horizon/drafting/DraftText.h"
#include "horizon/drafting/Layer.h"
#include "horizon/drafting/LineType.h"
#include "horizon/fileio/DxfFormat.h"
#include "horizon/modeling/SheetMetal.h"

namespace hz::io {

namespace {

constexpr const char* kOutline = "Flat Pattern";
constexpr const char* kBends = "Bend Lines";
constexpr double kRadToDeg = 57.29577951308232;

std::string angleText(double degrees) {
    char text[32];
    std::snprintf(text, sizeof text, "%.4g", degrees);
    return text;
}

}  // namespace

std::optional<FlatPattern> flatPatternOf(const doc::SheetMetalFeature& flange, std::string* why) {
    const auto strip = flange.strip(why);
    if (!strip) return std::nullopt;
    const model::SheetMetalParams params = flange.params();
    FlatPattern pattern;
    pattern.length = model::developedLength(*strip, params);
    pattern.width = flange.width();
    pattern.thickness = params.thickness;
    const std::vector<double> at = model::bendLinePositions(*strip, params);
    for (std::size_t i = 0; i < at.size() && i < strip->bendAngles.size(); ++i) {
        const double angle = strip->bendAngles[i];
        if (angle == 0.0) continue;  // a straight run: no bend to make
        pattern.bends.push_back({at[i], std::abs(angle) * kRadToDeg, angle > 0.0});
    }
    if (!(pattern.length > 0.0) || !(pattern.width > 0.0)) {
        if (why != nullptr) *why = "it unfolds to nothing";
        return std::nullopt;
    }
    return pattern;
}

void drawFlatPattern(const FlatPattern& pattern, doc::Document& into) {
    draft::LayerProperties outline;
    outline.name = kOutline;
    into.layerManager().addLayer(outline);
    draft::LayerProperties bends;
    bends.name = kBends;
    bends.lineType = static_cast<int>(draft::LineType::DashDot);
    into.layerManager().addLayer(bends);

    const double l = pattern.length;
    const double w = pattern.width;
    const math::Vec2 corners[4] = {{0.0, 0.0}, {l, 0.0}, {l, w}, {0.0, w}};
    for (int i = 0; i < 4; ++i) {
        auto edge = std::make_shared<draft::DraftLine>(corners[i], corners[(i + 1) % 4]);
        edge->setLayer(kOutline);
        into.addEntity(std::move(edge));
    }
    // A note the height of a small text, kept inside narrow blanks.
    const double height = std::min(2.5, w / 8.0);
    for (const FlatPattern::Bend& bend : pattern.bends) {
        auto line =
            std::make_shared<draft::DraftLine>(math::Vec2(bend.at, 0.0), math::Vec2(bend.at, w));
        line->setLayer(kBends);
        into.addEntity(std::move(line));
        auto note = std::make_shared<draft::DraftText>(
            math::Vec2(bend.at + height * 0.4, w / 2.0),
            std::string(bend.up ? "UP " : "DOWN ") + angleText(bend.degrees) + "\xC2\xB0", height);
        note->setLayer(kBends);
        into.addEntity(std::move(note));
    }
}

bool saveFlatPatternDxf(const std::string& path, const FlatPattern& pattern, std::string* why) {
    doc::Document drawing;
    drawFlatPattern(pattern, drawing);
    return DxfFormat::save(path, drawing, why);
}

}  // namespace hz::io
