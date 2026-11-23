#pragma once

// A sheet-metal part's flat pattern: the blank a shop cuts before bending
// it, drawn at full size and written as a DXF.

#include <optional>
#include <string>
#include <vector>

namespace hz::doc {
class Document;
class SheetMetalFeature;
}  // namespace hz::doc

namespace hz::io {

struct FlatPattern {
    double length = 0.0;  ///< the developed length, along the strip
    double width = 0.0;   ///< across it
    double thickness = 0.0;
    /// Each bend: where its centre line crosses the blank (from the start of
    /// the length), its angle in degrees, and whether it bends up (toward
    /// the material side) or down.
    struct Bend {
        double at = 0.0;
        double degrees = 0.0;
        bool up = true;
    };
    std::vector<Bend> bends;
};

/// The flat pattern of @p flange; nullopt, with why, when it folds into none.
std::optional<FlatPattern> flatPatternOf(const doc::SheetMetalFeature& flange, std::string* why);

/// @p pattern drawn into @p into at full size: its outline on the layer "Flat
/// Pattern", each bend's centre line (dash-dot) on "Bend Lines" with a note
/// of its direction and angle ("UP 90°").
void drawFlatPattern(const FlatPattern& pattern, doc::Document& into);

/// @p pattern written to @p path as a DXF, at 1:1, for cutting. False, with
/// why.
bool saveFlatPatternDxf(const std::string& path, const FlatPattern& pattern, std::string* why);

}  // namespace hz::io
