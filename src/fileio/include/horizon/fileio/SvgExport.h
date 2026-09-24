#pragma once

#include <string>

#include "horizon/drafting/PlotScene.h"

namespace hz::io {

/// A drawing plotted to SVG: a page the paper's size in millimetres, one
/// polyline per stroke with its weight, dash pattern and colour, and text at
/// its height. Numbers are written in the C locale, whatever the user's.
class SvgExport {
public:
    static std::string toString(const draft::PlotScene& scene, const draft::PlotLayout& layout);

    /// Write it to @p path, atomically. False with @p error on failure.
    static bool save(const std::string& path, const draft::PlotScene& scene,
                     const draft::PlotLayout& layout, std::string* error = nullptr);
};

}  // namespace hz::io
