#include "horizon/fileio/SvgExport.h"

#include <array>
#include <charconv>
#include <cmath>
#include <cstdio>
#include <string_view>

#include "horizon/fileio/AtomicFile.h"
#include "horizon/math/Constants.h"

namespace hz::io {

namespace {

/// @p value with three decimals (a micrometre on paper), in the C locale.
std::string num(double value) {
    std::array<char, 64> buffer{};
    if (!std::isfinite(value)) value = 0.0;
    const auto [end, error] = std::to_chars(buffer.data(), buffer.data() + buffer.size(), value,
                                            std::chars_format::fixed, 3);
    if (error != std::errc()) return "0";
    std::string text(buffer.data(), end);
    // "12.500" → "12.5", "3.000" → "3": the file is a third smaller.
    while (!text.empty() && text.back() == '0') text.pop_back();
    if (!text.empty() && text.back() == '.') text.pop_back();
    return text == "-0" ? "0" : text;
}

std::string colour(uint32_t argb) {
    std::array<char, 8> buffer{};
    std::snprintf(buffer.data(), buffer.size(), "#%06x", static_cast<unsigned>(argb & 0xFFFFFFu));
    return buffer.data();
}

std::string escaped(std::string_view text) {
    std::string out;
    out.reserve(text.size());
    for (const char c : text) {
        switch (c) {
            case '&':
                out += "&amp;";
                break;
            case '<':
                out += "&lt;";
                break;
            case '>':
                out += "&gt;";
                break;
            case '"':
                out += "&quot;";
                break;
            default:
                out += c;
        }
    }
    return out;
}

}  // namespace

std::string SvgExport::toString(const draft::PlotScene& scene, const draft::PlotLayout& layout) {
    const draft::PlotTransform t = draft::plotTransform(scene, layout);
    const std::string w = num(layout.paperWidthMm);
    const std::string h = num(layout.paperHeightMm);
    std::string svg;
    svg += "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n";
    svg += "<svg xmlns=\"http://www.w3.org/2000/svg\" width=\"" + w + "mm\" height=\"" + h +
           "mm\" viewBox=\"0 0 " + w + " " + h + "\">\n";
    svg += "<rect width=\"" + w + "\" height=\"" + h + "\" fill=\"white\"/>\n";
    svg += "<g fill=\"none\" stroke-linejoin=\"round\">\n";
    for (const auto& stroke : scene.strokes) {
        svg += stroke.closed ? "<polygon points=\"" : "<polyline points=\"";
        for (size_t i = 0; i < stroke.points.size(); ++i) {
            const math::Vec2 p = t.toPaper(stroke.points[i]);
            if (i > 0) svg += ' ';
            svg += num(p.x) + "," + num(p.y);
        }
        svg += "\" stroke=\"" + colour(draft::plotColor(stroke.color, layout.monochrome)) +
               "\" stroke-width=\"" + num(draft::plotWeightMm(stroke.width)) + "\"";
        const auto dashes = draft::plotDashMm(stroke.lineType);
        if (dashes.empty()) {
            svg += " stroke-linecap=\"round\"";
        } else {
            svg += " stroke-dasharray=\"";
            for (size_t i = 0; i < dashes.size(); ++i) {
                if (i > 0) svg += ' ';
                svg += num(dashes[i]);
            }
            svg += "\"";
        }
        svg += "/>\n";
    }
    svg += "</g>\n";
    for (const auto& text : scene.texts) {
        const math::Vec2 p = t.toPaper(text.position);
        // The height is the capitals'; the font's size is its em, about a
        // third more.
        const double em = text.height * t.scale / 0.7;
        const char* anchor = text.alignment == draft::TextAlignment::Left     ? "start"
                             : text.alignment == draft::TextAlignment::Center ? "middle"
                                                                              : "end";
        svg += "<text x=\"" + num(p.x) + "\" y=\"" + num(p.y) +
               "\" font-family=\"sans-serif\" font-size=\"" + num(em) + "\" text-anchor=\"" +
               anchor + "\" fill=\"" + colour(draft::plotColor(text.color, layout.monochrome)) +
               "\"";
        if (std::abs(text.rotation) > 1e-12) {
            // Counter-clockwise in the drawing is clockwise on the page, whose
            // y runs down.
            svg += " transform=\"rotate(" + num(-text.rotation * math::kRadToDeg) + " " + num(p.x) +
                   " " + num(p.y) + ")\"";
        }
        svg += ">" + escaped(text.text) + "</text>\n";
    }
    svg += "</svg>\n";
    return svg;
}

bool SvgExport::save(const std::string& path, const draft::PlotScene& scene,
                     const draft::PlotLayout& layout, std::string* error) {
    return writeFileAtomically(pathFromUtf8(path), toString(scene, layout), error);
}

}  // namespace hz::io
