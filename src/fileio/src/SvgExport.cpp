#include "horizon/fileio/SvgExport.h"

#include <algorithm>
#include <array>
#include <charconv>
#include <cmath>
#include <cstddef>
#include <cstdio>
#include <string>
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

/// The length of the UTF-8 character at @p at in @p text; 0 when the bytes
/// there are not one (a stray byte, a surrogate or an overlong form).
size_t utf8Length(std::string_view text, size_t at) {
    const auto byte = [&text](size_t i) {
        return i < text.size() ? static_cast<unsigned char>(text[i]) : 0u;
    };
    const auto continues = [](unsigned b) { return (b & 0xC0u) == 0x80u; };
    const unsigned lead = byte(at);
    const unsigned second = byte(at + 1);
    if (lead >= 0xC2 && lead <= 0xDF) return continues(second) ? 2 : 0;
    if (lead >= 0xE0 && lead <= 0xEF) {
        const bool whole = continues(second) && continues(byte(at + 2)) &&
                           (lead != 0xE0 || second >= 0xA0) && (lead != 0xED || second < 0xA0);
        return whole ? 3 : 0;
    }
    if (lead >= 0xF0 && lead <= 0xF4) {
        const bool whole = continues(second) && continues(byte(at + 2)) &&
                           continues(byte(at + 3)) && (lead != 0xF0 || second >= 0x90) &&
                           (lead != 0xF4 || second < 0x90);
        return whole ? 4 : 0;
    }
    return 0;
}

/// @p text as XML character data. Text read from a DXF can hold what XML
/// has no character for: a control character (\U+0001, a raw 0x0B) made the
/// file not well-formed, and a reader refuses all of it. Such a control
/// character is left out; anything else that is not a character, a byte
/// that is not UTF-8 among them, is U+FFFD.
std::string escaped(std::string_view text) {
    std::string out;
    out.reserve(text.size());
    for (size_t i = 0; i < text.size();) {
        const char c = text[i];
        if (static_cast<unsigned char>(c) >= 0x80) {
            const size_t length = utf8Length(text, i);
            const std::string_view character = text.substr(i, std::max<size_t>(length, 1));
            // U+FFFE and U+FFFF are UTF-8, but not XML characters.
            const bool xmlHasIt =
                length > 0 && character != "\xEF\xBF\xBE" && character != "\xEF\xBF\xBF";
            out += xmlHasIt ? character : std::string_view("\xEF\xBF\xBD");
            i += character.size();
            continue;
        }
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
                // Of the controls, XML has only tab, line feed and return.
                if (static_cast<unsigned char>(c) >= 0x20 || c == '\t' || c == '\n' || c == '\r') {
                    out += c;
                }
        }
        ++i;
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
