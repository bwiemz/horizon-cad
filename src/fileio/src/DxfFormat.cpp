#include "horizon/fileio/DxfFormat.h"

#include <algorithm>
#include <charconv>
#include <cmath>
#include <exception>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <map>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <string_view>
#include <system_error>
#include <utility>
#include <vector>

#include "DxfCodec.h"
#include "horizon/document/Document.h"
#include "horizon/drafting/DimensionStyle.h"
#include "horizon/drafting/DraftAngularDimension.h"
#include "horizon/drafting/DraftArc.h"
#include "horizon/drafting/DraftBlockRef.h"
#include "horizon/drafting/DraftCircle.h"
#include "horizon/drafting/DraftDimension.h"
#include "horizon/drafting/DraftEllipse.h"
#include "horizon/drafting/DraftHatch.h"
#include "horizon/drafting/DraftLeader.h"
#include "horizon/drafting/DraftLine.h"
#include "horizon/drafting/DraftLinearDimension.h"
#include "horizon/drafting/DraftPolyline.h"
#include "horizon/drafting/DraftRadialDimension.h"
#include "horizon/drafting/DraftRectangle.h"
#include "horizon/drafting/DraftSpline.h"
#include "horizon/drafting/DraftText.h"
#include "horizon/drafting/Layer.h"
#include "horizon/drafting/LineType.h"
#include "horizon/fileio/AtomicFile.h"
#include "horizon/math/BoundingBox.h"
#include "horizon/math/Constants.h"

namespace hz::io {

namespace {

using dxf::aciToArgb;
using dxf::argbToAci;

// ===========================================================================
// DXF Group Code Writer
// ===========================================================================

void writeGroup(std::ostream& out, int code, const std::string& value) {
    // A line break inside a value would end it early and shift every group
    // after it, so none is written. Text values encode theirs as ^J first.
    if (value.find_first_of("\r\n") == std::string::npos) {
        out << "  " << code << "\n" << value << "\n";
        return;
    }
    std::string flat = value;
    const auto isBreak = [](char c) { return c == '\r' || c == '\n'; };
    std::replace_if(flat.begin(), flat.end(), isBreak, ' ');
    out << "  " << code << "\n" << flat << "\n";
}

void writeGroup(std::ostream& out, int code, int value) {
    out << "  " << code << "\n" << value << "\n";
}

void writeGroup(std::ostream& out, int code, double value) {
    out << "  " << code << "\n" << std::fixed << std::setprecision(6) << value << "\n";
}

int g_handleCounter = 0;

std::string nextHandle() {
    std::ostringstream ss;
    ss << std::hex << std::uppercase << g_handleCounter++;
    return ss.str();
}

// ===========================================================================
// Common entity property writer
// ===========================================================================

void writeCommonProps(std::ostream& out, const draft::DraftEntity& entity) {
    writeGroup(out, 5, nextHandle());
    writeGroup(out, 8, entity.layer());
    uint32_t c = entity.color();
    if (c != 0x00000000) {
        writeGroup(out, 62, argbToAci(c));
        // A colour the index does not hold goes as a true colour too.
        if (!dxf::isAciColor(c)) writeGroup(out, 420, static_cast<int>(c & 0xFFFFFFu));
    }
    if (entity.lineType() > 0) {
        writeGroup(
            out, 6,
            std::string(draft::lineTypeDxfName(static_cast<draft::LineType>(entity.lineType()))));
    }
    if (entity.lineWidth() > 0.0) {
        writeGroup(out, 370, static_cast<int>(entity.lineWidth() * 100.0));
    }
}

// ===========================================================================
// Entity export functions
// ===========================================================================

void writeLine(std::ostream& out, const draft::DraftLine& line) {
    writeGroup(out, 0, std::string("LINE"));
    writeCommonProps(out, line);
    writeGroup(out, 100, std::string("AcDbEntity"));
    writeGroup(out, 100, std::string("AcDbLine"));
    writeGroup(out, 10, line.start().x);
    writeGroup(out, 20, line.start().y);
    writeGroup(out, 30, 0.0);
    writeGroup(out, 11, line.end().x);
    writeGroup(out, 21, line.end().y);
    writeGroup(out, 31, 0.0);
}

void writeCircle(std::ostream& out, const draft::DraftCircle& circle) {
    writeGroup(out, 0, std::string("CIRCLE"));
    writeCommonProps(out, circle);
    writeGroup(out, 100, std::string("AcDbEntity"));
    writeGroup(out, 100, std::string("AcDbCircle"));
    writeGroup(out, 10, circle.center().x);
    writeGroup(out, 20, circle.center().y);
    writeGroup(out, 30, 0.0);
    writeGroup(out, 40, circle.radius());
}

void writeArc(std::ostream& out, const draft::DraftArc& arc) {
    writeGroup(out, 0, std::string("ARC"));
    writeCommonProps(out, arc);
    writeGroup(out, 100, std::string("AcDbEntity"));
    writeGroup(out, 100, std::string("AcDbCircle"));
    writeGroup(out, 10, arc.center().x);
    writeGroup(out, 20, arc.center().y);
    writeGroup(out, 30, 0.0);
    writeGroup(out, 40, arc.radius());
    writeGroup(out, 100, std::string("AcDbArc"));
    writeGroup(out, 50, arc.startAngle() * math::kRadToDeg);
    writeGroup(out, 51, arc.endAngle() * math::kRadToDeg);
}

void writeLwPolyline(std::ostream& out, const std::vector<math::Vec2>& pts, bool closed,
                     const draft::DraftEntity& entity) {
    writeGroup(out, 0, std::string("LWPOLYLINE"));
    writeCommonProps(out, entity);
    writeGroup(out, 100, std::string("AcDbEntity"));
    writeGroup(out, 100, std::string("AcDbPolyline"));
    writeGroup(out, 90, static_cast<int>(pts.size()));
    writeGroup(out, 70, closed ? 1 : 0);
    for (const auto& p : pts) {
        writeGroup(out, 10, p.x);
        writeGroup(out, 20, p.y);
    }
}

void writeRectangle(std::ostream& out, const draft::DraftRectangle& rect) {
    auto c = rect.corners();
    std::vector<math::Vec2> pts = {c[0], c[1], c[2], c[3]};
    writeLwPolyline(out, pts, true, rect);
}

void writePolyline(std::ostream& out, const draft::DraftPolyline& poly) {
    writeLwPolyline(out, poly.points(), poly.closed(), poly);
}

void writeText(std::ostream& out, const draft::DraftText& text) {
    writeGroup(out, 0, std::string("TEXT"));
    writeCommonProps(out, text);
    writeGroup(out, 100, std::string("AcDbEntity"));
    writeGroup(out, 100, std::string("AcDbText"));
    writeGroup(out, 10, text.position().x);
    writeGroup(out, 20, text.position().y);
    writeGroup(out, 30, 0.0);
    writeGroup(out, 40, text.textHeight());
    writeGroup(out, 1, dxf::encodeText(text.text()));
    if (text.rotation() != 0.0) {
        writeGroup(out, 50, text.rotation() * math::kRadToDeg);
    }
    int hJust = 0;
    if (text.alignment() == draft::TextAlignment::Center)
        hJust = 1;
    else if (text.alignment() == draft::TextAlignment::Right)
        hJust = 2;
    if (hJust != 0) {
        writeGroup(out, 72, hJust);
        // Alignment point (same as insertion for simple text).
        writeGroup(out, 11, text.position().x);
        writeGroup(out, 21, text.position().y);
        writeGroup(out, 31, 0.0);
    }
    writeGroup(out, 100, std::string("AcDbText"));
}

void writeSpline(std::ostream& out, const draft::DraftSpline& spline) {
    writeGroup(out, 0, std::string("SPLINE"));
    writeCommonProps(out, spline);
    writeGroup(out, 100, std::string("AcDbEntity"));
    writeGroup(out, 100, std::string("AcDbSpline"));

    const auto& cps = spline.controlPoints();
    int n = static_cast<int>(cps.size());
    int degree = 3;
    int flags = spline.closed() ? 1 : 0;

    writeGroup(out, 70, flags);
    writeGroup(out, 71, degree);

    // Generate clamped uniform knot vector for open spline.
    int numKnots = n + degree + 1;
    writeGroup(out, 72, numKnots);
    writeGroup(out, 73, n);

    if (spline.closed()) {
        // Periodic knot vector.
        for (int i = 0; i < numKnots; ++i) {
            writeGroup(out, 40, static_cast<double>(i));
        }
    } else {
        // Clamped knot vector.
        int numInternal = n - degree - 1;
        for (int i = 0; i <= degree; ++i) writeGroup(out, 40, 0.0);
        for (int i = 1; i <= numInternal; ++i) {
            writeGroup(out, 40, static_cast<double>(i) / (numInternal + 1));
        }
        for (int i = 0; i <= degree; ++i) writeGroup(out, 40, 1.0);
    }

    for (const auto& cp : cps) {
        writeGroup(out, 10, cp.x);
        writeGroup(out, 20, cp.y);
        writeGroup(out, 30, 0.0);
    }
}

void writeHatch(std::ostream& out, const draft::DraftHatch& hatch) {
    writeGroup(out, 0, std::string("HATCH"));
    writeCommonProps(out, hatch);
    writeGroup(out, 100, std::string("AcDbEntity"));
    writeGroup(out, 100, std::string("AcDbHatch"));
    // Elevation point.
    writeGroup(out, 10, 0.0);
    writeGroup(out, 20, 0.0);
    writeGroup(out, 30, 0.0);
    // Extrusion direction.
    writeGroup(out, 210, 0.0);
    writeGroup(out, 220, 0.0);
    writeGroup(out, 230, 1.0);

    // Pattern name and type.
    bool isSolid = (hatch.pattern() == draft::HatchPattern::Solid);
    if (isSolid) {
        writeGroup(out, 2, std::string("SOLID"));
    } else if (hatch.pattern() == draft::HatchPattern::CrossHatch) {
        writeGroup(out, 2, std::string("ANSI37"));
    } else {
        writeGroup(out, 2, std::string("ANSI31"));
    }
    writeGroup(out, 70, isSolid ? 1 : 0);
    writeGroup(out, 71, 0);  // Non-associative.

    // Boundary path.
    const auto& boundary = hatch.boundary();
    writeGroup(out, 91, 1);  // 1 boundary path.
    writeGroup(out, 92, 2);  // Polyline boundary type.
    writeGroup(out, 72, 0);  // No bulge.
    writeGroup(out, 73, 1);  // Closed.
    writeGroup(out, 93, static_cast<int>(boundary.size()));
    for (const auto& pt : boundary) {
        writeGroup(out, 10, pt.x);
        writeGroup(out, 20, pt.y);
    }
    writeGroup(out, 97, 0);  // No source boundary objects.

    // Hatch style and pattern definition.
    writeGroup(out, 75, 0);  // Normal hatch style.
    writeGroup(out, 76, 1);  // Predefined pattern type.
    writeGroup(out, 52, hatch.angle() * math::kRadToDeg);
    writeGroup(out, 41, hatch.spacing());
    writeGroup(out, 78, 0);  // Number of pattern definition lines (0 for predefined).
}

void writeInsert(std::ostream& out, const draft::DraftBlockRef& ref) {
    writeGroup(out, 0, std::string("INSERT"));
    writeCommonProps(out, ref);
    writeGroup(out, 100, std::string("AcDbEntity"));
    writeGroup(out, 100, std::string("AcDbBlockReference"));
    writeGroup(out, 2, ref.blockName());
    writeGroup(out, 10, ref.insertPos().x);
    writeGroup(out, 20, ref.insertPos().y);
    writeGroup(out, 30, 0.0);
    writeGroup(out, 41, ref.uniformScale());
    writeGroup(out, 42, ref.uniformScale());
    writeGroup(out, 43, ref.uniformScale());
    if (ref.rotation() != 0.0) {
        writeGroup(out, 50, ref.rotation() * math::kRadToDeg);
    }
}

void writeEllipse(std::ostream& out, const draft::DraftEllipse& ellipse) {
    writeGroup(out, 0, std::string("ELLIPSE"));
    writeCommonProps(out, ellipse);
    writeGroup(out, 100, std::string("AcDbEntity"));
    writeGroup(out, 100, std::string("AcDbEllipse"));
    // Center point.
    writeGroup(out, 10, ellipse.center().x);
    writeGroup(out, 20, ellipse.center().y);
    writeGroup(out, 30, 0.0);
    // Endpoint of major axis relative to center.
    double cosR = std::cos(ellipse.rotation());
    double sinR = std::sin(ellipse.rotation());
    writeGroup(out, 11, ellipse.semiMajor() * cosR);
    writeGroup(out, 21, ellipse.semiMajor() * sinR);
    writeGroup(out, 31, 0.0);
    // Ratio of minor to major axis.
    double ratio = (ellipse.semiMajor() > 1e-12) ? ellipse.semiMinor() / ellipse.semiMajor() : 1.0;
    writeGroup(out, 40, ratio);
    // Start and end parameters (full ellipse = 0 to 2*PI).
    writeGroup(out, 41, 0.0);
    writeGroup(out, 42, math::kTwoPi);
}

// Export a dimension entity as decomposed LINE + TEXT entities.
void writeDimensionAsGeometry(std::ostream& out, const draft::DraftDimension& dim,
                              const draft::DimensionStyle& style) {
    // Extension lines.
    for (const auto& [s, e] : dim.extensionLines(style)) {
        writeGroup(out, 0, std::string("LINE"));
        writeCommonProps(out, dim);
        writeGroup(out, 100, std::string("AcDbEntity"));
        writeGroup(out, 100, std::string("AcDbLine"));
        writeGroup(out, 10, s.x);
        writeGroup(out, 20, s.y);
        writeGroup(out, 30, 0.0);
        writeGroup(out, 11, e.x);
        writeGroup(out, 21, e.y);
        writeGroup(out, 31, 0.0);
    }

    // Dimension lines.
    for (const auto& [s, e] : dim.dimensionLines(style)) {
        writeGroup(out, 0, std::string("LINE"));
        writeCommonProps(out, dim);
        writeGroup(out, 100, std::string("AcDbEntity"));
        writeGroup(out, 100, std::string("AcDbLine"));
        writeGroup(out, 10, s.x);
        writeGroup(out, 20, s.y);
        writeGroup(out, 30, 0.0);
        writeGroup(out, 11, e.x);
        writeGroup(out, 21, e.y);
        writeGroup(out, 31, 0.0);
    }

    // Arrowheads.
    for (const auto& [s, e] : dim.arrowheadLines(style)) {
        writeGroup(out, 0, std::string("LINE"));
        writeCommonProps(out, dim);
        writeGroup(out, 100, std::string("AcDbEntity"));
        writeGroup(out, 100, std::string("AcDbLine"));
        writeGroup(out, 10, s.x);
        writeGroup(out, 20, s.y);
        writeGroup(out, 30, 0.0);
        writeGroup(out, 11, e.x);
        writeGroup(out, 21, e.y);
        writeGroup(out, 31, 0.0);
    }

    // Text.
    auto textPos = dim.textPosition();
    auto displayText = dim.displayText(style);
    writeGroup(out, 0, std::string("TEXT"));
    writeCommonProps(out, dim);
    writeGroup(out, 100, std::string("AcDbEntity"));
    writeGroup(out, 100, std::string("AcDbText"));
    writeGroup(out, 10, textPos.x);
    writeGroup(out, 20, textPos.y);
    writeGroup(out, 30, 0.0);
    writeGroup(out, 40, style.textHeight);
    writeGroup(out, 1, dxf::encodeText(displayText));
    writeGroup(out, 72, 1);  // Center-justified.
    writeGroup(out, 11, textPos.x);
    writeGroup(out, 21, textPos.y);
    writeGroup(out, 31, 0.0);
    writeGroup(out, 100, std::string("AcDbText"));
}

void writeEntity(std::ostream& out, const draft::DraftEntity& entity,
                 const draft::DimensionStyle& dimStyle) {
    if (auto* line = dynamic_cast<const draft::DraftLine*>(&entity)) {
        writeLine(out, *line);
    } else if (auto* circle = dynamic_cast<const draft::DraftCircle*>(&entity)) {
        writeCircle(out, *circle);
    } else if (auto* arc = dynamic_cast<const draft::DraftArc*>(&entity)) {
        writeArc(out, *arc);
    } else if (auto* rect = dynamic_cast<const draft::DraftRectangle*>(&entity)) {
        writeRectangle(out, *rect);
    } else if (auto* poly = dynamic_cast<const draft::DraftPolyline*>(&entity)) {
        writePolyline(out, *poly);
    } else if (auto* text = dynamic_cast<const draft::DraftText*>(&entity)) {
        writeText(out, *text);
    } else if (auto* spline = dynamic_cast<const draft::DraftSpline*>(&entity)) {
        writeSpline(out, *spline);
    } else if (auto* hatch = dynamic_cast<const draft::DraftHatch*>(&entity)) {
        writeHatch(out, *hatch);
    } else if (auto* ellipse = dynamic_cast<const draft::DraftEllipse*>(&entity)) {
        writeEllipse(out, *ellipse);
    } else if (auto* ref = dynamic_cast<const draft::DraftBlockRef*>(&entity)) {
        writeInsert(out, *ref);
    } else if (auto* dim = dynamic_cast<const draft::DraftDimension*>(&entity)) {
        writeDimensionAsGeometry(out, *dim, dimStyle);
    }
}

// ===========================================================================
// DXF Import - Tokenizer
// ===========================================================================

struct DxfPair {
    int code = 0;
    std::string value;
};

/// A malformed or truncated DXF. The message becomes the load error.
struct DxfError : std::runtime_error {
    using std::runtime_error::runtime_error;
};

/// The input, how many lines of it have been read (for error messages), and
/// how its values become UTF-8.
struct DxfStream {
    std::istream& in;
    long line = 0;
    dxf::Decoder* decoder = nullptr;
};

void trim(std::string& s) {
    const size_t start = s.find_first_not_of(" \t\r\n");
    if (start == std::string::npos) {
        s.clear();
        return;
    }
    const size_t end = s.find_last_not_of(" \t\r\n");
    s = s.substr(start, end - start + 1);
}

/// Numbers are parsed with std::from_chars: std::stod follows the C locale,
/// which Qt sets from the environment on Unix, so under de_DE "1.5" read as 1.
double toDouble(std::string_view s) {
    if (!s.empty() && s.front() == '+') s.remove_prefix(1);  // from_chars rejects '+'
    double value = 0.0;
    const auto [ptr, ec] = std::from_chars(s.data(), s.data() + s.size(), value);
    return ec == std::errc{} && std::isfinite(value) ? value : 0.0;
}

int toInt(std::string_view s) {
    if (!s.empty() && s.front() == '+') s.remove_prefix(1);
    int value = 0;
    const auto [ptr, ec] = std::from_chars(s.data(), s.data() + s.size(), value);
    return ec == std::errc{} ? value : 0;
}

/// Read one code/value pair. False only at a clean end of input; a code with
/// no value, or a code line that is not a number, is a DxfError.
bool readPair(DxfStream& in, DxfPair& pair) {
    std::string codeLine;
    std::string valueLine;
    if (!std::getline(in.in, codeLine)) return false;
    if (in.line == 0 && codeLine.rfind("\xEF\xBB\xBF", 0) == 0) codeLine.erase(0, 3);  // a BOM
    ++in.line;
    trim(codeLine);
    if (!std::getline(in.in, valueLine)) {
        if (codeLine.empty()) return false;  // a trailing blank line
        throw DxfError("the file ends in the middle of a group (line " + std::to_string(in.line) +
                       ")");
    }
    ++in.line;

    int code = 0;
    const auto [ptr, ec] =
        std::from_chars(codeLine.data(), codeLine.data() + codeLine.size(), code);
    if (ec != std::errc{} || ptr != codeLine.data() + codeLine.size()) {
        throw DxfError("line " + std::to_string(in.line - 1) + ": expected a group code, found \"" +
                       codeLine.substr(0, 40) + "\"");
    }
    if (code == 1 || code == 3) {
        // Text keeps its spaces; only the line ending goes.
        while (!valueLine.empty() && (valueLine.back() == '\r' || valueLine.back() == '\n')) {
            valueLine.pop_back();
        }
    } else {
        trim(valueLine);
    }
    pair.code = code;
    pair.value = in.decoder ? in.decoder->decode(std::move(valueLine)) : std::move(valueLine);
    return true;
}

/// Read a pair inside a section, where the end of the input means the file was
/// cut short. Every section loop reads through this, so a truncated file is an
/// error instead of a silent partial load — or, as it used to be, a loop that
/// re-read the last pair forever.
bool nextPair(DxfStream& in, DxfPair& pair) {
    if (!readPair(in, pair)) {
        throw DxfError("the file ends before the end of a section (it may be truncated)");
    }
    return true;
}

// ===========================================================================
// DXF Import - Entity parsing helpers
// ===========================================================================

// Helper: extract common properties from a group list.
void applyCommonProps(std::shared_ptr<draft::DraftEntity>& entity,
                      const std::vector<DxfPair>& groups) {
    bool trueColor = false;
    for (const auto& g : groups) {
        if (g.code == 8) entity->setLayer(g.value);
        if (g.code == 420) {
            // A true colour: exact, and it wins over the index in 62.
            entity->setColor(dxf::trueColorToArgb(toInt(g.value)));
            trueColor = true;
        }
        if (g.code == 62 && !trueColor) entity->setColor(aciToArgb(toInt(g.value)));
        if (g.code == 6) entity->setLineType(static_cast<int>(draft::lineTypeFromDxfName(g.value)));
        if (g.code == 370) {
            int lw = toInt(g.value);
            entity->setLineWidth(lw <= 0 ? 0.0 : lw / 100.0);
        }
    }
}

// Helper: find first value for a given group code.
std::string findGroup(const std::vector<DxfPair>& groups, int code,
                      const std::string& defaultVal = "") {
    for (const auto& g : groups) {
        if (g.code == code) return g.value;
    }
    return defaultVal;
}

// Helper: find all values for a given group code (for repeated groups like 10/20).
std::vector<double> findAllDoubles(const std::vector<DxfPair>& groups, int code) {
    std::vector<double> result;
    for (const auto& g : groups) {
        if (g.code == code) result.push_back(toDouble(g.value));
    }
    return result;
}

std::shared_ptr<draft::DraftEntity> parseLine(const std::vector<DxfPair>& groups) {
    double x1 = toDouble(findGroup(groups, 10));
    double y1 = toDouble(findGroup(groups, 20));
    double x2 = toDouble(findGroup(groups, 11));
    double y2 = toDouble(findGroup(groups, 21));
    return std::make_shared<draft::DraftLine>(math::Vec2(x1, y1), math::Vec2(x2, y2));
}

std::shared_ptr<draft::DraftEntity> parseCircle(const std::vector<DxfPair>& groups) {
    double cx = toDouble(findGroup(groups, 10));
    double cy = toDouble(findGroup(groups, 20));
    double r = toDouble(findGroup(groups, 40));
    if (r <= 0.0) return nullptr;
    return std::make_shared<draft::DraftCircle>(math::Vec2(cx, cy), r);
}

std::shared_ptr<draft::DraftEntity> parseArc(const std::vector<DxfPair>& groups) {
    double cx = toDouble(findGroup(groups, 10));
    double cy = toDouble(findGroup(groups, 20));
    double r = toDouble(findGroup(groups, 40));
    double sa = toDouble(findGroup(groups, 50)) * math::kDegToRad;
    double ea = toDouble(findGroup(groups, 51)) * math::kDegToRad;
    if (r <= 0.0) return nullptr;
    return std::make_shared<draft::DraftArc>(math::Vec2(cx, cy), r, sa, ea);
}

std::shared_ptr<draft::DraftEntity> parseSpline(const std::vector<DxfPair>& groups) {
    int flags = toInt(findGroup(groups, 70, "0"));
    bool closed = (flags & 1) != 0;

    auto xs = findAllDoubles(groups, 10);
    auto ys = findAllDoubles(groups, 20);
    size_t count = std::min(xs.size(), ys.size());
    if (count < 2) return nullptr;

    std::vector<math::Vec2> cps;
    cps.reserve(count);
    for (size_t i = 0; i < count; ++i) {
        cps.emplace_back(xs[i], ys[i]);
    }
    return std::make_shared<draft::DraftSpline>(cps, closed);
}

std::shared_ptr<draft::DraftEntity> parseHatch(const std::vector<DxfPair>& groups) {
    std::string patternName = findGroup(groups, 2, "ANSI31");
    int solidFill = toInt(findGroup(groups, 70, "0"));
    double angle = toDouble(findGroup(groups, 52, "0")) * math::kDegToRad;
    double spacing = toDouble(findGroup(groups, 41, "1.0"));

    // Parse boundary vertices (first polyline boundary path).
    // Look for group 93 (vertex count), then subsequent 10/20 pairs.
    // In a hatch, boundary vertex 10/20 groups appear after the boundary header.
    auto xs = findAllDoubles(groups, 10);
    auto ys = findAllDoubles(groups, 20);

    // Skip the first 10/20 (elevation point) — boundary vertices start after.
    if (xs.size() > 1 && ys.size() > 1) {
        xs.erase(xs.begin());
        ys.erase(ys.begin());
    }

    size_t count = std::min(xs.size(), ys.size());
    if (count < 3) return nullptr;

    std::vector<math::Vec2> boundary;
    boundary.reserve(count);
    for (size_t i = 0; i < count; ++i) {
        boundary.emplace_back(xs[i], ys[i]);
    }

    draft::HatchPattern pattern = draft::HatchPattern::Lines;
    if (solidFill)
        pattern = draft::HatchPattern::Solid;
    else if (patternName == "ANSI37")
        pattern = draft::HatchPattern::CrossHatch;

    if (spacing <= 0.0) spacing = 1.0;
    return std::make_shared<draft::DraftHatch>(boundary, pattern, angle, spacing);
}

// ===========================================================================
// DXF Import - Section parsers
// ===========================================================================

void skipSection(DxfStream& in) {
    DxfPair pair;
    while (nextPair(in, pair)) {
        if (pair.code == 0 && pair.value == "ENDSEC") return;
    }
}

void parseLayerTable(DxfStream& in, doc::Document& doc) {
    DxfPair pair;
    while (nextPair(in, pair)) {
        if (pair.code == 0 && pair.value == "ENDTAB") return;
        if (pair.code == 0 && pair.value == "LAYER") {
            // Collect all groups for this layer entry.
            std::vector<DxfPair> groups;
            while (nextPair(in, pair)) {
                if (pair.code == 0) {
                    // Put back? We can't unread, so process what we have.
                    // Re-dispatch: this pair is the start of the next record.
                    std::string name = findGroup(groups, 2, "");
                    if (!name.empty()) {
                        int aci = toInt(findGroup(groups, 62, "7"));
                        int flags = toInt(findGroup(groups, 70, "0"));
                        int lw = toInt(findGroup(groups, 370, "-1"));

                        std::string ltName = findGroup(groups, 6, "CONTINUOUS");
                        draft::LayerProperties props;
                        props.name = name;
                        props.color =
                            aciToArgb(aci);  // a negative index (layer off) reads as its colour
                        if (const std::string rgb = findGroup(groups, 420, ""); !rgb.empty()) {
                            props.color = dxf::trueColorToArgb(toInt(rgb));
                        }
                        props.visible = (aci >= 0) && !(flags & 1);
                        props.locked = (flags & 4) != 0;
                        props.lineWidth = (lw <= 0) ? 1.0 : lw / 100.0;
                        props.lineType = static_cast<int>(draft::lineTypeFromDxfName(ltName));

                        if (name == "0") {
                            auto* existing = doc.layerManager().getLayer("0");
                            if (existing) *existing = props;
                        } else {
                            doc.layerManager().addLayer(props);
                        }
                    }

                    // Handle the re-dispatch.
                    if (pair.value == "ENDTAB") return;
                    if (pair.value == "LAYER") {
                        groups.clear();
                        continue;
                    }
                    return;
                }
                groups.push_back(pair);
            }
        }
    }
}

void parseTablesSection(DxfStream& in, doc::Document& doc) {
    DxfPair pair;
    while (nextPair(in, pair)) {
        if (pair.code == 0 && pair.value == "ENDSEC") return;
        if (pair.code == 0 && pair.value == "TABLE") {
            // Read table name.
            DxfPair namePair;
            if (!nextPair(in, namePair)) return;
            if (namePair.code == 2 && namePair.value == "LAYER") {
                parseLayerTable(in, doc);
            } else {
                // Skip other tables (LTYPE, STYLE, VIEW, etc.).
                while (nextPair(in, pair)) {
                    if (pair.code == 0 && pair.value == "ENDTAB") break;
                }
            }
        }
    }
}

using Entities = std::vector<std::shared_ptr<draft::DraftEntity>>;

/// One DXF entity as read: its type and group pairs, not yet built.
struct RawEntity {
    std::string type;
    std::vector<DxfPair> groups;
};

/// Read entities up to the code-0 pair `end` (ENDSEC or ENDBLK) or EOF. On
/// entry `pair` holds the first entity's code-0 pair; on exit, the terminator.
std::vector<RawEntity> readRawEntities(DxfStream& in, DxfPair& pair, const char* end) {
    std::vector<RawEntity> raws;
    while (!(pair.code == 0 && (pair.value == end || pair.value == "EOF"))) {
        if (pair.code != 0) {
            nextPair(in, pair);  // a stray group between entities
            continue;
        }
        RawEntity raw{pair.value, {}};
        while (nextPair(in, pair)) {
            if (pair.code == 0) break;
            raw.groups.push_back(pair);
        }
        raws.push_back(std::move(raw));
    }
    return raws;
}

/// What a DXF import saw besides the entities it made. Keys are
/// {entity type, what about it}.
struct ImportNotes {
    std::map<std::pair<std::string, std::string>, int> unread;
    std::map<std::pair<std::string, std::string>, int> approximated;
};

/// A block from the BLOCKS section, kept raw until every block is known, so
/// an INSERT inside one can refer to a block defined after it.
struct RawBlock {
    math::Vec2 base;
    std::vector<RawEntity> entities;
};

/// The header variables an import uses.
struct Header {
    std::string codepage;  ///< $DWGCODEPAGE
    int insunits = 0;      ///< $INSUNITS: 0 unitless, 1 inches, 4 millimetres...
};

struct Import {
    explicit Import(doc::Document& d) : doc(d) {}

    doc::Document& doc;
    ImportNotes notes;
    std::map<std::string, RawBlock> blocks;
    std::vector<std::string> building;  ///< Blocks being built: a cycle guard.
    dxf::Decoder decoder;
    Header header;
    std::vector<std::shared_ptr<draft::BlockDefinition>> built;  ///< Blocks this import made
    Entities added;                      ///< and the drawing entities it made.
    std::vector<std::string> converted;  ///< Changes that lose nothing (ImportReport::converted).
};

/// Report what a text's codes asked for that the drawing cannot show.
void noteLosses(const dxf::TextLosses& losses, const std::string& type, Import& im) {
    if (losses.styling) {
        ++im.notes.approximated[{type, "underline, colour or size changes inside it not kept"}];
    }
    if (losses.stacked) ++im.notes.approximated[{type, "stacked fractions written inline"}];
    if (losses.unreadable) {
        ++im.notes.approximated[{type, "characters in a multibyte code page not read"}];
    }
}

/// A TEXT entity. Left-aligned text on its baseline stands at its first
/// point (10/20); any other alignment is at its second point (11/21), which
/// is moved to the baseline this drawing's text stands on.
std::shared_ptr<draft::DraftEntity> parseText(const std::vector<DxfPair>& groups, Import& im) {
    double height = toDouble(findGroup(groups, 40, "2.5"));
    if (height <= 0.0) height = 2.5;
    double rotation = toDouble(findGroup(groups, 50, "0")) * math::kDegToRad;
    const int horizontal = toInt(findGroup(groups, 72, "0"));
    int vertical = toInt(findGroup(groups, 73, "0"));
    const math::Vec2 first(toDouble(findGroup(groups, 10)), toDouble(findGroup(groups, 20)));
    const bool hasSecond = !findGroup(groups, 11).empty();
    const math::Vec2 second(toDouble(findGroup(groups, 11)), toDouble(findGroup(groups, 21)));

    math::Vec2 anchor = first;
    draft::TextAlignment align = draft::TextAlignment::Left;
    if (horizontal == 3 || horizontal == 5) {
        // Aligned or fit: stretched to run from the first point to the second.
        if (hasSecond && (second - first).length() > 1e-12) {
            rotation = std::atan2(second.y - first.y, second.x - first.x);
        }
        vertical = 0;
        ++im.notes.approximated[{"TEXT", "fitted between two points, drawn at its own size"}];
    } else {
        if ((horizontal != 0 || vertical != 0) && hasSecond) anchor = second;
        if (horizontal == 1 || horizontal == 4) align = draft::TextAlignment::Center;
        if (horizontal == 2) align = draft::TextAlignment::Right;
        if (horizontal == 4) vertical = 2;  // "Middle": centred both ways
    }
    // How far the anchor is above the baseline: bottom (the descenders),
    // middle, or top (the capitals).
    double above = 0.0;
    if (vertical == 1) above = -0.25 * height;
    if (vertical == 2) above = 0.5 * height;
    if (vertical == 3) above = height;
    anchor = anchor + math::Vec2(std::sin(rotation) * above, -std::cos(rotation) * above);

    dxf::TextLosses losses;
    auto text = std::make_shared<draft::DraftText>(
        anchor, dxf::decodeText(findGroup(groups, 1), im.decoder, losses), height);
    text->setRotation(rotation);
    text->setAlignment(align);
    noteLosses(losses, "TEXT", im);
    return text;
}

/// An MTEXT entity, one text per line: its chunks (3) come before its last
/// part (1), and \P breaks a line. Lines are laid out from the attachment
/// point (71) as AutoCAD does: 5/3 of the height apart, times the spacing
/// factor (44).
Entities mtextEntities(const std::vector<DxfPair>& groups, Import& im) {
    std::string content;
    for (const auto& g : groups) {
        if (g.code == 3) content += g.value;
    }
    content += findGroup(groups, 1);
    dxf::TextLosses losses;
    const std::vector<std::string> lines = dxf::decodeMText(content, im.decoder, losses);
    noteLosses(losses, "MTEXT", im);

    double height = toDouble(findGroup(groups, 40, "2.5"));
    if (height <= 0.0) height = 2.5;
    const math::Vec2 at(toDouble(findGroup(groups, 10)), toDouble(findGroup(groups, 20)));
    // The x direction (11/21) when given; otherwise the angle (50), which
    // AutoCAD writes in degrees whatever the reference says.
    double rotation = toDouble(findGroup(groups, 50, "0")) * math::kDegToRad;
    const math::Vec2 direction(toDouble(findGroup(groups, 11)), toDouble(findGroup(groups, 21)));
    if (direction.length() > 1e-12) rotation = std::atan2(direction.y, direction.x);
    int attach = toInt(findGroup(groups, 71, "1"));
    if (attach < 1 || attach > 9) attach = 1;
    double spacing = toDouble(findGroup(groups, 44, "1"));
    if (spacing <= 0.0) spacing = 1.0;

    const double pitch = height * 5.0 / 3.0 * spacing;
    const double below = pitch * static_cast<double>(lines.size() - 1);
    double baseline = -height;  // top: the first line hangs below the point
    const int row = (attach - 1) / 3;
    if (row == 1) baseline = (height + below) / 2.0 - height;
    if (row == 2) baseline = below;  // bottom: the last line stands on it
    const int column = (attach - 1) % 3;
    const draft::TextAlignment align = column == 0   ? draft::TextAlignment::Left
                                       : column == 1 ? draft::TextAlignment::Center
                                                     : draft::TextAlignment::Right;
    const double c = std::cos(rotation);
    const double s = std::sin(rotation);

    Entities out;
    for (const auto& line : lines) {
        if (line.find_first_not_of(' ') != std::string::npos) {
            auto text = std::make_shared<draft::DraftText>(
                at + math::Vec2(-s * baseline, c * baseline), line, height);
            text->setRotation(rotation);
            text->setAlignment(align);
            out.push_back(std::move(text));
        }
        baseline -= pitch;
    }
    if (out.size() > 1) {
        const uint64_t group = im.doc.draftDocument().nextGroupId();
        for (auto& piece : out) piece->setGroupId(group);
        ++im.notes
              .approximated[{"MTEXT", "several lines, brought in as one text per line, grouped"}];
    }
    return out;
}

/// How an entity's object coordinate system (its extrusion direction, groups
/// 210/220/230) sits against the drawing: the same, mirrored (extrusion
/// (0, 0, -1): its x runs the other way), or out of the drawing's plane.
enum class Ocs { Plane, Mirrored, OutOfPlane };

Ocs ocsOf(const std::vector<DxfPair>& groups) {
    const double nx = toDouble(findGroup(groups, 210, "0"));
    const double ny = toDouble(findGroup(groups, 220, "0"));
    const double nz = toDouble(findGroup(groups, 230, "1"));
    if (std::abs(nx) > 1e-9 || std::abs(ny) > 1e-9) return Ocs::OutOfPlane;
    return nz < 0.0 ? Ocs::Mirrored : Ocs::Plane;
}

/// Give `to` the layer, colour, line width and line type of `from`.
void copyProperties(const draft::DraftEntity& from, draft::DraftEntity& to) {
    to.setLayer(from.layer());
    to.setColor(from.color());
    to.setLineWidth(from.lineWidth());
    to.setLineType(from.lineType());
}

/// Mirror in the y axis: x becomes -x. The drawing's view of a mirrored OCS.
void mirrorInYAxis(draft::DraftEntity& entity) {
    entity.mirror(math::Vec2(0, 0), math::Vec2(0, 1));
}

/// The arc a polyline segment with `bulge` (the tangent of a quarter of its
/// included angle; positive runs counterclockwise) makes from a to b.
std::shared_ptr<draft::DraftEntity> arcFromBulge(const math::Vec2& a, const math::Vec2& b,
                                                 double bulge) {
    const math::Vec2 chord = b - a;
    const double d = chord.length();
    if (d < 1e-12) return nullptr;
    const math::Vec2 left(-chord.y / d, chord.x / d);
    const double h = d * (1.0 - bulge * bulge) / (4.0 * bulge);
    const math::Vec2 c = (a + b) * 0.5 + left * h;
    const double r = std::abs(d * (1.0 + bulge * bulge) / (4.0 * bulge));
    const double angA = std::atan2(a.y - c.y, a.x - c.x);
    const double angB = std::atan2(b.y - c.y, b.x - c.x);
    return bulge > 0.0 ? std::make_shared<draft::DraftArc>(c, r, angA, angB)
                       : std::make_shared<draft::DraftArc>(c, r, angB, angA);
}

struct PolyVertex {
    math::Vec2 p;
    double bulge = 0.0;
};

/// A polyline's vertices, as lines and arcs when any segment is an arc (a
/// polyline here has straight segments only), kept together as one group.
Entities polylineEntities(const std::vector<PolyVertex>& v, bool closed, const std::string& type,
                          Import& im) {
    if (v.size() < 2) return {};
    const size_t segments = closed ? v.size() : v.size() - 1;
    bool arcs = false;
    for (size_t k = 0; k < segments; ++k) arcs = arcs || std::abs(v[k].bulge) > 1e-12;
    if (!arcs) {
        std::vector<math::Vec2> pts;
        pts.reserve(v.size());
        for (const auto& vertex : v) pts.push_back(vertex.p);
        return {std::make_shared<draft::DraftPolyline>(pts, closed)};
    }
    Entities pieces;
    const uint64_t group = im.doc.draftDocument().nextGroupId();
    for (size_t k = 0; k < segments; ++k) {
        const math::Vec2& a = v[k].p;
        const math::Vec2& b = v[(k + 1) % v.size()].p;
        std::shared_ptr<draft::DraftEntity> piece;
        if (std::abs(v[k].bulge) > 1e-12) {
            piece = arcFromBulge(a, b, v[k].bulge);
        } else if ((b - a).length() > 1e-12) {
            piece = std::make_shared<draft::DraftLine>(a, b);
        }
        if (!piece) continue;
        piece->setGroupId(group);
        pieces.push_back(std::move(piece));
    }
    ++im.notes.approximated[{type, "arcs brought in as lines and arcs, grouped"}];
    return pieces;
}

/// An LWPOLYLINE's vertices: each 10/20 starts one, and a 42 after it is its
/// bulge.
std::vector<PolyVertex> lwPolylineVertices(const std::vector<DxfPair>& groups) {
    std::vector<PolyVertex> v;
    for (const auto& g : groups) {
        if (g.code == 10) {
            v.push_back({math::Vec2(toDouble(g.value), 0.0), 0.0});
        } else if (g.code == 20 && !v.empty()) {
            v.back().p.y = toDouble(g.value);
        } else if (g.code == 42 && !v.empty()) {
            v.back().bulge = toDouble(g.value);
        }
    }
    return v;
}

/// A partial ELLIPSE (groups 41/42 its start and end parameters) as a
/// polyline; a whole one as an ellipse.
std::shared_ptr<draft::DraftEntity> ellipseEntity(const std::vector<DxfPair>& groups, Import& im) {
    const math::Vec2 c(toDouble(findGroup(groups, 10)), toDouble(findGroup(groups, 20)));
    const math::Vec2 major(toDouble(findGroup(groups, 11)), toDouble(findGroup(groups, 21)));
    const double ratio = toDouble(findGroup(groups, 40, "1.0"));
    const double semiMajor = major.length();
    if (semiMajor < 1e-12 || !(ratio > 0.0)) return nullptr;
    const double start = toDouble(findGroup(groups, 41, "0"));
    double end = toDouble(findGroup(groups, 42, std::to_string(math::kTwoPi)));
    if (end <= start) end += math::kTwoPi;
    if (std::abs(end - start - math::kTwoPi) < 1e-9) {
        return std::make_shared<draft::DraftEllipse>(c, semiMajor, semiMajor * ratio,
                                                     std::atan2(major.y, major.x));
    }
    // The minor axis follows the extrusion: N x major, scaled.
    const double nz = toDouble(findGroup(groups, 230, "1")) < 0.0 ? -1.0 : 1.0;
    const math::Vec2 minor = math::Vec2(-nz * major.y, nz * major.x) * ratio;
    const int steps = std::max(8, static_cast<int>(std::ceil(64.0 * (end - start) / math::kTwoPi)));
    std::vector<math::Vec2> pts;
    pts.reserve(static_cast<size_t>(steps) + 1);
    for (int k = 0; k <= steps; ++k) {
        const double t = start + (end - start) * k / steps;
        pts.push_back(c + major * std::cos(t) + minor * std::sin(t));
    }
    ++im.notes.approximated[{"ELLIPSE", "partial, brought in as a polyline"}];
    return std::make_shared<draft::DraftPolyline>(pts, false);
}

/// A block's entities placed by an insert: moved from the block's base, scaled
/// by (sx, sy), rotated, moved to `at`. Exact for equal scales (mirrored or
/// not); with unequal ones, lines, polylines, splines, hatches and circles stay
/// exact and arcs, ellipses and text are approximated.
Entities placeBlock(const draft::BlockDefinition& def, const math::Vec2& at, double rotation,
                    double sx, double sy, Import& im, int depth);

Entities placeEntity(const draft::DraftEntity& e, const math::Vec2& base, const math::Vec2& at,
                     double rotation, double sx, double sy, Import& im, int depth) {
    if (const auto* ref = dynamic_cast<const draft::DraftBlockRef*>(&e)) {
        // A block within the block: place its pieces, then these.
        Entities inner = placeBlock(*ref->definition(), ref->insertPos(), ref->rotation(),
                                    ref->uniformScale(), ref->uniformScale(), im, depth + 1);
        Entities out;
        for (const auto& piece : inner) {
            auto placed = placeEntity(*piece, base, at, rotation, sx, sy, im, depth + 1);
            out.insert(out.end(), placed.begin(), placed.end());
        }
        return out;
    }
    const double cr = std::cos(rotation);
    const double sr = std::sin(rotation);
    const auto T = [&](const math::Vec2& p) {
        const double x = (p.x - base.x) * sx;
        const double y = (p.y - base.y) * sy;
        return math::Vec2(at.x + x * cr - y * sr, at.y + x * sr + y * cr);
    };
    const auto all = [&](const std::vector<math::Vec2>& pts) {
        std::vector<math::Vec2> out;
        out.reserve(pts.size());
        for (const auto& p : pts) out.push_back(T(p));
        return out;
    };

    auto copy = e.clone();
    if (std::abs(std::abs(sx) - std::abs(sy)) <= 1e-12 * std::max(std::abs(sx), 1.0)) {
        // Equal scales: a similarity, which every entity takes exactly.
        copy->translate(math::Vec2(-base.x, -base.y));
        if (sx < 0.0) copy->mirror(math::Vec2(0, 0), math::Vec2(0, 1));
        if (sy < 0.0) copy->mirror(math::Vec2(0, 0), math::Vec2(1, 0));
        copy->scale(math::Vec2(0, 0), std::abs(sx));
        copy->rotate(math::Vec2(0, 0), rotation);
        copy->translate(at);
        return {copy};
    }
    if (auto* line = dynamic_cast<draft::DraftLine*>(copy.get())) {
        line->setStart(T(line->start()));
        line->setEnd(T(line->end()));
        return {copy};
    }
    if (auto* poly = dynamic_cast<draft::DraftPolyline*>(copy.get())) {
        poly->setPoints(all(poly->points()));
        return {copy};
    }
    if (auto* spline = dynamic_cast<draft::DraftSpline*>(copy.get())) {
        spline->setControlPoints(all(spline->controlPoints()));
        return {copy};
    }
    if (auto* hatch = dynamic_cast<draft::DraftHatch*>(copy.get())) {
        hatch->setBoundary(all(hatch->boundary()));
        return {copy};
    }
    if (const auto* circle = dynamic_cast<const draft::DraftCircle*>(&e)) {
        // An axis-aligned stretch, then a rotation: an ellipse, exactly.
        const double a = circle->radius() * std::abs(sx);
        const double b = circle->radius() * std::abs(sy);
        auto ellipse = std::make_shared<draft::DraftEllipse>(
            T(circle->center()), std::max(a, b), std::min(a, b),
            a >= b ? rotation : rotation + math::kPi / 2.0);
        copyProperties(e, *ellipse);
        return {ellipse};
    }
    if (auto* text = dynamic_cast<draft::DraftText*>(copy.get())) {
        text->setPosition(T(text->position()));
        text->setTextHeight(text->textHeight() * std::abs(sy));
        text->setRotation(text->rotation() + rotation);
        ++im.notes.approximated[{"TEXT", "in a block with unequal scales, kept its proportions"}];
        return {copy};
    }
    std::vector<math::Vec2> samples;
    if (const auto* arc = dynamic_cast<const draft::DraftArc*>(&e)) {
        const int steps =
            std::max(8, static_cast<int>(std::ceil(64.0 * arc->sweepAngle() / math::kTwoPi)));
        for (int k = 0; k <= steps; ++k) {
            const double t = arc->startAngle() + arc->sweepAngle() * k / steps;
            samples.push_back(arc->center() + math::Vec2(std::cos(t), std::sin(t)) * arc->radius());
        }
    } else if (const auto* ellipse = dynamic_cast<const draft::DraftEllipse*>(&e)) {
        samples = ellipse->evaluate(64);
    }
    if (!samples.empty()) {
        auto poly = std::make_shared<draft::DraftPolyline>(all(samples), false);
        copyProperties(e, *poly);
        ++im.notes.approximated[{"ARC/ELLIPSE",
                                 "in a block with unequal scales, brought in as a polyline"}];
        return {poly};
    }
    ++im.notes.unread[{"drawing", " in a block with unequal scales"}];
    return {};
}

Entities placeBlock(const draft::BlockDefinition& def, const math::Vec2& at, double rotation,
                    double sx, double sy, Import& im, int depth) {
    if (depth > 16) return {};  // blocks nested past any sensible depth: a cycle
    Entities out;
    for (const auto& e : def.entities) {
        auto placed = placeEntity(*e, def.basePoint, at, rotation, sx, sy, im, depth);
        out.insert(out.end(), placed.begin(), placed.end());
    }
    return out;
}

std::shared_ptr<draft::BlockDefinition> buildBlock(const std::string& name, Import& im);

/// The drawing entities raws[i] becomes; a POLYLINE takes its VERTEX entities
/// with it. Advances `i` past what it used. `inBlock` says whether this is a
/// block's content, where an INSERT is flattened into the block.
Entities readEntity(const std::vector<RawEntity>& raws, size_t& i, Import& im, bool inBlock) {
    const RawEntity& raw = raws[i++];
    const std::vector<DxfPair>& g = raw.groups;
    const std::string& type = raw.type;
    const std::string where = inBlock ? " inside a block" : "";
    const auto unread = [&](const std::string& why) -> Entities {
        ++im.notes.unread[{type, where + why}];
        return {};
    };

    // Entities placed in their object coordinate system. The rest (LINE,
    // SPLINE, ELLIPSE, MTEXT) are in world coordinates by the DXF reference;
    // an ELLIPSE's extrusion only sets which way its parameter runs (see
    // ellipseEntity).
    static const std::set<std::string> kOcsTypes = {"CIRCLE", "ARC",   "LWPOLYLINE", "POLYLINE",
                                                    "TEXT",   "HATCH", "INSERT"};
    Ocs ocs = Ocs::Plane;
    if (kOcsTypes.count(type)) {
        ocs = ocsOf(g);
        if (ocs == Ocs::OutOfPlane) {
            if (type == "POLYLINE") {
                // Its vertices go with it — and nothing after them, SEQEND
                // or not, so a missing SEQEND cannot swallow what follows.
                while (i < raws.size() && raws[i].type == "VERTEX") ++i;
                if (i < raws.size() && raws[i].type == "SEQEND") ++i;
            }
            return unread(" (not in the drawing's plane)");
        }
    }

    Entities out;
    if (type == "LINE") {
        out.push_back(parseLine(g));
    } else if (type == "CIRCLE") {
        out.push_back(parseCircle(g));
    } else if (type == "ARC") {
        out.push_back(parseArc(g));
    } else if (type == "LWPOLYLINE") {
        const bool closed = (toInt(findGroup(g, 70, "0")) & 1) != 0;
        out = polylineEntities(lwPolylineVertices(g), closed, type, im);
    } else if (type == "POLYLINE") {
        // The old form: its vertices follow as VERTEX entities, then SEQEND.
        const int flags = toInt(findGroup(g, 70, "0"));
        std::vector<PolyVertex> vertices;
        while (i < raws.size() && raws[i].type == "VERTEX") {
            const auto& vg = raws[i++].groups;
            if ((toInt(findGroup(vg, 70, "0")) & 16) != 0) continue;  // a spline frame point
            vertices.push_back(
                {math::Vec2(toDouble(findGroup(vg, 10)), toDouble(findGroup(vg, 20))),
                 toDouble(findGroup(vg, 42, "0"))});
        }
        if (i < raws.size() && raws[i].type == "SEQEND") ++i;
        if ((flags & (8 | 16 | 64)) != 0) return unread(" (a 3D polyline or mesh)");
        out = polylineEntities(vertices, (flags & 1) != 0, type, im);
    } else if (type == "TEXT") {
        out.push_back(parseText(g, im));
        if (ocs == Ocs::Mirrored) {
            ++im.notes.approximated[{type, "written mirrored, brought in unmirrored"}];
        }
    } else if (type == "MTEXT") {
        out = mtextEntities(g, im);
        if (out.empty()) return unread(" (it has no text)");
    } else if (type == "SPLINE") {
        out.push_back(parseSpline(g));
    } else if (type == "HATCH") {
        out.push_back(parseHatch(g));
    } else if (type == "ELLIPSE") {
        out.push_back(ellipseEntity(g, im));
    } else if (type == "INSERT") {
        const std::string name = findGroup(g, 2);
        auto def =
            inBlock ? buildBlock(name, im) : im.doc.draftDocument().blockTable().findBlock(name);
        if (!def) return unread(" (its block is missing)");
        const math::Vec2 at(toDouble(findGroup(g, 10)), toDouble(findGroup(g, 20)));
        const double sx = toDouble(findGroup(g, 41, "1.0"));
        const double sy = toDouble(findGroup(g, 42, "1.0"));
        const double rotation = toDouble(findGroup(g, 50, "0")) * math::kDegToRad;
        const bool plainRef = !inBlock && ocs == Ocs::Plane && sx > 0.0 &&
                              std::abs(sx - sy) <= 1e-12 * std::max(sx, 1.0);
        if (plainRef) {
            out.push_back(std::make_shared<draft::DraftBlockRef>(def, at, rotation, sx));
        } else {
            // A block reference here has one positive scale: an insert with
            // two, or a mirrored one, or one inside a block, is placed piece
            // by piece.
            out = placeBlock(*def, at, rotation, sx, sy, im, 0);
            if (inBlock) {
                ++im.notes.approximated[{type, "inside a block, flattened into it"}];
            } else {
                ++im.notes.approximated[{type, "unequal or mirrored scales, exploded"}];
            }
            // Block content on layer 0 takes the insert's layer and, when it
            // has no colour of its own, the insert's colour. Content on a
            // named layer keeps that layer's colour.
            const std::string layer = findGroup(g, 8, "0");
            const std::string aci = findGroup(g, 62, "");
            for (auto& piece : out) {
                if (piece->layer() != "0") continue;
                piece->setLayer(layer);
                if (piece->color() == 0 && !aci.empty()) piece->setColor(aciToArgb(toInt(aci)));
            }
            if (ocs == Ocs::Mirrored) {
                for (auto& piece : out) mirrorInYAxis(*piece);
            }
            return out;
        }
    } else {
        return unread("");
    }

    out.erase(std::remove(out.begin(), out.end(), nullptr), out.end());
    if (out.empty()) return unread(" (damaged)");
    for (auto& piece : out) {
        applyCommonProps(piece, g);
        if (ocs == Ocs::Mirrored) mirrorInYAxis(*piece);
    }
    return out;
}

/// Build a block from its raw entities, building any block it inserts first.
std::shared_ptr<draft::BlockDefinition> buildBlock(const std::string& name, Import& im) {
    if (auto built = im.doc.draftDocument().blockTable().findBlock(name)) return built;
    const auto raw = im.blocks.find(name);
    if (raw == im.blocks.end()) return nullptr;
    if (std::find(im.building.begin(), im.building.end(), name) != im.building.end()) {
        return nullptr;  // a block that contains itself
    }
    im.building.push_back(name);
    auto def = std::make_shared<draft::BlockDefinition>();
    def->name = name;
    def->basePoint = raw->second.base;
    const auto& raws = raw->second.entities;
    for (size_t i = 0; i < raws.size();) {
        auto pieces = readEntity(raws, i, im, true);
        def->entities.insert(def->entities.end(), pieces.begin(), pieces.end());
    }
    im.building.pop_back();
    im.doc.draftDocument().blockTable().addBlock(def);
    im.built.push_back(def);
    return def;
}

void parseBlocksSection(DxfStream& in, Import& im) {
    DxfPair pair;
    nextPair(in, pair);
    while (!(pair.code == 0 && (pair.value == "ENDSEC" || pair.value == "EOF"))) {
        if (!(pair.code == 0 && pair.value == "BLOCK")) {
            nextPair(in, pair);
            continue;
        }
        std::string name;
        math::Vec2 base;
        // The BLOCK header's groups, up to its first entity (or ENDBLK).
        while (nextPair(in, pair)) {
            if (pair.code == 0) break;
            if (pair.code == 2) name = pair.value;
            if (pair.code == 10) base.x = toDouble(pair.value);
            if (pair.code == 20) base.y = toDouble(pair.value);
        }
        RawBlock block{base, readRawEntities(in, pair, "ENDBLK")};
        if (pair.code == 0 && pair.value == "ENDBLK") {
            while (nextPair(in, pair)) {
                if (pair.code == 0) break;  // ENDBLK's own groups
            }
        }
        // Model and paper space blocks (names starting "*") hold the
        // drawing's layouts, not reusable blocks.
        if (!name.empty() && name[0] != '*') im.blocks[name] = std::move(block);
    }
    for (const auto& entry : im.blocks) buildBlock(entry.first, im);
}

void parseEntitiesSection(DxfStream& in, Import& im) {
    DxfPair pair;
    while (nextPair(in, pair)) {
        if (pair.code == 0) break;
    }
    const auto raws = readRawEntities(in, pair, "ENDSEC");
    for (size_t i = 0; i < raws.size();) {
        for (auto& entity : readEntity(raws, i, im, false)) {
            im.doc.draftDocument().addEntity(entity);
            im.added.push_back(entity);
        }
    }
}

/// The header variables the import uses: $DWGCODEPAGE, which from here on
/// reads every value, and $INSUNITS.
void parseHeaderSection(DxfStream& in, Import& im) {
    DxfPair pair;
    std::string variable;
    while (nextPair(in, pair)) {
        if (pair.code == 0 && pair.value == "ENDSEC") break;
        if (pair.code == 9) {
            variable = pair.value;
            continue;
        }
        std::string value = pair.value;
        trim(value);
        if (variable == "$DWGCODEPAGE" && pair.code == 3) im.header.codepage = value;
        if (variable == "$INSUNITS" && pair.code == 70) im.header.insunits = toInt(value);
    }
    if (!im.header.codepage.empty()) im.decoder.setCodepage(im.header.codepage);
}

/// A drawing in inches, feet or metres, scaled into millimetres. A block is
/// scaled once, in its definition; an insert of it only moves.
void convertUnits(Import& im) {
    std::string unit;
    const double mm = dxf::insunitsToMillimetres(im.header.insunits, &unit);
    if (!(mm > 0.0) || mm == 1.0) return;
    const auto scaleEntity = [mm](draft::DraftEntity& e) {
        if (auto* ref = dynamic_cast<draft::DraftBlockRef*>(&e)) {
            ref->setInsertPos(ref->insertPos() * mm);
        } else {
            e.scale(math::Vec2(0, 0), mm);
        }
    };
    for (const auto& def : im.built) {
        def->basePoint = def->basePoint * mm;
        for (const auto& e : def->entities) scaleEntity(*e);
    }
    for (const auto& e : im.added) scaleEntity(*e);
    std::ostringstream factor;
    factor << std::setprecision(10) << mm;
    im.converted.push_back("drawn in " + unit + ", scaled by " + factor.str() +
                           " into millimetres");
}

}  // anonymous namespace

// ===========================================================================
// Public API: save
// ===========================================================================

bool DxfFormat::save(const std::string& filePath, const doc::Document& doc, std::string* error) {
    // Build the whole file in memory, then replace the target atomically.
    std::ostringstream out;

    g_handleCounter = 0x100;

    // Compute bounding box for header.
    math::BoundingBox bbox;
    for (const auto& entity : doc.draftDocument().entities()) {
        auto eb = entity->boundingBox();
        if (eb.isValid()) bbox.expand(eb);
    }
    if (!bbox.isValid()) {
        bbox.expand(math::Vec3(0, 0, 0));
        bbox.expand(math::Vec3(100, 100, 0));
    }

    // ---- HEADER ----
    writeGroup(out, 0, std::string("SECTION"));
    writeGroup(out, 2, std::string("HEADER"));
    writeGroup(out, 9, std::string("$ACADVER"));
    writeGroup(out, 1, std::string("AC1027"));
    // Text is UTF-8 from AC1021 on; the code page is still expected.
    writeGroup(out, 9, std::string("$DWGCODEPAGE"));
    writeGroup(out, 3, std::string("ANSI_1252"));
    // The drawing's unit is the millimetre.
    writeGroup(out, 9, std::string("$INSUNITS"));
    writeGroup(out, 70, 4);
    writeGroup(out, 9, std::string("$MEASUREMENT"));
    writeGroup(out, 70, 1);
    writeGroup(out, 9, std::string("$INSBASE"));
    writeGroup(out, 10, 0.0);
    writeGroup(out, 20, 0.0);
    writeGroup(out, 30, 0.0);
    writeGroup(out, 9, std::string("$EXTMIN"));
    writeGroup(out, 10, bbox.min().x);
    writeGroup(out, 20, bbox.min().y);
    writeGroup(out, 30, 0.0);
    writeGroup(out, 9, std::string("$EXTMAX"));
    writeGroup(out, 10, bbox.max().x);
    writeGroup(out, 20, bbox.max().y);
    writeGroup(out, 30, 0.0);
    writeGroup(out, 0, std::string("ENDSEC"));

    // ---- TABLES ----
    writeGroup(out, 0, std::string("SECTION"));
    writeGroup(out, 2, std::string("TABLES"));

    // LTYPE table.
    writeGroup(out, 0, std::string("TABLE"));
    writeGroup(out, 2, std::string("LTYPE"));
    writeGroup(out, 5, nextHandle());
    writeGroup(out, 70, 7);

    // CONTINUOUS
    writeGroup(out, 0, std::string("LTYPE"));
    writeGroup(out, 5, nextHandle());
    writeGroup(out, 2, std::string("CONTINUOUS"));
    writeGroup(out, 70, 0);
    writeGroup(out, 3, std::string("Solid line"));
    writeGroup(out, 72, 65);
    writeGroup(out, 73, 0);
    writeGroup(out, 40, 0.0);

    // DASHED — dash=0.5, gap=0.3, period=0.8
    writeGroup(out, 0, std::string("LTYPE"));
    writeGroup(out, 5, nextHandle());
    writeGroup(out, 2, std::string("DASHED"));
    writeGroup(out, 70, 0);
    writeGroup(out, 3, std::string("Dashed __ __ __"));
    writeGroup(out, 72, 65);
    writeGroup(out, 73, 2);
    writeGroup(out, 40, 0.8);
    writeGroup(out, 49, 0.5);
    writeGroup(out, 49, -0.3);

    // DOT — dot=0.05, gap=0.25, period=0.3
    writeGroup(out, 0, std::string("LTYPE"));
    writeGroup(out, 5, nextHandle());
    writeGroup(out, 2, std::string("DOT"));
    writeGroup(out, 70, 0);
    writeGroup(out, 3, std::string("Dot . . . ."));
    writeGroup(out, 72, 65);
    writeGroup(out, 73, 2);
    writeGroup(out, 40, 0.3);
    writeGroup(out, 49, 0.05);
    writeGroup(out, 49, -0.25);

    // DASHDOT — 0.5, 0.15, 0.05, 0.15, period=0.85
    writeGroup(out, 0, std::string("LTYPE"));
    writeGroup(out, 5, nextHandle());
    writeGroup(out, 2, std::string("DASHDOT"));
    writeGroup(out, 70, 0);
    writeGroup(out, 3, std::string("Dash dot __ . __"));
    writeGroup(out, 72, 65);
    writeGroup(out, 73, 4);
    writeGroup(out, 40, 0.85);
    writeGroup(out, 49, 0.5);
    writeGroup(out, 49, -0.15);
    writeGroup(out, 49, 0.05);
    writeGroup(out, 49, -0.15);

    // CENTER — 1.0, 0.15, 0.2, 0.15, period=1.5
    writeGroup(out, 0, std::string("LTYPE"));
    writeGroup(out, 5, nextHandle());
    writeGroup(out, 2, std::string("CENTER"));
    writeGroup(out, 70, 0);
    writeGroup(out, 3, std::string("Center ____ _ ____ _"));
    writeGroup(out, 72, 65);
    writeGroup(out, 73, 4);
    writeGroup(out, 40, 1.5);
    writeGroup(out, 49, 1.0);
    writeGroup(out, 49, -0.15);
    writeGroup(out, 49, 0.2);
    writeGroup(out, 49, -0.15);

    // HIDDEN — 0.25, 0.15, period=0.4
    writeGroup(out, 0, std::string("LTYPE"));
    writeGroup(out, 5, nextHandle());
    writeGroup(out, 2, std::string("HIDDEN"));
    writeGroup(out, 70, 0);
    writeGroup(out, 3, std::string("Hidden __ __ __"));
    writeGroup(out, 72, 65);
    writeGroup(out, 73, 2);
    writeGroup(out, 40, 0.4);
    writeGroup(out, 49, 0.25);
    writeGroup(out, 49, -0.15);

    // PHANTOM — 1.0, 0.15, 0.2, 0.15, 0.2, 0.15, period=1.85
    writeGroup(out, 0, std::string("LTYPE"));
    writeGroup(out, 5, nextHandle());
    writeGroup(out, 2, std::string("PHANTOM"));
    writeGroup(out, 70, 0);
    writeGroup(out, 3, std::string("Phantom ______ _ _ ______"));
    writeGroup(out, 72, 65);
    writeGroup(out, 73, 6);
    writeGroup(out, 40, 1.85);
    writeGroup(out, 49, 1.0);
    writeGroup(out, 49, -0.15);
    writeGroup(out, 49, 0.2);
    writeGroup(out, 49, -0.15);
    writeGroup(out, 49, 0.2);
    writeGroup(out, 49, -0.15);

    writeGroup(out, 0, std::string("ENDTAB"));

    // LAYER table.
    auto layerNames = doc.layerManager().layerNames();
    writeGroup(out, 0, std::string("TABLE"));
    writeGroup(out, 2, std::string("LAYER"));
    writeGroup(out, 5, nextHandle());
    writeGroup(out, 70, static_cast<int>(layerNames.size()));
    for (const auto& name : layerNames) {
        const auto* lp = doc.layerManager().getLayer(name);
        if (!lp) continue;
        writeGroup(out, 0, std::string("LAYER"));
        writeGroup(out, 5, nextHandle());
        writeGroup(out, 2, name);
        int flags = 0;
        if (!lp->visible) flags |= 1;
        if (lp->locked) flags |= 4;
        writeGroup(out, 70, flags);
        int aci = argbToAci(lp->color);
        if (!lp->visible) aci = -aci;
        writeGroup(out, 62, aci);
        if (!dxf::isAciColor(lp->color)) {
            writeGroup(out, 420, static_cast<int>(lp->color & 0xFFFFFFu));
        }
        writeGroup(out, 6,
                   std::string(draft::lineTypeDxfName(static_cast<draft::LineType>(lp->lineType))));
    }
    writeGroup(out, 0, std::string("ENDTAB"));
    writeGroup(out, 0, std::string("ENDSEC"));

    // ---- BLOCKS ----
    writeGroup(out, 0, std::string("SECTION"));
    writeGroup(out, 2, std::string("BLOCKS"));
    for (const auto& name : doc.draftDocument().blockTable().blockNames()) {
        auto def = doc.draftDocument().blockTable().findBlock(name);
        if (!def) continue;
        writeGroup(out, 0, std::string("BLOCK"));
        writeGroup(out, 5, nextHandle());
        writeGroup(out, 8, std::string("0"));
        writeGroup(out, 2, name);
        writeGroup(out, 70, 0);
        writeGroup(out, 10, def->basePoint.x);
        writeGroup(out, 20, def->basePoint.y);
        writeGroup(out, 30, 0.0);
        draft::DimensionStyle dummyStyle;
        for (const auto& subEntity : def->entities) {
            writeEntity(out, *subEntity, dummyStyle);
        }
        writeGroup(out, 0, std::string("ENDBLK"));
        writeGroup(out, 5, nextHandle());
        writeGroup(out, 8, std::string("0"));
    }
    writeGroup(out, 0, std::string("ENDSEC"));

    // ---- ENTITIES ----
    writeGroup(out, 0, std::string("SECTION"));
    writeGroup(out, 2, std::string("ENTITIES"));
    const auto& dimStyle = doc.draftDocument().dimensionStyle();
    for (const auto& entity : doc.draftDocument().entities()) {
        writeEntity(out, *entity, dimStyle);
    }
    writeGroup(out, 0, std::string("ENDSEC"));

    // ---- EOF ----
    writeGroup(out, 0, std::string("EOF"));

    return writeFileAtomically(pathFromUtf8(filePath), out.str(), error);
}

// ===========================================================================
// Public API: load
// ===========================================================================

namespace {

/// The section loop behind DxfFormat::load. Returns false when the input has
/// no SECTION at all.
bool parseDxf(std::istream& input, Import& im) {
    DxfStream in{input, 0, &im.decoder};
    DxfPair pair;
    bool foundSection = false;

    for (;;) {
        try {
            if (!readPair(in, pair)) break;
        } catch (const DxfError&) {
            // Garbage before any section is simply not a DXF file.
            if (!foundSection) return false;
            throw;
        }
        if (pair.code == 0 && pair.value == "EOF") break;
        if (pair.code == 0 && pair.value == "SECTION") {
            foundSection = true;
            DxfPair namePair;
            nextPair(in, namePair);
            if (namePair.code == 2) {
                if (namePair.value == "HEADER") {
                    parseHeaderSection(in, im);
                } else if (namePair.value == "TABLES") {
                    parseTablesSection(in, im.doc);
                } else if (namePair.value == "BLOCKS") {
                    parseBlocksSection(in, im);
                } else if (namePair.value == "ENTITIES") {
                    parseEntitiesSection(in, im);
                } else {
                    skipSection(in);
                }
            }
        }
    }

    convertUnits(im);
    // Rebuild spatial index after loading all entities.
    im.doc.draftDocument().rebuildSpatialIndex();

    return foundSection;
}

}  // namespace

namespace {

bool failWith(std::string* error, std::string message) {
    if (error) *error = std::move(message);
    return false;
}

/// parseDxf with every failure turned into a reason, and what it did not
/// read counted into @p report.
bool parseChecked(std::istream& in, doc::Document& doc, std::string* error, ImportReport* report) {
    char head[18] = {};
    in.read(head, sizeof head);
    const bool binary = std::string_view(head, static_cast<size_t>(in.gcount())) ==
                        std::string_view("AutoCAD Binary DXF");
    in.clear();
    in.seekg(0);
    if (binary) {
        return failWith(error,
                        "this is a binary DXF file, which cannot be read; save it as a text "
                        "(ASCII) DXF");
    }
    Import im(doc);
    try {
        if (!parseDxf(in, im)) return failWith(error, "this is not a DXF file (it has no SECTION)");
    } catch (const DxfError& e) {
        return failWith(error, e.what());
    } catch (const std::exception& e) {
        return failWith(error, std::string("the DXF file is damaged: ") + e.what());
    }
    if (report) {
        const auto entities = [](int n) {
            return n == 1 ? std::string(" entity") : std::string(" entities");
        };
        for (const auto& [key, count] : im.notes.unread) {
            report->skipped.push_back(std::to_string(count) + " " + key.first + entities(count) +
                                      key.second + " not read");
        }
        for (const auto& [key, count] : im.notes.approximated) {
            report->approximated.push_back(std::to_string(count) + " " + key.first +
                                           entities(count) + ": " + key.second);
        }
        if (const int n = im.decoder.undecodable(); n > 0) {
            report->approximated.push_back(
                std::to_string(n) + (n == 1 ? " character" : " characters") + " in code page " +
                im.decoder.codepage() + " could not be read, shown as \xEF\xBF\xBD");
        }
        report->converted.insert(report->converted.end(), im.converted.begin(), im.converted.end());
    }
    return true;
}

}  // namespace

bool DxfFormat::load(const std::string& filePath, doc::Document& doc, std::string* error,
                     ImportReport* report) {
    const std::filesystem::path path = pathFromUtf8(filePath);
    if (std::string why = whyUnreadable(path); !why.empty()) return failWith(error, std::move(why));
    std::ifstream in(path, std::ios::binary);
    if (!in.is_open()) return failWith(error, "the file could not be read (check its permissions)");
    return parseChecked(in, doc, error, report);
}

bool DxfFormat::loadFromString(const std::string& text, doc::Document& doc, std::string* error,
                               ImportReport* report) {
    std::istringstream in(text);
    return parseChecked(in, doc, error, report);
}

}  // namespace hz::io
