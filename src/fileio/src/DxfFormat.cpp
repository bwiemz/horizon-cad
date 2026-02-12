#include "horizon/fileio/DxfFormat.h"
#include "horizon/document/Document.h"
#include "horizon/drafting/DraftLine.h"
#include "horizon/drafting/DraftCircle.h"
#include "horizon/drafting/DraftArc.h"
#include "horizon/drafting/DraftRectangle.h"
#include "horizon/drafting/DraftPolyline.h"
#include "horizon/drafting/DraftText.h"
#include "horizon/drafting/DraftSpline.h"
#include "horizon/drafting/DraftHatch.h"
#include "horizon/drafting/DraftEllipse.h"
#include "horizon/drafting/DraftBlockRef.h"
#include "horizon/drafting/DraftDimension.h"
#include "horizon/drafting/DraftLinearDimension.h"
#include "horizon/drafting/DraftRadialDimension.h"
#include "horizon/drafting/DraftAngularDimension.h"
#include "horizon/drafting/DraftLeader.h"
#include "horizon/drafting/DimensionStyle.h"
#include "horizon/drafting/Layer.h"
#include "horizon/math/Constants.h"
#include "horizon/math/BoundingBox.h"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <string>
#include <vector>

namespace hz::io {

// ===========================================================================
// ACI Color Table (subset for first 10 + white/black mapping)
// ===========================================================================

namespace {

struct AciEntry {
    int r, g, b;
};

// Standard AutoCAD Color Index (first 10 colors + key entries)
static const AciEntry kAciTable[] = {
    {0, 0, 0},         // 0 = BYBLOCK (unused in our mapping)
    {255, 0, 0},       // 1 = Red
    {255, 255, 0},     // 2 = Yellow
    {0, 255, 0},       // 3 = Green
    {0, 255, 255},     // 4 = Cyan
    {0, 0, 255},       // 5 = Blue
    {255, 0, 255},     // 6 = Magenta
    {255, 255, 255},   // 7 = White/Black (depends on background)
    {128, 128, 128},   // 8 = Dark gray
    {192, 192, 192},   // 9 = Light gray
};
static const int kAciTableSize = 10;

int argbToAci(uint32_t argb) {
    if (argb == 0x00000000) return 256;  // BYLAYER

    int r = (argb >> 16) & 0xFF;
    int g = (argb >> 8) & 0xFF;
    int b = argb & 0xFF;

    // Find closest ACI color (simple Euclidean distance).
    int bestAci = 7;
    int bestDist = 999999;
    for (int i = 1; i < kAciTableSize; ++i) {
        int dr = r - kAciTable[i].r;
        int dg = g - kAciTable[i].g;
        int db = b - kAciTable[i].b;
        int dist = dr * dr + dg * dg + db * db;
        if (dist < bestDist) {
            bestDist = dist;
            bestAci = i;
        }
    }
    return bestAci;
}

uint32_t aciToArgb(int aci) {
    if (aci == 256 || aci == 0) return 0x00000000;  // BYLAYER / BYBLOCK
    if (aci < 0) aci = -aci;  // Negative = layer off, use absolute
    if (aci < kAciTableSize) {
        auto& e = kAciTable[aci];
        return 0xFF000000u | (uint32_t(e.r) << 16) | (uint32_t(e.g) << 8) | uint32_t(e.b);
    }
    return 0xFFFFFFFF;  // Default white for unknown ACI
}

// ===========================================================================
// DXF Group Code Writer
// ===========================================================================

void writeGroup(std::ostream& out, int code, const std::string& value) {
    out << "  " << code << "\n" << value << "\n";
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

void writeLwPolyline(std::ostream& out, const std::vector<math::Vec2>& pts,
                     bool closed, const draft::DraftEntity& entity) {
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
    writeGroup(out, 1, text.text());
    if (text.rotation() != 0.0) {
        writeGroup(out, 50, text.rotation() * math::kRadToDeg);
    }
    int hJust = 0;
    if (text.alignment() == draft::TextAlignment::Center) hJust = 1;
    else if (text.alignment() == draft::TextAlignment::Right) hJust = 2;
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
    writeGroup(out, 75, 0);   // Normal hatch style.
    writeGroup(out, 76, 1);   // Predefined pattern type.
    writeGroup(out, 52, hatch.angle() * math::kRadToDeg);
    writeGroup(out, 41, hatch.spacing());
    writeGroup(out, 78, 0);   // Number of pattern definition lines (0 for predefined).
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
    double ratio = (ellipse.semiMajor() > 1e-12)
        ? ellipse.semiMinor() / ellipse.semiMajor() : 1.0;
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
    writeGroup(out, 1, displayText);
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

bool readPair(std::istream& in, DxfPair& pair) {
    std::string codeLine, valueLine;
    if (!std::getline(in, codeLine)) return false;
    if (!std::getline(in, valueLine)) return false;

    // Trim whitespace.
    auto trim = [](std::string& s) {
        size_t start = s.find_first_not_of(" \t\r\n");
        size_t end = s.find_last_not_of(" \t\r\n");
        if (start == std::string::npos) { s.clear(); return; }
        s = s.substr(start, end - start + 1);
    };
    trim(codeLine);
    trim(valueLine);

    try {
        pair.code = std::stoi(codeLine);
    } catch (...) {
        return false;
    }
    pair.value = valueLine;
    return true;
}

double toDouble(const std::string& s) {
    try { return std::stod(s); }
    catch (...) { return 0.0; }
}

int toInt(const std::string& s) {
    try { return std::stoi(s); }
    catch (...) { return 0; }
}

// ===========================================================================
// DXF Import - Entity parsing helpers
// ===========================================================================

// Helper: extract common properties from a group list.
void applyCommonProps(std::shared_ptr<draft::DraftEntity>& entity,
                      const std::vector<DxfPair>& groups) {
    for (const auto& g : groups) {
        if (g.code == 8)   entity->setLayer(g.value);
        if (g.code == 62)  entity->setColor(aciToArgb(toInt(g.value)));
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
    double r  = toDouble(findGroup(groups, 40));
    if (r <= 0.0) return nullptr;
    return std::make_shared<draft::DraftCircle>(math::Vec2(cx, cy), r);
}

std::shared_ptr<draft::DraftEntity> parseArc(const std::vector<DxfPair>& groups) {
    double cx = toDouble(findGroup(groups, 10));
    double cy = toDouble(findGroup(groups, 20));
    double r  = toDouble(findGroup(groups, 40));
    double sa = toDouble(findGroup(groups, 50)) * math::kDegToRad;
    double ea = toDouble(findGroup(groups, 51)) * math::kDegToRad;
    if (r <= 0.0) return nullptr;
    return std::make_shared<draft::DraftArc>(math::Vec2(cx, cy), r, sa, ea);
}

std::shared_ptr<draft::DraftEntity> parseLwPolyline(const std::vector<DxfPair>& groups) {
    int flags = toInt(findGroup(groups, 70, "0"));
    bool closed = (flags & 1) != 0;

    // Collect vertex points (multiple group 10/20 pairs).
    auto xs = findAllDoubles(groups, 10);
    auto ys = findAllDoubles(groups, 20);
    size_t count = std::min(xs.size(), ys.size());
    if (count < 2) return nullptr;

    std::vector<math::Vec2> pts;
    pts.reserve(count);
    for (size_t i = 0; i < count; ++i) {
        pts.emplace_back(xs[i], ys[i]);
    }
    return std::make_shared<draft::DraftPolyline>(pts, closed);
}

std::shared_ptr<draft::DraftEntity> parseText(const std::vector<DxfPair>& groups) {
    double x = toDouble(findGroup(groups, 10));
    double y = toDouble(findGroup(groups, 20));
    double height = toDouble(findGroup(groups, 40, "2.5"));
    std::string content = findGroup(groups, 1);
    double rotation = toDouble(findGroup(groups, 50, "0")) * math::kDegToRad;
    int hJust = toInt(findGroup(groups, 72, "0"));

    if (height <= 0.0) height = 2.5;
    auto entity = std::make_shared<draft::DraftText>(math::Vec2(x, y), content, height);
    entity->setRotation(rotation);

    draft::TextAlignment align = draft::TextAlignment::Left;
    if (hJust == 1) align = draft::TextAlignment::Center;
    else if (hJust == 2) align = draft::TextAlignment::Right;
    entity->setAlignment(align);

    return entity;
}

std::shared_ptr<draft::DraftEntity> parseMText(const std::vector<DxfPair>& groups) {
    double x = toDouble(findGroup(groups, 10));
    double y = toDouble(findGroup(groups, 20));
    double height = toDouble(findGroup(groups, 40, "2.5"));
    std::string content = findGroup(groups, 1);
    // Also check group 3 (additional text chunks).
    for (const auto& g : groups) {
        if (g.code == 3) content = g.value + content;
    }
    // Strip basic MTEXT formatting codes.
    std::string plain;
    for (size_t i = 0; i < content.size(); ++i) {
        if (content[i] == '\\' && i + 1 < content.size()) {
            char next = content[i + 1];
            if (next == 'P' || next == 'p') { plain += ' '; i++; continue; }
            // Skip formatting like \fArial|b0|i0|... until ';'
            if (next == 'f' || next == 'F' || next == 'H' || next == 'W' ||
                next == 'C' || next == 'T' || next == 'Q' || next == 'A') {
                size_t end = content.find(';', i + 1);
                if (end != std::string::npos) { i = end; continue; }
            }
            i++;  // Skip backslash + next char.
            continue;
        }
        if (content[i] == '{' || content[i] == '}') continue;
        plain += content[i];
    }

    if (height <= 0.0) height = 2.5;
    return std::make_shared<draft::DraftText>(math::Vec2(x, y), plain, height);
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
    if (solidFill) pattern = draft::HatchPattern::Solid;
    else if (patternName == "ANSI37") pattern = draft::HatchPattern::CrossHatch;

    if (spacing <= 0.0) spacing = 1.0;
    return std::make_shared<draft::DraftHatch>(boundary, pattern, angle, spacing);
}

std::shared_ptr<draft::DraftEntity> parseEllipse(const std::vector<DxfPair>& groups) {
    double cx = toDouble(findGroup(groups, 10));
    double cy = toDouble(findGroup(groups, 20));
    // Major axis endpoint relative to center.
    double mx = toDouble(findGroup(groups, 11));
    double my = toDouble(findGroup(groups, 21));
    double ratio = toDouble(findGroup(groups, 40, "1.0"));

    double semiMajor = std::sqrt(mx * mx + my * my);
    if (semiMajor < 1e-12) semiMajor = 1.0;
    double semiMinor = semiMajor * ratio;
    double rotation = std::atan2(my, mx);

    return std::make_shared<draft::DraftEllipse>(
        math::Vec2(cx, cy), semiMajor, semiMinor, rotation);
}

std::shared_ptr<draft::DraftEntity> parseInsert(
    const std::vector<DxfPair>& groups, const doc::Document& doc) {
    std::string blockName = findGroup(groups, 2);
    double x = toDouble(findGroup(groups, 10));
    double y = toDouble(findGroup(groups, 20));
    double xScale = toDouble(findGroup(groups, 41, "1.0"));
    double yScale = toDouble(findGroup(groups, 42, "1.0"));
    double rotation = toDouble(findGroup(groups, 50, "0")) * math::kDegToRad;

    auto def = doc.draftDocument().blockTable().findBlock(blockName);
    if (!def) return nullptr;

    double uniformScale = (std::abs(xScale) + std::abs(yScale)) / 2.0;
    if (std::abs(uniformScale) < 1e-9) uniformScale = 1.0;

    return std::make_shared<draft::DraftBlockRef>(def, math::Vec2(x, y),
                                                   rotation, uniformScale);
}

// ===========================================================================
// DXF Import - Section parsers
// ===========================================================================

void skipSection(std::istream& in) {
    DxfPair pair;
    while (readPair(in, pair)) {
        if (pair.code == 0 && pair.value == "ENDSEC") return;
    }
}

void parseLayerTable(std::istream& in, doc::Document& doc) {
    DxfPair pair;
    while (readPair(in, pair)) {
        if (pair.code == 0 && pair.value == "ENDTAB") return;
        if (pair.code == 0 && pair.value == "LAYER") {
            // Collect all groups for this layer entry.
            std::vector<DxfPair> groups;
            while (readPair(in, pair)) {
                if (pair.code == 0) {
                    // Put back? We can't unread, so process what we have.
                    // Re-dispatch: this pair is the start of the next record.
                    std::string name = findGroup(groups, 2, "");
                    if (!name.empty()) {
                        int aci = toInt(findGroup(groups, 62, "7"));
                        int flags = toInt(findGroup(groups, 70, "0"));
                        int lw = toInt(findGroup(groups, 370, "-1"));

                        draft::LayerProperties props;
                        props.name = name;
                        props.color = aciToArgb(std::abs(aci));
                        props.visible = (aci >= 0) && !(flags & 1);
                        props.locked = (flags & 4) != 0;
                        props.lineWidth = (lw <= 0) ? 1.0 : lw / 100.0;

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

void parseTablesSection(std::istream& in, doc::Document& doc) {
    DxfPair pair;
    while (readPair(in, pair)) {
        if (pair.code == 0 && pair.value == "ENDSEC") return;
        if (pair.code == 0 && pair.value == "TABLE") {
            // Read table name.
            DxfPair namePair;
            if (!readPair(in, namePair)) return;
            if (namePair.code == 2 && namePair.value == "LAYER") {
                parseLayerTable(in, doc);
            } else {
                // Skip other tables (LTYPE, STYLE, VIEW, etc.).
                while (readPair(in, pair)) {
                    if (pair.code == 0 && pair.value == "ENDTAB") break;
                }
            }
        }
    }
}

void parseBlocksSection(std::istream& in, doc::Document& doc) {
    DxfPair pair;
    while (readPair(in, pair)) {
        if (pair.code == 0 && pair.value == "ENDSEC") return;
        if (pair.code == 0 && pair.value == "BLOCK") {
            // Collect BLOCK header groups.
            std::vector<DxfPair> headerGroups;
            while (readPair(in, pair)) {
                if (pair.code == 0) break;
                headerGroups.push_back(pair);
            }

            std::string blockName = findGroup(headerGroups, 2, "");
            double bx = toDouble(findGroup(headerGroups, 10, "0"));
            double by = toDouble(findGroup(headerGroups, 20, "0"));

            // Skip special AutoCAD blocks.
            bool isSpecial = (!blockName.empty() && blockName[0] == '*');

            auto def = std::make_shared<draft::BlockDefinition>();
            def->name = blockName;
            def->basePoint = math::Vec2(bx, by);

            // Parse sub-entities until ENDBLK.
            // 'pair' already holds the first entity or ENDBLK.
            while (true) {
                if (pair.code == 0 && pair.value == "ENDBLK") break;
                if (pair.code == 0) {
                    std::string entityType = pair.value;
                    std::vector<DxfPair> groups;
                    while (readPair(in, pair)) {
                        if (pair.code == 0) break;
                        groups.push_back(pair);
                    }

                    if (!isSpecial) {
                        std::shared_ptr<draft::DraftEntity> entity;
                        if (entityType == "LINE") entity = parseLine(groups);
                        else if (entityType == "CIRCLE") entity = parseCircle(groups);
                        else if (entityType == "ARC") entity = parseArc(groups);
                        else if (entityType == "LWPOLYLINE") entity = parseLwPolyline(groups);
                        else if (entityType == "TEXT") entity = parseText(groups);
                        else if (entityType == "MTEXT") entity = parseMText(groups);
                        else if (entityType == "SPLINE") entity = parseSpline(groups);
                        else if (entityType == "ELLIPSE") entity = parseEllipse(groups);

                        if (entity) {
                            applyCommonProps(entity, groups);
                            def->entities.push_back(entity);
                        }
                    }
                    continue;  // pair already holds next entity's code 0 line.
                }
                if (!readPair(in, pair)) break;
            }

            if (!isSpecial && !blockName.empty()) {
                doc.draftDocument().blockTable().addBlock(def);
            }
        }
    }
}

void parseEntitiesSection(std::istream& in, doc::Document& doc) {
    DxfPair pair;
    // Read first entity type.
    while (readPair(in, pair)) {
        if (pair.code == 0) break;
    }

    while (true) {
        if (pair.code == 0 && pair.value == "ENDSEC") return;
        if (pair.code == 0 && pair.value == "EOF") return;

        std::string entityType = pair.value;
        std::vector<DxfPair> groups;

        // Collect all group codes until next code 0.
        while (readPair(in, pair)) {
            if (pair.code == 0) break;
            groups.push_back(pair);
        }

        // Parse entity.
        std::shared_ptr<draft::DraftEntity> entity;
        if (entityType == "LINE") entity = parseLine(groups);
        else if (entityType == "CIRCLE") entity = parseCircle(groups);
        else if (entityType == "ARC") entity = parseArc(groups);
        else if (entityType == "LWPOLYLINE") entity = parseLwPolyline(groups);
        else if (entityType == "TEXT") entity = parseText(groups);
        else if (entityType == "MTEXT") entity = parseMText(groups);
        else if (entityType == "SPLINE") entity = parseSpline(groups);
        else if (entityType == "HATCH") entity = parseHatch(groups);
        else if (entityType == "ELLIPSE") entity = parseEllipse(groups);
        else if (entityType == "INSERT") entity = parseInsert(groups, doc);
        // Unknown entity types are silently skipped.

        if (entity) {
            applyCommonProps(entity, groups);
            doc.draftDocument().addEntity(entity);
        }
    }
}

}  // anonymous namespace

// ===========================================================================
// Public API: save
// ===========================================================================

bool DxfFormat::save(const std::string& filePath, const doc::Document& doc) {
    std::ofstream out(filePath);
    if (!out.is_open()) return false;

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
    writeGroup(out, 70, 1);
    writeGroup(out, 0, std::string("LTYPE"));
    writeGroup(out, 5, nextHandle());
    writeGroup(out, 2, std::string("CONTINUOUS"));
    writeGroup(out, 70, 0);
    writeGroup(out, 3, std::string("Solid line"));
    writeGroup(out, 72, 65);
    writeGroup(out, 73, 0);
    writeGroup(out, 40, 0.0);
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
        writeGroup(out, 6, std::string("CONTINUOUS"));
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

    return out.good();
}

// ===========================================================================
// Public API: load
// ===========================================================================

bool DxfFormat::load(const std::string& filePath, doc::Document& doc) {
    std::ifstream in(filePath);
    if (!in.is_open()) return false;

    DxfPair pair;
    bool foundSection = false;

    while (readPair(in, pair)) {
        if (pair.code == 0 && pair.value == "EOF") break;
        if (pair.code == 0 && pair.value == "SECTION") {
            foundSection = true;
            DxfPair namePair;
            if (!readPair(in, namePair)) break;
            if (namePair.code == 2) {
                if (namePair.value == "TABLES") {
                    parseTablesSection(in, doc);
                } else if (namePair.value == "BLOCKS") {
                    parseBlocksSection(in, doc);
                } else if (namePair.value == "ENTITIES") {
                    parseEntitiesSection(in, doc);
                } else {
                    skipSection(in);
                }
            }
        }
    }

    return foundSection;
}

}  // namespace hz::io
