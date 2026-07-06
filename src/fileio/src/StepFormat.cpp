#include "horizon/fileio/StepFormat.h"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <map>
#include <optional>
#include <sstream>
#include <unordered_map>
#include <unordered_set>
#include <variant>

#include "horizon/geometry/curves/NurbsCurve.h"
#include "horizon/geometry/surfaces/NurbsSurface.h"
#include "horizon/math/Constants.h"
#include "horizon/math/Vec3.h"

namespace hz::io {

using hz::math::Vec3;

namespace {

thread_local std::string g_lastError;

constexpr double kMergeTol = 1e-9;

/// Piecewise degree-2 rational arc in an explicit placement frame.
/// Angles are measured from @p xAxis toward @p yAxis around the frame normal.
/// Unlike NurbsCurve::makeArc this takes the frame verbatim, which STEP
/// placements require.
std::shared_ptr<geo::NurbsCurve> makeArcInFrame(const Vec3& center, double radius, double a0,
                                                double a1, const Vec3& xAxis, const Vec3& yAxis) {
    double sweep = a1 - a0;
    while (sweep <= 1e-12) sweep += math::kTwoPi;
    while (sweep > math::kTwoPi + 1e-12) sweep -= math::kTwoPi;

    const int numSegments = std::max(1, static_cast<int>(std::ceil(sweep / math::kHalfPi - 1e-9)));
    const double segSweep = sweep / numSegments;
    const double wMid = std::cos(segSweep / 2.0);

    auto at = [&](double a, double scale) {
        return center + (xAxis * std::cos(a) + yAxis * std::sin(a)) * (radius * scale);
    };

    std::vector<Vec3> cps;
    std::vector<double> weights;
    cps.push_back(at(a0, 1.0));
    weights.push_back(1.0);
    double angle = a0;
    for (int seg = 0; seg < numSegments; ++seg) {
        const double mid = angle + segSweep / 2.0;
        const double end = angle + segSweep;
        cps.push_back(at(mid, 1.0 / wMid));
        weights.push_back(wMid);
        cps.push_back(at(end, 1.0));
        weights.push_back(1.0);
        angle = end;
    }

    std::vector<double> knots{0.0, 0.0, 0.0};
    for (int seg = 1; seg < numSegments; ++seg) {
        const double k = static_cast<double>(seg) / numSegments;
        knots.push_back(k);
        knots.push_back(k);
    }
    knots.insert(knots.end(), {1.0, 1.0, 1.0});

    return std::make_shared<geo::NurbsCurve>(std::move(cps), std::move(weights), std::move(knots),
                                             2);
}

/// Reverse a surface's U direction: rows and weights reversed, U knot vector
/// mirrored. Flips the surface normal while describing identical geometry —
/// used to honour ADVANCED_FACE same_sense = .F. on import (the kernel's
/// convention is surface normal == outward face normal).
std::shared_ptr<geo::NurbsSurface> reverseSurfaceU(const geo::NurbsSurface& s) {
    auto cps = s.controlPoints();
    auto wts = s.weights();
    std::reverse(cps.begin(), cps.end());
    std::reverse(wts.begin(), wts.end());
    const auto& k = s.knotsU();
    const double lo = k.front();
    const double hi = k.back();
    std::vector<double> rk(k.rbegin(), k.rend());
    for (double& v : rk) v = lo + hi - v;
    return std::make_shared<geo::NurbsSurface>(std::move(cps), std::move(wts), std::move(rk),
                                               s.knotsV(), s.degreeU(), s.degreeV());
}

// ===========================================================================
// Writer
// ===========================================================================

/// Format a real in Part-21 syntax: the mantissa always contains a '.'
/// (ISO 10303-21 clause 6.4.2 — "1E-07" is invalid, "1.E-07" is required).
std::string fmtReal(double v) {
    char buf[32];
    std::snprintf(buf, sizeof(buf), "%.15g", v);
    std::string s(buf);
    if (s.find("inf") != std::string::npos || s.find("nan") != std::string::npos) return "0.";
    // Part-21 uses upper-case E for exponents.
    std::replace(s.begin(), s.end(), 'e', 'E');
    if (s.find('.') == std::string::npos) {
        const size_t e = s.find('E');
        if (e == std::string::npos) {
            s += '.';
        } else {
            s.insert(e, ".");
        }
    }
    return s;
}

/// Group a raw knot vector into (unique knots, multiplicities).
void groupKnots(const std::vector<double>& raw, std::vector<double>& knots,
                std::vector<int>& mults) {
    knots.clear();
    mults.clear();
    for (double k : raw) {
        if (!knots.empty() && std::abs(k - knots.back()) < 1e-12) {
            ++mults.back();
        } else {
            knots.push_back(k);
            mults.push_back(1);
        }
    }
}

bool allUnitWeights(const std::vector<double>& w) {
    return std::all_of(w.begin(), w.end(), [](double x) { return std::abs(x - 1.0) < 1e-12; });
}

/// Incremental Part-21 DATA-section builder with monotonically increasing ids.
class StepWriter {
public:
    /// Add an instance; @p rhs is everything right of "#id = ". Returns the id.
    int add(const std::string& rhs) {
        m_body << '#' << m_next << " = " << rhs << ";\n";
        return m_next++;
    }

    int addPoint(const Vec3& p) {
        return add("CARTESIAN_POINT('',(" + fmtReal(p.x) + "," + fmtReal(p.y) + "," + fmtReal(p.z) +
                   "))");
    }

    static std::string refList(const std::vector<int>& ids) {
        std::string s = "(";
        for (size_t i = 0; i < ids.size(); ++i) {
            if (i) s += ',';
            s += '#' + std::to_string(ids[i]);
        }
        return s + ")";
    }

    static std::string realList(const std::vector<double>& vals) {
        std::string s = "(";
        for (size_t i = 0; i < vals.size(); ++i) {
            if (i) s += ',';
            s += fmtReal(vals[i]);
        }
        return s + ")";
    }

    static std::string intList(const std::vector<int>& vals) {
        std::string s = "(";
        for (size_t i = 0; i < vals.size(); ++i) {
            if (i) s += ',';
            s += std::to_string(vals[i]);
        }
        return s + ")";
    }

    std::string body() const { return m_body.str(); }
    int nextId() const { return m_next; }

private:
    std::ostringstream m_body;
    int m_next = 1;
};

int writeCurve(StepWriter& w, const geo::NurbsCurve& c) {
    std::vector<int> cpIds;
    cpIds.reserve(c.controlPoints().size());
    for (const Vec3& p : c.controlPoints()) cpIds.push_back(w.addPoint(p));

    std::vector<double> knots;
    std::vector<int> mults;
    groupKnots(c.knots(), knots, mults);

    const std::string common =
        std::to_string(c.degree()) + "," + StepWriter::refList(cpIds) + ",.UNSPECIFIED.,.F.,.F.";
    const std::string knotPart =
        StepWriter::intList(mults) + "," + StepWriter::realList(knots) + ",.UNSPECIFIED.";

    if (allUnitWeights(c.weights())) {
        return w.add("B_SPLINE_CURVE_WITH_KNOTS(''," + common + "," + knotPart + ")");
    }
    // Rational curves need the Part-21 complex (multi-leaf) instance form.
    return w.add("(BOUNDED_CURVE() B_SPLINE_CURVE(" + common + ") B_SPLINE_CURVE_WITH_KNOTS(" +
                 knotPart + ") CURVE() GEOMETRIC_REPRESENTATION_ITEM() RATIONAL_B_SPLINE_CURVE(" +
                 StepWriter::realList(c.weights()) + ") REPRESENTATION_ITEM(''))");
}

int writeSurface(StepWriter& w, const geo::NurbsSurface& s) {
    // Control net rows: STEP lists control points as a list of lists [u][v].
    std::string net = "(";
    bool rational = false;
    std::string weights = "(";
    for (size_t iu = 0; iu < s.controlPoints().size(); ++iu) {
        if (iu) {
            net += ',';
            weights += ',';
        }
        std::vector<int> row;
        for (const Vec3& p : s.controlPoints()[iu]) row.push_back(w.addPoint(p));
        net += StepWriter::refList(row);
        weights += StepWriter::realList(s.weights()[iu]);
        if (!allUnitWeights(s.weights()[iu])) rational = true;
    }
    net += ")";
    weights += ")";

    std::vector<double> knotsU, knotsV;
    std::vector<int> multsU, multsV;
    groupKnots(s.knotsU(), knotsU, multsU);
    groupKnots(s.knotsV(), knotsV, multsV);

    const std::string common = std::to_string(s.degreeU()) + "," + std::to_string(s.degreeV()) +
                               "," + net + ",.UNSPECIFIED.,.F.,.F.,.F.";
    const std::string knotPart = StepWriter::intList(multsU) + "," + StepWriter::intList(multsV) +
                                 "," + StepWriter::realList(knotsU) + "," +
                                 StepWriter::realList(knotsV) + ",.UNSPECIFIED.";

    if (!rational) {
        return w.add("B_SPLINE_SURFACE_WITH_KNOTS(''," + common + "," + knotPart + ")");
    }
    return w.add("(BOUNDED_SURFACE() B_SPLINE_SURFACE(" + common +
                 ") B_SPLINE_SURFACE_WITH_KNOTS(" + knotPart +
                 ") GEOMETRIC_REPRESENTATION_ITEM() RATIONAL_B_SPLINE_SURFACE(" + weights +
                 ") REPRESENTATION_ITEM('') SURFACE())");
}

/// Emit one solid; returns one MANIFOLD_SOLID_BREP id per shell (Horizon
/// solids may hold several disjoint shells, e.g. Pattern results — STEP
/// expresses those as sibling MANIFOLD_SOLID_BREPs in one representation).
std::vector<int> writeSolid(StepWriter& w, const topo::Solid& solid, int index) {
    // Vertices.
    std::unordered_map<const topo::Vertex*, int> vertexIds;
    for (const auto& v : solid.vertices()) {
        vertexIds[&v] = w.add("VERTEX_POINT('',#" + std::to_string(w.addPoint(v.point)) + ")");
    }

    // Edges. Orient the written curve along the recorded half-edge direction.
    std::unordered_map<const topo::Edge*, int> edgeIds;
    for (const auto& e : solid.edges()) {
        if (e.halfEdge == nullptr || e.curve == nullptr) continue;
        const topo::Vertex* start = e.halfEdge->origin;
        const topo::Vertex* end = e.halfEdge->twin->origin;
        // same_sense: does the curve run start → end?
        const Vec3 c0 = e.curve->evaluate(e.curve->tMin());
        const bool senseForward = (c0 - start->point).length() <= (c0 - end->point).length();
        const int curveId = writeCurve(w, *e.curve);
        edgeIds[&e] = w.add("EDGE_CURVE('',#" + std::to_string(vertexIds.at(start)) + ",#" +
                            std::to_string(vertexIds.at(end)) + ",#" + std::to_string(curveId) +
                            "," + (senseForward ? ".T." : ".F.") + ")");
    }

    // Faces.
    auto writeFace = [&](const topo::Face& f) -> std::optional<int> {
        if (f.outerLoop == nullptr || f.surface == nullptr) return std::nullopt;

        auto writeLoop = [&](const topo::Wire* wire, bool outer) -> std::optional<int> {
            std::vector<int> oriented;
            const topo::HalfEdge* start = wire->halfEdge;
            const topo::HalfEdge* cur = start;
            do {
                if (cur->edge == nullptr) return std::nullopt;
                auto it = edgeIds.find(cur->edge);
                if (it == edgeIds.end()) return std::nullopt;
                const bool forward = (cur == cur->edge->halfEdge);
                oriented.push_back(w.add("ORIENTED_EDGE('',*,*,#" + std::to_string(it->second) +
                                         "," + (forward ? ".T." : ".F.") + ")"));
                cur = cur->next;
            } while (cur != nullptr && cur != start);
            const int loop = w.add("EDGE_LOOP(''," + StepWriter::refList(oriented) + ")");
            return w.add(std::string(outer ? "FACE_OUTER_BOUND" : "FACE_BOUND") + "('',#" +
                         std::to_string(loop) + ",.T.)");
        };

        std::vector<int> bounds;
        if (auto b = writeLoop(f.outerLoop, true)) bounds.push_back(*b);
        for (const topo::Wire* inner : f.innerLoops) {
            if (auto b = writeLoop(inner, false)) bounds.push_back(*b);
        }
        if (bounds.empty()) return std::nullopt;

        const int surfId = writeSurface(w, *f.surface);
        return w.add("ADVANCED_FACE(''," + StepWriter::refList(bounds) + ",#" +
                     std::to_string(surfId) + ",.T.)");
    };

    auto writeShell = [&](const std::vector<const topo::Face*>& faces,
                          int shellIndex) -> std::optional<int> {
        std::vector<int> faceIds;
        for (const topo::Face* f : faces) {
            if (f == nullptr) continue;
            if (auto id = writeFace(*f)) faceIds.push_back(*id);
        }
        if (faceIds.empty()) return std::nullopt;
        const int shell = w.add("CLOSED_SHELL(''," + StepWriter::refList(faceIds) + ")");
        return w.add("MANIFOLD_SOLID_BREP('solid_" + std::to_string(index) + "_" +
                     std::to_string(shellIndex) + "',#" + std::to_string(shell) + ")");
    };

    std::vector<int> msbIds;
    for (const auto& sh : solid.shells()) {
        std::vector<const topo::Face*> faces(sh.faces.begin(), sh.faces.end());
        if (auto msb = writeShell(faces, static_cast<int>(msbIds.size()))) {
            msbIds.push_back(*msb);
        }
    }
    if (msbIds.empty()) {
        // Defensive fallback for faces not registered with any shell.
        std::vector<const topo::Face*> faces;
        for (const auto& f : solid.faces()) faces.push_back(&f);
        if (auto msb = writeShell(faces, 0)) msbIds.push_back(*msb);
    }
    return msbIds;
}

// ===========================================================================
// Parser — Part-21 tokenizer and instance model
// ===========================================================================

struct StepValue;
using StepList = std::vector<StepValue>;

struct StepValue {
    // NOLINTNEXTLINE(readability-enum-initial-value)
    enum Kind { Null, Star, Real, Str, Enum, Ref, List, Typed };
    Kind kind = Null;
    double num = 0.0;
    std::string text;                 // Str payload, Enum name, Typed name
    int ref = 0;                      // Ref payload
    std::shared_ptr<StepList> items;  // List / Typed payload

    bool isRef() const { return kind == Ref; }
    bool isList() const { return kind == List; }
};

/// A parsed instance: one or more (entityType, args) leaves. Simple instances
/// have exactly one leaf; complex instances have several.
struct StepInstance {
    std::vector<std::pair<std::string, StepList>> leaves;

    const StepList* leaf(const std::string& type) const {
        for (const auto& [t, args] : leaves) {
            if (t == type) return &args;
        }
        return nullptr;
    }
    bool hasType(const std::string& type) const { return leaf(type) != nullptr; }
    const std::string& primaryType() const {
        static const std::string empty;
        return leaves.empty() ? empty : leaves.front().first;
    }
};

class StepParser {
public:
    explicit StepParser(const std::string& text) : m_text(text) {}

    /// Parse the DATA section into the instance map. False on hard error.
    bool parse(std::string& error) {
        std::string stripped = stripComments(m_text);
        const size_t dataPos = stripped.find("DATA;");
        if (dataPos == std::string::npos) {
            error = "no DATA section found";
            return false;
        }
        size_t pos = dataPos + 5;
        const size_t endPos = stripped.find("ENDSEC;", dataPos);
        const size_t limit = (endPos == std::string::npos) ? stripped.size() : endPos;

        while (pos < limit) {
            // Find next instance start.
            while (pos < limit && stripped[pos] != '#') ++pos;
            if (pos >= limit) break;
            // Split at the terminating ';' outside of strings.
            size_t end = pos;
            bool inString = false;
            while (end < limit) {
                const char c = stripped[end];
                if (c == '\'') inString = !inString;
                if (c == ';' && !inString) break;
                ++end;
            }
            if (end >= limit) break;
            if (!parseInstance(stripped, pos, end, error)) return false;
            pos = end + 1;
        }
        return true;
    }

    const StepInstance* find(int id) const {
        auto it = m_instances.find(id);
        return it == m_instances.end() ? nullptr : &it->second;
    }

    /// All ids whose instance contains a leaf of @p type, in file order.
    std::vector<int> allOfType(const std::string& type) const {
        std::vector<int> out;
        for (int id : m_order) {
            if (m_instances.at(id).hasType(type)) out.push_back(id);
        }
        return out;
    }

private:
    static std::string stripComments(const std::string& in) {
        std::string out;
        out.reserve(in.size());
        size_t i = 0;
        bool inString = false;
        while (i < in.size()) {
            if (!inString && i + 1 < in.size() && in[i] == '/' && in[i + 1] == '*') {
                const size_t close = in.find("*/", i + 2);
                if (close == std::string::npos) break;
                i = close + 2;
                continue;
            }
            if (in[i] == '\'') inString = !inString;
            out += in[i++];
        }
        return out;
    }

    bool parseInstance(const std::string& s, size_t pos, size_t end, std::string& error) {
        // "#id = RHS" — RHS is TYPE(args) or (TYPE1(args) TYPE2(args) ...).
        size_t p = pos + 1;
        int id = 0;
        while (p < end && std::isdigit(static_cast<unsigned char>(s[p]))) {
            id = id * 10 + (s[p] - '0');
            ++p;
        }
        while (p < end &&
               (s[p] == ' ' || s[p] == '=' || s[p] == '\n' || s[p] == '\r' || s[p] == '\t')) {
            ++p;
        }
        if (p >= end || id == 0) {
            error = "malformed instance near offset " + std::to_string(pos);
            return false;
        }

        StepInstance inst;
        if (s[p] == '(') {
            // Complex instance: sequence of leaves inside the outer parens.
            ++p;
            while (p < end) {
                skipWs(s, p, end);
                if (p < end && s[p] == ')') break;
                if (!parseLeaf(s, p, end, inst, error)) return false;
            }
        } else {
            if (!parseLeaf(s, p, end, inst, error)) return false;
        }
        m_instances.emplace(id, std::move(inst));
        m_order.push_back(id);
        return true;
    }

    static void skipWs(const std::string& s, size_t& p, size_t end) {
        while (p < end && std::isspace(static_cast<unsigned char>(s[p]))) ++p;
    }

    bool parseLeaf(const std::string& s, size_t& p, size_t end, StepInstance& inst,
                   std::string& error) {
        skipWs(s, p, end);
        std::string type;
        while (p < end && (std::isalnum(static_cast<unsigned char>(s[p])) || s[p] == '_')) {
            type += s[p++];
        }
        skipWs(s, p, end);
        if (type.empty() || p >= end || s[p] != '(') {
            error = "expected entity leaf";
            return false;
        }
        ++p;  // consume '('
        StepList args;
        if (!parseArgs(s, p, end, args, error)) return false;
        inst.leaves.emplace_back(std::move(type), std::move(args));
        return true;
    }

    /// Parse a comma-separated argument list; consumes the closing ')'.
    bool parseArgs(const std::string& s, size_t& p, size_t end, StepList& out, std::string& error) {
        // parseArgs and parseValue are mutually recursive; adversarial files
        // with thousands of nested '(' would otherwise overflow the stack.
        if (m_depth >= kMaxNesting) {
            error = "argument nesting too deep";
            return false;
        }
        ++m_depth;
        const bool ok = parseArgsInner(s, p, end, out, error);
        --m_depth;
        return ok;
    }

    bool parseArgsInner(const std::string& s, size_t& p, size_t end, StepList& out,
                        std::string& error) {
        while (p < end) {
            skipWs(s, p, end);
            if (p >= end) break;
            if (s[p] == ')') {
                ++p;
                return true;
            }
            if (s[p] == ',') {
                ++p;
                continue;
            }
            StepValue v;
            if (!parseValue(s, p, end, v, error)) return false;
            out.push_back(std::move(v));
        }
        error = "unterminated argument list";
        return false;
    }

    bool parseValue(const std::string& s, size_t& p, size_t end, StepValue& v, std::string& error) {
        skipWs(s, p, end);
        if (p >= end) {
            error = "unexpected end of input";
            return false;
        }
        const char c = s[p];
        if (c == '$') {
            v.kind = StepValue::Null;
            ++p;
            return true;
        }
        if (c == '*') {
            v.kind = StepValue::Star;
            ++p;
            return true;
        }
        if (c == '#') {
            ++p;
            int id = 0;
            while (p < end && std::isdigit(static_cast<unsigned char>(s[p]))) {
                id = id * 10 + (s[p] - '0');
                ++p;
            }
            v.kind = StepValue::Ref;
            v.ref = id;
            return true;
        }
        if (c == '\'') {
            ++p;
            std::string str;
            while (p < end) {
                if (s[p] == '\'') {
                    if (p + 1 < end && s[p + 1] == '\'') {  // escaped quote
                        str += '\'';
                        p += 2;
                        continue;
                    }
                    ++p;
                    break;
                }
                str += s[p++];
            }
            v.kind = StepValue::Str;
            v.text = std::move(str);
            return true;
        }
        if (c == '.') {
            ++p;
            std::string name;
            while (p < end && s[p] != '.') name += s[p++];
            if (p < end) ++p;  // closing '.'
            v.kind = StepValue::Enum;
            v.text = std::move(name);
            return true;
        }
        if (c == '(') {
            ++p;
            v.kind = StepValue::List;
            v.items = std::make_shared<StepList>();
            return parseArgs(s, p, end, *v.items, error);
        }
        if (std::isdigit(static_cast<unsigned char>(c)) || c == '-' || c == '+') {
            size_t q = p;
            while (q < end && (std::isdigit(static_cast<unsigned char>(s[q])) || s[q] == '.' ||
                               s[q] == '-' || s[q] == '+' || s[q] == 'e' || s[q] == 'E')) {
                ++q;
            }
            v.kind = StepValue::Real;
            v.num = std::strtod(s.substr(p, q - p).c_str(), nullptr);
            p = q;
            return true;
        }
        if (std::isalpha(static_cast<unsigned char>(c)) || c == '_') {
            // Typed nested value: NAME(args).
            std::string name;
            while (p < end && (std::isalnum(static_cast<unsigned char>(s[p])) || s[p] == '_')) {
                name += s[p++];
            }
            skipWs(s, p, end);
            if (p < end && s[p] == '(') {
                ++p;
                v.kind = StepValue::Typed;
                v.text = std::move(name);
                v.items = std::make_shared<StepList>();
                return parseArgs(s, p, end, *v.items, error);
            }
            error = "unexpected token '" + name + "'";
            return false;
        }
        error = std::string("unexpected character '") + c + "'";
        return false;
    }

    static constexpr int kMaxNesting = 64;

    const std::string& m_text;
    std::unordered_map<int, StepInstance> m_instances;
    std::vector<int> m_order;
    int m_depth = 0;
};

// ===========================================================================
// Reconstruction — STEP entities → hz::topo::Solid
// ===========================================================================

class SolidBuilder {
public:
    SolidBuilder(const StepParser& parser, int solidIndex)
        : m_parser(parser), m_solidIndex(solidIndex) {}

    /// Build one topo::Solid from a group of MANIFOLD_SOLID_BREP instance ids
    /// (one shell per MSB — Horizon multi-shell solids export as siblings in
    /// one shape representation).
    std::unique_ptr<topo::Solid> build(const std::vector<int>& msbIds, std::string& error) {
        m_solid = std::make_unique<topo::Solid>();

        for (int msbId : msbIds) {
            const StepInstance* msb = m_parser.find(msbId);
            const StepList* args = msb ? msb->leaf("MANIFOLD_SOLID_BREP") : nullptr;
            if (args == nullptr || args->size() < 2 || !(*args)[1].isRef()) {
                error = "malformed MANIFOLD_SOLID_BREP #" + std::to_string(msbId);
                return nullptr;
            }
            const StepInstance* shellInst = m_parser.find((*args)[1].ref);
            const StepList* shellArgs = shellInst ? shellInst->leaf("CLOSED_SHELL") : nullptr;
            if (shellArgs == nullptr) {
                shellArgs = shellInst ? shellInst->leaf("OPEN_SHELL") : nullptr;
            }
            if (shellArgs == nullptr || shellArgs->size() < 2 || !(*shellArgs)[1].isList()) {
                error = "malformed shell for MANIFOLD_SOLID_BREP #" + std::to_string(msbId);
                return nullptr;
            }

            topo::Shell* shell = m_solid->allocShell();
            shell->solid = m_solid.get();

            for (const StepValue& faceRef : *(*shellArgs)[1].items) {
                if (!faceRef.isRef()) continue;
                if (!buildFace(faceRef.ref, shell, error)) return nullptr;
            }
        }

        if (!linkTwins(error)) return nullptr;

        if (!m_solid->isValid()) {
            error = "imported solid failed validation:\n" + m_solid->validationReport();
            return nullptr;
        }
        return std::move(m_solid);
    }

private:
    // -- Geometry ------------------------------------------------------------

    std::optional<Vec3> readPoint(int id) const {
        const StepInstance* inst = m_parser.find(id);
        const StepList* args = inst ? inst->leaf("CARTESIAN_POINT") : nullptr;
        if (args == nullptr || args->size() < 2 || !(*args)[1].isList() ||
            (*args)[1].items->size() < 3) {
            return std::nullopt;
        }
        const StepList& c = *(*args)[1].items;
        return Vec3{c[0].num, c[1].num, c[2].num};
    }

    std::optional<Vec3> readDirection(int id) const {
        const StepInstance* inst = m_parser.find(id);
        const StepList* args = inst ? inst->leaf("DIRECTION") : nullptr;
        if (args == nullptr || args->size() < 2 || !(*args)[1].isList() ||
            (*args)[1].items->size() < 3) {
            return std::nullopt;
        }
        const StepList& c = *(*args)[1].items;
        return Vec3{c[0].num, c[1].num, c[2].num};
    }

    /// AXIS2_PLACEMENT_3D → (origin, zAxis, xAxis).
    bool readPlacement(int id, Vec3& origin, Vec3& zAxis, Vec3& xAxis) const {
        const StepInstance* inst = m_parser.find(id);
        const StepList* args = inst ? inst->leaf("AXIS2_PLACEMENT_3D") : nullptr;
        if (args == nullptr || args->size() < 3) return false;
        auto o = (*args)[1].isRef() ? readPoint((*args)[1].ref) : std::nullopt;
        if (!o) return false;
        origin = *o;
        zAxis = Vec3{0, 0, 1};
        xAxis = Vec3{1, 0, 0};
        if ((*args)[2].isRef()) {
            if (auto z = readDirection((*args)[2].ref)) zAxis = z->normalized();
        }
        if (args->size() > 3 && (*args)[3].isRef()) {
            if (auto x = readDirection((*args)[3].ref)) xAxis = x->normalized();
        }
        // Re-orthogonalize X against Z; when the reference direction is absent
        // or (near-)parallel to the axis, derive any perpendicular instead.
        xAxis = xAxis - zAxis * xAxis.dot(zAxis);
        if (xAxis.length() < 1e-9) {
            const Vec3 seed = std::abs(zAxis.x) < 0.9 ? Vec3{1, 0, 0} : Vec3{0, 1, 0};
            xAxis = seed - zAxis * seed.dot(zAxis);
        }
        xAxis = xAxis.normalized();
        return true;
    }

    static std::vector<double> expandKnots(const StepList& mults, const StepList& knots) {
        // Multiplicities come straight from the file — bound them before
        // allocating, or a single absurd value drives an OOM.
        std::vector<double> out;
        size_t total = 0;
        for (size_t i = 0; i < mults.size() && i < knots.size(); ++i) {
            const double raw = mults[i].num;
            if (!(raw >= 1.0 && raw <= 1e4)) return {};
            const auto m = static_cast<size_t>(raw);
            total += m;
            if (total > 100000) return {};
            for (size_t j = 0; j < m; ++j) out.push_back(knots[i].num);
        }
        return out;
    }

    std::shared_ptr<geo::NurbsCurve> readBSplineCurve(const StepInstance& inst) const {
        // Attribute layout differs between the simple and complex forms.
        const StepList* simple = inst.leaf("B_SPLINE_CURVE_WITH_KNOTS");
        const StepList* core = inst.leaf("B_SPLINE_CURVE");
        const StepList* rational = inst.leaf("RATIONAL_B_SPLINE_CURVE");

        int degree = 0;
        const StepList* cpList = nullptr;
        const StepList* multsL = nullptr;
        const StepList* knotsL = nullptr;

        if (core != nullptr && simple != nullptr) {
            // Complex instance: B_SPLINE_CURVE(degree, cps, ...) +
            // B_SPLINE_CURVE_WITH_KNOTS(mults, knots, spec).
            if (core->size() < 2 || !(*core)[1].isList()) return nullptr;
            degree = static_cast<int>((*core)[0].num);
            cpList = (*core)[1].items.get();
            if (simple->size() < 2 || !(*simple)[0].isList() || !(*simple)[1].isList()) {
                return nullptr;
            }
            multsL = (*simple)[0].items.get();
            knotsL = (*simple)[1].items.get();
        } else if (simple != nullptr) {
            // Simple: ('', degree, cps, form, closed, selfint, mults, knots, spec).
            if (simple->size() < 8 || !(*simple)[2].isList() || !(*simple)[6].isList() ||
                !(*simple)[7].isList()) {
                return nullptr;
            }
            degree = static_cast<int>((*simple)[1].num);
            cpList = (*simple)[2].items.get();
            multsL = (*simple)[6].items.get();
            knotsL = (*simple)[7].items.get();
        } else {
            return nullptr;
        }

        std::vector<Vec3> cps;
        for (const StepValue& r : *cpList) {
            if (!r.isRef()) return nullptr;
            auto p = readPoint(r.ref);
            if (!p) return nullptr;
            cps.push_back(*p);
        }
        std::vector<double> weights(cps.size(), 1.0);
        if (rational != nullptr && !rational->empty() && (*rational)[0].isList()) {
            const StepList& wl = *(*rational)[0].items;
            for (size_t i = 0; i < wl.size() && i < weights.size(); ++i) weights[i] = wl[i].num;
        }
        std::vector<double> knots = expandKnots(*multsL, *knotsL);
        if (cps.size() < 2 || knots.size() != cps.size() + degree + 1) return nullptr;
        return std::make_shared<geo::NurbsCurve>(std::move(cps), std::move(weights),
                                                 std::move(knots), degree);
    }

    std::shared_ptr<geo::NurbsSurface> readBSplineSurface(const StepInstance& inst) const {
        const StepList* simple = inst.leaf("B_SPLINE_SURFACE_WITH_KNOTS");
        const StepList* core = inst.leaf("B_SPLINE_SURFACE");
        const StepList* rational = inst.leaf("RATIONAL_B_SPLINE_SURFACE");

        int degU = 0;
        int degV = 0;
        const StepList* net = nullptr;
        const StepList* multsU = nullptr;
        const StepList* multsV = nullptr;
        const StepList* knotsU = nullptr;
        const StepList* knotsV = nullptr;

        if (core != nullptr && simple != nullptr) {
            if (core->size() < 3 || !(*core)[2].isList()) return nullptr;
            degU = static_cast<int>((*core)[0].num);
            degV = static_cast<int>((*core)[1].num);
            net = (*core)[2].items.get();
            if (simple->size() < 4 || !(*simple)[0].isList() || !(*simple)[1].isList() ||
                !(*simple)[2].isList() || !(*simple)[3].isList()) {
                return nullptr;
            }
            multsU = (*simple)[0].items.get();
            multsV = (*simple)[1].items.get();
            knotsU = (*simple)[2].items.get();
            knotsV = (*simple)[3].items.get();
        } else if (simple != nullptr) {
            // ('', degU, degV, net, form, uClosed, vClosed, selfint,
            //  multsU, multsV, knotsU, knotsV, spec).
            if (simple->size() < 12 || !(*simple)[3].isList()) return nullptr;
            degU = static_cast<int>((*simple)[1].num);
            degV = static_cast<int>((*simple)[2].num);
            net = (*simple)[3].items.get();
            if (!(*simple)[8].isList() || !(*simple)[9].isList() || !(*simple)[10].isList() ||
                !(*simple)[11].isList()) {
                return nullptr;
            }
            multsU = (*simple)[8].items.get();
            multsV = (*simple)[9].items.get();
            knotsU = (*simple)[10].items.get();
            knotsV = (*simple)[11].items.get();
        } else {
            return nullptr;
        }

        std::vector<std::vector<Vec3>> cps;
        for (const StepValue& row : *net) {
            if (!row.isList()) return nullptr;
            std::vector<Vec3> r;
            for (const StepValue& v : *row.items) {
                if (!v.isRef()) return nullptr;
                auto p = readPoint(v.ref);
                if (!p) return nullptr;
                r.push_back(*p);
            }
            cps.push_back(std::move(r));
        }
        if (cps.empty() || cps.front().empty()) return nullptr;

        std::vector<std::vector<double>> weights(cps.size(),
                                                 std::vector<double>(cps.front().size(), 1.0));
        if (rational != nullptr && !rational->empty() && (*rational)[0].isList()) {
            const StepList& rows = *(*rational)[0].items;
            for (size_t i = 0; i < rows.size() && i < weights.size(); ++i) {
                if (!rows[i].isList()) continue;
                const StepList& wr = *rows[i].items;
                for (size_t j = 0; j < wr.size() && j < weights[i].size(); ++j) {
                    weights[i][j] = wr[j].num;
                }
            }
        }

        std::vector<double> ku = expandKnots(*multsU, *knotsU);
        std::vector<double> kv = expandKnots(*multsV, *knotsV);
        if (ku.size() != cps.size() + degU + 1 || kv.size() != cps.front().size() + degV + 1) {
            return nullptr;
        }
        return std::make_shared<geo::NurbsSurface>(std::move(cps), std::move(weights),
                                                   std::move(ku), std::move(kv), degU, degV);
    }

    /// Curve for an EDGE_CURVE, given the edge's endpoint positions.
    std::shared_ptr<geo::NurbsCurve> readEdgeGeometry(int curveId, const Vec3& start,
                                                      const Vec3& end, int depth = 0) const {
        const StepInstance* inst = m_parser.find(curveId);
        if (inst == nullptr || depth > 4) return nullptr;

        // OCC-style writers (FreeCAD et al.) wrap the 3D geometry: a
        // SURFACE_CURVE / SEAM_CURVE carries the real curve as its curve_3d
        // attribute. Depth-limit the hop to survive self-referential files.
        for (const char* wrapper : {"SURFACE_CURVE", "SEAM_CURVE"}) {
            if (const StepList* sc = inst->leaf(wrapper)) {
                if (sc->size() >= 2 && (*sc)[1].isRef()) {
                    return readEdgeGeometry((*sc)[1].ref, start, end, depth + 1);
                }
                return nullptr;
            }
        }

        if (inst->hasType("B_SPLINE_CURVE_WITH_KNOTS") || inst->hasType("B_SPLINE_CURVE")) {
            return readBSplineCurve(*inst);
        }
        if (inst->hasType("LINE")) {
            // STEP lines are unbounded; the edge vertices bound them exactly.
            return std::make_shared<geo::NurbsCurve>(std::vector<Vec3>{start, end},
                                                     std::vector<double>{1.0, 1.0},
                                                     std::vector<double>{0.0, 0.0, 1.0, 1.0}, 1);
        }
        if (const StepList* circ = inst->leaf("CIRCLE")) {
            if (circ->size() < 3 || !(*circ)[1].isRef()) return nullptr;
            Vec3 center;
            Vec3 zAxis;
            Vec3 xAxis;
            if (!readPlacement((*circ)[1].ref, center, zAxis, xAxis)) return nullptr;
            const double radius = (*circ)[2].num;
            const Vec3 yAxis = zAxis.cross(xAxis);
            if ((start - end).length() < kMergeTol) {
                // Closed seam edge: anchor the reconstructed curve at the
                // recorded seam vertex, not at the placement X-axis.
                const Vec3 d = start - center;
                const double a0 =
                    (d.length() > kMergeTol) ? std::atan2(d.dot(yAxis), d.dot(xAxis)) : 0.0;
                return makeArcInFrame(center, radius, a0, a0 + math::kTwoPi, xAxis, yAxis);
            }
            const Vec3 ds = start - center;
            const Vec3 de = end - center;
            const double a0 = std::atan2(ds.dot(yAxis), ds.dot(xAxis));
            const double a1 = std::atan2(de.dot(yAxis), de.dot(xAxis));
            return makeArcInFrame(center, radius, a0, a1, xAxis, yAxis);
        }
        return nullptr;
    }

    /// Surface for an ADVANCED_FACE; @p loopPoints bounds analytic surfaces.
    std::shared_ptr<geo::NurbsSurface> readFaceGeometry(int surfId,
                                                        const std::vector<Vec3>& loopPoints) const {
        const StepInstance* inst = m_parser.find(surfId);
        if (inst == nullptr) return nullptr;

        if (inst->hasType("B_SPLINE_SURFACE_WITH_KNOTS") || inst->hasType("B_SPLINE_SURFACE")) {
            return readBSplineSurface(*inst);
        }
        if (const StepList* plane = inst->leaf("PLANE")) {
            if (plane->size() < 2 || !(*plane)[1].isRef() || loopPoints.empty()) return nullptr;
            Vec3 origin;
            Vec3 zAxis;
            Vec3 xAxis;
            if (!readPlacement((*plane)[1].ref, origin, zAxis, xAxis)) return nullptr;
            const Vec3 yAxis = zAxis.cross(xAxis);
            double uMin = 1e300;
            double uMax = -1e300;
            double vMin = 1e300;
            double vMax = -1e300;
            for (const Vec3& p : loopPoints) {
                const Vec3 d = p - origin;
                uMin = std::min(uMin, d.dot(xAxis));
                uMax = std::max(uMax, d.dot(xAxis));
                vMin = std::min(vMin, d.dot(yAxis));
                vMax = std::max(vMax, d.dot(yAxis));
            }
            const double uSize = std::max(uMax - uMin, kMergeTol);
            const double vSize = std::max(vMax - vMin, kMergeTol);
            return std::make_shared<geo::NurbsSurface>(geo::NurbsSurface::makePlane(
                origin + xAxis * uMin + yAxis * vMin, xAxis, yAxis, uSize, vSize));
        }
        if (const StepList* cyl = inst->leaf("CYLINDRICAL_SURFACE")) {
            if (cyl->size() < 3 || !(*cyl)[1].isRef() || loopPoints.empty()) return nullptr;
            Vec3 origin;
            Vec3 zAxis;
            Vec3 xAxis;
            if (!readPlacement((*cyl)[1].ref, origin, zAxis, xAxis)) return nullptr;
            const double radius = (*cyl)[2].num;
            double hMin = 1e300;
            double hMax = -1e300;
            for (const Vec3& p : loopPoints) {
                const double h = (p - origin).dot(zAxis);
                hMin = std::min(hMin, h);
                hMax = std::max(hMax, h);
            }
            const double height = std::max(hMax - hMin, kMergeTol);
            return std::make_shared<geo::NurbsSurface>(
                geo::NurbsSurface::makeCylinder(origin + zAxis * hMin, zAxis, radius, height));
        }
        return nullptr;
    }

    // -- Topology --------------------------------------------------------------

    topo::Vertex* vertexFor(int vertexPointId) {
        auto it = m_vertices.find(vertexPointId);
        if (it != m_vertices.end()) return it->second;
        const StepInstance* inst = m_parser.find(vertexPointId);
        const StepList* args = inst ? inst->leaf("VERTEX_POINT") : nullptr;
        if (args == nullptr || args->size() < 2 || !(*args)[1].isRef()) return nullptr;
        auto p = readPoint((*args)[1].ref);
        if (!p) return nullptr;
        topo::Vertex* v = m_solid->allocVertex();
        v->point = *p;
        v->topoId = topo::TopologyID::make("step", "solid:" + std::to_string(m_solidIndex) +
                                                       "/v:" + std::to_string(vertexPointId));
        m_vertices.emplace(vertexPointId, v);
        return v;
    }

    struct EdgeRecord {
        topo::Edge* edge = nullptr;
        topo::Vertex* start = nullptr;
        topo::Vertex* end = nullptr;
        std::vector<topo::HalfEdge*> uses;  // half-edges referencing this edge
        std::vector<bool> forward;          // per-use: runs start → end?
    };

    EdgeRecord* edgeFor(int edgeCurveId, std::string& error) {
        auto it = m_edges.find(edgeCurveId);
        if (it != m_edges.end()) return &it->second;

        const StepInstance* inst = m_parser.find(edgeCurveId);
        const StepList* args = inst ? inst->leaf("EDGE_CURVE") : nullptr;
        if (args == nullptr || args->size() < 5 || !(*args)[1].isRef() || !(*args)[2].isRef() ||
            !(*args)[3].isRef()) {
            error = "malformed EDGE_CURVE #" + std::to_string(edgeCurveId);
            return nullptr;
        }
        topo::Vertex* start = vertexFor((*args)[1].ref);
        topo::Vertex* end = vertexFor((*args)[2].ref);
        if (start == nullptr || end == nullptr) {
            error = "EDGE_CURVE #" + std::to_string(edgeCurveId) + " has invalid vertices";
            return nullptr;
        }
        const bool sameSense = (*args)[4].kind == StepValue::Enum && (*args)[4].text == "T";
        auto curve = readEdgeGeometry((*args)[3].ref, sameSense ? start->point : end->point,
                                      sameSense ? end->point : start->point);
        if (curve == nullptr) {
            error = "unsupported curve geometry on EDGE_CURVE #" + std::to_string(edgeCurveId);
            return nullptr;
        }

        topo::Edge* e = m_solid->allocEdge();
        e->curve = std::move(curve);
        e->topoId = topo::TopologyID::make(
            "step", "solid:" + std::to_string(m_solidIndex) + "/e:" + std::to_string(edgeCurveId));
        EdgeRecord rec;
        rec.edge = e;
        rec.start = start;
        rec.end = end;
        auto [ins, ok] = m_edges.emplace(edgeCurveId, std::move(rec));
        (void)ok;
        return &ins->second;
    }

    bool buildFace(int faceId, topo::Shell* shell, std::string& error) {
        const StepInstance* inst = m_parser.find(faceId);
        const StepList* args = inst ? inst->leaf("ADVANCED_FACE") : nullptr;
        if (args == nullptr) args = inst ? inst->leaf("FACE_SURFACE") : nullptr;
        if (args == nullptr || args->size() < 3 || !(*args)[1].isList() || !(*args)[2].isRef()) {
            error = "malformed face #" + std::to_string(faceId);
            return false;
        }

        topo::Face* face = m_solid->allocFace();
        face->shell = shell;
        face->topoId = topo::TopologyID::make(
            "step", "solid:" + std::to_string(m_solidIndex) + "/f:" + std::to_string(faceId));
        shell->faces.push_back(face);

        std::vector<Vec3> loopPoints;

        for (const StepValue& boundRef : *(*args)[1].items) {
            if (!boundRef.isRef()) continue;
            const StepInstance* bound = m_parser.find(boundRef.ref);
            const StepList* bArgs = bound ? bound->leaf("FACE_OUTER_BOUND") : nullptr;
            const bool isOuter = bArgs != nullptr;
            if (bArgs == nullptr) bArgs = bound ? bound->leaf("FACE_BOUND") : nullptr;
            if (bArgs == nullptr || bArgs->size() < 3 || !(*bArgs)[1].isRef()) {
                error = "malformed face bound on face #" + std::to_string(faceId);
                return false;
            }
            const bool boundOrientation =
                (*bArgs)[2].kind == StepValue::Enum && (*bArgs)[2].text == "T";

            const StepInstance* loop = m_parser.find((*bArgs)[1].ref);
            const StepList* lArgs = loop ? loop->leaf("EDGE_LOOP") : nullptr;
            if (lArgs == nullptr || lArgs->size() < 2 || !(*lArgs)[1].isList()) {
                error = "malformed EDGE_LOOP on face #" + std::to_string(faceId);
                return false;
            }

            // Collect (edge, forward) pairs in loop order; honour bound orientation.
            std::vector<std::pair<EdgeRecord*, bool>> loopEdges;
            for (const StepValue& oeRef : *(*lArgs)[1].items) {
                if (!oeRef.isRef()) continue;
                const StepInstance* oe = m_parser.find(oeRef.ref);
                const StepList* oeArgs = oe ? oe->leaf("ORIENTED_EDGE") : nullptr;
                if (oeArgs == nullptr || oeArgs->size() < 5 || !(*oeArgs)[3].isRef()) {
                    error = "malformed ORIENTED_EDGE on face #" + std::to_string(faceId);
                    return false;
                }
                EdgeRecord* rec = edgeFor((*oeArgs)[3].ref, error);
                if (rec == nullptr) return false;
                const bool fwd = (*oeArgs)[4].kind == StepValue::Enum && (*oeArgs)[4].text == "T";
                loopEdges.emplace_back(rec, fwd);
            }
            if (!boundOrientation) {
                std::reverse(loopEdges.begin(), loopEdges.end());
                for (auto& [rec, fwd] : loopEdges) fwd = !fwd;
            }
            if (loopEdges.empty()) {
                error = "empty edge loop on face #" + std::to_string(faceId);
                return false;
            }

            // Materialize half-edges for this loop.
            topo::Wire* wire = m_solid->allocWire();
            std::vector<topo::HalfEdge*> hes;
            for (auto& [rec, fwd] : loopEdges) {
                topo::HalfEdge* he = m_solid->allocHalfEdge();
                he->origin = fwd ? rec->start : rec->end;
                he->edge = rec->edge;
                he->face = face;
                rec->uses.push_back(he);
                rec->forward.push_back(fwd);
                hes.push_back(he);
                loopPoints.push_back(he->origin->point);
                // Analytic carrier surfaces are sized from the loop extent;
                // curve control hulls bound curved edges (vertices alone
                // under-span e.g. a circular cap, whose single seam vertex
                // would collapse the patch to a point).
                if (rec->edge->curve != nullptr) {
                    for (const Vec3& p : rec->edge->curve->controlPoints()) {
                        loopPoints.push_back(p);
                    }
                }
            }
            for (size_t i = 0; i < hes.size(); ++i) {
                hes[i]->next = hes[(i + 1) % hes.size()];
                hes[i]->prev = hes[(i + hes.size() - 1) % hes.size()];
                if (hes[i]->origin->halfEdge == nullptr) hes[i]->origin->halfEdge = hes[i];
            }
            wire->halfEdge = hes.front();
            if (isOuter && face->outerLoop == nullptr) {
                face->outerLoop = wire;
            } else {
                face->innerLoops.push_back(wire);
            }
        }

        if (face->outerLoop == nullptr) {
            // Files that use plain FACE_BOUND for the outer loop: promote the
            // first inner loop.
            if (face->innerLoops.empty()) {
                error = "face #" + std::to_string(faceId) + " has no bounds";
                return false;
            }
            face->outerLoop = face->innerLoops.front();
            face->innerLoops.erase(face->innerLoops.begin());
        }

        face->surface = readFaceGeometry((*args)[2].ref, loopPoints);
        if (face->surface == nullptr) {
            error = "unsupported surface geometry on face #" + std::to_string(faceId);
            return false;
        }

        // ADVANCED_FACE same_sense: .F. means the face normal is the reverse
        // of the surface normal.  The kernel has no per-face sense flag (its
        // convention is surface normal == outward face normal), so bake the
        // flip into the surface itself.
        const bool sameSenseFace =
            args->size() < 4 || (*args)[3].kind != StepValue::Enum || (*args)[3].text != "F";
        if (!sameSenseFace) {
            face->surface = reverseSurfaceU(*face->surface);
        }
        return true;
    }

    bool linkTwins(std::string& error) {
        for (auto& [id, rec] : m_edges) {
            if (rec.uses.size() != 2) {
                error = "edge #" + std::to_string(id) + " used by " +
                        std::to_string(rec.uses.size()) + " loops (manifold solids need 2)";
                return false;
            }
            if (rec.forward[0] == rec.forward[1]) {
                error = "edge #" + std::to_string(id) + " has inconsistent loop orientations";
                return false;
            }
            rec.uses[0]->twin = rec.uses[1];
            rec.uses[1]->twin = rec.uses[0];
            // Record direction: the forward half-edge (start → end).
            rec.edge->halfEdge = rec.forward[0] ? rec.uses[0] : rec.uses[1];
        }
        return true;
    }

    const StepParser& m_parser;
    int m_solidIndex;
    std::unique_ptr<topo::Solid> m_solid;
    std::unordered_map<int, topo::Vertex*> m_vertices;
    std::map<int, EdgeRecord> m_edges;
};

}  // namespace

// ===========================================================================
// Public API
// ===========================================================================

std::string StepFormat::toString(const std::vector<const topo::Solid*>& solids) {
    StepWriter w;

    // Geometric representation context (SI millimetres) shared by all solids.
    const int lengthUnit = w.add("(LENGTH_UNIT() NAMED_UNIT(*) SI_UNIT(.MILLI.,.METRE.))");
    const int angleUnit = w.add("(NAMED_UNIT(*) PLANE_ANGLE_UNIT() SI_UNIT($,.RADIAN.))");
    const int solidAngleUnit = w.add("(NAMED_UNIT(*) SI_UNIT($,.STERADIAN.) SOLID_ANGLE_UNIT())");
    const int uncertainty =
        w.add("UNCERTAINTY_MEASURE_WITH_UNIT(LENGTH_MEASURE(1.E-7),#" + std::to_string(lengthUnit) +
              ",'distance_accuracy_value','confusion accuracy')");
    const int context = w.add(
        "(GEOMETRIC_REPRESENTATION_CONTEXT(3) GLOBAL_UNCERTAINTY_ASSIGNED_CONTEXT((#" +
        std::to_string(uncertainty) + ")) GLOBAL_UNIT_ASSIGNED_CONTEXT((#" +
        std::to_string(lengthUnit) + ",#" + std::to_string(angleUnit) + ",#" +
        std::to_string(solidAngleUnit) + ")) REPRESENTATION_CONTEXT('Context #1','3D Context'))");

    const int appContext = w.add("APPLICATION_CONTEXT('managed model based 3d engineering')");
    w.add(
        "APPLICATION_PROTOCOL_DEFINITION('international standard',"
        "'ap242_managed_model_based_3d_engineering',2020,#" +
        std::to_string(appContext) + ")");

    const int productContext =
        w.add("PRODUCT_CONTEXT('',#" + std::to_string(appContext) + ",'mechanical')");

    int index = 0;
    for (const topo::Solid* solid : solids) {
        if (solid == nullptr) continue;
        const std::vector<int> msbs = writeSolid(w, *solid, index);
        if (msbs.empty()) continue;

        const std::string name = "'part_" + std::to_string(index) + "'";
        const int product = w.add("PRODUCT(" + name + "," + name + ",'',(#" +
                                  std::to_string(productContext) + "))");
        w.add("PRODUCT_RELATED_PRODUCT_CATEGORY('part',$,(#" + std::to_string(product) + "))");
        const int formation =
            w.add("PRODUCT_DEFINITION_FORMATION('',$,#" + std::to_string(product) + ")");
        const int pdc = w.add("PRODUCT_DEFINITION_CONTEXT('part definition',#" +
                              std::to_string(appContext) + ",'design')");
        const int pd = w.add("PRODUCT_DEFINITION('design',$,#" + std::to_string(formation) + ",#" +
                             std::to_string(pdc) + ")");
        const int pds = w.add("PRODUCT_DEFINITION_SHAPE('',$,#" + std::to_string(pd) + ")");
        const int rep = w.add("ADVANCED_BREP_SHAPE_REPRESENTATION(" + name + "," +
                              StepWriter::refList(msbs) + ",#" + std::to_string(context) + ")");
        w.add("SHAPE_DEFINITION_REPRESENTATION(#" + std::to_string(pds) + ",#" +
              std::to_string(rep) + ")");
        ++index;
    }

    std::ostringstream out;
    out << "ISO-10303-21;\n"
        << "HEADER;\n"
        << "FILE_DESCRIPTION(('Horizon CAD B-Rep model'),'2;1');\n"
        << "FILE_NAME('','',('Horizon CAD'),(''),'Horizon STEP writer','Horizon CAD','');\n"
        << "FILE_SCHEMA(('AP242_MANAGED_MODEL_BASED_3D_ENGINEERING_MIM_LF { 1 0 10303 442 3 1 "
           "4 }'));\n"
        << "ENDSEC;\n"
        << "DATA;\n"
        << w.body() << "ENDSEC;\n"
        << "END-ISO-10303-21;\n";
    return out.str();
}

bool StepFormat::save(const std::string& filePath, const std::vector<const topo::Solid*>& solids) {
    g_lastError.clear();
    if (solids.empty()) {
        g_lastError = "no solids to export";
        return false;
    }
    std::ofstream file(filePath, std::ios::binary);
    if (!file) {
        g_lastError = "cannot open file for writing: " + filePath;
        return false;
    }
    file << toString(solids);
    return file.good();
}

std::vector<std::unique_ptr<topo::Solid>> StepFormat::fromString(const std::string& text) {
    g_lastError.clear();
    std::vector<std::unique_ptr<topo::Solid>> out;

    StepParser parser(text);
    std::string error;
    if (!parser.parse(error)) {
        g_lastError = "STEP parse error: " + error;
        return out;
    }

    // Group MANIFOLD_SOLID_BREPs by shape representation: sibling MSBs in one
    // ADVANCED_BREP_SHAPE_REPRESENTATION are the shells of a single solid.
    std::vector<std::vector<int>> groups;
    std::unordered_set<int> grouped;
    for (int repId : parser.allOfType("ADVANCED_BREP_SHAPE_REPRESENTATION")) {
        const StepInstance* rep = parser.find(repId);
        const StepList* args = rep ? rep->leaf("ADVANCED_BREP_SHAPE_REPRESENTATION") : nullptr;
        if (args == nullptr || args->size() < 2 || !(*args)[1].isList()) continue;
        std::vector<int> ids;
        for (const StepValue& item : *(*args)[1].items) {
            if (!item.isRef() || grouped.count(item.ref) != 0) continue;
            const StepInstance* it = parser.find(item.ref);
            if (it != nullptr && it->hasType("MANIFOLD_SOLID_BREP")) {
                ids.push_back(item.ref);
                grouped.insert(item.ref);
            }
        }
        if (!ids.empty()) groups.push_back(std::move(ids));
    }
    // MSBs outside any representation (minimal files) import one solid each.
    for (int id : parser.allOfType("MANIFOLD_SOLID_BREP")) {
        if (grouped.count(id) == 0) groups.push_back({id});
    }

    if (groups.empty()) {
        g_lastError = "no MANIFOLD_SOLID_BREP found in file";
        return out;
    }
    int index = 0;
    for (const std::vector<int>& group : groups) {
        SolidBuilder builder(parser, index);
        auto solid = builder.build(group, error);
        if (solid == nullptr) {
            g_lastError =
                "failed to reconstruct solid #" + std::to_string(group.front()) + ": " + error;
            out.clear();
            return out;
        }
        out.push_back(std::move(solid));
        ++index;
    }
    return out;
}

std::vector<std::unique_ptr<topo::Solid>> StepFormat::load(const std::string& filePath) {
    g_lastError.clear();
    std::ifstream file(filePath, std::ios::binary);
    if (!file) {
        g_lastError = "cannot open file: " + filePath;
        return {};
    }
    std::ostringstream ss;
    ss << file.rdbuf();
    return fromString(ss.str());
}

const std::string& StepFormat::lastError() {
    return g_lastError;
}

}  // namespace hz::io
