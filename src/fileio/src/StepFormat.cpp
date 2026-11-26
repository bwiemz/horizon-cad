#include "horizon/fileio/StepFormat.h"

#include <algorithm>
#include <atomic>
#include <cctype>
#include <charconv>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <functional>
#include <iomanip>
#include <limits>
#include <map>
#include <optional>
#include <set>
#include <sstream>
#include <string_view>
#include <system_error>
#include <unordered_map>
#include <unordered_set>
#include <variant>

#include "horizon/fileio/AtomicFile.h"
#include "horizon/fileio/ImportReport.h"
#include "horizon/geometry/Frame.h"
#include "horizon/geometry/Pcurve.h"
#include "horizon/geometry/curves/AnalyticCurves.h"
#include "horizon/geometry/curves/Curve2D.h"
#include "horizon/geometry/curves/NurbsCurve.h"
#include "horizon/geometry/surfaces/AnalyticSurfaces.h"
#include "horizon/geometry/surfaces/NurbsSurface.h"
#include "horizon/math/BoundingBox.h"
#include "horizon/math/CharConv.h"
#include "horizon/math/Constants.h"
#include "horizon/math/Mat4.h"
#include "horizon/math/ModelTolerance.h"
#include "horizon/math/Vec2.h"
#include "horizon/math/Vec3.h"
#include "horizon/modeling/Faceting.h"
#include "horizon/modeling/MassProperties.h"
#include "horizon/modeling/MateGeometry.h"
#include "horizon/modeling/Pattern.h"
#include "horizon/modeling/SeamInsertion.h"
#include "horizon/topology/ExactTolerance.h"
#include "horizon/topology/ExactValidator.h"
#include "horizon/topology/GeometryValidator.h"
#include "horizon/topology/LoopUnwrap.h"
#include "horizon/topology/Queries.h"

namespace hz::io {

using hz::math::Mat4;
using hz::math::Vec3;

namespace {

thread_local std::string g_lastError;
/// lastLengthUnitUnknown(): the last read stopped for its length unit.
thread_local bool g_lastUnitUnknown = false;

/// lastError() after a read that stopped for its length unit.
constexpr const char* kUnitUnknown =
    "it names no length unit, or one this version cannot read, so how large its parts are is "
    "not known";

/// @p unit's name in a sentence: "inches".
std::string unitName(math::LengthUnit unit) {
    switch (unit) {
        case math::LengthUnit::Millimetre:
            return "millimetres";
        case math::LengthUnit::Centimetre:
            return "centimetres";
        case math::LengthUnit::Metre:
            return "metres";
        case math::LengthUnit::Inch:
            return "inches";
        case math::LengthUnit::Foot:
            return "feet";
    }
    return "millimetres";
}

/// lastError() after a read stopped by its cancel flag.
constexpr const char* kCancelled = "cancelled";

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

/// Reverse a surface's U direction (NurbsSurface::reversedU): used to honour
/// ADVANCED_FACE same_sense = .F. on import (the kernel's convention is
/// surface normal == outward face normal).
std::shared_ptr<geo::NurbsSurface> reverseSurfaceU(const geo::NurbsSurface& s) {
    return std::make_shared<geo::NurbsSurface>(s.reversedU());
}

// ===========================================================================
// Writer
// ===========================================================================

/// Parse a Part-21 real. std::from_chars ignores the C locale (strtod read
/// "1.5" as 1 under a comma-decimal locale) and must consume the whole token,
/// so "1-2" is an error rather than a silent 1.
bool parseReal(std::string_view text, double& out) {
    if (!text.empty() && text.front() == '+') text.remove_prefix(1);  // from_chars rejects '+'
    const char* last = text.data() + text.size();
    const auto [ptr, ec] = math::fromChars(text.data(), last, out);
    return ec == std::errc{} && ptr == last;
}

/// Format a real in Part-21 syntax: the mantissa always contains a '.'
/// (ISO 10303-21 clause 6.4.2 — "1E-07" is invalid, "1.E-07" is required).
///
/// std::to_chars rather than printf: it ignores the C locale — which Qt sets
/// from the environment on Unix, so "%g" wrote "1,5" under de_DE — and gives
/// the shortest text that reads back as exactly the same double.
std::string fmtReal(double v) {
    if (!std::isfinite(v)) return "0.";
    char buf[64];
    const auto [end, ec] = std::to_chars(buf, buf + sizeof(buf), v);
    if (ec != std::errc{}) return "0.";
    std::string s(buf, end);
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

// ---------------------------------------------------------------------------
// Exact faces and edges as their kind (exact kernel, stage 2)
// ---------------------------------------------------------------------------

int writeDirection(StepWriter& w, const Vec3& d) {
    return w.add("DIRECTION('',(" + fmtReal(d.x) + "," + fmtReal(d.y) + "," + fmtReal(d.z) + "))");
}

/// An AXIS2_PLACEMENT_3D at @p origin, its axis @p axis and its x @p ref: always right-handed, as
/// STEP's are (its y is axis x ref).
int writePlacement(StepWriter& w, const Vec3& origin, const Vec3& axis, const Vec3& ref) {
    const int at = w.addPoint(origin);
    const int z = writeDirection(w, axis);
    const int x = writeDirection(w, ref);
    return w.add("AXIS2_PLACEMENT_3D('',#" + std::to_string(at) + ",#" + std::to_string(z) + ",#" +
                 std::to_string(x) + ")");
}

bool positiveFinite(double x) {
    return std::isfinite(x) && x > 0.0;
}

/// A surface as written: its id, and whether the normal of the surface written (du x dv) runs
/// against the normal of the surface it was written for.
struct WrittenSurface {
    int id = 0;
    bool flipped = false;
};

/// @p s written as its kind: a PLANE, CYLINDRICAL_SURFACE, CONICAL_SURFACE, SPHERICAL_SURFACE or
/// TOROIDAL_SURFACE placed by its frame, or its B-spline (writeSurface). STEP's typed surfaces
/// carry no parameter scale, and their placements are right-handed, so each is written as the
/// geometry it makes (Ruling 42): a plane through its frame's origin about x × y, with its x; a
/// surface of revolution about its frame's z with its x. A left-handed frame (a mirror image's),
/// or a negative axial scale, then runs the surface's u (or v) the other way, and the surface
/// written faces against it: said in `flipped`. Nullopt, with why in @p why ("its cone's
/// half-angle is not between 0 and 90 degrees"), where its numbers make none STEP holds (a radius
/// that is not positive, a cone's half-angle outside (0, pi/2), a scale of 0): the face goes out
/// on its carrier, as before the exact kernel, and the write report says so.
std::optional<WrittenSurface> writeExactSurface(StepWriter& w, const geo::Surface& s,
                                                std::string& why) {
    const auto none = [&why](const std::string& because) -> std::optional<WrittenSurface> {
        why = because;
        return std::nullopt;
    };
    const auto revolved = [&](const geo::Frame& f, double axialScale, const std::string& kind,
                              const std::string& radii) -> std::optional<WrittenSurface> {
        if (!std::isfinite(axialScale) || axialScale == 0.0) {
            return none("its axial scale is 0 or not finite");
        }
        const bool leftHanded = f.x.cross(f.y).dot(f.z) < 0.0;
        const int placement = writePlacement(w, f.origin, f.z, f.x);
        return WrittenSurface{w.add(kind + "('',#" + std::to_string(placement) + "," + radii + ")"),
                              leftHanded != (axialScale < 0.0)};
    };
    if (const auto* plane = dynamic_cast<const geo::Plane*>(&s)) {
        const geo::Frame& f = plane->frame();
        const Vec3 axis = f.x.cross(f.y);
        if (!positiveFinite(axis.length())) return none("its plane's axes span no plane");
        if (!std::isfinite(plane->scale()) || plane->scale() == 0.0) {
            return none("its plane's scale is 0 or not finite");
        }
        const int placement = writePlacement(w, f.origin, axis * (1.0 / axis.length()), f.x);
        return WrittenSurface{w.add("PLANE('',#" + std::to_string(placement) + ")"), false};
    }
    if (const auto* cylinder = dynamic_cast<const geo::Cylinder*>(&s)) {
        if (!positiveFinite(cylinder->radius())) {
            return none("its cylinder's radius is not positive");
        }
        return revolved(cylinder->frame(), cylinder->axialScale(), "CYLINDRICAL_SURFACE",
                        fmtReal(cylinder->radius()));
    }
    if (const auto* cone = dynamic_cast<const geo::Cone*>(&s)) {
        const double r = cone->radius();
        const double a = cone->halfAngle();
        if (!std::isfinite(r) || r < 0.0) return none("its cone's radius is negative");
        if (!(a > 0.0 && a < math::kHalfPi)) {
            return none("its cone's half-angle is not between 0 and 90 degrees");
        }
        return revolved(cone->frame(), cone->axialScale(), "CONICAL_SURFACE",
                        fmtReal(r) + "," + fmtReal(a));
    }
    if (const auto* sphere = dynamic_cast<const geo::Sphere*>(&s)) {
        if (!positiveFinite(sphere->radius())) return none("its sphere's radius is not positive");
        return revolved(sphere->frame(), 1.0, "SPHERICAL_SURFACE", fmtReal(sphere->radius()));
    }
    if (const auto* torus = dynamic_cast<const geo::Torus*>(&s)) {
        if (!positiveFinite(torus->majorRadius()) || !positiveFinite(torus->minorRadius())) {
            return none("its torus's radii are not positive");
        }
        return revolved(torus->frame(), 1.0, "TOROIDAL_SURFACE",
                        fmtReal(torus->majorRadius()) + "," + fmtReal(torus->minorRadius()));
    }
    if (const auto* nurbs = dynamic_cast<const geo::NurbsSurface*>(&s)) {
        return WrittenSurface{writeSurface(w, *nurbs), false};
    }
    return none("its surface is of no kind STEP has");
}

/// @p c run the other way over the same domain: its net and weights reversed, its knots
/// mirrored.
geo::NurbsCurve reversedSpline(const geo::NurbsCurve& c) {
    std::vector<Vec3> points(c.controlPoints().rbegin(), c.controlPoints().rend());
    std::vector<double> weights(c.weights().rbegin(), c.weights().rend());
    const std::vector<double>& knots = c.knots();
    std::vector<double> mirrored;
    mirrored.reserve(knots.size());
    for (auto k = knots.rbegin(); k != knots.rend(); ++k) {
        mirrored.push_back(knots.front() + knots.back() - *k);
    }
    return {std::move(points), std::move(weights), std::move(mirrored), c.degree()};
}

/// @p c written as its kind, parametrised as @p c is: a LINE whose VECTOR's magnitude is the
/// line's scale (a negative scale written as its direction turned round), a CIRCLE or an ELLIPSE
/// placed about its frame's x × y with its x (so its y is the frame's, whichever hand the frame
/// is), or its B-spline (writeCurve). With @p backward, the same curve running the other way: a
/// line's direction, a conic's y (and so its axis) turned round, a B-spline's net reversed.
/// Nullopt, with why in @p why, where its numbers make none STEP holds.
std::optional<int> writeExactCurve(StepWriter& w, const geo::Curve3& c, std::string& why,
                                   bool backward = false) {
    const auto none = [&why](const std::string& because) -> std::optional<int> {
        why = because;
        return std::nullopt;
    };
    if (const auto* line = dynamic_cast<const geo::LineCurve*>(&c)) {
        const double scale = line->scale();
        if (!std::isfinite(scale) || scale == 0.0) {
            return none("its line's magnitude is 0 or not finite");
        }
        const Vec3 direction =
            (scale < 0.0) != backward ? line->direction() * -1.0 : line->direction();
        const int point = w.addPoint(line->origin());
        const int vector = w.add("VECTOR('',#" + std::to_string(writeDirection(w, direction)) +
                                 "," + fmtReal(std::abs(scale)) + ")");
        return w.add("LINE('',#" + std::to_string(point) + ",#" + std::to_string(vector) + ")");
    }
    const auto placed = [&](const geo::Frame& f, const std::string& kind,
                            const std::string& radii) -> std::optional<int> {
        const Vec3 axis = f.x.cross(f.y) * (backward ? -1.0 : 1.0);
        if (!positiveFinite(axis.length())) return none("its axes span no plane");
        const int placement = writePlacement(w, f.origin, axis * (1.0 / axis.length()), f.x);
        return w.add(kind + "('',#" + std::to_string(placement) + "," + radii + ")");
    };
    if (const auto* circle = dynamic_cast<const geo::CircleCurve*>(&c)) {
        if (!positiveFinite(circle->radius())) return none("its circle's radius is not positive");
        return placed(circle->frame(), "CIRCLE", fmtReal(circle->radius()));
    }
    if (const auto* ellipse = dynamic_cast<const geo::EllipseCurve*>(&c)) {
        if (!positiveFinite(ellipse->majorRadius()) || !positiveFinite(ellipse->minorRadius())) {
            return none("its ellipse's radii are not positive");
        }
        return placed(ellipse->frame(), "ELLIPSE",
                      fmtReal(ellipse->majorRadius()) + "," + fmtReal(ellipse->minorRadius()));
    }
    if (const auto* nurbs = dynamic_cast<const geo::NurbsCurve*>(&c)) {
        return backward ? writeCurve(w, reversedSpline(*nurbs)) : writeCurve(w, *nurbs);
    }
    return none("its curve is of no kind STEP has");
}

// ---------------------------------------------------------------------------
// Curved faces as designed (Phase 151)
// ---------------------------------------------------------------------------

/// A circle: what a rim's chords stand in for.
struct Circle {
    Vec3 center;
    Vec3 normal;  ///< unit
    double radius = 0.0;
};

/// @p curve as a circle, if it is one: three points on it make a circle,
/// and every other point sampled must lie on that one.
std::optional<Circle> circleOf(const geo::NurbsCurve& curve) {
    const double t0 = curve.tMin();
    const double t1 = curve.tMax();
    const Vec3 a = curve.evaluate(t0);
    const Vec3 b = curve.evaluate(t0 + (t1 - t0) / 3.0);
    const Vec3 c = curve.evaluate(t0 + 2.0 * (t1 - t0) / 3.0);
    const Vec3 ab = b - a;
    const Vec3 ac = c - a;
    const Vec3 n = ab.cross(ac);
    const double n2 = n.dot(n);
    const double scale = std::max({ab.length(), ac.length(), 1e-300});
    if (n2 <= 1e-18 * scale * scale * scale * scale) return std::nullopt;  // collinear
    // The circumcentre of a, b, c.
    const Vec3 center =
        a + (n.cross(ab) * ac.dot(ac) + ac.cross(n) * ab.dot(ab)) * (1.0 / (2.0 * n2));
    Circle circle{center, n * (1.0 / std::sqrt(n2)), (a - center).length()};
    const double tolerance = 1e-9 * std::max(1.0, circle.radius);
    for (int i = 0; i <= 16; ++i) {
        const Vec3 p = curve.evaluate(t0 + (t1 - t0) * i / 16.0);
        const Vec3 off = p - circle.center;
        if (std::abs(off.length() - circle.radius) > tolerance ||
            std::abs(off.dot(circle.normal)) > tolerance) {
            return std::nullopt;
        }
    }
    return circle;
}

/// A face's plane, from its outer loop (Newell): its normal (unit, the way
/// the loop runs) and a point on it. None for a degenerate loop.
std::optional<std::pair<Vec3, Vec3>> planeOf(const topo::Face& f) {
    if (f.outerLoop == nullptr || f.outerLoop->halfEdge == nullptr) return std::nullopt;
    Vec3 normal;
    Vec3 centroid;
    int count = 0;
    const topo::HalfEdge* start = f.outerLoop->halfEdge;
    const topo::HalfEdge* he = start;
    do {
        if (he == nullptr || he->origin == nullptr || he->next == nullptr ||
            he->next->origin == nullptr) {
            return std::nullopt;
        }
        const Vec3& p = he->origin->point;
        const Vec3& q = he->next->origin->point;
        normal = normal + Vec3((p.y - q.y) * (p.z + q.z), (p.z - q.z) * (p.x + q.x),
                               (p.x - q.x) * (p.y + q.y));
        centroid = centroid + p;
        ++count;
        he = he->next;
    } while (he != start && count < 1000000);
    const double length = normal.length();
    if (length <= 0.0 || count == 0) return std::nullopt;
    return std::make_pair(normal * (1.0 / length), centroid * (1.0 / count));
}

/// The plane a face with no exact geometry is written on where it is flat (Ruling 13): its outer
/// loop's (planeOf: its normal the way the loop runs), where its carrier, if it has one, is flat,
/// and every corner of its loops and every control point of its edges' curves lies within
/// @p tolerance of that plane. A flat face goes out on its plane, not on its carrier, a patch
/// sized to its corners that the circles of its rims, written as designed, stand outside of.
std::optional<std::pair<Vec3, Vec3>> flatPlaneOf(const topo::Face& f, double tolerance) {
    const auto plane = planeOf(f);
    if (!plane) return std::nullopt;
    Vec3 carrierOrigin;
    Vec3 carrierNormal;
    if (f.surface != nullptr &&
        !topo::GeometryValidator::facePlane(f, carrierOrigin, carrierNormal)) {
        return std::nullopt;  // a curved carrier, whatever its corners do
    }
    const Vec3& normal = plane->first;
    const Vec3& at = plane->second;
    const auto on = [&](const Vec3& p) { return std::abs((p - at).dot(normal)) <= tolerance; };
    std::vector<const topo::Wire*> loops{f.outerLoop};
    loops.insert(loops.end(), f.innerLoops.begin(), f.innerLoops.end());
    for (const topo::Wire* wire : loops) {
        if (wire == nullptr || wire->halfEdge == nullptr) return std::nullopt;
        const topo::HalfEdge* he = wire->halfEdge;
        std::size_t count = 0;
        do {
            if (he->origin == nullptr || !on(he->origin->point)) return std::nullopt;
            if (he->edge != nullptr && he->edge->curve != nullptr) {
                const auto& points = he->edge->curve->controlPoints();
                if (!std::all_of(points.begin(), points.end(), on)) return std::nullopt;
            }
            he = he->next;
        } while (he != nullptr && he != wire->halfEdge && ++count < 1000000);
    }
    return plane;
}

/// Whether @p f's carrier faces the way its outer loop runs: the loop's winding in the carrier's
/// own (u, v), counter-clockwise about du x dv. Its corners, and points along each edge's
/// curve, are located on the carrier, each moved a whole period where the carrier closes, so
/// that the loop runs on unbroken; a loop that comes back a period over (a band round the
/// carrier), or whose (u, v) area is next to nothing (a seam there and back), says nothing, and
/// the carrier is taken to face as its face does, as the reader leaves every carrier it reads.
/// A point taken from the loop's corners alone does not do: on a face spanning more than half
/// its carrier their middle lies off the face, where the carrier faces the other way.
bool carrierRunsWithLoops(const topo::Face& f) {
    if (f.surface == nullptr || f.outerLoop == nullptr || f.outerLoop->halfEdge == nullptr) {
        return true;
    }
    const geo::NurbsSurface& s = *f.surface;
    const double periodU = s.closedU() ? s.uMax() - s.uMin() : 0.0;
    const double periodV = s.closedV() ? s.vMax() - s.vMin() : 0.0;
    constexpr int kAlong = 8;  // points along a curved edge, its start among them
    std::vector<math::Vec2> loop;
    const auto unwrapped = [](double x, double previous, double period) {
        return period > 0.0 ? x - period * std::round((x - previous) / period) : x;
    };
    const auto put = [&](const Vec3& p) {
        const auto [u, v] = s.locate(p);
        if (!std::isfinite(u) || !std::isfinite(v)) return false;
        if (loop.empty()) {
            loop.emplace_back(u, v);
        } else {
            loop.emplace_back(unwrapped(u, loop.back().x, periodU),
                              unwrapped(v, loop.back().y, periodV));
        }
        return true;
    };
    const topo::HalfEdge* he = f.outerLoop->halfEdge;
    std::size_t count = 0;
    do {
        if (he->origin == nullptr || he->next == nullptr || he->next->origin == nullptr) {
            return true;
        }
        if (!put(he->origin->point)) return true;
        const geo::NurbsCurve* c = he->edge != nullptr ? he->edge->curve.get() : nullptr;
        if (c != nullptr && c->degree() > 1) {
            // Along the half-edge: a curve runs from the end nearer its start, and a closed one
            // the way its edge's own half-edge does.
            const Vec3& from = he->origin->point;
            const Vec3& to = he->next->origin->point;
            const bool closed = (from - to).length() <= 1e-12 * std::max(1.0, from.length());
            const bool forward = closed ? he->edge->halfEdge == he
                                        : (c->evaluate(c->tMin()) - from).length() <=
                                              (c->evaluate(c->tMax()) - from).length();
            for (int k = 1; k < kAlong; ++k) {
                const double f0 = static_cast<double>(k) / kAlong;
                const double at = c->tMin() + (c->tMax() - c->tMin()) * (forward ? f0 : 1.0 - f0);
                if (!put(c->evaluate(at))) return true;
            }
        }
        he = he->next;
    } while (he != nullptr && he != f.outerLoop->halfEdge && ++count < 100000);
    if (loop.size() < 3) return true;
    // Back where it started, not a period over.
    const math::Vec2 back{unwrapped(loop.front().x, loop.back().x, periodU),
                          unwrapped(loop.front().y, loop.back().y, periodV)};
    if ((periodU > 0.0 && std::abs(back.x - loop.front().x) > 0.5 * periodU) ||
        (periodV > 0.0 && std::abs(back.y - loop.front().y) > 0.5 * periodV)) {
        return true;
    }
    double area = 0.0;
    math::Vec2 lo = loop.front();
    math::Vec2 hi = lo;
    for (std::size_t i = 0; i < loop.size(); ++i) {
        const math::Vec2& a = loop[i];
        const math::Vec2& b = i + 1 < loop.size() ? loop[i + 1] : back;
        area += a.x * b.y - b.x * a.y;
        lo = {std::min(lo.x, a.x), std::min(lo.y, a.y)};
        hi = {std::max(hi.x, a.x), std::max(hi.y, a.y)};
    }
    if (!std::isfinite(area) || std::abs(area) <= 1e-9 * (hi.x - lo.x) * (hi.y - lo.y)) {
        return true;
    }
    return area > 0.0;
}

/// A curved face as designed: the facets that stand in for one analytic
/// surface, joined by their shared edges, and how they are written as one
/// face on it. `why` says why they are not, when they are not.
struct CurvedFace {
    std::vector<const topo::Face*> facets;
    const geo::NurbsSurface* surface = nullptr;
    /// Its outline: the half-edges of its facets whose other side is not
    /// one of them, in loops as its facets run.
    std::vector<std::vector<const topo::HalfEdge*>> loops;
    /// A closed face's (a cylinder's side): the edges between its two
    /// outlines it is cut along, one straight line of them, as half-edges
    /// from the first outline to the second.
    std::vector<const topo::HalfEdge*> seam;
    /// Its surface cut to close along its seam, where it closed elsewhere.
    std::optional<geo::NurbsSurface> cut;
    bool sameSense = true;
    std::string why;  ///< empty: written as designed
};

/// The curved faces of @p solid (Phase 151): the facets of each analytic
/// surface, joined over their shared edges, and whether each can be written
/// as one face on its surface. One that cannot keeps its facets, and says
/// why. @p curved receives the edges written on their ideal curves, and
/// @p rims the circles recovered for those whose ideal was lost (a hole's
/// rim, cut by a Boolean: where a cylinder meets a plane square to it).
std::vector<CurvedFace> planCurvedFaces(const topo::Solid& solid,
                                        std::unordered_set<const topo::Edge*>& curved,
                                        std::unordered_map<const topo::Edge*, Circle>& rims) {
    // Facets of one surface, joined over their shared edges.
    std::unordered_map<const topo::Face*, const topo::Face*> parent;
    const std::function<const topo::Face*(const topo::Face*)> root = [&](const topo::Face* f) {
        const topo::Face* r = f;
        while (parent.at(r) != r) r = parent.at(r);
        while (parent.at(f) != r) {
            const topo::Face* next = parent.at(f);
            parent[f] = r;
            f = next;
        }
        return r;
    };
    for (const auto& f : solid.faces()) {
        if (f.analyticSurface) parent[&f] = &f;
    }
    for (const auto& e : solid.edges()) {
        const topo::HalfEdge* h = e.halfEdge;
        if (h == nullptr || h->twin == nullptr || h->face == nullptr || h->twin->face == nullptr) {
            continue;
        }
        const topo::Face* a = h->face;
        const topo::Face* b = h->twin->face;
        if (a == b || !a->analyticSurface || a->analyticSurface != b->analyticSurface) continue;
        const topo::Face* ra = root(a);
        const topo::Face* rb = root(b);
        if (ra != rb) parent[ra] = rb;
    }
    std::unordered_map<const topo::Face*, std::size_t> groupOf;
    std::vector<CurvedFace> faces;
    for (const auto& f : solid.faces()) {  // in the solid's order, so the file is too
        if (!f.analyticSurface) continue;
        const topo::Face* r = root(&f);
        auto it = groupOf.find(r);
        if (it == groupOf.end()) {
            it = groupOf.emplace(r, faces.size()).first;
            faces.emplace_back();
            faces.back().surface = f.analyticSurface.get();
        }
        groupOf[&f] = it->second;
        faces[it->second].facets.push_back(&f);
    }
    const auto groupIndex = [&](const topo::Face* f) -> std::optional<std::size_t> {
        const auto it = groupOf.find(f);
        if (it == groupOf.end()) return std::nullopt;
        return it->second;
    };
    const auto inGroup = [&](const topo::Face* f, std::size_t g) {
        const auto i = groupIndex(f);
        return i && *i == g;
    };
    const auto onSurface = [](const geo::NurbsSurface& surface, const Vec3& p, double scale) {
        const auto [u, v] = surface.locate(p);
        return (surface.evaluate(u, v) - p).length() <= 1e-7 * std::max(1.0, scale);
    };

    for (std::size_t g = 0; g < faces.size(); ++g) {
        CurvedFace& face = faces[g];
        const geo::NurbsSurface& surface = *face.surface;
        // Its outline, and whether any of its corners is inside it: a pole,
        // or an apex, which no one outline goes round.
        std::vector<const topo::HalfEdge*> outline;
        std::unordered_set<const topo::Vertex*> corners;
        std::unordered_set<const topo::Vertex*> onOutline;
        std::size_t halfEdges = 0;
        for (const topo::Face* f : face.facets) {
            std::vector<const topo::Wire*> wires{f->outerLoop};
            wires.insert(wires.end(), f->innerLoops.begin(), f->innerLoops.end());
            for (const topo::Wire* wire : wires) {
                if (wire == nullptr || wire->halfEdge == nullptr) continue;
                const topo::HalfEdge* start = wire->halfEdge;
                const topo::HalfEdge* he = start;
                do {
                    ++halfEdges;
                    corners.insert(he->origin);
                    if (he->twin == nullptr || !inGroup(he->twin->face, g)) {
                        outline.push_back(he);
                        onOutline.insert(he->origin);
                        if (he->next != nullptr) onOutline.insert(he->next->origin);
                    }
                    he = he->next;
                } while (he != nullptr && he != start && halfEdges < 10000000);
            }
        }
        if (outline.empty()) {
            face.why = "it is closed all round (a sphere or a torus)";
            continue;
        }
        // A corner inside it is a pole or an apex only where the surface
        // folds to a point (its tangents parallel, or nothing): a fillet
        // band round a rim (Phase 164) has its arc's samples inside it, on a
        // torus that is smooth there.
        // How big it is: its area element there is next to nothing at a
        // pole, where the parameters fold (the tangents there keep their
        // angle, so it is their size that tells).
        math::BoundingBox extent;
        for (const topo::Vertex* corner : corners) extent.expand(corner->point);
        const double size = extent.isValid() ? (extent.max() - extent.min()).length() : 0.0;
        bool pole = false;
        for (const topo::Vertex* corner : corners) {
            if (onOutline.count(corner) != 0) continue;
            const auto [u, v] = surface.locate(corner->point);
            const auto at = surface.evaluateWithDerivatives(u, v);
            if (!(at.du.cross(at.dv).length() > 1e-8 * size * size)) {
                pole = true;
                break;
            }
        }
        if (pole) {
            face.why = "it comes to a point inside it (a cone's apex, a pole)";
            continue;
        }
        // The outline in loops: from a half-edge on it to the next, round
        // the vertex it ends at, over the facets' own edges.
        std::unordered_set<const topo::HalfEdge*> left(outline.begin(), outline.end());
        std::size_t nextStart = 0;
        while (!left.empty()) {
            std::vector<const topo::HalfEdge*> loop;
            // The same start each time, whatever the set's order: the first
            // of the outline, in its own order, still left.
            while (left.count(outline[nextStart]) == 0) ++nextStart;
            const topo::HalfEdge* he = outline[nextStart];
            bool closed = false;
            for (std::size_t guard = 0; guard <= halfEdges; ++guard) {
                loop.push_back(he);
                left.erase(he);
                const topo::HalfEdge* next = he->next;
                std::size_t turns = 0;
                while (next != nullptr && next->twin != nullptr && inGroup(next->twin->face, g) &&
                       turns++ <= halfEdges) {
                    next = next->twin->next;
                }
                if (next == nullptr) break;
                if (next == loop.front()) {
                    closed = true;
                    break;
                }
                if (left.count(next) == 0) break;  // not a simple outline
                he = next;
            }
            if (!closed) {
                face.why = "its outline does not close";
                break;
            }
            face.loops.push_back(std::move(loop));
        }
        if (!face.why.empty()) continue;
        if (face.loops.size() > 2) {
            face.why = "it has more than two outlines";
            continue;
        }

        // Every edge of its outline, on it: a circle, a curve of its own
        // span, a straight edge along it (a cylinder's rulings), or a rim
        // where it meets a plane square to it, whose circle a Boolean did
        // not keep.
        const auto frame = model::MateGeometry::frameForFace(*face.facets.front());
        // A rim's circle from its own corners, which lie on it exactly: the
        // circle through three of them, the rest checked against it. (The
        // cylinder's own centre is fitted to its facets, 5e-6 off.)
        bool rimOffCircle = false;  // a rim found, its corners not all on one circle
        const auto rimOf = [&](const std::vector<const topo::HalfEdge*>& loop,
                               const topo::HalfEdge* he) -> std::optional<Circle> {
            if (!frame || frame->kind != model::MateFrameKind::Cylindrical || he->twin == nullptr ||
                groupIndex(he->twin->face)) {
                return std::nullopt;
            }
            const auto plane = planeOf(*he->twin->face);
            if (!plane || std::abs(std::abs(plane->first.dot(frame->direction)) - 1.0) > 1e-9) {
                return std::nullopt;
            }
            // Its corners: of the outline's edges into that plane, whatever
            // faces it is in (a Boolean leaves a plane in triangles).
            std::vector<Vec3> corners;
            for (const topo::HalfEdge* h : loop) {
                if (h->twin == nullptr || h->twin->face == nullptr) continue;
                const auto other = planeOf(*h->twin->face);
                if (!other || other->first.dot(plane->first) < 1.0 - 1e-12 ||
                    std::abs((other->second - plane->second).dot(plane->first)) >
                        1e-9 * std::max(1.0, plane->second.length())) {
                    continue;
                }
                corners.push_back(h->origin->point);
            }
            if (corners.size() < 3) return std::nullopt;
            const Vec3& p = corners[0];
            const Vec3& q = corners[corners.size() / 3];
            const Vec3& r = corners[2 * corners.size() / 3];
            const Vec3 pq = q - p;
            const Vec3 pr = r - p;
            const Vec3 n = pq.cross(pr);
            const double n2 = n.dot(n);
            if (n2 <= 0.0) return std::nullopt;
            const Vec3 centre =
                p + (n.cross(pq) * pr.dot(pr) + pr.cross(n) * pq.dot(pq)) * (1.0 / (2.0 * n2));
            const Circle rim{centre, plane->first, (p - centre).length()};
            const double tol = 1e-9 * std::max(1.0, rim.radius + centre.length());
            // The cylinder's own rim, every corner on it: a Boolean may split
            // a chord in two, at a point inside the circle, and then no one
            // circle goes through its corners.
            bool onCircle =
                std::abs(rim.radius - frame->radius) <= 1e-5 * std::max(1.0, rim.radius);
            for (const Vec3& corner : corners) {
                const Vec3 off = corner - rim.center;
                onCircle = onCircle && std::abs(off.length() - rim.radius) <= tol &&
                           std::abs(off.dot(rim.normal)) <= tol;
            }
            if (!onCircle) {
                rimOffCircle = true;
                return std::nullopt;
            }
            return rim;
        };
        double scale = 0.0;
        for (const auto* he : outline) {
            scale = std::max(scale, he->origin->point.length());
        }
        for (const auto& loop : face.loops) {
            for (const topo::HalfEdge* he : loop) {
                const topo::Edge* e = he->edge;
                if (e == nullptr || e->halfEdge == nullptr || e->halfEdge->twin == nullptr) {
                    face.why = "an edge of its outline is incomplete";
                    break;
                }
                const Vec3& a = e->halfEdge->origin->point;
                const Vec3& b = e->halfEdge->twin->origin->point;
                if (e->analyticCurve) {
                    // A rim's circle is shared by its chords, and a Boolean
                    // gives it to the pieces of a chord it cuts, whose cut
                    // end is inside the circle: this edge's own ends on it.
                    if (const auto circle = circleOf(*e->analyticCurve)) {
                        const double tol = 1e-9 * std::max(1.0, circle->radius);
                        const auto onIt = [&](const Vec3& p) {
                            const Vec3 off = p - circle->center;
                            return std::abs(off.length() - circle->radius) <= tol &&
                                   std::abs(off.dot(circle->normal)) <= tol;
                        };
                        if (onIt(a) && onIt(b)) continue;
                        face.why =
                            "an edge of its outline has an end off its circle (a cut "
                            "chord)";
                        break;
                    }
                    const Vec3 c0 = e->analyticCurve->evaluate(e->analyticCurve->tMin());
                    const Vec3 c1 = e->analyticCurve->evaluate(e->analyticCurve->tMax());
                    const double tol = 1e-9 * std::max(1.0, scale);
                    if ((((c0 - a).length() <= tol && (c1 - b).length() <= tol) ||
                         ((c0 - b).length() <= tol && (c1 - a).length() <= tol))) {
                        continue;  // its own span
                    }
                    face.why = "an edge of its outline is part of a curve other than a circle";
                    break;
                }
                if (const auto rim = rimOf(loop, he)) {
                    rims[e] = *rim;
                    continue;
                }
                if (!onSurface(surface, (a + b) * 0.5, scale)) {
                    face.why = rimOffCircle
                                   ? "its rim's corners are not all on one circle (a Boolean "
                                     "split its chords)"
                                   : "a straight edge of its outline leaves its surface";
                    break;
                }
            }
            if (!face.why.empty()) break;
        }
        if (!face.why.empty()) continue;

        // Two outlines (a cylinder's side): cut along a line of its edges
        // from one to the other, on the surface (a ruling, not a triangle's
        // diagonal) — one edge, or several where its facets are a grid —
        // where the surface closes if one does. A face cut elsewhere crosses
        // where it closes, and a reader that takes a closed B-spline surface
        // for an open one (OpenCASCADE) trims it there, measuring it short
        // (#184): where no line is, the surface is cut to close along one.
        if (face.loops.size() == 2) {
            std::unordered_set<const topo::Vertex*> first;
            std::unordered_set<const topo::Vertex*> second;
            for (const auto* he : face.loops[0]) first.insert(he->origin);
            for (const auto* he : face.loops[1]) second.insert(he->origin);
            const double tol = 1e-7 * std::max(1.0, scale);
            // Straight: a ruling of the file's own, which keeps its line.
            const auto straight = [tol](const topo::Edge& e) {
                if (!e.analyticCurve) return true;
                const Vec3 a = e.analyticCurve->evaluate(e.analyticCurve->tMin());
                const Vec3 b = e.analyticCurve->evaluate(e.analyticCurve->tMax());
                const Vec3 m = e.analyticCurve->evaluate(
                    0.5 * (e.analyticCurve->tMin() + e.analyticCurve->tMax()));
                return ((a + b) * 0.5 - m).length() <= tol;
            };
            // Along where it closes: both its ends on that line of it.
            const auto whereItCloses = [&](const Vec3& a, const Vec3& b) {
                const auto [ua, va] = surface.locate(a);
                const auto [ub, vb] = surface.locate(b);
                return (surface.closedU() &&
                        (surface.evaluate(surface.uMin(), va) - a).length() <= tol &&
                        (surface.evaluate(surface.uMin(), vb) - b).length() <= tol) ||
                       (surface.closedV() &&
                        (surface.evaluate(ua, surface.vMin()) - a).length() <= tol &&
                        (surface.evaluate(ub, surface.vMin()) - b).length() <= tol);
            };
            // Its edges inside it, by the vertex they leave; a ruling's.
            std::unordered_map<const topo::Vertex*, std::vector<const topo::HalfEdge*>> leaving;
            for (const topo::Face* f : face.facets) {
                const topo::HalfEdge* start = f->outerLoop->halfEdge;
                const topo::HalfEdge* he = start;
                do {
                    if (he->twin != nullptr && inGroup(he->twin->face, g) && he->next != nullptr) {
                        leaving[he->origin].push_back(he);
                    }
                    he = he->next;
                } while (he != nullptr && he != start);
            }
            const auto ruling = [&](const topo::HalfEdge* he) {
                const Vec3 mid = (he->origin->point + he->next->origin->point) * 0.5;
                return straight(*he->edge) && onSurface(surface, mid, scale);
            };
            // From @p up, on to the second outline straight on: the line of
            // edges, or none.
            const auto lineFrom = [&](const topo::HalfEdge* up) {
                std::vector<const topo::HalfEdge*> line{up};
                const Vec3 way = up->next->origin->point - up->origin->point;
                const double length = way.length();
                if (!(length > 0.0)) return std::vector<const topo::HalfEdge*>{};
                const Vec3 along = way * (1.0 / length);
                while (second.count(line.back()->next->origin) == 0) {
                    const topo::Vertex* at = line.back()->next->origin;
                    if (first.count(at) != 0 || line.size() > face.facets.size())
                        return std::vector<const topo::HalfEdge*>{};
                    const topo::HalfEdge* next = nullptr;
                    const auto out = leaving.find(at);
                    if (out != leaving.end()) {
                        for (const topo::HalfEdge* he : out->second) {
                            const Vec3 step = he->next->origin->point - at->point;
                            const double stepLength = step.length();
                            if (stepLength > 0.0 && step.dot(along) >= (1.0 - 1e-9) * stepLength &&
                                ruling(he)) {
                                next = he;
                                break;
                            }
                        }
                    }
                    if (next == nullptr) return std::vector<const topo::HalfEdge*>{};
                    line.push_back(next);
                }
                return line;
            };
            bool onJoin = false;
            for (const topo::HalfEdge* rim : face.loops[0]) {  // in its order: the file's too
                const auto out = leaving.find(rim->origin);
                if (out == leaving.end()) continue;
                for (const topo::HalfEdge* up : out->second) {
                    if (second.count(up->origin) != 0 || !ruling(up)) continue;
                    auto line = lineFrom(up);
                    if (line.empty()) continue;
                    const bool closesHere = whereItCloses(line.front()->origin->point,
                                                          line.back()->next->origin->point);
                    if (face.seam.empty() || closesHere) face.seam = std::move(line);
                    if (closesHere) {
                        onJoin = true;
                        break;
                    }
                }
                if (onJoin) break;
            }
            if (face.seam.empty()) {
                face.why =
                    "no line of edges from one of its outlines to the other lies on its surface";
                continue;
            }
            if (!onJoin && (surface.closedU() || surface.closedV())) {
                // Cut the way it goes round: across the seam, whose ends
                // share their parameter that way (a torus closes both ways).
                const Vec3& a = face.seam.front()->origin->point;
                const Vec3& b = face.seam.back()->next->origin->point;
                const auto [ua, va] = surface.locate(a);
                const auto [ub, vb] = surface.locate(b);
                const auto apart = [](double x, double y, double lo, double hi) {
                    const double d = std::fmod(std::abs(x - y), hi - lo);
                    return std::min(d, hi - lo - d) / (hi - lo);
                };
                const bool inU =
                    surface.closedU() &&
                    (!surface.closedV() || apart(ua, ub, surface.uMin(), surface.uMax()) <=
                                               apart(va, vb, surface.vMin(), surface.vMax()));
                const auto [u, v] = surface.locate((a + b) * 0.5);
                face.cut = inU ? surface.startingAtU(u) : surface.startingAtV(v);
                if (!face.cut) {
                    face.why = "its surface closes where no line joining its outlines lies";
                    continue;
                }
            }
        }

        // Which way it faces: each facet's normal against the surface's.
        int along = 0;
        int against = 0;
        const std::size_t step = std::max<std::size_t>(1, face.facets.size() / 16);
        for (std::size_t i = 0; i < face.facets.size(); i += step) {
            const auto plane = planeOf(*face.facets[i]);
            if (!plane) continue;
            const auto [u, v] = surface.locate(plane->second);
            const double d = surface.normal(u, v).dot(plane->first);
            if (d > 1e-6) ++along;
            if (d < -1e-6) ++against;
        }
        if (along > 0 && against > 0) {
            face.why = "its facets face both ways on its surface";
            continue;
        }
        if (along == 0 && against == 0) {
            face.why = "which way it faces cannot be told";
            continue;
        }
        face.sameSense = along > 0;
    }

    // The edges of the outlines written on their ideal curves; each must be
    // on the face across it too: another written as designed, or a plane
    // the curve lies in (a cylinder's cap). A face kept in facets is
    // bounded by chords, and one written as designed beside it cannot be.
    for (bool changed = true; changed;) {
        changed = false;
        curved.clear();
        for (std::size_t g = 0; g < faces.size(); ++g) {
            CurvedFace& face = faces[g];
            if (!face.why.empty()) continue;
            for (const auto& loop : face.loops) {
                for (const topo::HalfEdge* he : loop) {
                    const topo::Edge* e = he->edge;
                    const auto rim = rims.find(e);
                    if (!e->analyticCurve && rim == rims.end()) continue;
                    const topo::Face* across = he->twin->face;
                    const auto other = groupIndex(across);
                    bool accepted = other && faces[*other].why.empty();
                    if (!accepted && !other) {
                        const auto plane = planeOf(*across);
                        accepted = plane.has_value();
                        for (int i = 0; accepted && i <= 8; ++i) {
                            Vec3 p;
                            if (e->analyticCurve) {
                                p = e->analyticCurve->evaluate(
                                    e->analyticCurve->tMin() +
                                    (e->analyticCurve->tMax() - e->analyticCurve->tMin()) * i /
                                        8.0);
                            } else {
                                // A recovered rim lies in the plane it was
                                // found from: its circle's points, by angle.
                                const Circle& c = rim->second;
                                const Vec3 ref =
                                    std::abs(c.normal.x) < 0.9 ? Vec3(1, 0, 0) : Vec3(0, 1, 0);
                                const Vec3 u =
                                    c.normal.cross(ref) * (1.0 / c.normal.cross(ref).length());
                                const Vec3 w = c.normal.cross(u);
                                const double angle = 2.0 * math::kPi * i / 8.0;
                                p = c.center +
                                    (u * std::cos(angle) + w * std::sin(angle)) * c.radius;
                            }
                            accepted = std::abs((p - plane->second).dot(plane->first)) <=
                                       1e-7 * std::max(1.0, p.length());
                        }
                    }
                    if (!accepted) {
                        face.why = "a face beside it is kept in facets";
                        changed = true;
                        break;
                    }
                    curved.insert(e);
                }
                if (!face.why.empty()) break;
            }
        }
    }
    return faces;
}

/// Emit one solid; returns one solid id per body (Horizon solids may hold
/// several disjoint bodies, e.g. Pattern results — STEP expresses those as
/// sibling solids in one representation): a MANIFOLD_SOLID_BREP, or a
/// BREP_WITH_VOIDS for a body with cavities.
std::vector<int> writeSolid(StepWriter& w, const topo::Solid& solid, int index, bool asDesigned,
                            std::vector<std::string>* faceted) {
    // Its curved faces as designed, where they can be (Phase 151).
    std::unordered_set<const topo::Edge*> curved;
    std::unordered_map<const topo::Edge*, Circle> rims;
    std::vector<CurvedFace> designed;
    if (asDesigned) designed = planCurvedFaces(solid, curved, rims);
    std::unordered_map<const topo::Face*, const CurvedFace*> designedOf;
    std::unordered_set<const topo::Edge*> seams;
    for (const CurvedFace& face : designed) {
        if (!face.why.empty()) {
            if (faceted != nullptr) {
                faceted->push_back("a curved face of " + std::to_string(face.facets.size()) +
                                   " facets: " + face.why);
            }
            continue;
        }
        for (const topo::Face* f : face.facets) designedOf[f] = &face;
        for (const topo::HalfEdge* he : face.seam) seams.insert(he->edge);
    }
    // An edge between two facets of one face written as designed is inside
    // it, and not written: but for the seam it is cut along.
    const auto inside = [&](const topo::Edge& e) {
        const topo::HalfEdge* h = e.halfEdge;
        if (h == nullptr || h->twin == nullptr || seams.count(&e) != 0) return false;
        const auto a = designedOf.find(h->face);
        const auto b = designedOf.find(h->twin->face);
        return a != designedOf.end() && b != designedOf.end() && a->second == b->second;
    };

    // Vertices.
    std::unordered_map<const topo::Vertex*, int> vertexIds;
    for (const auto& v : solid.vertices()) {
        vertexIds[&v] = w.add("VERTEX_POINT('',#" + std::to_string(w.addPoint(v.point)) + ")");
    }

    // An exact face or edge whose geometry STEP holds no entity for: said in the report, with
    // what took its place.
    const auto say = [faceted](const char* what, const topo::TopologyID& id, const std::string& why,
                               bool onCarrier) {
        if (faceted == nullptr) return;
        faceted->push_back(std::string(what) + " " + (id.isValid() ? id.tag() : "(unnamed)") +
                           ": not written as its kind (" + why + "); " +
                           (onCarrier ? "written on its carrier" : "left out"));
    };

    // Edges. Orient the written curve along the recorded half-edge direction.
    std::unordered_map<const topo::Edge*, int> edgeIds;
    for (const auto& e : solid.edges()) {
        if (e.halfEdge == nullptr || e.halfEdge->twin == nullptr ||
            (e.curve == nullptr && e.exactCurve == nullptr) || inside(e)) {
            continue;
        }
        const topo::Vertex* start = e.halfEdge->origin;
        const topo::Vertex* end = e.halfEdge->twin->origin;
        const std::string ends =
            "#" + std::to_string(vertexIds.at(start)) + ",#" + std::to_string(vertexIds.at(end));
        if (e.exactCurve != nullptr) {
            // On its exact curve as its kind, from its start at curveStart to its end at
            // curveEnd: with the curve where that runs up it. A reader can tell which way round a
            // closed edge runs only from its sense, and takes its curve's way for it (Horizon's
            // reader its carrier's), so a closed edge goes out on its curve turned, where it runs
            // back down it, and with it. An open edge on a B-spline goes out on its own span of
            // it: on a closed one, an end at the join is at either end of the curve, and no
            // reader can tell which half the edge runs over.
            const bool closed = start == end;
            const bool backward = e.curveEnd < e.curveStart;
            const auto* spline = dynamic_cast<const geo::NurbsCurve*>(e.exactCurve.get());
            const auto spanned = [&]() -> bool {
                if (closed || spline == nullptr) return false;
                const double a =
                    std::clamp(std::min(e.curveStart, e.curveEnd), spline->tMin(), spline->tMax());
                const double b =
                    std::clamp(std::max(e.curveStart, e.curveEnd), spline->tMin(), spline->tMax());
                return b - a > 1e-12 * (spline->tMax() - spline->tMin());
            };
            std::string why;
            const std::optional<int> curveId =
                spanned()
                    ? std::optional<int>(writeCurve(w, spline->segment(e.curveStart, e.curveEnd)))
                    : writeExactCurve(w, *e.exactCurve, why, closed && backward);
            if (curveId) {
                edgeIds[&e] = w.add("EDGE_CURVE(''," + ends + ",#" + std::to_string(*curveId) +
                                    "," + (closed || !backward ? ".T." : ".F.") + ")");
                continue;
            }
            say("edge", e.topoId, why, e.curve != nullptr);
            if (e.curve == nullptr) continue;
        }
        if (curved.count(&e) != 0) {
            const auto rim = rims.find(&e);
            const std::optional<Circle> circle =
                rim != rims.end() ? std::optional<Circle>(rim->second) : circleOf(*e.analyticCurve);
            if (circle) {
                // On its circle, the short way from its start to its end: the
                // circle's axis turned so that way runs anticlockwise.
                const Vec3 from = start->point - circle->center;
                const Vec3 to = end->point - circle->center;
                const Vec3 axis = from.cross(to).dot(circle->normal) >= 0.0 ? circle->normal
                                                                            : circle->normal * -1.0;
                const Vec3 ref = from * (1.0 / from.length());
                const int placement = w.add(
                    "AXIS2_PLACEMENT_3D('',#" + std::to_string(w.addPoint(circle->center)) + ",#" +
                    std::to_string(w.add("DIRECTION('',(" + fmtReal(axis.x) + "," +
                                         fmtReal(axis.y) + "," + fmtReal(axis.z) + "))")) +
                    ",#" +
                    std::to_string(w.add("DIRECTION('',(" + fmtReal(ref.x) + "," + fmtReal(ref.y) +
                                         "," + fmtReal(ref.z) + "))")) +
                    ")");
                const int circleId = w.add("CIRCLE('',#" + std::to_string(placement) + "," +
                                           fmtReal(circle->radius) + ")");
                edgeIds[&e] =
                    w.add("EDGE_CURVE(''," + ends + ",#" + std::to_string(circleId) + ",.T.)");
                continue;
            }
            // A curve of the edge's own span.
            const Vec3 c0 = e.analyticCurve->evaluate(e.analyticCurve->tMin());
            const bool senseForward = (c0 - start->point).length() <= (c0 - end->point).length();
            const int curveId = writeCurve(w, *e.analyticCurve);
            edgeIds[&e] = w.add("EDGE_CURVE(''," + ends + ",#" + std::to_string(curveId) + "," +
                                (senseForward ? ".T." : ".F.") + ")");
            continue;
        }
        // same_sense: does the curve run start → end?
        const Vec3 c0 = e.curve->evaluate(e.curve->tMin());
        const bool senseForward = (c0 - start->point).length() <= (c0 - end->point).length();
        const int curveId = writeCurve(w, *e.curve);
        edgeIds[&e] = w.add("EDGE_CURVE(''," + ends + ",#" + std::to_string(curveId) + "," +
                            (senseForward ? ".T." : ".F.") + ")");
    }

    // How near a flat face's corners must lie to one plane for it to go out on that plane.
    const double flatWithin = math::ModelTolerance::forExtent(topo::exactExtent(solid)).linear;

    // Faces; a cavity's @p reversed, facing out of its void as a closed
    // shell of the void must (its bounds and its sense turned round).
    const auto sense = [](bool forward) { return forward ? std::string(".T.") : ".F."; };
    auto writeFace = [&](const topo::Face& f, bool reversed) -> std::optional<int> {
        if (f.outerLoop == nullptr) return std::nullopt;
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
                         std::to_string(loop) + "," + sense(!reversed) + ")");
        };

        std::vector<int> bounds;
        if (auto b = writeLoop(f.outerLoop, true)) bounds.push_back(*b);
        for (const topo::Wire* inner : f.innerLoops) {
            if (auto b = writeLoop(inner, false)) bounds.push_back(*b);
        }
        if (bounds.empty()) return std::nullopt;

        // Its surface, and whether that surface's normal runs the way its loops do, so that
        // its same_sense says truly which way it faces: an exact face on its exact surface as
        // its kind; a flat one with none on its plane (Ruling 13); another on its carrier, as
        // is an exact face whose surface's numbers make none STEP holds.
        std::optional<int> surfId;
        bool withLoops = true;
        if (f.exactSurface != nullptr) {
            std::string why;
            if (const auto written = writeExactSurface(w, *f.exactSurface, why)) {
                // Its loops run about its outward normal: its surface's, turned where reversed.
                surfId = written->id;
                withLoops = written->flipped == f.reversed;
            } else {
                say("face", f.topoId, why, f.surface != nullptr);
            }
        }
        if (!surfId) {
            const auto plane =
                f.exactSurface == nullptr ? flatPlaneOf(f, flatWithin) : std::nullopt;
            if (plane) {
                const auto& [normal, at] = *plane;
                const geo::Frame frame = geo::Frame::fromAxis(at, normal);
                surfId = w.add("PLANE('',#" +
                               std::to_string(writePlacement(w, at, frame.z, frame.x)) + ")");
                withLoops = true;  // its normal is its loop's
            } else if (f.surface != nullptr) {
                surfId = writeSurface(w, *f.surface);
                withLoops = carrierRunsWithLoops(f);
            } else {
                return std::nullopt;
            }
        }

        return w.add("ADVANCED_FACE(''," + StepWriter::refList(bounds) + ",#" +
                     std::to_string(*surfId) + "," + sense(withLoops != reversed) + ")");
    };

    // A curved face as designed: one face on its surface, round its outline;
    // a closed one's two outlines joined into one loop along its seam, used
    // once each way, as other systems (and this one's reader) expect.
    auto writeDesigned = [&](const CurvedFace& face, bool reversed) -> std::optional<int> {
        const auto oriented = [&](const topo::HalfEdge* he) -> std::optional<int> {
            const auto it = edgeIds.find(he->edge);
            if (it == edgeIds.end()) return std::nullopt;
            const bool forward = he->origin == he->edge->halfEdge->origin;
            return w.add("ORIENTED_EDGE('',*,*,#" + std::to_string(it->second) + "," +
                         (forward ? ".T." : ".F.") + ")");
        };
        std::vector<const topo::HalfEdge*> path;
        const auto from = [](const std::vector<const topo::HalfEdge*>& loop,
                             const topo::Vertex* v) {
            std::vector<const topo::HalfEdge*> turned;
            const auto at = std::find_if(loop.begin(), loop.end(),
                                         [v](const topo::HalfEdge* he) { return he->origin == v; });
            if (at == loop.end()) return turned;
            turned.insert(turned.end(), at, loop.end());
            turned.insert(turned.end(), loop.begin(), at);
            return turned;
        };
        if (face.seam.empty()) {
            path = face.loops.front();
        } else {
            const auto first = from(face.loops[0], face.seam.front()->origin);
            const auto second = from(face.loops[1], face.seam.back()->next->origin);
            if (first.empty() || second.empty()) return std::nullopt;
            path = first;
            path.insert(path.end(), face.seam.begin(), face.seam.end());
            path.insert(path.end(), second.begin(), second.end());
            for (auto he = face.seam.rbegin(); he != face.seam.rend(); ++he) {
                path.push_back((*he)->twin);
            }
        }
        std::vector<int> edges;
        for (const topo::HalfEdge* he : path) {
            const auto id = oriented(he);
            if (!id) return std::nullopt;
            edges.push_back(*id);
        }
        const int loop = w.add("EDGE_LOOP(''," + StepWriter::refList(edges) + ")");
        const int bound =
            w.add("FACE_OUTER_BOUND('',#" + std::to_string(loop) + "," + sense(!reversed) + ")");
        const int surfId = writeSurface(w, face.cut ? *face.cut : *face.surface);
        return w.add("ADVANCED_FACE('',(#" + std::to_string(bound) + "),#" +
                     std::to_string(surfId) + "," + sense(face.sameSense != reversed) + ")");
    };

    // A CLOSED_SHELL of @p faces.
    auto writeShell = [&](const std::vector<const topo::Face*>& faces,
                          bool reversed) -> std::optional<int> {
        std::vector<int> faceIds;
        std::unordered_set<const CurvedFace*> written;
        for (const topo::Face* f : faces) {
            if (f == nullptr) continue;
            const auto designedFace = designedOf.find(f);
            if (designedFace != designedOf.end()) {
                // Its facets are the one face: written once, for the first.
                if (!written.insert(designedFace->second).second) continue;
                if (auto id = writeDesigned(*designedFace->second, reversed)) {
                    faceIds.push_back(*id);
                }
                continue;
            }
            if (auto id = writeFace(*f, reversed)) faceIds.push_back(*id);
        }
        if (faceIds.empty()) return std::nullopt;
        return w.add("CLOSED_SHELL(''," + StepWriter::refList(faceIds) + ")");
    };
    const auto name = [index](std::size_t body) {
        return "'solid_" + std::to_string(index) + "_" + std::to_string(body) + "'";
    };

    // A body wound facing in (the kernel's box is: its builders wind either way) goes out
    // facing out, as STEP has every solid: each of its shells turned round. Only where its loops
    // measure it, its faces having no exact surface, which faces out as its `reversed` says, and
    // its edges being chords, not curves whose few corners measure nothing.
    const bool loopsMeasure = !model::describedByCurves(solid);
    const auto woundIn = [loopsMeasure](const topo::Shell& shell) {
        return loopsMeasure &&
               std::none_of(shell.faces.begin(), shell.faces.end(),
                            [](const topo::Face* f) { return f->exactSurface != nullptr; }) &&
               topo::signedVolume(shell) < 0.0;
    };

    // Each body: its outer shell, and a void for each cavity it encloses.
    std::vector<int> solidIds;
    for (const auto& shells : model::Pattern::bodyShells(solid)) {
        const auto facesOf = [](const topo::Shell* shell) {
            return std::vector<const topo::Face*>(shell->faces.begin(), shell->faces.end());
        };
        const bool turned = woundIn(*shells.front());
        const auto outer = writeShell(facesOf(shells.front()), turned);
        if (!outer) continue;
        std::vector<int> voids;
        for (std::size_t i = 1; i < shells.size(); ++i) {
            if (const auto cavity = writeShell(facesOf(shells[i]), !turned)) {
                voids.push_back(
                    w.add("ORIENTED_CLOSED_SHELL('',*,#" + std::to_string(*cavity) + ",.F.)"));
            }
        }
        solidIds.push_back(voids.empty() ? w.add("MANIFOLD_SOLID_BREP(" + name(solidIds.size()) +
                                                 ",#" + std::to_string(*outer) + ")")
                                         : w.add("BREP_WITH_VOIDS(" + name(solidIds.size()) + ",#" +
                                                 std::to_string(*outer) + "," +
                                                 StepWriter::refList(voids) + ")"));
    }
    if (solidIds.empty()) {
        // Defensive fallback for faces not registered with any shell.
        std::vector<const topo::Face*> faces;
        for (const auto& f : solid.faces()) faces.push_back(&f);
        if (auto shell = writeShell(faces, false)) {
            solidIds.push_back(
                w.add("MANIFOLD_SOLID_BREP(" + name(0) + ",#" + std::to_string(*shell) + ")"));
        }
    }
    return solidIds;
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

    /// Parse the DATA section into the instance map. False on hard error,
    /// or once @p cancelled is set.
    bool parse(std::string& error, const std::atomic<bool>* cancelled = nullptr) {
        std::string stripped = stripComments(m_text);
        const size_t dataPos = stripped.find("DATA;");
        if (dataPos == std::string::npos) {
            error = "no DATA section found";
            return false;
        }
        size_t pos = dataPos + 5;
        const size_t endPos = stripped.find("ENDSEC;", dataPos);
        const size_t limit = (endPos == std::string::npos) ? stripped.size() : endPos;

        size_t read = 0;
        while (pos < limit) {
            if (cancelled != nullptr && (++read & 1023U) == 0 &&
                cancelled->load(std::memory_order_relaxed)) {
                error = kCancelled;
                return false;
            }
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

    /// Read the digits of an entity number ("#123") at `p`. False when the
    /// number does not fit an int — accumulating it unchecked is signed
    /// overflow, and a wrapped id could alias another entity.
    static bool parseEntityNumber(const std::string& s, size_t& p, size_t end, int& id) {
        id = 0;
        while (p < end && std::isdigit(static_cast<unsigned char>(s[p]))) {
            const int digit = s[p] - '0';
            if (id > (std::numeric_limits<int>::max() - digit) / 10) return false;
            id = id * 10 + digit;
            ++p;
        }
        return true;
    }

    bool parseInstance(const std::string& s, size_t pos, size_t end, std::string& error) {
        // "#id = RHS" — RHS is TYPE(args) or (TYPE1(args) TYPE2(args) ...).
        size_t p = pos + 1;
        int id = 0;
        if (!parseEntityNumber(s, p, end, id)) {
            error = "entity number too large near offset " + std::to_string(pos);
            return false;
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
            if (!parseEntityNumber(s, p, end, id)) {
                error = "entity reference too large near offset " + std::to_string(p);
                return false;
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
            if (!parseReal(std::string_view(s).substr(p, q - p), v.num)) {
                error = "malformed number near offset " + std::to_string(p);
                return false;
            }
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
    /// @p lengthUncertainty is the file's, in its own length unit; 0 where it gives none.
    SolidBuilder(const StepParser& parser, int solidIndex,
                 const std::atomic<bool>* cancelled = nullptr, double radiansPerUnit = 1.0,
                 double lengthUncertainty = 0.0)
        : m_parser(parser),
          m_solidIndex(solidIndex),
          m_cancelled(cancelled),
          m_radiansPerUnit(radiansPerUnit),
          m_lengthUncertainty(lengthUncertainty) {}

    /// Build one topo::Solid from a group of solid ids: MANIFOLD_SOLID_BREPs,
    /// one shell each (Horizon multi-shell solids export as siblings in one
    /// shape representation), and BREP_WITH_VOIDS, an outer shell and a
    /// shell round each void, facing into it: a cavity, as a Boolean leaves
    /// one.
    std::unique_ptr<topo::Solid> build(const std::vector<int>& solidIds, bool surfaces,
                                       std::string& error) {
        std::vector<ShellRef> plan;
        if (surfaces) {
            // A surface model's shells: each that closes (Creo writes its
            // solids so) a body; one that does not, an open surface, is left
            // out, and counted.
            for (int shell : solidIds) {
                if (closes(shell)) {
                    plan.push_back({shell, false, false, plan.size()});
                } else {
                    ++m_openShells;
                }
            }
            if (plan.empty()) {
                error = m_openShells == 1 ? "its one shell is an open surface, not a solid"
                                          : "its " + std::to_string(m_openShells) +
                                                " shells are open surfaces, not solids";
                return nullptr;
            }
        } else {
            for (int solidId : solidIds) {
                if (!shellsOf(solidId, plan, error)) return nullptr;
            }
        }
        m_vertexSize = vertexSize(plan);
        auto solid = assemble(plan, error);
        if (solid == nullptr) return nullptr;

        // A void faces into itself, away from its body's material: a closed
        // shell of the void turned round, as the standard has it. Some files
        // (a SolidWorks one) write the closed shell facing in already, and
        // mark it turned round too, which would make the void more material:
        // one that measures as its outer shell does is turned round again.
        // A surface model's shell faces whichever way its writer wound it:
        // one that measures inside out is turned round.
        bool turned = false;
        if (surfaces ||
            std::any_of(plan.begin(), plan.end(), [](const ShellRef& s) { return s.isVoid; })) {
            const auto measures = model::measureShells(*solid);
            for (std::size_t i = 0; i < plan.size() && measures.size() == plan.size(); ++i) {
                const bool wrongWay =
                    plan[i].isVoid ? measures[i].volume * measures[plan[i].outer].volume > 0.0
                                   : surfaces && measures[i].volume < 0.0;
                if (wrongWay) {
                    plan[i].reversed = !plan[i].reversed;
                    turned = true;
                }
            }
        }
        m_plan = plan;
        return turned ? assemble(plan, error) : std::move(solid);
    }

    /// Which shells of the solid built are voids, in its shell order (its shells are built in
    /// the order of its plan).
    std::vector<bool> voidShells() const {
        std::vector<bool> voids;
        voids.reserve(m_plan.size());
        for (const ShellRef& shell : m_plan) voids.push_back(shell.isVoid);
        return voids;
    }

    /// The solid built again with each of @p shells (indices in its shell order) turned round, as
    /// build() turns a void or a surface model's shell that measures the wrong way: each of its
    /// faces' loops run the other way and its face reversed, so its topology and its geometry
    /// agree. Null, with @p error said, where it cannot be built.
    std::unique_ptr<topo::Solid> turned(const std::vector<std::size_t>& shells,
                                        std::string& error) {
        for (const std::size_t i : shells) {
            if (i < m_plan.size()) m_plan[i].reversed = !m_plan[i].reversed;
        }
        return assemble(m_plan, error);
    }

    /// How many of a surface model's shells were open surfaces, left out.
    std::size_t openShells() const { return m_openShells; }

    /// Why the solid built has no exact form, where a face's surface or an edge's curve could not
    /// be read exactly (its carrier could); empty when every face and edge has one.
    const std::string& exactProblem() const { return m_exactProblem; }

private:
    /// A shell to build: its CLOSED_SHELL (or OPEN_SHELL), and whether its
    /// faces are to be turned round; a void's, and the index of its body's
    /// outer shell.
    struct ShellRef {
        int id = 0;
        bool reversed = false;
        bool isVoid = false;
        std::size_t outer = 0;
    };

    /// The size of the solid @p plan builds, as its vertices measure it (the diagonal of the box
    /// round them): known before any edge is read, which the solid's own extent, from its exact
    /// curves, is not. A round part's vertices may measure it smaller; 0 when none reads.
    double vertexSize(const std::vector<ShellRef>& plan) const {
        math::BoundingBox box;
        std::unordered_set<int> seen;
        const auto vertex = [&](const StepValue& ref) {
            const StepInstance* v =
                ref.isRef() && seen.insert(ref.ref).second ? m_parser.find(ref.ref) : nullptr;
            const StepList* args = v != nullptr ? v->leaf("VERTEX_POINT") : nullptr;
            if (args == nullptr || args->size() < 2 || !(*args)[1].isRef()) return;
            if (const auto p = readPoint((*args)[1].ref); p && finite(*p)) box.expand(*p);
        };
        for (const ShellRef& shellRef : plan) {
            const StepInstance* shell = m_parser.find(shellRef.id);
            const StepList* faces = shell ? shell->leaf("CLOSED_SHELL") : nullptr;
            if (faces == nullptr && shell != nullptr) faces = shell->leaf("OPEN_SHELL");
            if (faces == nullptr || faces->size() < 2 || !(*faces)[1].isList()) continue;
            for (const StepValue& faceRef : *(*faces)[1].items) {
                const StepInstance* face = faceRef.isRef() ? m_parser.find(faceRef.ref) : nullptr;
                const StepList* fArgs = face ? face->leaf("ADVANCED_FACE") : nullptr;
                if (fArgs == nullptr && face != nullptr) fArgs = face->leaf("FACE_SURFACE");
                if (fArgs == nullptr || fArgs->size() < 2 || !(*fArgs)[1].isList()) continue;
                for (const StepValue& boundRef : *(*fArgs)[1].items) {
                    const StepInstance* bound =
                        boundRef.isRef() ? m_parser.find(boundRef.ref) : nullptr;
                    const StepList* b = bound ? bound->leaf("FACE_OUTER_BOUND") : nullptr;
                    if (b == nullptr && bound != nullptr) b = bound->leaf("FACE_BOUND");
                    const StepInstance* loop = b != nullptr && b->size() >= 2 && (*b)[1].isRef()
                                                   ? m_parser.find((*b)[1].ref)
                                                   : nullptr;
                    if (loop == nullptr) continue;
                    if (const StepList* vl = loop->leaf("VERTEX_LOOP"); vl && vl->size() >= 2) {
                        vertex((*vl)[1]);
                    }
                    const StepList* edges = loop->leaf("EDGE_LOOP");
                    if (edges == nullptr || edges->size() < 2 || !(*edges)[1].isList()) continue;
                    for (const StepValue& oeRef : *(*edges)[1].items) {
                        const StepInstance* oe = oeRef.isRef() ? m_parser.find(oeRef.ref) : nullptr;
                        const StepList* oeArgs = oe ? oe->leaf("ORIENTED_EDGE") : nullptr;
                        if (oeArgs == nullptr || oeArgs->size() < 4 || !(*oeArgs)[3].isRef()) {
                            continue;
                        }
                        const StepInstance* ec = m_parser.find((*oeArgs)[3].ref);
                        const StepList* ecArgs = ec ? ec->leaf("EDGE_CURVE") : nullptr;
                        if (ecArgs == nullptr || ecArgs->size() < 3) continue;
                        vertex((*ecArgs)[1]);
                        vertex((*ecArgs)[2]);
                    }
                }
            }
        }
        const double size = box.isValid() ? (box.max() - box.min()).length() : 0.0;
        return std::isfinite(size) ? size : 0.0;
    }

    /// The solid of the shells in @p plan, each built as it says.
    std::unique_ptr<topo::Solid> assemble(const std::vector<ShellRef>& plan, std::string& error) {
        m_solid = std::make_unique<topo::Solid>();
        m_vertices.clear();
        m_edges.clear();
        m_exactProblem.clear();
        for (const ShellRef& shellRef : plan) {
            if (!buildShell(shellRef, error)) return nullptr;
        }

        if (!linkTwins(error)) return nullptr;

        if (m_solid->faceCount() == 0) {
            error = "it has no faces";
            return nullptr;
        }
        if (!m_solid->isValid()) {
            error = "imported solid failed validation:\n" + m_solid->validationReport();
            return nullptr;
        }
        return std::move(m_solid);
    }

    /// Whether shell @p shellId closes: each of its edges bounds two of its
    /// faces, once each way, as linkTwins needs (a surface model's shell
    /// may be a solid's, or a surface's; one whose faces disagree which way
    /// they run is no solid either).
    bool closes(int shellId) const {
        const StepInstance* shell = m_parser.find(shellId);
        const StepList* args = shell ? shell->leaf("OPEN_SHELL") : nullptr;
        if (args == nullptr && shell != nullptr) args = shell->leaf("CLOSED_SHELL");
        if (args == nullptr || args->size() < 2 || !(*args)[1].isList()) return false;
        std::map<int, std::pair<int, int>> uses;  // by edge: its uses each way
        for (const StepValue& faceRef : *(*args)[1].items) {
            const StepInstance* face = faceRef.isRef() ? m_parser.find(faceRef.ref) : nullptr;
            const StepList* fArgs = face ? face->leaf("ADVANCED_FACE") : nullptr;
            if (fArgs == nullptr && face != nullptr) fArgs = face->leaf("FACE_SURFACE");
            if (fArgs == nullptr || fArgs->size() < 2 || !(*fArgs)[1].isList()) return false;
            for (const StepValue& boundRef : *(*fArgs)[1].items) {
                const StepInstance* bound =
                    boundRef.isRef() ? m_parser.find(boundRef.ref) : nullptr;
                const StepList* b = bound ? bound->leaf("FACE_OUTER_BOUND") : nullptr;
                if (b == nullptr && bound != nullptr) b = bound->leaf("FACE_BOUND");
                const StepInstance* loop = b != nullptr && b->size() >= 2 && (*b)[1].isRef()
                                               ? m_parser.find((*b)[1].ref)
                                               : nullptr;
                const StepList* edges = loop ? loop->leaf("EDGE_LOOP") : nullptr;
                if (edges == nullptr || edges->size() < 2 || !(*edges)[1].isList()) continue;
                const bool boundSense =
                    b->size() < 3 || (*b)[2].kind != StepValue::Enum || (*b)[2].text == "T";
                for (const StepValue& oeRef : *(*edges)[1].items) {
                    const StepInstance* oe = oeRef.isRef() ? m_parser.find(oeRef.ref) : nullptr;
                    const StepList* oeArgs = oe ? oe->leaf("ORIENTED_EDGE") : nullptr;
                    if (oeArgs != nullptr && oeArgs->size() >= 5 && (*oeArgs)[3].isRef()) {
                        const bool along = ((*oeArgs)[4].kind == StepValue::Enum &&
                                            (*oeArgs)[4].text == "T") == boundSense;
                        auto& [forward, backward] = uses[(*oeArgs)[3].ref];
                        ++(along ? forward : backward);
                    }
                }
            }
        }
        return std::all_of(uses.begin(), uses.end(), [](const auto& u) {
            return u.second.first == 1 && u.second.second == 1;
        });
    }

    /// The shells of solid @p solidId, onto @p shells: a MANIFOLD_SOLID_BREP's
    /// one, or a BREP_WITH_VOIDS' outer shell and its voids. Either as a
    /// simple instance, BREP_WITH_VOIDS(name, outer, (voids)), or a complex
    /// one, each of whose parts has its own attributes alone:
    /// (BREP_WITH_VOIDS((voids)) MANIFOLD_SOLID_BREP(outer) ... ).
    bool shellsOf(int solidId, std::vector<ShellRef>& shells, std::string& error) const {
        const StepInstance* solid = m_parser.find(solidId);
        const StepList* msb = solid ? solid->leaf("MANIFOLD_SOLID_BREP") : nullptr;
        const StepList* withVoids = solid ? solid->leaf("BREP_WITH_VOIDS") : nullptr;
        const StepValue* outer = nullptr;
        const StepValue* voids = nullptr;
        if (withVoids != nullptr && withVoids->size() >= 3) {
            outer = &(*withVoids)[1];
            voids = &(*withVoids)[2];
        } else if (msb != nullptr && !msb->empty()) {
            outer = &(*msb)[msb->size() >= 2 ? 1 : 0];
            if (withVoids != nullptr && !withVoids->empty()) voids = &(*withVoids)[0];
        }
        if (outer == nullptr || !outer->isRef() || (voids != nullptr && !voids->isList())) {
            error = "malformed solid #" + std::to_string(solidId);
            return false;
        }
        const std::size_t body = shells.size();
        shells.push_back({outer->ref, false, false, body});
        if (voids == nullptr) return true;
        for (const StepValue& voidRef : *voids->items) {
            // An ORIENTED_CLOSED_SHELL(name, *, closed shell, orientation):
            // .F., as the standard has every void, turns the closed shell's
            // faces, which face out of the void, round to face into it.
            const StepInstance* oriented = voidRef.isRef() ? m_parser.find(voidRef.ref) : nullptr;
            const StepList* args = oriented ? oriented->leaf("ORIENTED_CLOSED_SHELL") : nullptr;
            if (args != nullptr && args->size() >= 4 && (*args)[2].isRef()) {
                const bool sameWay = (*args)[3].kind == StepValue::Enum && (*args)[3].text == "T";
                shells.push_back({(*args)[2].ref, !sameWay, true, body});
            } else if (oriented != nullptr && oriented->hasType("CLOSED_SHELL")) {
                shells.push_back({voidRef.ref, true, true, body});  // a closed shell alone: as .F.
            } else {
                error = "malformed void of solid #" + std::to_string(solidId);
                return false;
            }
        }
        return true;
    }

    bool buildShell(const ShellRef& shellRef, std::string& error) {
        const StepInstance* shellInst = m_parser.find(shellRef.id);
        const StepList* shellArgs = shellInst ? shellInst->leaf("CLOSED_SHELL") : nullptr;
        if (shellArgs == nullptr) {
            shellArgs = shellInst ? shellInst->leaf("OPEN_SHELL") : nullptr;
        }
        if (shellArgs == nullptr || shellArgs->size() < 2 || !(*shellArgs)[1].isList()) {
            error = "malformed shell #" + std::to_string(shellRef.id);
            return false;
        }

        topo::Shell* shell = m_solid->allocShell();
        shell->solid = m_solid.get();

        for (const StepValue& faceRef : *(*shellArgs)[1].items) {
            if (m_cancelled != nullptr && m_cancelled->load(std::memory_order_relaxed)) {
                error = kCancelled;
                return false;
            }
            if (!faceRef.isRef()) continue;
            if (!buildFace(faceRef.ref, shell, shellRef.reversed, error)) return false;
        }
        return true;
    }

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

    /// A B-spline's degree from the file, when it is a whole number an int
    /// holds; 0 otherwise, which no B-spline has. A real cast to int out of
    /// its range, as a degree of 1E300, is undefined behaviour.
    static int degreeFrom(const StepValue& value) {
        const double d = value.num;
        if (value.kind != StepValue::Real || !(d >= 0.0 && d <= std::numeric_limits<int>::max()) ||
            d != std::floor(d)) {
            return 0;
        }
        return static_cast<int>(d);
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
            degree = degreeFrom((*core)[0]);
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
            degree = degreeFrom((*simple)[1]);
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
            degU = degreeFrom((*core)[0]);
            degV = degreeFrom((*core)[1]);
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
            degU = degreeFrom((*simple)[1]);
            degV = degreeFrom((*simple)[2]);
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

    /// @p curve cut to the span of it an edge from @p start to @p end uses.
    /// Another system may give an edge a longer curve (IronCAD runs a blade's
    /// edges along the whole side of its surface), and the kernel samples an
    /// edge's curve from one end to the other: a face of two such edges was
    /// read inside out. A closed edge, or one whose ends are not on the
    /// curve, keeps the curve whole.
    static std::shared_ptr<geo::NurbsCurve> spanOf(std::shared_ptr<geo::NurbsCurve> curve,
                                                   const Vec3& start, const Vec3& end) {
        if (curve == nullptr || (start - end).length() < kMergeTol) return curve;
        math::BoundingBox hull;
        for (const Vec3& p : curve->controlPoints()) hull.expand(p);
        const double near = std::max(1e-2, 1e-4 * (hull.max() - hull.min()).length());
        const double ts = curve->closestPoint(start);
        const double te = curve->closestPoint(end);
        if ((curve->evaluate(ts) - start).length() > near ||
            (curve->evaluate(te) - end).length() > near) {
            return curve;
        }
        const double domain = curve->tMax() - curve->tMin();
        const double a = std::min(ts, te);
        const double b = std::max(ts, te);
        if (a - curve->tMin() <= 1e-9 * domain && curve->tMax() - b <= 1e-9 * domain) return curve;
        if (!(b - a > 1e-9 * domain)) return curve;
        return std::make_shared<geo::NurbsCurve>(curve->segment(a, b));
    }

    /// Curve for an EDGE_CURVE, given the edge's endpoint positions. A B-spline's, as read
    /// before it is cut to the edge, into @p whole when given: the edge's exact curve.
    std::shared_ptr<geo::NurbsCurve> readEdgeGeometry(
        int curveId, const Vec3& start, const Vec3& end, int depth = 0,
        std::shared_ptr<geo::NurbsCurve>* whole = nullptr) const {
        const StepInstance* inst = m_parser.find(curveId);
        if (inst == nullptr || depth > 4) return nullptr;

        // OCC-style writers (FreeCAD et al.) wrap the 3D geometry: a
        // SURFACE_CURVE / SEAM_CURVE carries the real curve as its curve_3d
        // attribute. Depth-limit the hop to survive self-referential files.
        // An INTERSECTION_CURVE keeps it there too (IronCAD's).
        for (const char* wrapper : {"SURFACE_CURVE", "SEAM_CURVE", "INTERSECTION_CURVE"}) {
            if (const StepList* sc = inst->leaf(wrapper)) {
                if (sc->size() >= 2 && (*sc)[1].isRef()) {
                    return readEdgeGeometry((*sc)[1].ref, start, end, depth + 1, whole);
                }
                return nullptr;
            }
        }

        if (inst->hasType("B_SPLINE_CURVE_WITH_KNOTS") || inst->hasType("B_SPLINE_CURVE")) {
            auto read = readBSplineCurve(*inst);
            if (whole != nullptr) *whole = read;
            return spanOf(std::move(read), start, end);
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
        // An ellipse, where a hole meets a slanted face (Inventor's): the
        // circle's arc stretched along its axes, exact, a rational arc's
        // affine image being one. Angles are its parameter's.
        if (const StepList* ellipse = inst->leaf("ELLIPSE")) {
            if (ellipse->size() < 4 || !(*ellipse)[1].isRef()) return nullptr;
            Vec3 center;
            Vec3 zAxis;
            Vec3 xAxis;
            if (!readPlacement((*ellipse)[1].ref, center, zAxis, xAxis)) return nullptr;
            const double major = (*ellipse)[2].num;
            const double minor = (*ellipse)[3].num;
            if (!(major > 0.0) || !(minor > 0.0)) return nullptr;
            const Vec3 yAxis = zAxis.cross(xAxis);
            const auto angleOf = [&](const Vec3& p) {
                const Vec3 d = p - center;
                return std::atan2(d.dot(yAxis) / minor, d.dot(xAxis) / major);
            };
            const double a0 = angleOf(start);
            const double a1 = (start - end).length() < kMergeTol ? a0 + math::kTwoPi : angleOf(end);
            return makeArcInFrame(center, 1.0, a0, a1, xAxis * major, yAxis * minor);
        }
        return nullptr;
    }

    /// A SURFACE_OF_LINEAR_EXTRUSION(name, swept curve, VECTOR): the curve
    /// swept along the vector's direction (an ellipse's, a cylinder round
    /// it, in Unigraphics'), as far as @p loopPoints reach along it. A
    /// closed conic sweeps its whole round; a B-spline, itself.
    std::shared_ptr<geo::NurbsSurface> linearExtrusion(const StepList& args,
                                                       const std::vector<Vec3>& loopPoints) const {
        if (args.size() < 3 || !args[1].isRef() || !args[2].isRef() || loopPoints.empty()) {
            return nullptr;
        }
        const StepInstance* vector = m_parser.find(args[2].ref);
        const StepList* vArgs = vector ? vector->leaf("VECTOR") : nullptr;
        if (vArgs == nullptr || vArgs->size() < 2 || !(*vArgs)[1].isRef()) return nullptr;
        const auto along = readDirection((*vArgs)[1].ref);
        if (!along || !(along->length() > 0.0)) return nullptr;
        const Vec3 d = along->normalized();

        const StepInstance* curve = m_parser.find(args[1].ref);
        if (curve == nullptr) return nullptr;
        std::shared_ptr<geo::NurbsCurve> profile;
        if (curve->hasType("B_SPLINE_CURVE_WITH_KNOTS") || curve->hasType("B_SPLINE_CURVE")) {
            profile = readBSplineCurve(*curve);
        } else {
            const StepList* circle = curve->leaf("CIRCLE");
            const StepList* ellipse = curve->leaf("ELLIPSE");
            const StepList* conic = circle != nullptr ? circle : ellipse;
            Vec3 centre;
            Vec3 zAxis;
            Vec3 xAxis;
            if (conic == nullptr || conic->size() < (ellipse != nullptr ? 4u : 3u) ||
                !(*conic)[1].isRef() || !readPlacement((*conic)[1].ref, centre, zAxis, xAxis)) {
                return nullptr;
            }
            const double a = (*conic)[2].num;
            const double b = ellipse != nullptr ? (*conic)[3].num : a;
            if (!(a > 0.0) || !(b > 0.0)) return nullptr;
            profile =
                makeArcInFrame(centre, 1.0, 0.0, math::kTwoPi, xAxis * a, zAxis.cross(xAxis) * b);
        }
        if (profile == nullptr) return nullptr;

        // From below the lowest point of the face to above its highest,
        // measured along the sweep from the profile.
        double low = 1e300;
        double high = -1e300;
        for (const Vec3& p : loopPoints) {
            low = std::min(low, p.dot(d));
            high = std::max(high, p.dot(d));
        }
        double from = 1e300;
        double to = -1e300;
        for (const Vec3& q : profile->controlPoints()) {
            from = std::min(from, q.dot(d));
            to = std::max(to, q.dot(d));
        }
        const double start = low - to;
        const double end = std::max(high - from, start + kMergeTol);
        std::vector<std::vector<Vec3>> points;
        std::vector<std::vector<double>> weights;
        for (std::size_t i = 0; i < profile->controlPoints().size(); ++i) {
            const Vec3& q = profile->controlPoints()[i];
            points.push_back({q + d * start, q + d * end});
            weights.push_back({profile->weights()[i], profile->weights()[i]});
        }
        return std::make_shared<geo::NurbsSurface>(
            std::move(points), std::move(weights), profile->knots(),
            std::vector<double>{0.0, 0.0, 1.0, 1.0}, profile->degree(), 1);
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
        if (const StepList* swept = inst->leaf("SURFACE_OF_LINEAR_EXTRUSION")) {
            return linearExtrusion(*swept, loopPoints);
        }
        if (const StepList* cone = inst->leaf("CONICAL_SURFACE")) {
            // The radius at the placement, widening along its axis at the
            // semi-angle: the apex is behind it.
            if (cone->size() < 4 || !(*cone)[1].isRef() || loopPoints.empty()) return nullptr;
            Vec3 origin;
            Vec3 zAxis;
            Vec3 xAxis;
            if (!readPlacement((*cone)[1].ref, origin, zAxis, xAxis)) return nullptr;
            const double radius = (*cone)[2].num;
            const double semi = (*cone)[3].num * m_radiansPerUnit;
            if (!(radius >= 0.0) || !(semi > 0.0) || !(semi < math::kPi / 2)) return nullptr;
            const Vec3 apex = origin - zAxis * (radius / std::tan(semi));
            // The nappe the face is on, as far as it reaches.
            double side = 0.0;
            double reach = 0.0;
            for (const Vec3& p : loopPoints) {
                const double h = (p - apex).dot(zAxis);
                side += h;
                reach = std::max(reach, std::abs(h));
            }
            return std::make_shared<geo::NurbsSurface>(geo::NurbsSurface::makeCone(
                apex, side >= 0.0 ? zAxis : zAxis * -1.0, semi, std::max(reach, kMergeTol)));
        }
        if (const StepList* sphere = inst->leaf("SPHERICAL_SURFACE")) {
            if (sphere->size() < 3 || !(*sphere)[1].isRef()) return nullptr;
            Vec3 origin;
            Vec3 zAxis;
            Vec3 xAxis;
            if (!readPlacement((*sphere)[1].ref, origin, zAxis, xAxis)) return nullptr;
            const double radius = (*sphere)[2].num;
            if (!(radius > 0.0)) return nullptr;
            // Turned into the placement's frame, so the file's poles are the
            // surface's; and turned inside out, as NurbsSurface's sphere
            // faces in and a STEP sphere out.
            const auto unit = geo::NurbsSurface::makeSphere(Vec3(), radius);
            const Vec3 yAxis = zAxis.cross(xAxis);
            auto points = unit.controlPoints();
            for (auto& row : points) {
                for (auto& q : row) q = origin + xAxis * q.x + yAxis * q.y + zAxis * q.z;
            }
            const geo::NurbsSurface placed(std::move(points), unit.weights(), unit.knotsU(),
                                           unit.knotsV(), unit.degreeU(), unit.degreeV());
            return reverseSurfaceU(placed);
        }
        if (const StepList* torus = inst->leaf("TOROIDAL_SURFACE")) {
            if (torus->size() < 4 || !(*torus)[1].isRef()) return nullptr;
            Vec3 origin;
            Vec3 zAxis;
            Vec3 xAxis;
            if (!readPlacement((*torus)[1].ref, origin, zAxis, xAxis)) return nullptr;
            const double major = (*torus)[2].num;
            const double minor = (*torus)[3].num;
            // Any torus with a ring: one whose tube is wider than its ring (a
            // spindle torus, round a screw's head in EUCLID's) crosses its
            // axis, a surface of revolution still. No ring is a sphere gone
            // round twice, each point two places on it: not read.
            if (!(minor > 0.0) || !(major > 0.0)) return nullptr;
            return std::make_shared<geo::NurbsSurface>(
                geo::NurbsSurface::makeTorus(origin, zAxis, major, minor));
        }
        return nullptr;
    }

    // -- Exact geometry (exact kernel, stage 2) ---------------------------------
    //
    // Beside each carrier, the geometry the file gives, as it gives it: STEP's parametrisations
    // of its analytic surfaces and curves are the kernel's. A face or an edge whose numbers make
    // no exact surface or curve (though they made a carrier) leaves the solid without its exact
    // form: m_exactProblem says why, and the solid is not read.

    static bool finite(const Vec3& v) {
        return std::isfinite(v.x) && std::isfinite(v.y) && std::isfinite(v.z);
    }

    /// AXIS2_PLACEMENT_3D @p id as a frame, its axes as readPlacement reads them (the carriers'
    /// own); nullopt where its numbers are not finite or its axis has no direction.
    std::optional<geo::Frame> readFrame(int id) const {
        Vec3 origin;
        Vec3 zAxis;
        Vec3 xAxis;
        if (!readPlacement(id, origin, zAxis, xAxis) || !finite(origin) || !finite(zAxis) ||
            !finite(xAxis) || !(zAxis.length() > 0.5) || !(xAxis.length() > 0.5)) {
            return std::nullopt;
        }
        return geo::Frame::fromAxis(origin, zAxis, xAxis);
    }

    /// The frame of the placement @p args holds at @p i.
    std::optional<geo::Frame> frameAt(const StepList& args, std::size_t i) const {
        return i < args.size() && args[i].isRef() ? readFrame(args[i].ref) : std::nullopt;
    }

    /// The real @p args holds at @p i when it is finite and positive; 0 otherwise.
    static double positiveAt(const StepList& args, std::size_t i) {
        const double v = i < args.size() && args[i].kind == StepValue::Real ? args[i].num : 0.0;
        return std::isfinite(v) && v > 0.0 ? v : 0.0;
    }

    /// Notes the first reason the solid has no exact form: @p why, at @p where in the file.
    void noteExact(const std::string& where, const std::string& why) {
        if (m_exactProblem.empty()) m_exactProblem = why + " (" + where + ")";
    }

    /// The exact surface of face @p faceId, on SURFACE @p surfId: a PLANE, CYLINDRICAL_SURFACE,
    /// CONICAL_SURFACE, SPHERICAL_SURFACE or TOROIDAL_SURFACE as itself, in its placement's
    /// frame; a B-spline as @p read, the carrier read for it before any turning round. A
    /// SURFACE_OF_LINEAR_EXTRUSION has no typed surface yet: its carrier, a B-spline, is its
    /// exact surface (the one kind that does not come in as the file gives it). Null, and noted,
    /// where the file's numbers make none. A surface several faces lie on is one surface.
    std::shared_ptr<const geo::Surface> exactSurfaceOf(
        int faceId, int surfId, const std::shared_ptr<geo::NurbsSurface>& read) {
        if (const auto it = m_exactSurfaces.find(surfId); it != m_exactSurfaces.end()) {
            return it->second;
        }
        const StepInstance* inst = m_parser.find(surfId);
        if (inst == nullptr) return nullptr;  // its carrier was read, so it is there
        std::shared_ptr<const geo::Surface> exact;
        std::string why;
        if (inst->hasType("B_SPLINE_SURFACE_WITH_KNOTS") || inst->hasType("B_SPLINE_SURFACE")) {
            exact = read;
        } else if (const StepList* plane = inst->leaf("PLANE")) {
            if (const auto frame = frameAt(*plane, 1)) {
                exact = std::make_shared<geo::Plane>(*frame);
            } else {
                why = "its plane's placement cannot be read";
            }
        } else if (const StepList* cyl = inst->leaf("CYLINDRICAL_SURFACE")) {
            const auto frame = frameAt(*cyl, 1);
            const double radius = positiveAt(*cyl, 2);
            if (frame && radius > 0.0) {
                exact = std::make_shared<geo::Cylinder>(*frame, radius);
            } else {
                why = "its cylinder's placement or radius cannot be read";
            }
        } else if (inst->hasType("SURFACE_OF_LINEAR_EXTRUSION")) {
            return read;  // sized to its own face's loops: one face's alone
        } else if (const StepList* cone = inst->leaf("CONICAL_SURFACE")) {
            const auto frame = frameAt(*cone, 1);
            const double radius =
                cone->size() > 2 && (*cone)[2].kind == StepValue::Real ? (*cone)[2].num : -1.0;
            const double semi = positiveAt(*cone, 3) * m_radiansPerUnit;
            if (frame && std::isfinite(radius) && radius >= 0.0 && semi > 0.0 &&
                semi < math::kPi / 2) {
                exact = std::make_shared<geo::Cone>(*frame, radius, semi);
            } else {
                why = "its cone's placement, radius or half-angle cannot be read";
            }
        } else if (const StepList* sphere = inst->leaf("SPHERICAL_SURFACE")) {
            const auto frame = frameAt(*sphere, 1);
            const double radius = positiveAt(*sphere, 2);
            if (frame && radius > 0.0) {
                exact = std::make_shared<geo::Sphere>(*frame, radius);
            } else {
                why = "its sphere's placement or radius cannot be read";
            }
        } else if (const StepList* torus = inst->leaf("TOROIDAL_SURFACE")) {
            const auto frame = frameAt(*torus, 1);
            const double major = positiveAt(*torus, 2);
            const double minor = positiveAt(*torus, 3);
            if (frame && major > 0.0 && minor > 0.0) {
                exact = std::make_shared<geo::Torus>(*frame, major, minor);
            } else {
                why = "its torus's placement or radii cannot be read";
            }
        }
        if (exact == nullptr) {
            noteExact("face #" + std::to_string(faceId),
                      why.empty() ? "its surface cannot be read exactly" : why);
            return nullptr;
        }
        m_exactSurfaces.emplace(surfId, exact);
        return exact;
    }

    /// The exact curve of EDGE_CURVE geometry @p curveId: a LINE through its point along its
    /// VECTOR, the vector's magnitude a unit of its parameter; a CIRCLE or an ELLIPSE in its
    /// placement; a B-spline as @p whole, read for the carrier before it was cut to the edge. A
    /// SURFACE_CURVE's, SEAM_CURVE's or INTERSECTION_CURVE's own curve, as the carrier is read.
    /// Null, with @p why said, where the file's numbers make none.
    std::shared_ptr<const geo::Curve3> exactCurveOf(int curveId,
                                                    const std::shared_ptr<geo::NurbsCurve>& whole,
                                                    std::string& why, int depth = 0) const {
        const StepInstance* inst = depth > 4 ? nullptr : m_parser.find(curveId);
        if (inst == nullptr) {
            why = "its curve cannot be read";
            return nullptr;
        }
        for (const char* wrapper : {"SURFACE_CURVE", "SEAM_CURVE", "INTERSECTION_CURVE"}) {
            if (const StepList* sc = inst->leaf(wrapper)) {
                if (sc->size() >= 2 && (*sc)[1].isRef()) {
                    return exactCurveOf((*sc)[1].ref, whole, why, depth + 1);
                }
                why = "its curve cannot be read";
                return nullptr;
            }
        }
        if (inst->hasType("B_SPLINE_CURVE_WITH_KNOTS") || inst->hasType("B_SPLINE_CURVE")) {
            if (whole == nullptr) why = "its B-spline curve cannot be read";
            return whole;
        }
        if (const StepList* line = inst->leaf("LINE")) {
            // LINE(name, point, VECTOR(name, direction, magnitude)), the vector an entity of its
            // own or written in place.
            const auto origin =
                line->size() >= 3 && (*line)[1].isRef() ? readPoint((*line)[1].ref) : std::nullopt;
            const StepList* vector = nullptr;
            if (line->size() >= 3 && (*line)[2].isRef()) {
                const StepInstance* v = m_parser.find((*line)[2].ref);
                vector = v != nullptr ? v->leaf("VECTOR") : nullptr;
            } else if (line->size() >= 3 && (*line)[2].kind == StepValue::Typed &&
                       (*line)[2].text == "VECTOR" && (*line)[2].items) {
                vector = (*line)[2].items.get();
            }
            const auto direction = vector != nullptr && vector->size() >= 3 && (*vector)[1].isRef()
                                       ? readDirection((*vector)[1].ref)
                                       : std::nullopt;
            const double magnitude = vector != nullptr ? positiveAt(*vector, 2) : 0.0;
            if (!origin || !direction || !finite(*origin) || !finite(*direction) ||
                !(direction->length() > 0.0) || !(magnitude > 0.0)) {
                why = "its line's point, direction or magnitude cannot be read";
                return nullptr;
            }
            return std::make_shared<geo::LineCurve>(*origin, *direction, magnitude);
        }
        if (const StepList* circle = inst->leaf("CIRCLE")) {
            const auto frame = frameAt(*circle, 1);
            const double radius = positiveAt(*circle, 2);
            if (frame && radius > 0.0) return std::make_shared<geo::CircleCurve>(*frame, radius);
            why = "its circle's placement or radius cannot be read";
            return nullptr;
        }
        if (const StepList* ellipse = inst->leaf("ELLIPSE")) {
            const auto frame = frameAt(*ellipse, 1);
            const double a = positiveAt(*ellipse, 2);
            const double b = positiveAt(*ellipse, 3);
            if (frame && a > 0.0 && b > 0.0)
                return std::make_shared<geo::EllipseCurve>(*frame, a, b);
            why = "its ellipse's placement or semi-axes cannot be read";
            return nullptr;
        }
        why = "its curve cannot be read exactly";
        return nullptr;
    }

    /// Puts edge @p e on @p curve, from @p start at curveStart to @p end at curveEnd: the
    /// parameters its vertices invert to, the way round @p sameSense says (EDGE_CURVE's sense
    /// against its curve's). A closed edge runs one whole period, or the whole of a closed
    /// B-spline, which is not periodic here, from its join. False, with @p why said, for an edge
    /// that runs across a closed B-spline's join, which no one range of it covers, and a closed
    /// edge on a B-spline that does not close, or whose vertex is off the join.
    bool placeOnCurve(topo::Edge& e, std::shared_ptr<const geo::Curve3> curve,
                      const topo::Vertex* start, const topo::Vertex* end, bool sameSense,
                      std::string& why) const {
        const bool closed = start == end || (start->point - end->point).length() < kMergeTol;
        double t0 = curve->closestParameter(start->point);
        double t1 = curve->closestParameter(end->point);
        const geo::Interval range = curve->range();
        if (curve->periodic()) {
            const double period = range.max - range.min;
            if (closed) {
                t1 = t0 + (sameSense ? period : -period);
            } else if (sameSense && t1 <= t0) {
                t1 += period;
            } else if (!sameSense && t1 >= t0) {
                t1 -= period;
            }
        } else if (const auto* nurbs = dynamic_cast<const geo::NurbsCurve*>(curve.get())) {
            const Vec3 first = nurbs->evaluate(range.min);
            const Vec3 last = nurbs->evaluate(range.max);
            math::BoundingBox hull;
            for (const Vec3& p : nurbs->controlPoints()) hull.expand(p);
            // How near its ends must be to join, and a vertex to be at the join: the file's
            // uncertainty, no finer than the curve's resolution and no coarser than the
            // solid's limit, a thousandth of its size. The solid's exact extent is not known
            // while its edges are read, so its size is its vertices' (vertexSize), or the
            // curve's where that is larger: vertices alone may measure a round part smaller.
            const double size = (hull.max() - hull.min()).length();
            const double cap = 1e-3 * std::max(m_vertexSize, size);
            const double near = std::max(std::min(m_lengthUncertainty, cap),
                                         math::ModelTolerance::forExtent(size).linear);
            const auto atJoin = [&](const Vec3& p) {
                return (p - first).length() <= near || (p - last).length() <= near;
            };
            const bool joins = (first - last).length() <= near;
            if (closed) {
                // One period of a closed B-spline is its whole range, from its join round to it.
                if (!joins) {
                    why = "it is closed, but its B-spline curve is not";
                    return false;
                }
                if (!atJoin(start->point)) {
                    why = "its vertex is off where its closed B-spline curve joins";
                    return false;
                }
                t0 = sameSense ? range.min : range.max;
                t1 = sameSense ? range.max : range.min;
            } else if (joins) {
                if (atJoin(start->point)) t0 = sameSense ? range.min : range.max;
                if (atJoin(end->point)) t1 = sameSense ? range.max : range.min;
                if ((t1 > t0) != sameSense) {
                    why = "it runs across where its closed B-spline curve joins";
                    return false;
                }
            }
        }
        if (!std::isfinite(t0) || !std::isfinite(t1)) {
            why = "its vertices cannot be placed on its curve";
            return false;
        }
        e.exactCurve = std::move(curve);
        e.curveStart = t0;
        e.curveEnd = t1;
        return true;
    }

    /// Edge @p e on @p curve from @p t0 to @p t1: a seam the reader makes, its curve its own.
    static void placeSeam(topo::Edge& e, std::shared_ptr<const geo::Curve3> curve, double t0,
                          double t1) {
        e.exactCurve = std::move(curve);
        e.curveStart = t0;
        e.curveEnd = t1;
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
        std::shared_ptr<geo::NurbsCurve> whole;
        auto curve = readEdgeGeometry((*args)[3].ref, sameSense ? start->point : end->point,
                                      sameSense ? end->point : start->point, 0, &whole);
        if (curve == nullptr) {
            error = "unsupported curve geometry on EDGE_CURVE #" + std::to_string(edgeCurveId);
            return nullptr;
        }

        topo::Edge* e = m_solid->allocEdge();
        e->curve = std::move(curve);
        e->topoId = topo::TopologyID::make(
            "step", "solid:" + std::to_string(m_solidIndex) + "/e:" + std::to_string(edgeCurveId));
        // Its exact curve, beside the carrier: from its start vertex, where its half-edge
        // running start to end (linkTwins makes it the edge's) begins.
        std::string why;
        auto exact = exactCurveOf((*args)[3].ref, whole, why);
        if (exact == nullptr || !placeOnCurve(*e, std::move(exact), start, end, sameSense, why)) {
            noteExact("edge #" + std::to_string(edgeCurveId), why);
        }
        EdgeRecord rec;
        rec.edge = e;
        rec.start = start;
        rec.end = end;
        auto [ins, ok] = m_edges.emplace(edgeCurveId, std::move(rec));
        (void)ok;
        return &ins->second;
    }

    /// The loop a whole closed face is bounded by, into @p loopEdges, as
    /// Open CASCADE writes it: a sphere's seam from pole to pole, there and
    /// back, on its placement's x side; a torus's two seam circles through
    /// a point on its outer equator, each there and back. For a face whose
    /// only bound is a VERTEX_LOOP; false, said why, for another surface.
    bool seamsOf(int faceId, int surfaceId, std::vector<std::pair<EdgeRecord*, bool>>& loopEdges,
                 std::string& error) {
        const StepInstance* surface = m_parser.find(surfaceId);
        const StepList* sphere = surface ? surface->leaf("SPHERICAL_SURFACE") : nullptr;
        const StepList* torus = surface ? surface->leaf("TOROIDAL_SURFACE") : nullptr;
        const StepList* args = sphere != nullptr ? sphere : torus;
        Vec3 origin;
        Vec3 zAxis;
        Vec3 xAxis;
        if (args == nullptr || args->size() < (torus != nullptr ? 4u : 3u) || !(*args)[1].isRef() ||
            !readPlacement((*args)[1].ref, origin, zAxis, xAxis)) {
            error = "face #" + std::to_string(faceId) +
                    " is bounded by a point alone, on a surface that is not a sphere or a torus";
            return false;
        }
        const Vec3 yAxis = zAxis.cross(xAxis);
        const std::string at =
            "solid:" + std::to_string(m_solidIndex) + "/f:" + std::to_string(faceId) + "/seam";
        const auto vertexAt = [&](const Vec3& point, const std::string& name) {
            topo::Vertex* v = m_solid->allocVertex();
            v->point = point;
            v->topoId = topo::TopologyID::make("step", at + "/v:" + name);
            return v;
        };
        // Keyed below every edge the file names (they are all positive).
        const auto seam = [&](int k, topo::Vertex* start, topo::Vertex* end,
                              std::shared_ptr<geo::NurbsCurve> curve) {
            topo::Edge* e = m_solid->allocEdge();
            e->curve = std::move(curve);
            e->topoId = topo::TopologyID::make("step", at + "/e:" + std::to_string(k));
            EdgeRecord rec;
            rec.edge = e;
            rec.start = start;
            rec.end = end;
            return &m_edges.emplace(-4 * faceId - k, std::move(rec)).first->second;
        };
        // The seams' exact curves: the carriers' circles, in the surface's frame.
        const auto frame = readFrame((*args)[1].ref);
        if (!frame) noteExact("face #" + std::to_string(faceId), "its placement cannot be read");
        if (sphere != nullptr) {
            const double r = (*args)[2].num;
            if (!(r > 0.0)) return false;
            topo::Vertex* south = vertexAt(origin - zAxis * r, "s");
            topo::Vertex* north = vertexAt(origin + zAxis * r, "n");
            EdgeRecord* meridian =
                seam(1, south, north,
                     makeArcInFrame(origin, r, -math::kPi / 2.0, math::kPi / 2.0, xAxis, zAxis));
            if (frame) {
                // In the plane of x and z, from the south pole to the north.
                placeSeam(
                    *meridian->edge,
                    std::make_shared<geo::CircleCurve>(
                        geo::Frame::fromAxis(frame->origin, frame->x.cross(frame->z), frame->x), r),
                    -math::kPi / 2.0, math::kPi / 2.0);
            }
            loopEdges = {{meridian, true}, {meridian, false}};
            return true;
        }
        const double major = (*args)[2].num;
        const double minor = (*args)[3].num;
        if (!(minor > 0.0) || !(major > 0.0)) return false;  // as the torus is read
        topo::Vertex* corner = vertexAt(origin + xAxis * (major + minor), "p");
        EdgeRecord* round =
            seam(1, corner, corner,
                 makeArcInFrame(origin, major + minor, 0.0, math::kTwoPi, xAxis, yAxis));
        EdgeRecord* tube =
            seam(2, corner, corner,
                 makeArcInFrame(origin + xAxis * major, minor, 0.0, math::kTwoPi, xAxis, zAxis));
        if (frame) {
            placeSeam(*round->edge, std::make_shared<geo::CircleCurve>(*frame, major + minor), 0.0,
                      math::kTwoPi);
            placeSeam(*tube->edge,
                      std::make_shared<geo::CircleCurve>(
                          geo::Frame::fromAxis(frame->origin + frame->x * major,
                                               frame->x.cross(frame->z), frame->x),
                          minor),
                      0.0, math::kTwoPi);
        }
        loopEdges = {{round, true}, {tube, true}, {round, false}, {tube, false}};
        return true;
    }

    /// The seam of face @p faceId from @p rim up its surface to @p apex: a
    /// cone's generator, a sphere's meridian. Null for another surface.
    EdgeRecord* seamTo(int faceId, int surfaceId, topo::Vertex* rim, topo::Vertex* apex) {
        const StepInstance* surface = m_parser.find(surfaceId);
        std::shared_ptr<geo::NurbsCurve> curve;
        // Its exact curve, from the rim at curveStart to the apex at the end.
        std::shared_ptr<const geo::Curve3> exact;
        double end = 0.0;
        if (surface != nullptr && surface->hasType("CONICAL_SURFACE")) {
            curve = std::make_shared<geo::NurbsCurve>(std::vector<Vec3>{rim->point, apex->point},
                                                      std::vector<double>{1.0, 1.0},
                                                      std::vector<double>{0.0, 0.0, 1.0, 1.0}, 1);
            end = (apex->point - rim->point).length();
            if (end > 0.0 && std::isfinite(end)) {
                exact = std::make_shared<geo::LineCurve>(rim->point, apex->point - rim->point);
            }
        } else if (const StepList* sphere =
                       surface != nullptr ? surface->leaf("SPHERICAL_SURFACE") : nullptr) {
            Vec3 centre;
            Vec3 zAxis;
            Vec3 xAxis;
            if (sphere->size() < 3 || !(*sphere)[1].isRef() ||
                !readPlacement((*sphere)[1].ref, centre, zAxis, xAxis)) {
                return nullptr;
            }
            const double r = (*sphere)[2].num;
            const Vec3 from = rim->point - centre;
            Vec3 toward = (apex->point - centre) - from * ((apex->point - centre).dot(from) /
                                                           std::max(from.dot(from), 1e-300));
            if (!(r > 0.0) || !(toward.length() > 1e-12 * r)) return nullptr;
            toward = toward.normalized();
            const double angle = std::atan2((apex->point - centre).dot(toward),
                                            (apex->point - centre).dot(from.normalized()));
            curve = makeArcInFrame(centre, r, 0.0, angle, from.normalized(), toward);
            const Vec3 along = from.normalized();
            if (finite(centre) && finite(toward) && along.length() > 0.5 && std::isfinite(angle)) {
                // In the plane of the rim and the apex, from the rim.
                exact = std::make_shared<geo::CircleCurve>(
                    geo::Frame::fromAxis(centre, along.cross(toward), along), r);
                end = angle;
            }
        }
        if (curve == nullptr) return nullptr;
        topo::Edge* e = m_solid->allocEdge();
        e->curve = std::move(curve);
        e->topoId = topo::TopologyID::make("step", "solid:" + std::to_string(m_solidIndex) +
                                                       "/f:" + std::to_string(faceId) + "/seam");
        if (exact != nullptr) {
            placeSeam(*e, std::move(exact), 0.0, end);
        } else {
            noteExact("face #" + std::to_string(faceId), "its seam up to its apex cannot be made");
        }
        EdgeRecord rec;
        rec.edge = e;
        rec.start = rim;
        rec.end = apex;
        return &m_edges.emplace(-4 * faceId - 3, std::move(rec)).first->second;
    }

    /// Face @p faceId into @p shell; @p reversed, turned round: its loops
    /// run the other way and its normal is its surface's reversed.
    bool buildFace(int faceId, topo::Shell* shell, bool reversed, std::string& error) {
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

        // A point beside one other bound (a cone's apex, a sphere's pole, as
        // Unigraphics writes them): the other loop runs from its rim up a
        // seam to the point and back, as other writers give it, so that the
        // face reaches the point.
        topo::Vertex* apex = nullptr;
        if ((*args)[1].items->size() == 2) {
            for (const StepValue& boundRef : *(*args)[1].items) {
                const StepInstance* bound =
                    boundRef.isRef() ? m_parser.find(boundRef.ref) : nullptr;
                const StepList* b = bound ? bound->leaf("FACE_BOUND") : nullptr;
                if (b == nullptr && bound != nullptr) b = bound->leaf("FACE_OUTER_BOUND");
                const StepInstance* loop = b != nullptr && b->size() >= 2 && (*b)[1].isRef()
                                               ? m_parser.find((*b)[1].ref)
                                               : nullptr;
                const StepList* vl = loop ? loop->leaf("VERTEX_LOOP") : nullptr;
                if (vl != nullptr && vl->size() >= 2 && (*vl)[1].isRef()) {
                    apex = vertexFor((*vl)[1].ref);
                }
            }
        }

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
                ((*bArgs)[2].kind == StepValue::Enum && (*bArgs)[2].text == "T") != reversed;

            const StepInstance* loop = m_parser.find((*bArgs)[1].ref);
            std::vector<std::pair<EdgeRecord*, bool>> loopEdges;
            if (loop != nullptr && loop->hasType("VERTEX_LOOP")) {
                // A point bounds nothing beside other bounds (a cone's apex).
                // Alone, it bounds a closed surface's whole face (a sphere's,
                // a torus's, as Unigraphics writes them): the seams other
                // writers give it instead, which the kernel's loops need.
                if ((*args)[1].items->size() > 1) continue;
                if (!seamsOf(faceId, (*args)[2].ref, loopEdges, error)) return false;
            }
            const StepList* lArgs = loop ? loop->leaf("EDGE_LOOP") : nullptr;
            if (loopEdges.empty() &&
                (lArgs == nullptr || lArgs->size() < 2 || !(*lArgs)[1].isList())) {
                error = "malformed EDGE_LOOP on face #" + std::to_string(faceId);
                return false;
            }

            // Collect (edge, forward) pairs in loop order; honour bound orientation.
            for (const StepValue& oeRef : lArgs != nullptr ? *(*lArgs)[1].items : StepList{}) {
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
            if (apex != nullptr && !loopEdges.empty()) {
                const auto& [first, forward] = loopEdges.front();
                topo::Vertex* rim = forward ? first->start : first->end;
                if (EdgeRecord* up = seamTo(faceId, (*args)[2].ref, rim, apex)) {
                    loopEdges.emplace_back(up, true);
                    loopEdges.emplace_back(up, false);
                }
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
            // Files that use plain FACE_BOUND for every loop (Alibre's): the
            // outer is the one that encloses the others, so the widest. The
            // first was taken, and an annulus whose inner circle came first
            // was read inside out.
            if (face->innerLoops.empty()) {
                error = "face #" + std::to_string(faceId) + " has no bounds";
                return false;
            }
            const auto widthOf = [](const topo::Wire* wire) {
                // Its vertices and its curves' control points, which bound
                // them: a circle's loop has one vertex.
                math::BoundingBox box;
                const topo::HalfEdge* he = wire->halfEdge;
                do {
                    box.expand(he->origin->point);
                    if (he->edge != nullptr && he->edge->curve != nullptr) {
                        for (const Vec3& p : he->edge->curve->controlPoints()) box.expand(p);
                    }
                    he = he->next;
                } while (he != nullptr && he != wire->halfEdge);
                return box.isValid() ? (box.max() - box.min()).length() : 0.0;
            };
            auto widest = face->innerLoops.begin();
            for (auto it = std::next(widest); it != face->innerLoops.end(); ++it) {
                if (widthOf(*it) > widthOf(*widest)) widest = it;
            }
            face->outerLoop = *widest;
            face->innerLoops.erase(widest);
        }

        face->surface = readFaceGeometry((*args)[2].ref, loopPoints);
        if (face->surface == nullptr) {
            error = "unsupported surface geometry on face #" + std::to_string(faceId);
            return false;
        }

        // ADVANCED_FACE same_sense: .F. means the face normal is the reverse
        // of the surface normal.  The carrier has no per-face sense flag (its
        // convention is surface normal == outward face normal), so bake the
        // flip into the surface itself.
        const bool sameSenseFace = (args->size() < 4 || (*args)[3].kind != StepValue::Enum ||
                                    (*args)[3].text != "F") != reversed;
        // The exact face keeps its surface as the file gives it, and says which way it faces.
        face->exactSurface = exactSurfaceOf(faceId, (*args)[2].ref, face->surface);
        face->reversed = !sameSenseFace;
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
    const std::atomic<bool>* m_cancelled;
    double m_radiansPerUnit;     ///< of the file's plane angle unit
    double m_lengthUncertainty;  ///< the file's, in its length unit; 0 where it gives none
    double m_vertexSize = 0.0;   ///< the solid's size as its vertices measure it (vertexSize)
    std::unique_ptr<topo::Solid> m_solid;
    std::unordered_map<int, topo::Vertex*> m_vertices;
    std::map<int, EdgeRecord> m_edges;
    /// The shells build() built, each as it turned them: the solid's shells, in order.
    std::vector<ShellRef> m_plan;
    std::size_t m_openShells = 0;
    /// Each typed or B-spline surface's exact surface, by its SURFACE: shared by its faces.
    std::unordered_map<int, std::shared_ptr<const geo::Surface>> m_exactSurfaces;
    std::string m_exactProblem;
};

// ===========================================================================
// Units
// ===========================================================================

/// Millimetres per unit of the length unit instance `id`: an SI_UNIT of
/// metres with its prefix, or a CONVERSION_BASED_UNIT through its measure
/// ('INCH' is 25.4 of a millimetre unit). 0 when it cannot be read.
double unitMillimetres(const StepParser& parser, int id, std::string* name, int depth = 0) {
    const StepInstance* inst = depth > 8 ? nullptr : parser.find(id);
    if (inst == nullptr) return 0.0;
    if (const StepList* si = inst->leaf("SI_UNIT")) {
        if (si->size() < 2 || (*si)[1].kind != StepValue::Enum || (*si)[1].text != "METRE") {
            return 0.0;
        }
        static const std::map<std::string, double> kPrefixes = {
            {"EXA", 1e18},  {"PETA", 1e15},  {"TERA", 1e12},   {"GIGA", 1e9},
            {"MEGA", 1e6},  {"KILO", 1e3},   {"HECTO", 1e2},   {"DECA", 1e1},
            {"DECI", 1e-1}, {"CENTI", 1e-2}, {"MILLI", 1e-3},  {"MICRO", 1e-6},
            {"NANO", 1e-9}, {"PICO", 1e-12}, {"FEMTO", 1e-15}, {"ATTO", 1e-18}};
        double metres = 1.0;
        std::string prefix;
        if ((*si)[0].kind == StepValue::Enum) {
            const auto it = kPrefixes.find((*si)[0].text);
            if (it == kPrefixes.end()) return 0.0;
            metres = it->second;
            prefix = it->first;
            std::transform(prefix.begin(), prefix.end(), prefix.begin(),
                           [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
        }
        if (name) *name = prefix + "metres";
        return metres * 1000.0;
    }
    if (const StepList* cb = inst->leaf("CONVERSION_BASED_UNIT")) {
        if (cb->size() < 2 || !(*cb)[1].isRef()) return 0.0;
        const StepInstance* measure = parser.find((*cb)[1].ref);
        const StepList* args = measure ? measure->leaf("LENGTH_MEASURE_WITH_UNIT") : nullptr;
        if (args == nullptr) args = measure ? measure->leaf("MEASURE_WITH_UNIT") : nullptr;
        if (args == nullptr || args->size() < 2 || !(*args)[1].isRef()) return 0.0;
        // The value: LENGTH_MEASURE(25.4), or a bare real.
        double value = 0.0;
        const StepValue& v = (*args)[0];
        if (v.kind == StepValue::Real) {
            value = v.num;
        } else if (v.kind == StepValue::Typed && v.items && !v.items->empty() &&
                   v.items->front().kind == StepValue::Real) {
            value = v.items->front().num;
        }
        const double base = unitMillimetres(parser, (*args)[1].ref, nullptr, depth + 1);
        if (!(value > 0.0) || !(base > 0.0) || !std::isfinite(value * base)) return 0.0;
        if (name) {
            std::string unit = (*cb)[0].kind == StepValue::Str ? (*cb)[0].text : "";
            std::transform(unit.begin(), unit.end(), unit.begin(),
                           [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
            if (unit == "inch") unit = "inches";
            if (unit == "foot") unit = "feet";
            *name = unit.empty() ? "a converted unit" : unit;
        }
        return value * base;
    }
    return 0.0;
}

/// Radians per unit of the plane angle unit instance `id`: an SI_UNIT of
/// radians, or a CONVERSION_BASED_UNIT through its measure ('DEGREE' is
/// 0.01745 of a radian). 0 when it cannot be read.
double unitRadians(const StepParser& parser, int id, int depth = 0) {
    const StepInstance* inst = depth > 8 ? nullptr : parser.find(id);
    if (inst == nullptr) return 0.0;
    if (const StepList* si = inst->leaf("SI_UNIT")) {
        const bool radian =
            si->size() >= 2 && (*si)[1].kind == StepValue::Enum && (*si)[1].text == "RADIAN";
        return radian ? 1.0 : 0.0;
    }
    if (const StepList* cb = inst->leaf("CONVERSION_BASED_UNIT")) {
        if (cb->size() < 2 || !(*cb)[1].isRef()) return 0.0;
        const StepInstance* measure = parser.find((*cb)[1].ref);
        const StepList* args = measure ? measure->leaf("PLANE_ANGLE_MEASURE_WITH_UNIT") : nullptr;
        if (args == nullptr) args = measure ? measure->leaf("MEASURE_WITH_UNIT") : nullptr;
        if (args == nullptr || args->size() < 2 || !(*args)[1].isRef()) return 0.0;
        double value = 0.0;
        const StepValue& v = (*args)[0];
        if (v.kind == StepValue::Real) {
            value = v.num;
        } else if (v.kind == StepValue::Typed && v.items && !v.items->empty() &&
                   v.items->front().kind == StepValue::Real) {
            value = v.items->front().num;
        }
        const double base = unitRadians(parser, (*args)[1].ref, depth + 1);
        if (!(value > 0.0) || !(base > 0.0) || !std::isfinite(value * base)) return 0.0;
        return value * base;
    }
    return 0.0;
}

/// Radians per plane angle unit in the representation context `contextId`:
/// 1 when it names none, or one that cannot be read.
double contextRadians(const StepParser& parser, int contextId) {
    const StepInstance* ctx = parser.find(contextId);
    const StepList* units = ctx ? ctx->leaf("GLOBAL_UNIT_ASSIGNED_CONTEXT") : nullptr;
    if (units == nullptr || units->empty() || !(*units)[0].isList()) return 1.0;
    for (const StepValue& u : *(*units)[0].items) {
        if (!u.isRef()) continue;
        const StepInstance* unit = parser.find(u.ref);
        if (unit != nullptr && unit->hasType("PLANE_ANGLE_UNIT")) {
            const double radians = unitRadians(parser, u.ref);
            return radians > 0.0 ? radians : 1.0;
        }
    }
    return 1.0;
}

/// Millimetres per length unit in the representation context `contextId`:
/// the LENGTH_UNIT of its GLOBAL_UNIT_ASSIGNED_CONTEXT. 0 when it names
/// none, or one that cannot be read: neither is taken for the millimetre.
double contextMillimetres(const StepParser& parser, int contextId, std::string* name) {
    const StepInstance* ctx = parser.find(contextId);
    const StepList* units = ctx ? ctx->leaf("GLOBAL_UNIT_ASSIGNED_CONTEXT") : nullptr;
    if (units == nullptr || units->empty() || !(*units)[0].isList()) return 0.0;
    for (const StepValue& u : *(*units)[0].items) {
        if (!u.isRef()) continue;
        const StepInstance* unit = parser.find(u.ref);
        if (unit != nullptr && unit->hasType("LENGTH_UNIT")) {
            return unitMillimetres(parser, u.ref, name);
        }
    }
    return 0.0;
}

/// The length uncertainty the representation context `contextId` gives
/// (GLOBAL_UNCERTAINTY_ASSIGNED_CONTEXT), in millimetres: the largest of its
/// UNCERTAINTY_MEASURE_WITH_UNITs that measure a length, each in its own
/// unit, or in `mm` per unit (the solid's) where a LENGTH_MEASURE's cannot
/// be read. 0 when it gives none.
double contextUncertaintyMillimetres(const StepParser& parser, int contextId, double mm) {
    const StepInstance* ctx = parser.find(contextId);
    const StepList* list = ctx ? ctx->leaf("GLOBAL_UNCERTAINTY_ASSIGNED_CONTEXT") : nullptr;
    if (list == nullptr || list->empty() || !(*list)[0].isList()) return 0.0;
    double largest = 0.0;
    for (const StepValue& u : *(*list)[0].items) {
        const StepInstance* measure = u.isRef() ? parser.find(u.ref) : nullptr;
        const StepList* args = measure ? measure->leaf("UNCERTAINTY_MEASURE_WITH_UNIT") : nullptr;
        if (args == nullptr || args->size() < 2) continue;
        const double unit =
            (*args)[1].isRef() ? unitMillimetres(parser, (*args)[1].ref, nullptr) : 0.0;
        // LENGTH_MEASURE(1.E-5), or a bare real in a length unit.
        const StepValue& v = (*args)[0];
        double value = 0.0;
        if (v.kind == StepValue::Typed && v.text == "LENGTH_MEASURE" && v.items &&
            !v.items->empty() && v.items->front().kind == StepValue::Real) {
            value = v.items->front().num * (unit > 0.0 ? unit : mm);
        } else if (v.kind == StepValue::Real && unit > 0.0) {
            value = v.num * unit;
        }
        if (std::isfinite(value) && value > largest) largest = value;
    }
    return largest;
}

// ===========================================================================
// Product structure (Phase 153)
// ===========================================================================

/// A Part-21 string's text as UTF-8: its \\ and its \X2\...\X0\ (UTF-16),
/// \X4\...\X0\ (UTF-32), \X\hh and \S\c (ISO 8859-1) escapes undone.
std::string decodeStepText(const std::string& in) {
    std::string out;
    const auto put = [&out](std::uint32_t cp) {
        if (cp < 0x80) {
            out += static_cast<char>(cp);
        } else if (cp < 0x800) {
            out += static_cast<char>(0xC0 | (cp >> 6));
            out += static_cast<char>(0x80 | (cp & 0x3F));
        } else if (cp < 0x10000) {
            out += static_cast<char>(0xE0 | (cp >> 12));
            out += static_cast<char>(0x80 | ((cp >> 6) & 0x3F));
            out += static_cast<char>(0x80 | (cp & 0x3F));
        } else if (cp < 0x110000) {
            out += static_cast<char>(0xF0 | (cp >> 18));
            out += static_cast<char>(0x80 | ((cp >> 12) & 0x3F));
            out += static_cast<char>(0x80 | ((cp >> 6) & 0x3F));
            out += static_cast<char>(0x80 | (cp & 0x3F));
        }
    };
    const auto hex = [&in](std::size_t at, std::size_t digits, std::uint32_t& value) {
        if (at + digits > in.size()) return false;
        value = 0;
        for (std::size_t k = 0; k < digits; ++k) {
            const char c = in[at + k];
            const int d = std::isdigit(static_cast<unsigned char>(c)) != 0 ? c - '0'
                          : (c >= 'A' && c <= 'F')                         ? c - 'A' + 10
                          : (c >= 'a' && c <= 'f')                         ? c - 'a' + 10
                                                                           : -1;
            if (d < 0) return false;
            value = value * 16 + static_cast<std::uint32_t>(d);
        }
        return true;
    };
    std::size_t i = 0;
    while (i < in.size()) {
        if (in[i] != '\\') {
            out += in[i++];
            continue;
        }
        if (in.compare(i, 2, "\\\\") == 0) {
            out += '\\';
            i += 2;
        } else if (in.compare(i, 4, "\\X2\\") == 0 || in.compare(i, 4, "\\X4\\") == 0) {
            const std::size_t digits = in[i + 2] == '2' ? 4 : 8;
            std::size_t at = i + 4;
            std::uint32_t unit = 0;
            std::uint32_t high = 0;  // a UTF-16 high surrogate waiting for its low
            // What is not a character (half a pair, a surrogate on its own,
            // beyond Unicode) is the replacement character, not dropped.
            constexpr std::uint32_t kReplacement = 0xFFFD;
            while (at < in.size() && in[at] != '\\' && hex(at, digits, unit)) {
                const bool surrogate = unit >= 0xD800 && unit < 0xE000;
                const bool highHalf = digits == 4 && unit >= 0xD800 && unit < 0xDC00;
                const bool lowHalf = digits == 4 && unit >= 0xDC00 && unit < 0xE000;
                if (high != 0 && !lowHalf) {
                    put(kReplacement);
                    high = 0;
                }
                if (highHalf) {
                    high = unit;
                } else if (lowHalf && high != 0) {
                    put(0x10000 + ((high - 0xD800) << 10) + (unit - 0xDC00));
                    high = 0;
                } else {
                    put(surrogate || unit > 0x10FFFF ? kReplacement : unit);
                }
                at += digits;
            }
            if (high != 0) put(kReplacement);
            i = in.compare(at, 4, "\\X0\\") == 0 ? at + 4 : at;
        } else if (in.compare(i, 3, "\\X\\") == 0) {
            std::uint32_t byte = 0;
            if (hex(i + 3, 2, byte)) {
                put(byte);
                i += 5;
            } else {
                out += in[i++];
            }
        } else if (in.compare(i, 3, "\\S\\") == 0 && i + 3 < in.size()) {
            put(static_cast<unsigned char>(in[i + 3]) + 128U);
            i += 4;
        } else if (in.compare(i, 2, "\\P") == 0 && i + 3 < in.size() && in[i + 3] == '\\') {
            i += 4;  // a code page chosen for \S\: taken as ISO 8859-1
        } else {
            out += in[i++];
        }
    }
    return out;
}

/// @p text (UTF-8) as a Part-21 string, quoted: its quote and backslash
/// doubled, printable ASCII as it is, and all else \X2\ or \X4\ escaped.
std::string stepText(const std::string& text) {
    std::string out = "'";
    std::string wide;  // a run of UTF-16 code units, in hex
    const auto flush = [&] {
        if (wide.empty()) return;
        out += "\\X2\\" + wide + "\\X0\\";
        wide.clear();
    };
    const auto hex = [](std::uint32_t v, int digits) {
        std::string h(static_cast<std::size_t>(digits), '0');
        for (int k = digits - 1; k >= 0; --k, v >>= 4) {
            h[static_cast<std::size_t>(k)] = "0123456789ABCDEF"[v & 0xF];
        }
        return h;
    };
    std::size_t i = 0;
    while (i < text.size()) {
        const auto c = static_cast<unsigned char>(text[i]);
        std::uint32_t cp = '?';
        std::size_t length = 1;
        if (c < 0x80) {
            cp = c;
        } else if ((c & 0xE0) == 0xC0) {
            cp = c & 0x1FU;
            length = 2;
        } else if ((c & 0xF0) == 0xE0) {
            cp = c & 0x0FU;
            length = 3;
        } else if ((c & 0xF8) == 0xF0) {
            cp = c & 0x07U;
            length = 4;
        }
        bool valid = c < 0x80 || length > 1;
        for (std::size_t k = 1; valid && k < length; ++k) {
            const auto next =
                i + k < text.size() ? static_cast<unsigned char>(text[i + k]) : std::uint8_t{0};
            valid = (next & 0xC0) == 0x80;
            cp = (cp << 6) | (next & 0x3FU);
        }
        if (!valid) {
            cp = '?';
            length = 1;
        }
        i += length;
        if (cp >= 0x20 && cp < 0x7F) {
            flush();
            if (cp == '\'')
                out += "''";
            else if (cp == '\\')
                out += "\\\\";
            else
                out += static_cast<char>(cp);
        } else if (cp < 0x10000) {
            wide += hex(cp, 4);
        } else {
            flush();
            out += "\\X4\\" + hex(cp, 8) + "\\X0\\";
        }
    }
    flush();
    return out + "'";
}

/// The text of @p value when it is a string; empty when it is not.
std::string textOf(const StepValue& value) {
    return value.kind == StepValue::Str ? decodeStepText(value.text) : std::string();
}

/// Whether @p text is empty or only spaces.
bool blank(const std::string& text) {
    return std::all_of(text.begin(), text.end(),
                       [](char c) { return std::isspace(static_cast<unsigned char>(c)) != 0; });
}

/// The context of representation @p repId: the last attribute of any
/// representation, simple or complex ('name', (items), #context).
int representationContext(const StepParser& parser, int repId) {
    const StepInstance* rep = parser.find(repId);
    if (rep == nullptr) return 0;
    for (const auto& [type, args] : rep->leaves) {
        if (args.size() >= 3 && args[1].isList() && args[2].isRef()) return args[2].ref;
    }
    return 0;
}

/// A relationship between two representations, simple or complex:
/// (name, description, rep_1, rep_2), and its transformation's id (0 for
/// none).
struct Relationship {
    int rep1 = 0;
    int rep2 = 0;
    int transformation = 0;
};

std::optional<Relationship> relationshipOf(const StepInstance& inst) {
    Relationship r;
    const StepList* withTransform = inst.leaf("REPRESENTATION_RELATIONSHIP_WITH_TRANSFORMATION");
    if (withTransform != nullptr && !withTransform->empty()) {
        // Complex: its own one attribute. Simple (as some write it): after
        // the relationship's four.
        const StepValue& t = withTransform->size() >= 5 ? (*withTransform)[4] : (*withTransform)[0];
        if (t.isRef()) r.transformation = t.ref;
    }
    for (const char* type : {"REPRESENTATION_RELATIONSHIP", "SHAPE_REPRESENTATION_RELATIONSHIP",
                             "REPRESENTATION_RELATIONSHIP_WITH_TRANSFORMATION"}) {
        const StepList* args = inst.leaf(type);
        if (args != nullptr && args->size() >= 4 && (*args)[2].isRef() && (*args)[3].isRef()) {
            r.rep1 = (*args)[2].ref;
            r.rep2 = (*args)[3].ref;
            return r;
        }
    }
    return std::nullopt;
}

/// The matrix taking an AXIS2_PLACEMENT_3D's own coordinates into its
/// representation's, its origin in millimetres by @p mm.
std::optional<Mat4> axisMatrix(const StepParser& parser, int id, double mm) {
    const auto triple = [&parser](const StepValue& value, const char* type) -> std::optional<Vec3> {
        const StepInstance* inst = value.isRef() ? parser.find(value.ref) : nullptr;
        const StepList* args = inst ? inst->leaf(type) : nullptr;
        if (args == nullptr || args->size() < 2 || !(*args)[1].isList() ||
            (*args)[1].items->size() < 3) {
            return std::nullopt;
        }
        const StepList& c = *(*args)[1].items;
        const Vec3 v{c[0].num, c[1].num, c[2].num};
        if (!std::isfinite(v.x) || !std::isfinite(v.y) || !std::isfinite(v.z)) return std::nullopt;
        return v;
    };
    const StepInstance* inst = parser.find(id);
    const StepList* args = inst ? inst->leaf("AXIS2_PLACEMENT_3D") : nullptr;
    if (args == nullptr || args->size() < 2) return std::nullopt;
    const auto origin = triple((*args)[1], "CARTESIAN_POINT");
    if (!origin) return std::nullopt;
    Vec3 z{0, 0, 1};
    Vec3 x{1, 0, 0};
    if (args->size() > 2) {
        if (const auto d = triple((*args)[2], "DIRECTION"); d && d->length() > 1e-12) {
            z = d->normalized();
        }
    }
    if (args->size() > 3) {
        if (const auto d = triple((*args)[3], "DIRECTION")) x = *d;
    }
    x = x - z * x.dot(z);
    if (x.length() < 1e-9) {
        const Vec3 seed = std::abs(z.x) < 0.9 ? Vec3{1, 0, 0} : Vec3{0, 1, 0};
        x = seed - z * seed.dot(z);
    }
    x = x.normalized();
    const Vec3 y = z.cross(x);
    Mat4 m = Mat4::identity();
    for (int row = 0; row < 3; ++row) {
        m.at(row, 0) = row == 0 ? x.x : row == 1 ? x.y : x.z;
        m.at(row, 1) = row == 0 ? y.x : row == 1 ? y.y : y.z;
        m.at(row, 2) = row == 0 ? z.x : row == 1 ? z.y : z.z;
    }
    m.at(0, 3) = origin->x * mm;
    m.at(1, 3) = origin->y * mm;
    m.at(2, 3) = origin->z * mm;
    return m;
}

/// The inverse of a rotation and a move: exact, as a general inverse is not.
Mat4 rigidInverse(const Mat4& m) {
    Mat4 inv = Mat4::identity();
    for (int r = 0; r < 3; ++r) {
        for (int c = 0; c < 3; ++c) inv.at(r, c) = m.at(c, r);
    }
    for (int r = 0; r < 3; ++r) {
        inv.at(r, 3) =
            -(inv.at(r, 0) * m.at(0, 3) + inv.at(r, 1) * m.at(1, 3) + inv.at(r, 2) * m.at(2, 3));
    }
    return inv;
}

bool isIdentity(const Mat4& m) {
    for (int r = 0; r < 4; ++r) {
        for (int c = 0; c < 4; ++c) {
            if (m.at(r, c) != (r == c ? 1.0 : 0.0)) return false;
        }
    }
    return true;
}

/// The solids (MANIFOLD_SOLID_BREPs, and BREP_WITH_VOIDS) that make one
/// topo::Solid: those in one shape representation, with its context (which
/// holds its units).
struct SolidGroup {
    std::vector<int> msbs;
    /// A surface model's (SHELL_BASED_SURFACE_MODEL): msbs are its shells,
    /// each a solid if it closes.
    bool surfaces = false;
    int context = 0;
    /// The ADVANCED_BREP_SHAPE_REPRESENTATION holding them; 0 for one
    /// outside any.
    int rep = 0;
};

/// A file's solids, grouped into the shapes they make.
std::vector<SolidGroup> solidGroups(const StepParser& parser) {
    std::vector<SolidGroup> groups;
    std::unordered_set<int> grouped;
    // The solids in an ADVANCED_BREP representation, then those a plain
    // SHAPE_REPRESENTATION holds itself, as Open CASCADE writes a compound
    // of solids (KiCad's boards): read as no product's, each came in once,
    // where drawn, however often the assembly used the product.
    for (const char* type : {"ADVANCED_BREP_SHAPE_REPRESENTATION", "SHAPE_REPRESENTATION"}) {
        for (int repId : parser.allOfType(type)) {
            const StepInstance* rep = parser.find(repId);
            const StepList* args = rep ? rep->leaf(type) : nullptr;
            if (args == nullptr || args->size() < 2 || !(*args)[1].isList()) continue;
            SolidGroup group;
            group.rep = repId;
            if (args->size() > 2 && (*args)[2].isRef()) group.context = (*args)[2].ref;
            for (const StepValue& item : *(*args)[1].items) {
                if (!item.isRef() || grouped.count(item.ref) != 0) continue;
                const StepInstance* it = parser.find(item.ref);
                if (it != nullptr &&
                    (it->hasType("MANIFOLD_SOLID_BREP") || it->hasType("BREP_WITH_VOIDS"))) {
                    group.msbs.push_back(item.ref);
                    grouped.insert(item.ref);
                }
            }
            if (!group.msbs.empty()) groups.push_back(std::move(group));
        }
    }
    // Surface models: their shells, which another system may write for a
    // solid (Creo's closed OPEN_SHELLs), one group a representation.
    std::unordered_set<int> modelled;
    const auto shellsOfModel = [&parser](int model, std::vector<int>& shells) {
        const StepList* args = parser.find(model)->leaf("SHELL_BASED_SURFACE_MODEL");
        if (args == nullptr || args->size() < 2 || !(*args)[1].isList()) return;
        for (const StepValue& shell : *(*args)[1].items) {
            const StepInstance* it = shell.isRef() ? parser.find(shell.ref) : nullptr;
            if (it != nullptr && (it->hasType("OPEN_SHELL") || it->hasType("CLOSED_SHELL"))) {
                shells.push_back(shell.ref);
            }
        }
    };
    for (const char* type : {"MANIFOLD_SURFACE_SHAPE_REPRESENTATION", "SHAPE_REPRESENTATION"}) {
        for (int repId : parser.allOfType(type)) {
            const StepList* args = parser.find(repId)->leaf(type);
            if (args == nullptr || args->size() < 2 || !(*args)[1].isList()) continue;
            SolidGroup group;
            group.rep = repId;
            group.surfaces = true;
            if (args->size() > 2 && (*args)[2].isRef()) group.context = (*args)[2].ref;
            for (const StepValue& item : *(*args)[1].items) {
                const StepInstance* it = item.isRef() ? parser.find(item.ref) : nullptr;
                if (it == nullptr || !it->hasType("SHELL_BASED_SURFACE_MODEL") ||
                    !modelled.insert(item.ref).second) {
                    continue;
                }
                shellsOfModel(item.ref, group.msbs);
            }
            if (!group.msbs.empty()) groups.push_back(std::move(group));
        }
    }

    // MSBs outside any representation (minimal files) import one solid each,
    // in the file's first context with units, if it has one.
    int fileContext = 0;
    for (int id : parser.allOfType("GLOBAL_UNIT_ASSIGNED_CONTEXT")) {
        fileContext = id;
        break;
    }
    for (const char* type : {"MANIFOLD_SOLID_BREP", "BREP_WITH_VOIDS"}) {
        for (int id : parser.allOfType(type)) {
            if (grouped.insert(id).second) groups.push_back({{id}, false, fileContext, 0});
        }
    }
    for (int id : parser.allOfType("SHELL_BASED_SURFACE_MODEL")) {
        if (!modelled.insert(id).second) continue;
        SolidGroup group;
        group.surfaces = true;
        group.context = fileContext;
        shellsOfModel(id, group.msbs);
        if (!group.msbs.empty()) groups.push_back(std::move(group));
    }
    return groups;
}

/// A file's product structure: its products with shapes, and every
/// placement of each, found by walking its assemblies from the top.
struct ProductStructure {
    struct Product {
        std::string name;
        std::vector<std::size_t> groups;  ///< its solids: indices into the groups
    };
    std::vector<Product> products;
    /// Each product's placements; StepOccurrence::part indexes products.
    std::vector<StepOccurrence> placed;
    /// Whether any product is placed by an assembly.
    bool structured = false;
    /// What could only be followed in part, and what was left out, said in
    /// a report.
    std::vector<std::string> notes;
    std::vector<std::string> leftOut;
    /// A placement's length unit is not named, or cannot be read, and none
    /// was chosen: the file is not read.
    bool unitUnknown = false;
};

/// At most this many solids placed, and this many assemblies walked into: a
/// file whose assemblies multiply beyond them (a part used ten times in each
/// of ten nested levels, or a part of many solids used many times) is cut
/// short.
constexpr std::size_t kMaxPlacements = 20000;
constexpr std::size_t kMaxVisits = 1000000;
/// Assemblies nested deeper than this are taken for a loop.
constexpr int kMaxDepth = 64;

ProductStructure readStructure(const StepParser& parser, const std::vector<SolidGroup>& groups,
                               const StepReadOptions& options) {
    ProductStructure structure;
    // A representation's groups: its solids', and its surface model's (a
    // plain SHAPE_REPRESENTATION may hold both).
    std::unordered_map<int, std::vector<std::size_t>> groupsOfRep;
    for (std::size_t g = 0; g < groups.size(); ++g) {
        if (groups[g].rep != 0) groupsOfRep[groups[g].rep].push_back(g);
    }

    // Representations related without a transformation are one shape
    // (a part's SHAPE_REPRESENTATION and the ADVANCED_BREP one holding its
    // solids); those with one place one in the other.
    std::unordered_map<int, std::vector<int>> joined;
    std::map<int, Relationship> placing;  // by the relationship's id
    std::set<int> relationships;
    for (const char* type : {"REPRESENTATION_RELATIONSHIP", "SHAPE_REPRESENTATION_RELATIONSHIP",
                             "REPRESENTATION_RELATIONSHIP_WITH_TRANSFORMATION"}) {
        for (int id : parser.allOfType(type)) relationships.insert(id);
    }
    for (int id : relationships) {
        const auto r = relationshipOf(*parser.find(id));
        if (!r) continue;
        if (r->transformation != 0) {
            placing.emplace(id, *r);
        } else {
            joined[r->rep1].push_back(r->rep2);
            joined[r->rep2].push_back(r->rep1);
        }
    }
    const auto shapeOf = [&](const std::vector<int>& reps) {
        std::set<int> seen(reps.begin(), reps.end());
        std::vector<int> stack(reps.begin(), reps.end());
        while (!stack.empty()) {
            const int rep = stack.back();
            stack.pop_back();
            const auto found = joined.find(rep);
            if (found == joined.end()) continue;
            for (int other : found->second) {
                if (seen.insert(other).second) stack.push_back(other);
            }
        }
        return seen;
    };

    // What each PRODUCT_DEFINITION_SHAPE defines: a product definition, or
    // an occurrence of one in an assembly.
    std::unordered_map<int, int> definedBy;
    for (int id : parser.allOfType("PRODUCT_DEFINITION_SHAPE")) {
        const StepList* args = parser.find(id)->leaf("PRODUCT_DEFINITION_SHAPE");
        if (args != nullptr && args->size() >= 3 && (*args)[2].isRef()) {
            definedBy.emplace(id, (*args)[2].ref);
        }
    }
    // Each product definition's representations.
    std::unordered_map<int, std::vector<int>> repsOf;
    for (int id : parser.allOfType("SHAPE_DEFINITION_REPRESENTATION")) {
        const StepList* args = parser.find(id)->leaf("SHAPE_DEFINITION_REPRESENTATION");
        if (args == nullptr || args->size() < 2 || !(*args)[0].isRef() || !(*args)[1].isRef()) {
            continue;
        }
        const auto definition = definedBy.find((*args)[0].ref);
        if (definition != definedBy.end()) repsOf[definition->second].push_back((*args)[1].ref);
    }

    // The product definitions, in the file's order, and their names.
    std::set<int> definitions;
    for (const char* type :
         {"PRODUCT_DEFINITION", "PRODUCT_DEFINITION_WITH_ASSOCIATED_DOCUMENTS"}) {
        for (int id : parser.allOfType(type)) definitions.insert(id);
    }
    const auto nameOf = [&parser](int definition) {
        const StepInstance* pd = parser.find(definition);
        const StepList* args = pd ? pd->leaf("PRODUCT_DEFINITION") : nullptr;
        if (args == nullptr && pd != nullptr) {
            args = pd->leaf("PRODUCT_DEFINITION_WITH_ASSOCIATED_DOCUMENTS");
        }
        const StepInstance* formation = args != nullptr && args->size() >= 3 && (*args)[2].isRef()
                                            ? parser.find((*args)[2].ref)
                                            : nullptr;
        const StepList* fArgs =
            formation ? formation->leaf("PRODUCT_DEFINITION_FORMATION") : nullptr;
        if (fArgs == nullptr && formation != nullptr) {
            fArgs = formation->leaf("PRODUCT_DEFINITION_FORMATION_WITH_SPECIFIED_SOURCE");
        }
        const StepInstance* product = fArgs != nullptr && fArgs->size() >= 3 && (*fArgs)[2].isRef()
                                          ? parser.find((*fArgs)[2].ref)
                                          : nullptr;
        const StepList* pArgs = product ? product->leaf("PRODUCT") : nullptr;
        if (pArgs == nullptr || pArgs->size() < 2) return std::string();
        const std::string name = textOf((*pArgs)[1]);
        return blank(name) ? textOf((*pArgs)[0]) : name;
    };

    // Products: the definitions with solids. Which rep of theirs each is,
    // to tell which side of a placement a part is on.
    std::unordered_map<int, std::size_t> productOf;
    std::unordered_map<int, std::set<int>> shapeReps;
    std::vector<bool> inProduct(groups.size(), false);
    for (int definition : definitions) {
        const auto reps = repsOf.find(definition);
        if (reps == repsOf.end()) continue;
        std::set<int> shape = shapeOf(reps->second);
        ProductStructure::Product product;
        for (int rep : shape) {
            const auto found = groupsOfRep.find(rep);
            if (found != groupsOfRep.end()) {
                product.groups.insert(product.groups.end(), found->second.begin(),
                                      found->second.end());
            }
        }
        std::sort(product.groups.begin(), product.groups.end());
        shapeReps.emplace(definition, std::move(shape));
        if (product.groups.empty()) continue;
        for (std::size_t g : product.groups) inProduct[g] = true;
        product.name = nameOf(definition);
        productOf.emplace(definition, structure.products.size());
        structure.products.push_back(std::move(product));
    }

    // Each use of a product in an assembly, and the relationship placing it.
    struct Use {
        int occurrence = 0;
        int parent = 0;
        int child = 0;
        /// What it calls this use of its part: its name, unless that is
        /// blank or a writer's stock text; then the part's.
        std::string name;
    };
    std::map<int, std::vector<Use>> usesIn;  // by the assembly's definition
    std::set<int> used;
    for (int id : parser.allOfType("NEXT_ASSEMBLY_USAGE_OCCURRENCE")) {
        const StepList* args = parser.find(id)->leaf("NEXT_ASSEMBLY_USAGE_OCCURRENCE");
        if (args == nullptr || args->size() < 5 || !(*args)[3].isRef() || !(*args)[4].isRef()) {
            continue;
        }
        std::string name = textOf((*args)[1]);
        if (blank(name) || name == "Next assembly relationship") name = nameOf((*args)[4].ref);
        usesIn[(*args)[3].ref].push_back({id, (*args)[3].ref, (*args)[4].ref, std::move(name)});
        used.insert((*args)[4].ref);
    }
    std::unordered_map<int, int> placedBy;  // occurrence → its relationship
    for (int id : parser.allOfType("CONTEXT_DEPENDENT_SHAPE_REPRESENTATION")) {
        const StepList* args = parser.find(id)->leaf("CONTEXT_DEPENDENT_SHAPE_REPRESENTATION");
        if (args == nullptr || args->size() < 2 || !(*args)[0].isRef() || !(*args)[1].isRef()) {
            continue;
        }
        const auto occurrence = definedBy.find((*args)[1].ref);
        if (occurrence != definedBy.end()) placedBy.emplace(occurrence->second, (*args)[0].ref);
    }
    std::unordered_map<int, double> mmOfRep;
    const auto mmOf = [&](int rep) {
        const auto found = mmOfRep.find(rep);
        if (found != mmOfRep.end()) return found->second;
        const int context = representationContext(parser, rep);
        double mm = context != 0 ? contextMillimetres(parser, context, nullptr) : 0.0;
        if (!(mm > 0.0)) {
            // Not named, or not readable: the unit chosen, else unknown.
            if (options.unknownLengthUnit) {
                mm = math::millimetresPer(*options.unknownLengthUnit);
            } else {
                structure.unitUnknown = true;
                mm = 1.0;
            }
        }
        mmOfRep.emplace(rep, mm);
        return mm;
    };
    // The use's part's coordinates into its assembly's: the transformation
    // takes rep_1's into rep_2's, the part's into the assembly's as written
    // by most, inverted when the part is rep_2.
    const auto placement = [&](const Use& use) -> std::optional<Mat4> {
        const auto by = placedBy.find(use.occurrence);
        if (by == placedBy.end()) return std::nullopt;
        const auto r = placing.find(by->second);
        if (r == placing.end()) return std::nullopt;
        const StepInstance* idt = parser.find(r->second.transformation);
        const StepList* args = idt ? idt->leaf("ITEM_DEFINED_TRANSFORMATION") : nullptr;
        if (args == nullptr || args->size() < 4 || !(*args)[2].isRef() || !(*args)[3].isRef()) {
            return std::nullopt;
        }
        const auto from = axisMatrix(parser, (*args)[2].ref, mmOf(r->second.rep1));
        const auto to = axisMatrix(parser, (*args)[3].ref, mmOf(r->second.rep2));
        if (!from || !to) return std::nullopt;
        const Mat4 t = *to * rigidInverse(*from);
        const auto part = shapeReps.find(use.child);
        const bool partSecond = part != shapeReps.end() &&
                                part->second.count(r->second.rep2) != 0 &&
                                part->second.count(r->second.rep1) == 0;
        return partSecond ? rigidInverse(t) : t;
    };

    // Uses a CONTEXT_DEPENDENT_SHAPE_REPRESENTATION does not place, placed by
    // a MAPPED_ITEM instead (Unigraphics'): the assembly's representation
    // holds a MAPPED_ITEM(name, REPRESENTATION_MAP(origin, the part's
    // representation), target), the part's origin taken to the target. Each
    // use of a part in an assembly takes the next such item mapping the
    // part's shape, in the file's order.
    std::unordered_map<int, Mat4> mappedPlacement;  // by the use's occurrence
    {
        struct Mapped {
            int container = 0;  ///< the representation holding the item
            int mapped = 0;     ///< the part's representation it places
            int origin = 0;
            int target = 0;
        };
        std::vector<Mapped> items;
        std::unordered_map<int, std::vector<int>> containersOf;  // item → its representations
        for (const char* type : {"SHAPE_REPRESENTATION", "ADVANCED_BREP_SHAPE_REPRESENTATION"}) {
            for (int repId : parser.allOfType(type)) {
                const StepList* args = parser.find(repId)->leaf(type);
                if (args == nullptr || args->size() < 2 || !(*args)[1].isList()) continue;
                for (const StepValue& item : *(*args)[1].items) {
                    if (item.isRef()) containersOf[item.ref].push_back(repId);
                }
            }
        }
        for (int id : parser.allOfType("MAPPED_ITEM")) {
            const StepList* args = parser.find(id)->leaf("MAPPED_ITEM");
            if (args == nullptr || args->size() < 3 || !(*args)[1].isRef() || !(*args)[2].isRef()) {
                continue;
            }
            const StepInstance* map = parser.find((*args)[1].ref);
            const StepList* mapArgs = map ? map->leaf("REPRESENTATION_MAP") : nullptr;
            if (mapArgs == nullptr || mapArgs->size() < 2 || !(*mapArgs)[0].isRef() ||
                !(*mapArgs)[1].isRef()) {
                continue;
            }
            const auto containers = containersOf.find(id);
            if (containers == containersOf.end()) continue;
            for (int container : containers->second) {
                items.push_back({container, (*mapArgs)[1].ref, (*mapArgs)[0].ref, (*args)[2].ref});
            }
        }
        std::vector<bool> taken(items.size(), false);
        for (const auto& [assembly, uses] : usesIn) {
            const auto whole = shapeReps.find(assembly);
            if (whole == shapeReps.end()) continue;
            for (const Use& use : uses) {
                if (placedBy.count(use.occurrence) != 0) continue;
                const auto part = shapeReps.find(use.child);
                if (part == shapeReps.end()) continue;
                for (std::size_t k = 0; k < items.size(); ++k) {
                    const Mapped& item = items[k];
                    if (taken[k] || whole->second.count(item.container) == 0 ||
                        part->second.count(item.mapped) == 0) {
                        continue;
                    }
                    taken[k] = true;
                    const auto from = axisMatrix(parser, item.origin, mmOf(item.mapped));
                    const auto to = axisMatrix(parser, item.target, mmOf(item.container));
                    if (from && to)
                        mappedPlacement.emplace(use.occurrence, *to * rigidInverse(*from));
                    break;
                }
            }
        }
    }

    // From each top assembly (a definition no assembly uses) down.
    std::set<int> onPath;
    std::vector<bool> placedProduct(structure.products.size(), false);
    bool cut = false;
    std::size_t visits = 0;
    std::size_t solids = 0;  // placed so far, counting each of a part's
    std::set<int> unplaced;  // uses with no placement that can be read
    std::set<int> loops;     // uses of an assembly within itself
    // Each placement is named by the uses down to it ("Sub:1/Bolt:2"); a
    // top product by its own name.
    std::function<void(int, const Mat4&, int, const std::string&)> walk =
        [&](int definition, const Mat4& world, int depth, const std::string& path) {
            if (solids >= kMaxPlacements || ++visits > kMaxVisits) {
                cut = true;
                return;
            }
            const auto product = productOf.find(definition);
            if (product != productOf.end()) {
                solids += structure.products[product->second].groups.size();
                structure.placed.push_back(
                    {product->second, depth > 0 ? path : structure.products[product->second].name,
                     world});
                placedProduct[product->second] = true;
                if (depth > 0) structure.structured = true;
            }
            const auto uses = usesIn.find(definition);
            if (uses == usesIn.end()) return;
            onPath.insert(definition);
            for (const Use& use : uses->second) {
                if (onPath.count(use.child) != 0 || depth >= kMaxDepth) {
                    loops.insert(use.occurrence);
                    continue;
                }
                auto t = placement(use);
                if (!t) {
                    const auto mapped = mappedPlacement.find(use.occurrence);
                    if (mapped != mappedPlacement.end()) t = mapped->second;
                }
                if (!t) {
                    unplaced.insert(use.occurrence);
                    t = Mat4::identity();
                }
                walk(use.child, world * *t, depth + 1,
                     path.empty() ? use.name : path + "/" + use.name);
            }
            onPath.erase(definition);
        };
    for (int definition : definitions) {
        if (used.count(definition) == 0) walk(definition, Mat4::identity(), 0, std::string());
    }
    // Which uses, by their ids: the first few.
    const auto listed = [](const std::set<int>& ids) {
        std::string text;
        std::size_t n = 0;
        for (int id : ids) {
            if (n++ == 5) return text + ", ...";
            text += (text.empty() ? "#" : ", #") + std::to_string(id);
        }
        return text;
    };
    if (!unplaced.empty()) {
        structure.notes.push_back(
            std::to_string(unplaced.size()) +
            (unplaced.size() == 1 ? " use of a part has" : " uses of parts have") +
            " no placement that can be read, and " + (unplaced.size() == 1 ? "is" : "are") +
            " placed where drawn (" + listed(unplaced) + ")");
    }
    if (!loops.empty()) {
        structure.notes.push_back("an assembly that contains itself is followed once (" +
                                  listed(loops) + ")");
    }
    if (cut) {
        structure.leftOut.push_back("the assembly places its parts more times than can be read (" +
                                    std::to_string(structure.placed.size()) +
                                    " placements read): the rest are left out");
    }
    // A product no walk reached (only under a loop, or under a missing
    // assembly), where drawn; and solids of no product, each its own.
    for (std::size_t p = 0; p < structure.products.size(); ++p) {
        if (!placedProduct[p]) {
            structure.placed.push_back({p, structure.products[p].name, Mat4::identity()});
        }
    }
    for (std::size_t g = 0; g < groups.size(); ++g) {
        if (inProduct[g]) continue;
        const StepInstance* msb = parser.find(groups[g].msbs.front());
        const StepList* args = msb ? msb->leaf("MANIFOLD_SOLID_BREP") : nullptr;
        if (args == nullptr && msb != nullptr) args = msb->leaf("BREP_WITH_VOIDS");
        std::string name = args != nullptr && !args->empty() ? textOf((*args)[0]) : std::string();
        if (blank(name)) name = "solid " + std::to_string(g + 1);
        structure.placed.push_back({structure.products.size(), name, Mat4::identity()});
        structure.products.push_back({name, {g}});
    }
    return structure;
}

// ===========================================================================
// Reading and writing, shared
// ===========================================================================

/// Parse @p text into @p parser; false, with g_lastError said, when it
/// cannot be, or once @p cancelled is set.
bool parseText(StepParser& parser, const std::atomic<bool>* cancelled) {
    std::string error;
    bool parsed = false;
    try {
        parsed = parser.parse(error, cancelled);
    } catch (const std::exception& e) {
        error = e.what();
    }
    if (cancelled != nullptr && cancelled->load(std::memory_order_relaxed)) {
        g_lastError = kCancelled;
        return false;
    }
    if (!parsed) g_lastError = "STEP parse error: " + error;
    return parsed;
}

// ===========================================================================
// The exact form (exact kernel, stage 2)
// ===========================================================================

/// A pcurve moved a whole period along its surface: a seam's other side.
class ShiftedCurve2D final : public geo::Curve2D {
public:
    ShiftedCurve2D(std::shared_ptr<const geo::Curve2D> base, const math::Vec2& shift)
        : m_base(std::move(base)), m_shift(shift) {}

    math::Vec2 evaluate(double t) const override { return m_base->evaluate(t) + m_shift; }
    math::Vec2 derivative(double t, int order) const override {
        return m_base->derivative(t, order);
    }
    double tMin() const override { return m_base->tMin(); }
    double tMax() const override { return m_base->tMax(); }
    bool isClosed() const override { return m_base->isClosed(); }

private:
    std::shared_ptr<const geo::Curve2D> m_base;
    math::Vec2 m_shift;
};

/// Where in its file @p id names, for a report: "face #27", "the seam of face #27" (one the
/// reader made) or "edge #18". Empty for another name.
std::string fileEntity(const topo::TopologyID& id) {
    const std::string& tag = id.tag();
    const auto solid = tag.find("solid:");
    const auto slash = solid == std::string::npos ? std::string::npos : tag.find('/', solid);
    if (slash == std::string::npos) return {};
    const std::string tail = tag.substr(slash + 1);
    std::size_t end = 2;
    while (end < tail.size() && std::isdigit(static_cast<unsigned char>(tail[end])) != 0) ++end;
    const std::string number = tail.substr(std::min<std::size_t>(2, tail.size()), end - 2);
    if (tail.rfind("f:", 0) == 0) {
        return (tail.find("/seam") != std::string::npos ? "the seam of face #" : "face #") + number;
    }
    if (tail.rfind("e:", 0) == 0) return "edge #" + number;
    return {};
}

/// " (face #27, edge #18)": where in its file an exact problem lies; empty when not known.
std::string whereIn(const topo::Face* face, const topo::Edge* edge) {
    std::string where = face != nullptr ? fileEntity(face->topoId) : std::string();
    const std::string onEdge = edge != nullptr ? fileEntity(edge->topoId) : std::string();
    if (!onEdge.empty() && onEdge != where) where += (where.empty() ? "" : ", ") + onEdge;
    return where.empty() ? where : " (" + where + ")";
}

/// The half-edges of @p wire, in order; empty when its cycle breaks within @p limit.
std::vector<topo::HalfEdge*> ringOf(const topo::Wire& wire, std::size_t limit) {
    std::vector<topo::HalfEdge*> ring;
    topo::HalfEdge* h = wire.halfEdge;
    while (h != nullptr && ring.size() <= limit) {
        ring.push_back(h);
        h = h->next;
        if (h == wire.halfEdge) return ring;
    }
    return {};
}

/// What the exact validator says of a loop that closes only after winding round its surface.
constexpr const char* kWindsRound = "loop closes only after winding round the surface";

/// The signed area in (u, v) of the loop @p ring of a face on @p s, unwrapped as the exact
/// validator unwraps it (with @p sense, its runs round a pole laid as that sense says, where the
/// loop leaves them free); nullopt where it cannot be (a loop that winds round the surface). An
/// area within rounding of none (the box round the loop's samples times 1e-12) is 0: it says
/// nothing of which way the loop runs.
std::optional<double> loopArea(const std::vector<topo::HalfEdge*>& ring, const geo::Surface& s,
                               const math::ModelTolerance& tolerance,
                               topo::LoopSense sense = topo::LoopSense::Either) {
    const std::vector<const topo::HalfEdge*> walk(ring.begin(), ring.end());
    std::vector<double> allowances;
    allowances.reserve(walk.size());
    for (const topo::HalfEdge* h : walk) allowances.push_back(topo::joinAllowance(*h, tolerance));
    topo::LoopPolygon poly;
    if (walk.empty() || !topo::unwrapLoop(walk, topo::kLoopSamples, s, topo::uPeriod(s),
                                          topo::vPeriod(s), allowances, poly, sense)) {
        return std::nullopt;
    }
    math::Vec2 lo = poly.front().a;
    math::Vec2 hi = lo;
    for (const topo::LoopSegment& seg : poly) {
        lo = {std::min(lo.x, seg.a.x), std::min(lo.y, seg.a.y)};
        hi = {std::max(hi.x, seg.a.x), std::max(hi.y, seg.a.y)};
    }
    const double area = topo::signedArea(poly);
    return std::abs(area) <= 1e-12 * (hi.x - lo.x) * (hi.y - lo.y) ? 0.0 : area;
}

/// Points along the edge of @p s's domain where u (@p alongV false) or v (true) is @p at: 17 of
/// them, both ends included.
std::vector<Vec3> domainEdge(const geo::NurbsSurface& s, bool alongV, double at) {
    constexpr int kSteps = 16;
    std::vector<Vec3> points;
    points.reserve(kSteps + 1);
    for (int k = 0; k <= kSteps; ++k) {
        const double f = static_cast<double>(k) / kSteps;
        points.push_back(alongV ? s.evaluate(s.uMin() + (s.uMax() - s.uMin()) * f, at)
                                : s.evaluate(at, s.vMin() + (s.vMax() - s.vMin()) * f));
    }
    return points;
}

/// Whether a loop on @p s determines the region it bounds: on a plane, a cylinder, a cone (a
/// swept surface's B-spline among them) and a B-spline neither closed both ways nor coming to a
/// pole, the region off the loop's disc reaches an edge of the surface no edge of the face
/// bounds, so the face is the disc. On a sphere, a torus, or a B-spline closed both ways or with
/// a pole, a loop bounds two regions, and its (u, v) area always picks the disc. A B-spline is
/// judged within @p near (the solid's limit, a thousandth of its size), not NurbsSurface's own
/// closure: an edge of its domain whose points spread no further is a pole, and a direction
/// whose two edges come no further apart is closed, so a fitted surface's pole opened a hair, or
/// a torus-like surface closed only nearly, is not taken for one the loops decide. Erring that
/// way only leaves a solid out.
bool loopsDetermineRegion(const geo::Surface& s, double near) {
    switch (s.kind()) {
        case geo::SurfaceKind::Plane:
        case geo::SurfaceKind::Cylinder:
        case geo::SurfaceKind::Cone:
            return true;
        case geo::SurfaceKind::Sphere:
        case geo::SurfaceKind::Torus:
            return false;
        case geo::SurfaceKind::Nurbs: {
            const auto* nurbs = dynamic_cast<const geo::NurbsSurface*>(&s);
            if (nurbs == nullptr) return false;
            const auto spread = [](const std::vector<Vec3>& points) {
                math::BoundingBox box;
                for (const Vec3& p : points) box.expand(p);
                return (box.max() - box.min()).length();
            };
            const auto apart = [](const std::vector<Vec3>& a, const std::vector<Vec3>& b) {
                double most = 0.0;
                for (std::size_t k = 0; k < a.size() && k < b.size(); ++k) {
                    most = std::max(most, (a[k] - b[k]).length());
                }
                return most;
            };
            const auto uLow = domainEdge(*nurbs, false, nurbs->uMin());
            const auto uHigh = domainEdge(*nurbs, false, nurbs->uMax());
            const auto vLow = domainEdge(*nurbs, true, nurbs->vMin());
            const auto vHigh = domainEdge(*nurbs, true, nurbs->vMax());
            const bool pole = spread(uLow) <= near || spread(uHigh) <= near ||
                              spread(vLow) <= near || spread(vHigh) <= near;
            const bool closedBothWays = apart(uLow, uHigh) <= near && apart(vLow, vHigh) <= near;
            return !pole && !closedBothWays;
        }
    }
    return false;
}

/// Gives @p face its sense, or says why it has none this stage represents.
///
/// Where its loops determine its region (loopsDetermineRegion within @p near), or its outer loop
/// uses a seam of its own (both sides of one edge, a period apart, so its (u, v) region is
/// fixed), the face faces the way its outer loop runs in (u, v), counter-clockwise about its
/// outward normal: the loops are the solid's topology, and same_sense only repeats it (Horizon's
/// own writer gave same_sense .T. whichever way a face's surface faced). Elsewhere the face
/// keeps its file's sense (same_sense, with its shell's and bounds' flags); where its loop's
/// unwrapped area says the other way, either the face is the surface outside its loop (a ball's
/// skin round a flat), which holds a pole or wraps the surface and this stage cannot represent,
/// or its file's same_sense is wrong: which, it cannot tell, so it says both. A loop whose area
/// says nothing (a seam joined only at poles, laid out next) leaves the face as its file says.
/// A loop that runs round a pole between two pcurves without a seam (a lune between two
/// meridians) does not fix its region: which way round the pole it runs is the file's sense's,
/// not the window its next pcurve is written in, so its area is taken again with its runs laid
/// as that sense says (topo::unwrapLoop) before the face is refused.
///
/// An outer loop with no seam that closes only after winding round the surface (a band) says so
/// in the validator's words, kWindsRound, and leaves the face as it was: finishExact gives it its
/// seam and asks again. Said of a face that still has none, it is placed as the validator places
/// it (at the face and its outer loop's first edge).
std::string faceSense(topo::Face& face, const math::ModelTolerance& tolerance, std::size_t limit,
                      double near) {
    if (face.outerLoop == nullptr) return {};
    const std::vector<topo::HalfEdge*> ring = ringOf(*face.outerLoop, limit);
    const bool seamed = std::any_of(ring.begin(), ring.end(), [&face](const topo::HalfEdge* h) {
        return h->twin != nullptr && h->twin->face == &face;
    });
    const auto area = loopArea(ring, *face.exactSurface, tolerance);
    if (!area) {
        // A seamed loop that does not close yet may once its seam is laid out a period over
        // (both sides of a seam share one window, and a join at a pole shifts nothing): that is
        // layOutSeams's to try, and the validator's to judge. Only a loop with no seam is a band.
        return seamed || ring.empty() ? std::string() : std::string(kWindsRound);
    }
    if (!std::isfinite(*area) || *area == 0.0) return {};
    const bool loopsSayReversed = *area < 0.0;
    if (seamed || loopsDetermineRegion(*face.exactSurface, near)) {
        face.reversed = loopsSayReversed;
        return {};
    }
    if (loopsSayReversed == face.reversed) return {};
    const auto laid =
        loopArea(ring, *face.exactSurface, tolerance,
                 face.reversed ? topo::LoopSense::Clockwise : topo::LoopSense::CounterClockwise);
    if (laid && std::isfinite(*laid) && *laid != 0.0 && (*laid < 0.0) == face.reversed) return {};
    return "its loops run against its same_sense: either the face is the surface outside its "
           "loop (it holds a pole or wraps the surface), which this stage cannot represent, or "
           "the file's same_sense is wrong";
}

/// How many layouts layOutSeams tries for one face, at most. A well-formed face has a seam along
/// each periodic direction at most (a torus's two), each of two sides, each tried at 4 shifts:
/// 16 tries; only a malformed face (many seams, none that orients it) reaches the cap.
constexpr int kSeamTries = 16;

/// Lays the seams of @p face out so that each of its loops runs the way the face needs in
/// (u, v): counter-clockwise for its outer loop, unless the face is reversed. A seam's two
/// sides lie on one line of the surface; where the loop joins them only at poles (a sphere's
/// meridian from pole to pole and back, as OpenCASCADE writes a ball), the unwrapping cannot
/// tell them a period apart, and one side is moved the period that runs the loop the right way.
/// Elsewhere the unwrapping shifts whole periods itself, and the layout is left as it is. Only
/// half-edges whose twin bounds the same face are tried, at most kSeamTries layouts for the face
/// (each an unwrap of its loop): past that, or once @p cancelled is set, it stops and says why;
/// otherwise empty.
std::string layOutSeams(topo::Face& face, const math::ModelTolerance& tolerance, std::size_t limit,
                        const std::atomic<bool>* cancelled) {
    const geo::Surface& s = *face.exactSurface;
    const double pu = topo::uPeriod(s);
    const double pv = topo::vPeriod(s);
    if (pu == 0.0 && pv == 0.0) return {};
    std::vector<topo::Wire*> loops{face.outerLoop};
    loops.insert(loops.end(), face.innerLoops.begin(), face.innerLoops.end());
    int tries = 0;
    for (topo::Wire* wire : loops) {
        if (wire == nullptr) continue;
        const std::vector<topo::HalfEdge*> ring = ringOf(*wire, limit);
        const bool seamed = std::any_of(ring.begin(), ring.end(), [&face](const topo::HalfEdge* h) {
            return h->twin != nullptr && h->twin->face == &face;
        });
        if (!seamed) continue;
        const bool wantPositive = (wire == face.outerLoop) != face.reversed;
        const auto runsRight = [&] {
            const auto area = loopArea(ring, s, tolerance);
            return area && *area != 0.0 && (*area > 0.0) == wantPositive;
        };
        if (runsRight()) continue;
        const math::Vec2 shifts[] = {{pu, 0.0}, {-pu, 0.0}, {0.0, pv}, {0.0, -pv}};
        bool laid = false;
        for (topo::HalfEdge* h : ring) {
            if (laid || h->twin == nullptr || h->twin->face != &face) continue;
            const std::shared_ptr<const geo::Curve2D> original = h->pcurve;
            for (const math::Vec2& shift : shifts) {
                if (shift.x == 0.0 && shift.y == 0.0) continue;
                if (cancelled != nullptr && cancelled->load(std::memory_order_relaxed)) {
                    h->pcurve = original;
                    return kCancelled;
                }
                if (tries == kSeamTries) {
                    h->pcurve = original;
                    return "seam layout gave up after " + std::to_string(kSeamTries) + " tries";
                }
                ++tries;
                h->pcurve = std::make_shared<ShiftedCurve2D>(original, shift);
                laid = runsRight();
                if (laid) break;
            }
            if (!laid) h->pcurve = original;
        }
    }
    return {};
}

/// What making a solid's exact form came to: why it could not be made, or how exact it is.
struct ExactRead {
    std::string problem;      ///< empty when the solid is exact
    double resolution = 0.0;  ///< the model resolution its size gives it
    double largestEdgeTolerance = 0.0;
};

/// A length for a report, to three figures.
std::string threeFigures(double x) {
    std::ostringstream out;
    out << std::setprecision(3) << x;
    return out.str();
}

/// Finishes the exact form of @p solid, as read (each face's exact surface, each edge's exact
/// curve over its range): builds each half-edge's pcurve with geo::pcurveOn, grows each edge's
/// and vertex's tolerance to cover how far the file has it stray, and checks the whole with
/// topo::validateExact at the solid's model resolution.
///
/// Each pcurve is fitted to the larger of the model resolution of the solid's size and the
/// curve's own stand-off from its surface (measured, never chased): a clean edge to the
/// resolution, one that stands off to its gap. As an OpenCASCADE reader's per-entity tolerances
/// do, each edge's tolerance (and each vertex's) grows to cover what is measured, its vertices'
/// gaps, its pcurves' deviations and stand-offs, up to the limit of a thousandth of the solid's
/// size (the resolution, where its size cannot be measured), and where every pcurve the validator
/// finds past its tolerances is past them by rounding alone, to the validator's measure (Rulings 64
/// and 66, topo::roundingShortfalls). Past the limit the solid is not exact, and the problem says
/// so; so it does for a problem the validator finds, or a pcurve that cannot be built. The file's
/// UNCERTAINTY_MEASURE sets neither.
///
/// A periodic face whose loops wind round its surface with no seam (a band between two rims) is
/// given one by model::insertSeams once every face's pcurves exist, for its seam splits edges
/// the faces after it share; its sense and its seams' layout follow. Every other face's problem
/// leaves the solid out as soon as that face's pcurves are built.
ExactRead finishExact(topo::Solid& solid, const std::atomic<bool>* cancelled) {
    ExactRead read;
    const topo::ToleranceGrowth growth(topo::exactExtent(solid));
    const math::ModelTolerance& resolution = growth.resolution;
    read.resolution = resolution.linear;
    const double limit = growth.limit;
    const std::string limitSays = "beyond the limit of " + threeFigures(limit) +
                                  (growth.sized ? " (a thousandth of the solid's size)"
                                                : " (the model resolution: the solid's size could "
                                                  "not be measured)");
    const double base = resolution.linear;
    const auto stop = [&] {
        if (cancelled == nullptr || !cancelled->load(std::memory_order_relaxed)) return false;
        read.problem = kCancelled;
        return true;
    };
    // Grows @p tolerance to cover @p by; past the limit, the first such says why: "<what> <by>
    // <from>, beyond the limit of …".
    const auto grow = [&](double& tolerance, double by, const char* what, const char* from,
                          const topo::Face* f, const topo::Edge* e) {
        if (growth.grow(tolerance, by) || !read.problem.empty()) return;
        read.problem = std::string(what) + " " + threeFigures(by) + " " + from + ", " + limitSays +
                       whereIn(f, e);
    };

    // Each edge's curve ends at its vertices, as near as the file has them.
    for (auto& e : solid.edges()) {
        if (!e.alive || e.halfEdge == nullptr || e.halfEdge->twin == nullptr || !e.exactCurve) {
            continue;
        }
        topo::Vertex* ends[2] = {e.halfEdge->origin, e.halfEdge->twin->origin};
        const double at[2] = {e.curveStart, e.curveEnd};
        for (int k = 0; k < 2; ++k) {
            const double gap = (e.exactCurve->evaluate(at[k]) - ends[k]->point).length();
            grow(e.tolerance, gap, "its curve ends", "from its vertex", nullptr, &e);
            grow(ends[k]->tolerance, gap, "its curve ends", "from its vertex", nullptr, &e);
        }
    }
    if (!read.problem.empty()) return read;

    // Each half-edge's course on its face, built (a file's PCURVEs are not read): from its
    // origin's parameter on its edge's curve to its next's.
    const std::size_t limitHalfEdges = 2 * solid.edges().size() + 1;
    std::vector<topo::Face*> bands;
    for (auto& f : solid.faces()) {
        if (stop()) return read;
        if (!f.alive) continue;
        if (!f.exactSurface) {
            read.problem = "a face has no exact surface" + whereIn(&f, nullptr);
            return read;
        }
        std::vector<topo::Wire*> loops{f.outerLoop};
        loops.insert(loops.end(), f.innerLoops.begin(), f.innerLoops.end());
        for (topo::Wire* wire : loops) {
            const std::vector<topo::HalfEdge*> ring =
                wire != nullptr ? ringOf(*wire, limitHalfEdges) : std::vector<topo::HalfEdge*>{};
            for (topo::HalfEdge* h : ring) {
                if (stop()) return read;
                topo::Edge& e = *h->edge;
                if (!e.exactCurve) {
                    read.problem = "an edge has no exact curve" + whereIn(&f, &e);
                    return read;
                }
                if (h->twin != nullptr && h->twin->face == &f && h->twin->pcurve) {
                    // A seam's other side: the same course, the other way.
                    h->pcurve = h->twin->pcurve;
                    h->pcurveStart = h->twin->pcurveEnd;
                    h->pcurveEnd = h->twin->pcurveStart;
                } else {
                    const bool forward = h == e.halfEdge;
                    const double t0 = forward ? e.curveStart : e.curveEnd;
                    const double t1 = forward ? e.curveEnd : e.curveStart;
                    // Fitted to the resolution; where the curve stands off its surface within
                    // the limit, no pcurve comes nearer, so it is fitted to its gap (twice the
                    // gap measured, which may peak between the samples that measured it, and the
                    // resolution) in the same call, and the edge's tolerance covers what is
                    // measured.
                    geo::Pcurve pc =
                        geo::pcurveOn(*f.exactSurface, *e.exactCurve, t0, t1, base, limit);
                    if (!pc.problem.empty() || pc.curve == nullptr) {
                        read.problem = std::isfinite(pc.standOff) && pc.standOff > limit
                                           ? "its curve stands " + threeFigures(pc.standOff) +
                                                 " off its face's surface, " + limitSays
                                           : pc.problem;
                        read.problem += whereIn(&f, &e);
                        return read;
                    }
                    h->pcurve = std::move(pc.curve);
                    h->pcurveStart = pc.start;
                    h->pcurveEnd = pc.end;
                    grow(e.tolerance, pc.deviation, "its pcurve strays", "from its curve", &f, &e);
                }
                // Its ends, on the surface, at its vertices.
                const math::Vec2 a = h->pcurve->evaluate(h->pcurveStart);
                const math::Vec2 b = h->pcurve->evaluate(h->pcurveEnd);
                grow(e.tolerance, (f.exactSurface->evaluate(a.x, a.y) - h->origin->point).length(),
                     "its pcurve ends", "from its vertex", &f, &e);
                grow(e.tolerance,
                     (f.exactSurface->evaluate(b.x, b.y) - h->next->origin->point).length(),
                     "its pcurve ends", "from its vertex", &f, &e);
                if (!read.problem.empty()) return read;
            }
        }
        // The face's sense, and its seams laid out, as soon as its pcurves exist: a face that
        // cannot have them leaves the solid out before the faces after it are built. A band
        // waits for its seam, below.
        if (std::string why = faceSense(f, resolution, limitHalfEdges, limit); !why.empty()) {
            if (why == kWindsRound) {
                bands.push_back(&f);
                continue;
            }
            read.problem = why + whereIn(&f, nullptr);
            return read;
        }
        if (std::string why = layOutSeams(f, resolution, limitHalfEdges, cancelled); !why.empty()) {
            read.problem = why == kCancelled ? why : why + whereIn(&f, nullptr);
            return read;
        }
    }

    // Each band given its seam, now that every face's pcurves exist (its seam splits edges that
    // faces built after it share), then its sense and its seams' layout, as every other face had.
    // The tolerances its seam and splits are given, measured as grow measures, are held to the
    // limit as every other is: the seam's, how far its line passes from the vertices it joins; a
    // split's vertex's, how far its edge's pcurves have it.
    for (topo::Face* f : bands) {
        if (stop()) return read;
        if (std::string why = model::insertSeams(solid, *f, resolution); !why.empty()) {
            read.problem = why + whereIn(f, f->outerLoop->halfEdge->edge);
            return read;
        }
        std::vector<topo::Wire*> loops{f->outerLoop};
        loops.insert(loops.end(), f->innerLoops.begin(), f->innerLoops.end());
        for (topo::Wire* wire : loops) {
            const std::vector<topo::HalfEdge*> ring =
                wire != nullptr ? ringOf(*wire, 2 * solid.edges().size() + 1)
                                : std::vector<topo::HalfEdge*>{};
            for (const topo::HalfEdge* h : ring) {
                if (h->edge->tolerance > limit) {
                    read.problem = "its seam passes " + threeFigures(h->edge->tolerance) +
                                   " from a vertex it joins, " + limitSays + whereIn(f, h->edge);
                    return read;
                }
                if (h->origin->tolerance > limit) {
                    read.problem = "its seam splits an edge where the edge's pcurves part by " +
                                   threeFigures(h->origin->tolerance) + ", " + limitSays +
                                   whereIn(f, h->edge);
                    return read;
                }
            }
        }
    }
    const std::size_t limitSeamed = 2 * solid.edges().size() + 1;
    for (topo::Face* f : bands) {
        if (stop()) return read;
        if (std::string why = faceSense(*f, resolution, limitSeamed, limit); !why.empty()) {
            const topo::Edge* first = why == kWindsRound ? f->outerLoop->halfEdge->edge : nullptr;
            read.problem = why + whereIn(f, first);
            return read;
        }
        if (std::string why = layOutSeams(*f, resolution, limitSeamed, cancelled); !why.empty()) {
            read.problem = why == kCancelled ? why : why + whereIn(f, nullptr);
            return read;
        }
    }

    if (stop()) return read;
    auto issues = topo::validateExact(solid, resolution);
    // Where the validator finds pcurves past their tolerances by rounding alone (Rulings 64 and
    // 66): each tolerance was grown to pcurveOn's measure of the gap, at its own points and by its
    // own route, of a pcurve that may since have been reversed for a seam's other side, moved a
    // period, or split by a band's seam, and the validator's measure of the same gap can come out
    // a last bit larger (where sin, cos and atan2 round otherwise, as macOS's do). Only where every
    // issue is such a shortfall does each tolerance grow to the validator's own measure, held to
    // the limit as every other gap is, and the solid is validated again; otherwise the first issue
    // that is not one is reported as ever, and nothing is taken up, so that no shortfall's growth
    // covers a pcurve that strays beside it.
    if (!issues.empty()) {
        const topo::RoundingTakeUp takeUp = topo::roundingShortfalls(solid, issues, resolution);
        if (takeUp.beyond != nullptr) {
            read.problem =
                takeUp.beyond->detail + whereIn(takeUp.beyond->face, takeUp.beyond->edge);
            return read;
        }
        for (const topo::RoundingShortfall& s : takeUp.shortfalls) {
            const bool stray = s.issue->kind == topo::ExactIssueKind::PcurveOffCurve;
            grow(*s.tolerance, s.measured, stray ? "its pcurve strays" : "its pcurve ends",
                 stray ? "from its curve" : "from its vertex", s.issue->face, s.issue->edge);
        }
        if (!read.problem.empty()) return read;
        if (stop()) return read;
        issues = topo::validateExact(solid, resolution);
    }
    if (!issues.empty()) {
        read.problem = issues.front().detail + whereIn(issues.front().face, issues.front().edge);
        return read;
    }
    for (const auto& e : solid.edges()) {
        if (e.alive) read.largestEdgeTolerance = std::max(read.largestEdgeTolerance, e.tolerance);
    }
    return read;
}

/// How many times its estimated error a shell's coarse volume must be, the way its role wants it
/// (an outer shell's positive, a void's negative), for the coarse pass to settle it.
constexpr double kSignMargin = 4.0;

/// The shells of @p solid, by their index in its shell order, that face the wrong way for what
/// they are, by the signed volume each encloses integrated on its exact faces: an outer shell
/// whose faces face into it (written inside out, as a careless writer may: Horizon's own wrote
/// its kernel's box so), or a void's whose faces face out of the void. @p voids says which
/// shells are voids. A coarse integral settles each shell its volume leaves beyond doubt the way
/// its role wants; the solid is integrated as mass properties integrate it only where one is not
/// (a shell facing the wrong way, or too thin for the coarse pass to tell), and only that
/// integral turns a shell. None where the volumes cannot be measured: the mass properties of
/// such a solid say why when asked.
std::vector<std::size_t> shellsInsideOut(const topo::Solid& solid, const std::vector<bool>& voids,
                                         const std::atomic<bool>* cancelled) {
    using model::MassPropertiesCalculator;
    const auto coarse = MassPropertiesCalculator::shellVolumes(solid, true, nullptr, cancelled);
    // The shells the coarse pass leaves in doubt (all, where it could not be made), alone.
    std::vector<bool> doubtful(voids.size(), coarse.size() != voids.size());
    for (std::size_t i = 0; i < coarse.size() && i < voids.size(); ++i) {
        const double way = voids[i] ? -coarse[i].volume : coarse[i].volume;
        doubtful[i] = !(way > kSignMargin * coarse[i].error);
    }
    if (std::none_of(doubtful.begin(), doubtful.end(), [](bool d) { return d; })) return {};
    const auto exact =
        MassPropertiesCalculator::shellVolumes(solid, false, nullptr, cancelled, doubtful);
    if (exact.size() != voids.size()) return {};
    // Turned only where it faces the wrong way beyond its error: one that encloses next to
    // nothing is left as it is.
    std::vector<std::size_t> wrong;
    for (std::size_t i = 0; i < exact.size(); ++i) {
        const double way = voids[i] ? -exact[i].volume : exact[i].volume;
        if (doubtful[i] && way < 0.0 && -way > exact[i].error) wrong.push_back(i);
    }
    return wrong;
}

/// @p solid without its exact form, as master reads it: every exact field cleared, so nothing
/// half-exact survives a solid kept though its exact form failed part way.
void clearExact(topo::Solid& solid) {
    for (auto& v : solid.vertices()) v.tolerance = 0.0;
    for (auto& e : solid.edges()) {
        e.exactCurve = nullptr;
        e.curveStart = 0.0;
        e.curveEnd = 0.0;
        e.tolerance = 0.0;
    }
    const std::size_t limit = 2 * solid.edges().size() + 1;
    for (auto& f : solid.faces()) {
        f.exactSurface = nullptr;
        f.reversed = false;
        std::vector<topo::Wire*> loops{f.outerLoop};
        loops.insert(loops.end(), f.innerLoops.begin(), f.innerLoops.end());
        for (topo::Wire* wire : loops) {
            if (wire == nullptr) continue;
            for (topo::HalfEdge* h : ringOf(*wire, limit)) {
                h->pcurve = nullptr;
                h->pcurveStart = 0.0;
                h->pcurveEnd = 0.0;
            }
        }
    }
}

/// A file's solid groups, each built once, in millimetres: null where one
/// could not be. What fell short, for a report.
struct BuiltSolids {
    std::vector<std::unique_ptr<topo::Solid>> solids;
    std::vector<std::string> failures;
    std::vector<std::string> approximated;
    std::map<std::string, int> conversions;
    /// The solids exact only to their file's tolerance, each with how far its edges stray.
    std::vector<std::string> exactTo;
    /// The solids kept without their exact form (StepReadOptions::keepWhenNotExact), with why.
    std::vector<std::string> keptAsItWas;
    std::string firstError;
    /// A solid's length unit is not named, or cannot be read, and none was
    /// chosen: nothing more is built, and the file is not read.
    bool unitUnknown = false;
};

BuiltSolids buildSolids(const StepParser& parser, const std::vector<SolidGroup>& groups,
                        bool reporting, const std::atomic<bool>* cancelled,
                        const StepReadOptions& options) {
    BuiltSolids built;
    built.solids.resize(groups.size());
    // Each solid on its own: one that cannot be rebuilt is reported, and the
    // others still come in. Its index in the file names its topology, so a
    // solid's names do not depend on whether the ones before it were read.
    for (size_t index = 0; index < groups.size(); ++index) {
        if (cancelled != nullptr && cancelled->load(std::memory_order_relaxed)) return built;
        const SolidGroup& group = groups[index];
        const std::string which =
            "solid " + std::to_string(index + 1) + " (#" + std::to_string(group.msbs.front()) + ")";
        // Millimetres per unit of its lengths. Not named, or not readable:
        // the unit chosen, and said; none chosen, the file is not read.
        std::string unit;
        double mm = group.context != 0 ? contextMillimetres(parser, group.context, &unit) : 0.0;
        const bool chosen = !(mm > 0.0);
        if (chosen) {
            if (!options.unknownLengthUnit) {
                built.unitUnknown = true;
                return built;
            }
            mm = math::millimetresPer(*options.unknownLengthUnit);
            unit = unitName(*options.unknownLengthUnit);
        }
        // How far the file says its geometry may stray, in millimetres.
        const double uncertainty =
            group.context != 0 ? contextUncertaintyMillimetres(parser, group.context, mm) : 0.0;
        const double radians = group.context != 0 ? contextRadians(parser, group.context) : 1.0;
        SolidBuilder builder(parser, static_cast<int>(index), cancelled, radians, uncertainty / mm);
        // The solid as the file has it, in millimetres, built by @p from: its topology and
        // carriers, and the exact fields read with them, as no exact pass has yet changed them.
        // Throws as the geometry constructors do.
        const auto asTheFileHasIt = [&](SolidBuilder& from, std::string& why) {
            std::unique_ptr<topo::Solid> built = from.build(group.msbs, group.surfaces, why);
            if (built != nullptr && mm != 1.0) {
                built = model::Pattern::transformed(*built, math::Mat4::scale(mm));
            }
            return built;
        };
        // Its exact form, or nothing (decision 1b): a solid whose faces or edges cannot be read
        // exactly, or whose exact form does not hold together, is left out, with why. Nothing
        // faceted comes in in its place. (STEP embedded in a native part file is kept as it was
        // read before, without its exact form: old parts still open.)
        const std::string solidN = "solid " + std::to_string(index + 1);
        std::unique_ptr<topo::Solid> solid;
        std::string error;
        std::string notExact;
        ExactRead exact;
        try {
            solid = asTheFileHasIt(builder, error);
            if (solid != nullptr) notExact = builder.exactProblem();
        } catch (const std::exception& e) {
            // The geometry constructors throw on inputs the reader's own
            // checks let through (a degree-0 B-spline, ragged control rows,
            // a scale into millimetres that overflows): no solid at all.
            error = e.what();
            solid = nullptr;
        }
        // Whether the exact pass ran on the solid, which changes its topology as it goes (a band's
        // seam inserted, the rims it crosses split) and may fail after it has.
        const bool exactPassRan = solid != nullptr && notExact.empty();
        if (exactPassRan) {
            // A throw making the exact form leaves the solid, as read, without one: left out of
            // an import, kept as it was in a native file's embedded STEP.
            try {
                exact = finishExact(*solid, cancelled);
                // A shell written inside out (every face facing into what it bounds, consistently,
                // so the validator cannot tell) is turned outward as build() turns a void that
                // measures the wrong way: built again with its loops run the other way and its
                // faces reversed, and made exact again. A void stays facing into itself.
                const std::vector<std::size_t> inside =
                    exact.problem.empty() ? shellsInsideOut(*solid, builder.voidShells(), cancelled)
                                          : std::vector<std::size_t>{};
                if (!inside.empty()) {
                    std::string why;
                    auto again = builder.turned(inside, why);
                    if (again == nullptr || !builder.exactProblem().empty()) {
                        notExact = "its shell written inside out could not be turned round: " +
                                   (why.empty() ? builder.exactProblem() : why);
                    } else {
                        if (mm != 1.0) {
                            again = model::Pattern::transformed(*again, math::Mat4::scale(mm));
                        }
                        exact = finishExact(*again, cancelled);
                        solid = std::move(again);
                    }
                }
            } catch (const std::exception& e) {
                notExact = std::string("its exact form could not be made: ") + e.what();
            }
        }
        if (cancelled != nullptr && cancelled->load(std::memory_order_relaxed)) return built;
        if (solid == nullptr) {
            if (built.firstError.empty()) {
                built.firstError = "failed to reconstruct solid #" +
                                   std::to_string(group.msbs.front()) + ": " + error;
            }
            built.failures.push_back(which + ": " + error);
            continue;
        }
        if (const std::size_t open = builder.openShells(); open > 0) {
            built.failures.push_back(which + ": " + std::to_string(open) +
                                     (open == 1 ? " of its shells is an open surface"
                                                : " of its shells are open surfaces") +
                                     ", not a solid, and left out");
        }
        if (notExact.empty()) {
            notExact = exact.problem;
            if (notExact.empty() && exact.largestEdgeTolerance > exact.resolution) {
                built.exactTo.push_back(solidN + ": exact to " +
                                        threeFigures(exact.largestEdgeTolerance) +
                                        " mm, as the file gives its edges");
            }
        }
        if (!notExact.empty()) {
            if (!options.keepWhenNotExact) {
                if (built.firstError.empty()) {
                    built.firstError = "solid #" + std::to_string(group.msbs.front()) +
                                       " could not be made exact: " + notExact;
                }
                built.failures.push_back(solidN + ": " + notExact);
                continue;
            }
            // Kept as master read it: as the file has it, untouched by an exact pass that failed
            // part way (Ruling 10). What the pass changed is not the file's, and may not even hold
            // together (a seam inserted on one band and the next refused, a ring left merged by
            // a split that threw): the solid is built again from the file, and the exact fields
            // the builder reads with it cleared.
            if (exactPassRan) {
                std::string why;
                try {
                    SolidBuilder again(parser, static_cast<int>(index), cancelled, radians,
                                       uncertainty / mm);
                    solid = asTheFileHasIt(again, why);
                } catch (const std::exception& e) {
                    why = e.what();
                    solid = nullptr;
                }
                if (cancelled != nullptr && cancelled->load(std::memory_order_relaxed)) {
                    return built;
                }
                if (solid == nullptr) {
                    // It built once from the same file: not reached, yet said if it is.
                    if (built.firstError.empty()) {
                        built.firstError = "failed to reconstruct solid #" +
                                           std::to_string(group.msbs.front()) + ": " + why;
                    }
                    built.failures.push_back(which + ": " + why);
                    continue;
                }
            }
            clearExact(*solid);
            built.keptAsItWas.push_back(solidN + ": not exact (" + notExact + "); kept as it was");
        }
        if (chosen) {
            ++built.conversions["read in " + unit +
                                ", as chosen: the file names no length unit this version reads"];
        } else if (mm != 1.0) {
            std::ostringstream factor;
            factor << std::setprecision(10) << mm;
            ++built.conversions["drawn in " + unit + ", scaled by " + factor.str() +
                                " into millimetres"];
        }
        // Its curved faces go in as facets (ImportedBodyFeature): say where
        // that falls short. (A body read again from a native file was said
        // so of when it was imported.)
        if (reporting && !options.keepWhenNotExact) {
            const auto faceted = model::facetCurved(*solid);
            if (!faceted.solid) {
                built.approximated.push_back(
                    which + ": its curved faces could not be cut into facets (" + faceted.error +
                    "), so its volume and Booleans follow its corners alone");
            } else if (!faceted.outlined.empty()) {
                const size_t n = faceted.outlined.size();
                built.approximated.push_back(
                    which + ": " + std::to_string(n) +
                    (n == 1 ? " curved face is" : " curved faces are") +
                    " not bounded by its surface's own edges, and is one facet, its outline");
            }
        }
        built.solids[index] = std::move(solid);
    }
    return built;
}

/// What @p built and @p structure fell short of, into @p report.
void reportRead(const BuiltSolids& built, const ProductStructure& structure, ImportReport* report) {
    if (report == nullptr) return;
    for (const auto& failure : built.failures) report->skipped.push_back(failure);
    for (const auto& note : structure.leftOut) report->skipped.push_back(note);
    for (const auto& note : structure.notes) report->approximated.push_back(note);
    for (const auto& line : built.approximated) report->approximated.push_back(line);
    for (const auto& [what, count] : built.conversions) {
        report->converted.push_back(
            (count == 1 ? std::string("1 solid ") : std::to_string(count) + " solids ") + what);
    }
    for (const auto& line : built.exactTo) report->converted.push_back(line);
    for (const auto& line : built.keptAsItWas) report->converted.push_back(line);
}

/// @p solid's names moved from solid @p from's to solid @p to's: a second
/// placement of a part named as a solid of its own.
void renameSolid(topo::Solid& solid, std::size_t from, std::size_t to) {
    const std::string was = "step/solid:" + std::to_string(from) + "/";
    const std::string now = "step/solid:" + std::to_string(to) + "/";
    const auto rename = [&](topo::TopologyID& id) {
        if (id.tag().rfind(was, 0) == 0) {
            id = topo::TopologyID::fromTag(now + id.tag().substr(was.size()));
        }
    };
    for (auto& v : solid.vertices()) rename(v.topoId);
    for (auto& e : solid.edges()) rename(e.topoId);
    for (auto& f : solid.faces()) rename(f.topoId);
}

/// The entities a file starts with: its units and context, its
/// application. Their ids.
struct Preamble {
    int context = 0;
    int appContext = 0;
    int productContext = 0;
};

/// The uncertainty a file states for its solids, in millimetres, where nothing is given a
/// tolerance: the writer's own resolution.
constexpr double kWrittenUncertainty = 1e-7;

/// The uncertainty a file of @p solids states, in millimetres: kWrittenUncertainty, or the largest
/// tolerance a vertex of theirs was given, where that is larger. A vertex's tolerance covers how
/// far it stands from the ends of its edges' curves, as its file had it. The reader joins a closed
/// B-spline's ends, and takes a vertex to be at that join, within its file's uncertainty
/// (SolidBuilder::placeOnCurve): stated finer than its vertices stand, a solid read exact within
/// a loose file's uncertainty would be read back with a vertex off its join, and not exact.
double uncertaintyOf(const std::vector<const topo::Solid*>& solids) {
    double largest = kWrittenUncertainty;
    for (const topo::Solid* solid : solids) {
        if (solid == nullptr) continue;
        for (const auto& v : solid->vertices()) {
            if (v.alive && std::isfinite(v.tolerance)) largest = std::max(largest, v.tolerance);
        }
    }
    return largest;
}

Preamble writePreamble(StepWriter& w, double uncertaintyMillimetres) {
    Preamble pre;
    // Geometric representation context (SI millimetres) shared by all solids.
    const int lengthUnit = w.add("(LENGTH_UNIT() NAMED_UNIT(*) SI_UNIT(.MILLI.,.METRE.))");
    const int angleUnit = w.add("(NAMED_UNIT(*) PLANE_ANGLE_UNIT() SI_UNIT($,.RADIAN.))");
    const int solidAngleUnit = w.add("(NAMED_UNIT(*) SI_UNIT($,.STERADIAN.) SOLID_ANGLE_UNIT())");
    const std::string measure = uncertaintyMillimetres > kWrittenUncertainty
                                    ? fmtReal(uncertaintyMillimetres)
                                    : std::string("1.E-7");
    const int uncertainty =
        w.add("UNCERTAINTY_MEASURE_WITH_UNIT(LENGTH_MEASURE(" + measure + "),#" +
              std::to_string(lengthUnit) + ",'distance_accuracy_value','confusion accuracy')");
    pre.context = w.add(
        "(GEOMETRIC_REPRESENTATION_CONTEXT(3) GLOBAL_UNCERTAINTY_ASSIGNED_CONTEXT((#" +
        std::to_string(uncertainty) + ")) GLOBAL_UNIT_ASSIGNED_CONTEXT((#" +
        std::to_string(lengthUnit) + ",#" + std::to_string(angleUnit) + ",#" +
        std::to_string(solidAngleUnit) + ")) REPRESENTATION_CONTEXT('Context #1','3D Context'))");

    pre.appContext = w.add("APPLICATION_CONTEXT('managed model based 3d engineering')");
    w.add(
        "APPLICATION_PROTOCOL_DEFINITION('international standard',"
        "'ap242_managed_model_based_3d_engineering',2020,#" +
        std::to_string(pre.appContext) + ")");

    pre.productContext =
        w.add("PRODUCT_CONTEXT('',#" + std::to_string(pre.appContext) + ",'mechanical')");
    return pre;
}

/// A product and its definition: the definition's id, and its shape's.
struct ProductIds {
    int definition = 0;
    int shape = 0;
};

/// A product, @p quoted its name as Part-21 text, of @p category.
ProductIds writeProduct(StepWriter& w, const Preamble& pre, const std::string& quoted,
                        const char* category) {
    const int product = w.add("PRODUCT(" + quoted + "," + quoted + ",'',(#" +
                              std::to_string(pre.productContext) + "))");
    w.add(std::string("PRODUCT_RELATED_PRODUCT_CATEGORY('") + category + "',$,(#" +
          std::to_string(product) + "))");
    const int formation =
        w.add("PRODUCT_DEFINITION_FORMATION('',$,#" + std::to_string(product) + ")");
    const int pdc = w.add("PRODUCT_DEFINITION_CONTEXT('part definition',#" +
                          std::to_string(pre.appContext) + ",'design')");
    ProductIds ids;
    ids.definition = w.add("PRODUCT_DEFINITION('design',$,#" + std::to_string(formation) + ",#" +
                           std::to_string(pdc) + ")");
    ids.shape = w.add("PRODUCT_DEFINITION_SHAPE('',$,#" + std::to_string(ids.definition) + ")");
    return ids;
}

std::string fileText(const StepWriter& w) {
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

/// Whether @p m is a rotation and a move alone: what an axis placement
/// can say.
bool isRigid(const Mat4& m) {
    constexpr double kTol = 1e-6;
    if (std::abs(m.at(3, 0)) > kTol || std::abs(m.at(3, 1)) > kTol || std::abs(m.at(3, 2)) > kTol ||
        std::abs(m.at(3, 3) - 1.0) > kTol) {
        return false;
    }
    const Vec3 x{m.at(0, 0), m.at(1, 0), m.at(2, 0)};
    const Vec3 y{m.at(0, 1), m.at(1, 1), m.at(2, 1)};
    const Vec3 z{m.at(0, 2), m.at(1, 2), m.at(2, 2)};
    for (double v : {m.at(0, 3), m.at(1, 3), m.at(2, 3)}) {
        if (!std::isfinite(v)) return false;
    }
    return std::abs(x.length() - 1.0) < kTol && std::abs(y.length() - 1.0) < kTol &&
           std::abs(z.length() - 1.0) < kTol && std::abs(x.dot(y)) < kTol &&
           std::abs(y.dot(z)) < kTol && std::abs(z.dot(x)) < kTol && x.cross(y).dot(z) > 0.0;
}

}  // namespace

// ===========================================================================
// Public API
// ===========================================================================

std::string StepFormat::toString(const std::vector<const topo::Solid*>& solids,
                                 const WriteOptions& options, WriteReport* report) {
    StepWriter w;
    if (report != nullptr) {
        report->faceted.clear();
        report->leftOut.clear();
    }
    const Preamble pre = writePreamble(w, uncertaintyOf(solids));

    int index = 0;
    for (const topo::Solid* solid : solids) {
        if (solid == nullptr) continue;
        const std::vector<int> msbs = writeSolid(w, *solid, index, options.asDesigned,
                                                 report != nullptr ? &report->faceted : nullptr);
        if (msbs.empty()) continue;

        const std::string name = "'part_" + std::to_string(index) + "'";
        const ProductIds ids = writeProduct(w, pre, name, "part");
        const int rep = w.add("ADVANCED_BREP_SHAPE_REPRESENTATION(" + name + "," +
                              StepWriter::refList(msbs) + ",#" + std::to_string(pre.context) + ")");
        w.add("SHAPE_DEFINITION_REPRESENTATION(#" + std::to_string(ids.shape) + ",#" +
              std::to_string(rep) + ")");
        ++index;
    }
    return fileText(w);
}

std::string StepFormat::assemblyToString(const std::string& name,
                                         const std::vector<StepWritePart>& parts,
                                         const std::vector<StepOccurrence>& occurrences,
                                         const WriteOptions& options, WriteReport* report) {
    StepWriter w;
    if (report != nullptr) {
        report->faceted.clear();
        report->leftOut.clear();
    }
    const auto leaveOut = [report](const std::string& why) {
        if (report != nullptr) report->leftOut.push_back(why);
    };
    std::vector<const topo::Solid*> bodies;
    for (const StepWritePart& part : parts) {
        bodies.insert(bodies.end(), part.bodies.begin(), part.bodies.end());
    }
    const Preamble pre = writePreamble(w, uncertaintyOf(bodies));
    const auto direction = [&w](const Vec3& d) {
        return w.add("DIRECTION('',(" + fmtReal(d.x) + "," + fmtReal(d.y) + "," + fmtReal(d.z) +
                     "))");
    };
    // The frame @p m takes the part's own into: where it is placed.
    const auto axis = [&](const Mat4& m) {
        const int origin = w.addPoint({m.at(0, 3), m.at(1, 3), m.at(2, 3)});
        const int z = direction(Vec3{m.at(0, 2), m.at(1, 2), m.at(2, 2)}.normalized());
        const int x = direction(Vec3{m.at(0, 0), m.at(1, 0), m.at(2, 0)}.normalized());
        return w.add("AXIS2_PLACEMENT_3D(''," + std::string("#") + std::to_string(origin) + ",#" +
                     std::to_string(z) + ",#" + std::to_string(x) + ")");
    };

    // The occurrences that can be placed, and the parts they use.
    std::vector<const StepOccurrence*> placeable;
    std::vector<bool> used(parts.size(), false);
    for (const StepOccurrence& occurrence : occurrences) {
        const std::string what = "\"" + occurrence.name + "\"";
        if (occurrence.part >= parts.size()) {
            leaveOut(what + ": its part is not in the assembly");
        } else if (!isRigid(occurrence.transform)) {
            leaveOut(what + ": its placement is not a rotation and a move, which STEP places by");
        } else {
            placeable.push_back(&occurrence);
            used[occurrence.part] = true;
        }
    }

    // Each part used, once: its origin, the frame its placements move, in a
    // shape representation of its own; and each of its solids in an
    // advanced B-rep representation related to it (as OCC writes a part),
    // so each comes back a solid of its own, not a shell of one.
    struct Written {
        int definition = 0;
        int rep = 0;
        int origin = 0;
    };
    std::vector<std::optional<Written>> written(parts.size());
    int index = 0;
    for (std::size_t p = 0; p < parts.size(); ++p) {
        if (!used[p]) continue;
        const std::string quoted = stepText(parts[p].name);
        std::vector<int> bodies;
        for (const topo::Solid* body : parts[p].bodies) {
            if (body == nullptr) continue;
            const std::vector<int> msbs =
                writeSolid(w, *body, index++, options.asDesigned,
                           report != nullptr ? &report->faceted : nullptr);
            if (msbs.empty()) continue;
            bodies.push_back(w.add("ADVANCED_BREP_SHAPE_REPRESENTATION(" + quoted + "," +
                                   StepWriter::refList(msbs) + ",#" + std::to_string(pre.context) +
                                   ")"));
        }
        if (bodies.empty()) continue;
        Written part;
        part.origin = axis(Mat4::identity());
        const ProductIds ids = writeProduct(w, pre, quoted, "part");
        part.definition = ids.definition;
        part.rep = w.add("SHAPE_REPRESENTATION(" + quoted + ",(#" + std::to_string(part.origin) +
                         "),#" + std::to_string(pre.context) + ")");
        w.add("SHAPE_DEFINITION_REPRESENTATION(#" + std::to_string(ids.shape) + ",#" +
              std::to_string(part.rep) + ")");
        for (const int body : bodies) {
            w.add("SHAPE_REPRESENTATION_RELATIONSHIP('','',#" + std::to_string(part.rep) + ",#" +
                  std::to_string(body) + ")");
        }
        written[p] = part;
    }

    // The assembly: its origin and a frame for each placement, and a use
    // of each part at it.
    std::vector<int> items{axis(Mat4::identity())};
    struct Placement {
        const StepOccurrence* occurrence = nullptr;
        Written part;
        int frame = 0;
    };
    std::vector<Placement> placements;
    for (const StepOccurrence* occurrence : placeable) {
        const auto& part = written[occurrence->part];
        if (!part) {
            leaveOut("\"" + occurrence->name + "\": its part has no solid to write");
            continue;
        }
        placements.push_back({occurrence, *part, axis(occurrence->transform)});
        items.push_back(placements.back().frame);
    }
    const std::string quoted = stepText(name);
    const ProductIds assembly = writeProduct(w, pre, quoted, "assembly");
    const int rep = w.add("SHAPE_REPRESENTATION(" + quoted + "," + StepWriter::refList(items) +
                          ",#" + std::to_string(pre.context) + ")");
    w.add("SHAPE_DEFINITION_REPRESENTATION(#" + std::to_string(assembly.shape) + ",#" +
          std::to_string(rep) + ")");
    int number = 0;
    for (const auto& [occurrence, part, frame] : placements) {
        const int use =
            w.add("NEXT_ASSEMBLY_USAGE_OCCURRENCE('" + std::to_string(++number) + "'," +
                  stepText(occurrence->name) + ",'',#" + std::to_string(assembly.definition) +
                  ",#" + std::to_string(part.definition) + ",$)");
        const int shape = w.add("PRODUCT_DEFINITION_SHAPE('','',#" + std::to_string(use) + ")");
        const int transformation =
            w.add("ITEM_DEFINED_TRANSFORMATION('','',#" + std::to_string(part.origin) + ",#" +
                  std::to_string(frame) + ")");
        const int relationship =
            w.add("(REPRESENTATION_RELATIONSHIP('','',#" + std::to_string(part.rep) + ",#" +
                  std::to_string(rep) + ") REPRESENTATION_RELATIONSHIP_WITH_TRANSFORMATION(#" +
                  std::to_string(transformation) + ") SHAPE_REPRESENTATION_RELATIONSHIP())");
        w.add("CONTEXT_DEPENDENT_SHAPE_REPRESENTATION(#" + std::to_string(relationship) + ",#" +
              std::to_string(shape) + ")");
    }
    return fileText(w);
}

bool StepFormat::save(const std::string& filePath, const std::vector<const topo::Solid*>& solids,
                      const WriteOptions& options, WriteReport* report) {
    g_lastError.clear();
    if (solids.empty()) {
        g_lastError = "no solids to export";
        return false;
    }
    std::string error;
    if (!writeFileAtomically(pathFromUtf8(filePath), toString(solids, options, report), &error)) {
        g_lastError = error;
        return false;
    }
    return true;
}

bool StepFormat::saveAssembly(const std::string& filePath, const std::string& name,
                              const std::vector<StepWritePart>& parts,
                              const std::vector<StepOccurrence>& occurrences,
                              const WriteOptions& options, WriteReport* report) {
    g_lastError.clear();
    WriteReport own;
    WriteReport& said = report != nullptr ? *report : own;
    const std::string text = assemblyToString(name, parts, occurrences, options, &said);
    if (said.leftOut.size() >= occurrences.size()) {
        g_lastError = "no component to place";
        return false;
    }
    std::string error;
    if (!writeFileAtomically(pathFromUtf8(filePath), text, &error)) {
        g_lastError = error;
        return false;
    }
    return true;
}

namespace {

/// Say that a read stopped for its length unit; true.
bool stoppedForTheUnit(bool unknown) {
    if (unknown) {
        g_lastError = kUnitUnknown;
        g_lastUnitUnknown = true;
    }
    return unknown;
}

}  // namespace

std::vector<std::unique_ptr<topo::Solid>> StepFormat::fromString(const std::string& text,
                                                                 ImportReport* report,
                                                                 const std::atomic<bool>* cancelled,
                                                                 const StepReadOptions& options) {
    g_lastError.clear();
    g_lastUnitUnknown = false;
    const auto stopped = [cancelled] {
        return cancelled != nullptr && cancelled->load(std::memory_order_relaxed);
    };
    StepParser parser(text);
    if (!parseText(parser, cancelled)) return {};
    const std::vector<SolidGroup> groups = solidGroups(parser);
    if (groups.empty()) {
        g_lastError = "no MANIFOLD_SOLID_BREP found in file";
        return {};
    }
    BuiltSolids built = buildSolids(parser, groups, report != nullptr, cancelled, options);
    if (stoppedForTheUnit(built.unitUnknown)) return {};
    if (stopped()) {
        g_lastError = kCancelled;
        return {};
    }
    const ProductStructure structure = readStructure(parser, groups, options);
    if (stoppedForTheUnit(structure.unitUnknown)) return {};

    std::vector<std::unique_ptr<topo::Solid>> out;
    if (!structure.structured) {
        // Parts alone: each solid where it was drawn, as the file has them.
        for (auto& solid : built.solids) {
            if (solid) out.push_back(std::move(solid));
        }
    } else {
        // Each placement of each part's solids. A solid's first keeps its
        // names (as a file of the part alone names it); each after it is
        // named as a solid of its own, numbered after the file's.
        std::vector<std::size_t> uses(groups.size(), 0);
        for (const StepOccurrence& placed : structure.placed) {
            for (std::size_t g : structure.products[placed.part].groups) ++uses[g];
        }
        std::vector<bool> named(groups.size(), false);
        std::size_t next = groups.size();
        std::set<std::size_t> parts;  // the products placed
        for (const StepOccurrence& placed : structure.placed) {
            if (stopped()) {
                g_lastError = kCancelled;
                return {};
            }
            for (std::size_t g : structure.products[placed.part].groups) {
                auto& solid = built.solids[g];
                if (!solid) continue;
                std::unique_ptr<topo::Solid> copy =
                    --uses[g] == 0 && isIdentity(placed.transform)
                        ? std::move(solid)
                        : model::Pattern::transformed(*solid, placed.transform);
                if (named[g]) renameSolid(*copy, g, next++);
                named[g] = true;
                out.push_back(std::move(copy));
                parts.insert(placed.part);
            }
        }
        if (report != nullptr && !out.empty()) {
            report->converted.push_back("an assembly: " + std::to_string(parts.size()) +
                                        (parts.size() == 1 ? " part placed" : " parts placed") +
                                        " where it puts them, as " + std::to_string(out.size()) +
                                        (out.size() == 1 ? " solid" : " solids"));
        }
    }
    if (out.empty()) {
        g_lastError = built.firstError.empty() ? "no solid could be read" : built.firstError;
        return out;
    }
    reportRead(built, structure, report);
    return out;
}

StepAssembly StepFormat::assemblyFromString(const std::string& text, ImportReport* report,
                                            const std::atomic<bool>* cancelled,
                                            const StepReadOptions& options) {
    g_lastError.clear();
    g_lastUnitUnknown = false;
    StepAssembly assembly;
    StepParser parser(text);
    if (!parseText(parser, cancelled)) return assembly;
    const std::vector<SolidGroup> groups = solidGroups(parser);
    if (groups.empty()) {
        g_lastError = "no MANIFOLD_SOLID_BREP found in file";
        return assembly;
    }
    BuiltSolids built = buildSolids(parser, groups, report != nullptr, cancelled, options);
    if (stoppedForTheUnit(built.unitUnknown)) return {};
    if (cancelled != nullptr && cancelled->load(std::memory_order_relaxed)) {
        g_lastError = kCancelled;
        return {};
    }
    const ProductStructure structure = readStructure(parser, groups, options);
    if (stoppedForTheUnit(structure.unitUnknown)) return {};

    // Each product with a solid that could be read, a part; its solids
    // moved into it, or copied when another product has them too.
    std::vector<std::size_t> uses(groups.size(), 0);
    for (const auto& product : structure.products) {
        for (std::size_t g : product.groups) ++uses[g];
    }
    constexpr std::size_t kNone = std::numeric_limits<std::size_t>::max();
    std::vector<std::size_t> partOf(structure.products.size(), kNone);
    for (std::size_t p = 0; p < structure.products.size(); ++p) {
        const auto& product = structure.products[p];
        StepAssembly::Part part;
        part.name = blank(product.name) ? "part " + std::to_string(p + 1) : product.name;
        for (std::size_t g : product.groups) {
            auto& solid = built.solids[g];
            if (!solid) continue;
            part.bodies.push_back(--uses[g] == 0
                                      ? std::move(solid)
                                      : model::Pattern::transformed(*solid, Mat4::identity()));
        }
        if (part.bodies.empty()) continue;
        partOf[p] = assembly.parts.size();
        assembly.parts.push_back(std::move(part));
    }
    for (const StepOccurrence& placed : structure.placed) {
        if (partOf[placed.part] == kNone) continue;
        StepOccurrence occurrence = placed;
        occurrence.part = partOf[placed.part];
        if (blank(occurrence.name)) occurrence.name = assembly.parts[occurrence.part].name;
        assembly.occurrences.push_back(std::move(occurrence));
    }
    assembly.structured = structure.structured;
    if (assembly.parts.empty()) {
        g_lastError = built.firstError.empty() ? "no solid could be read" : built.firstError;
        return assembly;
    }
    reportRead(built, structure, report);
    return assembly;
}

namespace {

/// The bytes of the file at @p filePath; nullopt, with g_lastError said,
/// when it cannot be opened.
std::optional<std::string> fileBytes(const std::string& filePath) {
    std::ifstream file(pathFromUtf8(filePath), std::ios::binary);
    if (!file) {
        g_lastError = "cannot open file: " + filePath;
        return std::nullopt;
    }
    std::ostringstream ss;
    ss << file.rdbuf();
    return ss.str();
}

}  // namespace

std::vector<std::unique_ptr<topo::Solid>> StepFormat::load(const std::string& filePath,
                                                           ImportReport* report,
                                                           const std::atomic<bool>* cancelled,
                                                           const StepReadOptions& options) {
    g_lastError.clear();
    g_lastUnitUnknown = false;
    const auto text = fileBytes(filePath);
    return text ? fromString(*text, report, cancelled, options)
                : std::vector<std::unique_ptr<topo::Solid>>{};
}

StepAssembly StepFormat::loadAssembly(const std::string& filePath, ImportReport* report,
                                      const std::atomic<bool>* cancelled,
                                      const StepReadOptions& options) {
    g_lastError.clear();
    g_lastUnitUnknown = false;
    const auto text = fileBytes(filePath);
    return text ? assemblyFromString(*text, report, cancelled, options) : StepAssembly{};
}

const std::string& StepFormat::lastError() {
    return g_lastError;
}

bool StepFormat::lastLengthUnitUnknown() {
    return g_lastUnitUnknown;
}

}  // namespace hz::io
