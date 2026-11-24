#include "horizon/geometry/surfaces/SurfaceRecognition.h"

#include <algorithm>
#include <cmath>
#include <functional>
#include <optional>
#include <vector>

#include "horizon/geometry/surfaces/AnalyticSurfaces.h"
#include "horizon/math/Constants.h"

namespace hz::geo {

namespace {

using math::Vec3;

/// The samples per direction of the grid a NURBS is checked on.
constexpr int kN = 33;

Vec3 unit(const Vec3& v) {
    const double l = v.length();
    return l > 0.0 ? v / l : Vec3{0.0, 0.0, 0.0};
}

bool finite(const Vec3& v) {
    return std::isfinite(v.x) && std::isfinite(v.y) && std::isfinite(v.z);
}

// ---------------------------------------------------------------------------
// Small dense linear algebra (row-major n x n)
// ---------------------------------------------------------------------------

/// Solves A x = b by Gaussian elimination with partial pivoting; false when A is
/// singular to working precision.
bool solve(std::vector<double> a, std::vector<double> b, int n, std::vector<double>& x) {
    double maxAbs = 0.0;
    for (double e : a) {
        maxAbs = std::max(maxAbs, std::abs(e));
    }
    if (!(maxAbs > 0.0)) {
        return false;
    }
    for (int c = 0; c < n; ++c) {
        int piv = c;
        for (int r = c + 1; r < n; ++r) {
            if (std::abs(a[r * n + c]) > std::abs(a[piv * n + c])) {
                piv = r;
            }
        }
        if (std::abs(a[piv * n + c]) < 1e-12 * maxAbs) {
            return false;
        }
        if (piv != c) {
            for (int k = 0; k < n; ++k) {
                std::swap(a[c * n + k], a[piv * n + k]);
            }
            std::swap(b[c], b[piv]);
        }
        for (int r = c + 1; r < n; ++r) {
            const double f = a[r * n + c] / a[c * n + c];
            for (int k = c; k < n; ++k) {
                a[r * n + k] -= f * a[c * n + k];
            }
            b[r] -= f * b[c];
        }
    }
    x.assign(n, 0.0);
    for (int r = n - 1; r >= 0; --r) {
        double s = b[r];
        for (int k = r + 1; k < n; ++k) {
            s -= a[r * n + k] * x[k];
        }
        x[r] = s / a[r * n + r];
    }
    return true;
}

/// Eigen-decomposition of a symmetric matrix by cyclic Jacobi rotations. Column k of
/// @p vecs (row-major) is the eigenvector of vals[k]; the values come out ascending.
void symmetricEigen(std::vector<double> a, int n, std::vector<double>& vals,
                    std::vector<double>& vecs) {
    std::vector<double> v(n * n, 0.0);
    for (int i = 0; i < n; ++i) {
        v[i * n + i] = 1.0;
    }
    double total = 0.0;
    for (double e : a) {
        total += e * e;
    }
    for (int sweep = 0; sweep < 100; ++sweep) {
        double off = 0.0;
        for (int p = 0; p < n; ++p) {
            for (int q = p + 1; q < n; ++q) {
                off += a[p * n + q] * a[p * n + q];
            }
        }
        if (off <= 1e-30 * total) {
            break;
        }
        for (int p = 0; p < n; ++p) {
            for (int q = p + 1; q < n; ++q) {
                const double apq = a[p * n + q];
                if (apq == 0.0) {
                    continue;
                }
                const double theta = (a[q * n + q] - a[p * n + p]) / (2.0 * apq);
                const double t = (theta >= 0.0 ? 1.0 : -1.0) /
                                 (std::abs(theta) + std::sqrt(theta * theta + 1.0));
                const double c = 1.0 / std::sqrt(t * t + 1.0);
                const double s = t * c;
                for (int k = 0; k < n; ++k) {
                    const double akp = a[k * n + p];
                    const double akq = a[k * n + q];
                    a[k * n + p] = c * akp - s * akq;
                    a[k * n + q] = s * akp + c * akq;
                }
                for (int k = 0; k < n; ++k) {
                    const double apk = a[p * n + k];
                    const double aqk = a[q * n + k];
                    a[p * n + k] = c * apk - s * aqk;
                    a[q * n + k] = s * apk + c * aqk;
                }
                for (int k = 0; k < n; ++k) {
                    const double vkp = v[k * n + p];
                    const double vkq = v[k * n + q];
                    v[k * n + p] = c * vkp - s * vkq;
                    v[k * n + q] = s * vkp + c * vkq;
                }
            }
        }
    }
    std::vector<int> order(n);
    for (int i = 0; i < n; ++i) {
        order[i] = i;
    }
    std::sort(order.begin(), order.end(),
              [&](int x, int y) { return a[x * n + x] < a[y * n + y]; });
    vals.assign(n, 0.0);
    vecs.assign(n * n, 0.0);
    for (int k = 0; k < n; ++k) {
        vals[k] = a[order[k] * n + order[k]];
        for (int r = 0; r < n; ++r) {
            vecs[r * n + k] = v[r * n + order[k]];
        }
    }
}

// ---------------------------------------------------------------------------
// Samples
// ---------------------------------------------------------------------------

struct Sample {
    Vec3 p;
    Vec3 n;  ///< unit normal; meaningful only when hasNormal
    bool hasNormal = false;
};

/// A kN x kN grid over the domain, its boundary included.
struct Grid {
    std::vector<Sample> samples;
    double size = 0.0;  ///< the diagonal of the samples' bounding box
    Vec3 centroid;
    int validNormals = 0;

    const Sample& at(int i, int j) const { return samples[i * kN + j]; }
};

std::optional<Grid> sample(const NurbsSurface& s) {
    const double u0 = s.uMin();
    const double u1 = s.uMax();
    const double v0 = s.vMin();
    const double v1 = s.vMax();
    if (!(std::isfinite(u0) && std::isfinite(u1) && std::isfinite(v0) && std::isfinite(v1)) ||
        !(u1 > u0) || !(v1 > v0)) {
        return std::nullopt;
    }
    Grid g;
    g.samples.resize(kN * kN);
    std::vector<Vec3> du(kN * kN);
    std::vector<Vec3> dv(kN * kN);
    Vec3 lo{1e300, 1e300, 1e300};
    Vec3 hi{-1e300, -1e300, -1e300};
    for (int i = 0; i < kN; ++i) {
        const double u = i == kN - 1 ? u1 : u0 + (u1 - u0) * i / (kN - 1);
        for (int j = 0; j < kN; ++j) {
            const double v = j == kN - 1 ? v1 : v0 + (v1 - v0) * j / (kN - 1);
            const SurfaceDerivatives d = s.derivatives(u, v);
            if (!finite(d.point) || !finite(d.du) || !finite(d.dv)) {
                return std::nullopt;
            }
            g.samples[i * kN + j].p = d.point;
            du[i * kN + j] = d.du;
            dv[i * kN + j] = d.dv;
            lo = {std::min(lo.x, d.point.x), std::min(lo.y, d.point.y), std::min(lo.z, d.point.z)};
            hi = {std::max(hi.x, d.point.x), std::max(hi.y, d.point.y), std::max(hi.z, d.point.z)};
            g.centroid += d.point;
        }
    }
    g.centroid = g.centroid / double(kN * kN);
    g.size = (hi - lo).length();
    if (!(g.size > 0.0)) {
        return std::nullopt;
    }
    for (int k = 0; k < kN * kN; ++k) {
        const double lu = du[k].length();
        const double lv = dv[k].length();
        const Vec3 c = du[k].cross(dv[k]);
        // Degenerate where a derivative vanishes (a pole, an apex) or du and dv are parallel.
        if (lu > 1e-8 * g.size && lv > 1e-8 * g.size && c.length() > 1e-6 * lu * lv) {
            g.samples[k].n = unit(c);
            g.samples[k].hasNormal = true;
            ++g.validNormals;
        }
    }
    return g;
}

// ---------------------------------------------------------------------------
// Candidates and their verification
// ---------------------------------------------------------------------------

/// A typed surface proposed for a NURBS: its distance and normal in closed form, the
/// axis it revolves about (when it does), and how to build it.
struct Candidate {
    std::function<double(const Vec3&)> distance;
    /// The unit normal at the foot of a point; false where undefined.
    std::function<bool(const Vec3&, Vec3&)> normal;
    std::function<std::unique_ptr<Surface>()> build;
    bool hasAxis = false;
    Vec3 axisOrigin;
    Vec3 axisDir;
    /// A torus's major radius, or negative: the tube's angle is checked as well.
    double ringRadius = -1.0;
};

/// Whether the angles @p angles (radians, in sequence) sweep monotonically, by less than
/// the @p period: a line of the parameter domain that doubles back or wraps twice folds.
bool sweepsOnce(const std::vector<double>& angles) {
    const double pi = math::kPi;
    double total = 0.0;
    double dir = 0.0;
    for (std::size_t k = 1; k < angles.size(); ++k) {
        double step = std::fmod(angles[k] - angles[k - 1], 2.0 * pi);
        if (step > pi) {
            step -= 2.0 * pi;
        } else if (step < -pi) {
            step += 2.0 * pi;
        }
        if (std::abs(step) > 1e-6) {
            const double sgn = step > 0.0 ? 1.0 : -1.0;
            if (dir != 0.0 && sgn != dir) {
                return false;
            }
            dir = sgn;
        }
        total += step;
    }
    return std::abs(total) <= 2.0 * pi + 1e-6;
}

bool noFolding(const Grid& g, const Candidate& c) {
    const Frame f = Frame::fromAxis(c.axisOrigin, c.axisDir);
    const double minRho = 1e-3 * g.size;
    // The angle about the axis, and the tube's angle for a torus, along every grid line.
    for (int pass = 0; pass < (c.ringRadius > 0.0 ? 2 : 1); ++pass) {
        for (int dirn = 0; dirn < 2; ++dirn) {
            for (int a = 0; a < kN; ++a) {
                std::vector<double> angles;
                for (int b = 0; b < kN; ++b) {
                    const Sample& s = dirn == 0 ? g.at(a, b) : g.at(b, a);
                    const Vec3 d = s.p - c.axisOrigin;
                    const double x = d.dot(f.x);
                    const double y = d.dot(f.y);
                    const double h = d.dot(f.z);
                    const double rho = std::hypot(x, y);
                    if (pass == 0) {
                        if (rho > minRho) {
                            angles.push_back(std::atan2(y, x));
                        }
                    } else {
                        angles.push_back(std::atan2(h, rho - c.ringRadius));
                    }
                }
                if (!sweepsOnce(angles)) {
                    return false;
                }
            }
        }
    }
    return true;
}

/// Whether every sample lies on @p c within @p tol, its normal agrees with c's up to a
/// sign that is the same throughout, and the grid does not fold.
bool verify(const Grid& g, const Candidate& c, double tol, double normalTol) {
    double sign = 0.0;
    int checked = 0;
    for (const Sample& s : g.samples) {
        if (!(c.distance(s.p) <= tol)) {
            return false;
        }
        Vec3 nc;
        if (s.hasNormal && c.normal(s.p, nc)) {
            if (s.n.cross(nc).length() > normalTol) {
                return false;
            }
            const double d = s.n.dot(nc);
            if (sign == 0.0) {
                sign = d > 0.0 ? 1.0 : -1.0;
            } else if (d * sign < 0.0) {
                return false;
            }
            ++checked;
        }
    }
    if (checked < kN * kN / 4) {
        return false;
    }
    return !c.hasAxis || noFolding(g, c);
}

/// A frame about @p axis at @p origin whose x points towards @p towards.
Frame frameToward(const Vec3& origin, const Vec3& axis, const Vec3& towards) {
    return Frame::fromAxis(origin, axis, towards - origin);
}

// ---------------------------------------------------------------------------
// Fits
// ---------------------------------------------------------------------------

std::optional<Candidate> fitPlane(const NurbsSurface& s, const Grid& g) {
    std::vector<double> cov(9, 0.0);
    for (const Sample& smp : g.samples) {
        const Vec3 d = (smp.p - g.centroid) / g.size;
        const double c[3] = {d.x, d.y, d.z};
        for (int r = 0; r < 3; ++r) {
            for (int k = 0; k < 3; ++k) {
                cov[r * 3 + k] += c[r] * c[k];
            }
        }
    }
    std::vector<double> vals;
    std::vector<double> vecs;
    symmetricEigen(cov, 3, vals, vecs);
    if (!(vals[1] > 1e-12 * double(g.samples.size()))) {
        return std::nullopt;  // collinear or a point
    }
    Vec3 n = unit({vecs[0], vecs[3], vecs[6]});
    const Sample& mid = g.at(kN / 2, kN / 2);
    if (mid.hasNormal && mid.n.dot(n) < 0.0) {
        n = -n;
    }
    const Vec3 origin = g.at(0, 0).p;
    Vec3 hint = s.derivatives(s.uMin(), s.vMin()).du;
    if (hint.length() < 1e-12 * g.size) {
        hint = g.at(kN - 1, 0).p - origin;
    }
    const Frame frame = Frame::fromAxis(origin, n, hint);
    Candidate c;
    c.distance = [origin, n](const Vec3& p) { return std::abs(n.dot(p - origin)); };
    c.normal = [n](const Vec3&, Vec3& out) {
        out = n;
        return true;
    };
    c.build = [frame]() { return std::make_unique<Plane>(frame); };
    return c;
}

/// Whether the v edge of the grid at column @p j collapses to a point.
bool edgeCollapsed(const Grid& g, int j, double tol) {
    for (int i = 1; i < kN; ++i) {
        if ((g.at(i, j).p - g.at(0, j).p).length() > tol) {
            return false;
        }
    }
    return true;
}

std::optional<Candidate> fitSphere(const NurbsSurface&, const Grid& g) {
    // The centre is nearest the normal lines: sum (I - n n^T) (X - P) = 0.
    std::vector<double> a(9, 0.0);
    std::vector<double> b(3, 0.0);
    int count = 0;
    for (const Sample& smp : g.samples) {
        if (!smp.hasNormal) {
            continue;
        }
        const Vec3 d = (smp.p - g.centroid) / g.size;
        const double n[3] = {smp.n.x, smp.n.y, smp.n.z};
        const double pd[3] = {d.x, d.y, d.z};
        for (int r = 0; r < 3; ++r) {
            for (int k = 0; k < 3; ++k) {
                const double m = (r == k ? 1.0 : 0.0) - n[r] * n[k];
                a[r * 3 + k] += m;
                b[r] += m * pd[k];
            }
        }
        ++count;
    }
    std::vector<double> x;
    if (count < 8 || !solve(a, b, 3, x)) {
        return std::nullopt;
    }
    const Vec3 centre = g.centroid + Vec3{x[0], x[1], x[2]} * g.size;
    double radius = 0.0;
    for (const Sample& smp : g.samples) {
        radius += (smp.p - centre).length();
    }
    radius /= double(g.samples.size());
    if (!(radius > 1e-9 * g.size)) {
        return std::nullopt;
    }

    Candidate c;
    c.distance = [centre, radius](const Vec3& p) {
        return std::abs((p - centre).length() - radius);
    };
    c.normal = [centre](const Vec3& p, Vec3& out) {
        out = unit(p - centre);
        return out.length() > 0.0;
    };
    // The axis through the pole a v edge collapses to, pointing towards the larger v.
    const double collapse = 1e-7 * g.size;
    const bool south = edgeCollapsed(g, 0, collapse);
    const bool north = edgeCollapsed(g, kN - 1, collapse);
    Vec3 axis = unit(g.at(0, kN / 2).p - centre);
    if (south && north) {
        axis = unit(g.at(0, kN - 1).p - g.at(0, 0).p);
        c.hasAxis = true;
    } else if (south) {
        axis = unit(centre - g.at(0, 0).p);
        c.hasAxis = true;
    } else if (north) {
        axis = unit(g.at(0, kN - 1).p - centre);
        c.hasAxis = true;
    }
    c.axisOrigin = centre;
    c.axisDir = axis;
    const Frame frame = frameToward(centre, axis, g.at(0, kN / 2).p);
    c.build = [frame, radius]() { return std::make_unique<Sphere>(frame, radius); };
    return c;
}

/// The axis of a cylinder or cone: the direction the normals, which lie on a circle of
/// the Gauss sphere, are all at the same angle to. @p cosAngle is the mean n . axis.
bool axisFromNormals(const Grid& g, Vec3& axis, double& cosAngle) {
    Vec3 mean;
    int count = 0;
    for (const Sample& s : g.samples) {
        if (s.hasNormal) {
            mean += s.n;
            ++count;
        }
    }
    if (count < 8) {
        return false;
    }
    mean = mean / double(count);
    std::vector<double> cov(9, 0.0);
    for (const Sample& s : g.samples) {
        if (!s.hasNormal) {
            continue;
        }
        const Vec3 d = s.n - mean;
        const double c[3] = {d.x, d.y, d.z};
        for (int r = 0; r < 3; ++r) {
            for (int k = 0; k < 3; ++k) {
                cov[r * 3 + k] += c[r] * c[k] / double(count);
            }
        }
    }
    std::vector<double> vals;
    std::vector<double> vecs;
    symmetricEigen(cov, 3, vals, vecs);
    if (!(vals[1] > 1e-9)) {
        return false;  // the normals do not span a circle: no axis
    }
    axis = unit({vecs[0], vecs[3], vecs[6]});
    cosAngle = mean.dot(axis);
    return true;
}

std::optional<Candidate> fitCylinder(const NurbsSurface&, const Grid& g) {
    Vec3 axis;
    double cosAngle = 0.0;
    if (!axisFromNormals(g, axis, cosAngle)) {
        return std::nullopt;
    }
    // Along v (or u, were the cylinder's parameters the other way round) the axis points
    // the way the parameter grows.
    const Vec3 alongV = g.at(0, kN - 1).p - g.at(0, 0).p;
    const Vec3 alongU = g.at(kN - 1, 0).p - g.at(0, 0).p;
    const double along = std::abs(alongV.dot(axis)) >= std::abs(alongU.dot(axis))
                             ? alongV.dot(axis)
                             : alongU.dot(axis);
    if (along < 0.0) {
        axis = -axis;
    }
    // Kasa's circle fit in the plane perpendicular to the axis.
    const Frame basis = Frame::fromAxis(g.centroid, axis);
    std::vector<double> a(9, 0.0);
    std::vector<double> b(3, 0.0);
    for (const Sample& s : g.samples) {
        const Vec3 d = (s.p - g.centroid) / g.size;
        const double x = d.dot(basis.x);
        const double y = d.dot(basis.y);
        const double row[3] = {2.0 * x, 2.0 * y, 1.0};
        for (int r = 0; r < 3; ++r) {
            for (int k = 0; k < 3; ++k) {
                a[r * 3 + k] += row[r] * row[k];
            }
            b[r] += row[r] * (x * x + y * y);
        }
    }
    std::vector<double> x;
    if (!solve(a, b, 3, x)) {
        return std::nullopt;
    }
    const double r2 = x[2] + x[0] * x[0] + x[1] * x[1];
    if (!(r2 > 0.0)) {
        return std::nullopt;
    }
    const double radius = std::sqrt(r2) * g.size;
    const Vec3 onAxis = g.centroid + (basis.x * x[0] + basis.y * x[1]) * g.size;
    const Vec3 start = g.at(0, 0).p;
    const Vec3 origin = onAxis + axis * (start - onAxis).dot(axis);
    if (!(radius > 1e-9 * g.size)) {
        return std::nullopt;
    }

    Candidate c;
    c.distance = [origin, axis, radius](const Vec3& p) {
        const Vec3 d = p - origin;
        return std::abs((d - axis * d.dot(axis)).length() - radius);
    };
    c.normal = [origin, axis](const Vec3& p, Vec3& out) {
        const Vec3 d = p - origin;
        out = unit(d - axis * d.dot(axis));
        return out.length() > 0.0;
    };
    c.hasAxis = true;
    c.axisOrigin = origin;
    c.axisDir = axis;
    const Frame frame = frameToward(origin, axis, g.at(0, kN / 2).p);
    c.build = [frame, radius]() { return std::make_unique<Cylinder>(frame, radius); };
    return c;
}

std::optional<Candidate> fitCone(const NurbsSurface&, const Grid& g) {
    Vec3 axis;
    double cosAngle = 0.0;
    if (!axisFromNormals(g, axis, cosAngle)) {
        return std::nullopt;
    }
    const double sinHalf = std::abs(cosAngle);  // n . axis = -+ sin(half-angle)
    if (!(sinHalf > 1e-6 && sinHalf < 1.0 - 1e-9)) {
        return std::nullopt;
    }
    const double halfAngle = std::asin(sinHalf);
    // Every tangent plane passes through the apex: n . (X - P) = 0.
    std::vector<double> a(9, 0.0);
    std::vector<double> b(3, 0.0);
    for (const Sample& s : g.samples) {
        if (!s.hasNormal) {
            continue;
        }
        const Vec3 d = (s.p - g.centroid) / g.size;
        const double n[3] = {s.n.x, s.n.y, s.n.z};
        const double nd = s.n.dot(d);
        for (int r = 0; r < 3; ++r) {
            for (int k = 0; k < 3; ++k) {
                a[r * 3 + k] += n[r] * n[k];
            }
            b[r] += n[r] * nd;
        }
    }
    std::vector<double> x;
    if (!solve(a, b, 3, x)) {
        return std::nullopt;
    }
    const Vec3 apex = g.centroid + Vec3{x[0], x[1], x[2]} * g.size;
    // The axis points from the apex into the surface.
    double above = 0.0;
    for (const Sample& s : g.samples) {
        above += (s.p - apex).dot(axis);
    }
    if (above < 0.0) {
        axis = -axis;
    }

    Candidate c;
    const double cosH = std::cos(halfAngle);
    c.distance = [apex, axis, sinHalf, cosH](const Vec3& p) {
        const Vec3 d = p - apex;
        const double h = d.dot(axis);
        const double rho = (d - axis * h).length();
        return std::abs(rho * cosH - std::abs(h) * sinHalf);
    };
    c.normal = [apex, axis, sinHalf, cosH](const Vec3& p, Vec3& out) {
        const Vec3 d = p - apex;
        const double h = d.dot(axis);
        const Vec3 radial = unit(d - axis * h);
        if (radial.length() == 0.0) {
            return false;
        }
        out = radial * cosH - axis * (h >= 0.0 ? sinHalf : -sinHalf);
        return true;
    };
    c.hasAxis = true;
    c.axisOrigin = apex;
    c.axisDir = axis;
    const Frame frame = frameToward(apex, axis, g.at(0, kN / 2).p);
    c.build = [frame, halfAngle]() { return std::make_unique<Cone>(frame, 0.0, halfAngle); };
    return c;
}

std::optional<Candidate> fitTorus(const NurbsSurface&, const Grid& g) {
    // Every normal line meets the axis. A line (direction a, moment m) meets the line
    // through P along n when a . (P x n) + m . n = 0: the axis is the null vector of the
    // sum of w w^T, w = (P x n, n).
    std::vector<double> cov(36, 0.0);
    int count = 0;
    for (const Sample& s : g.samples) {
        if (!s.hasNormal) {
            continue;
        }
        const Vec3 p = (s.p - g.centroid) / g.size;
        const Vec3 pn = p.cross(s.n);
        const double w[6] = {pn.x, pn.y, pn.z, s.n.x, s.n.y, s.n.z};
        for (int r = 0; r < 6; ++r) {
            for (int k = 0; k < 6; ++k) {
                cov[r * 6 + k] += w[r] * w[k];
            }
        }
        ++count;
    }
    if (count < 16) {
        return std::nullopt;
    }
    std::vector<double> vals;
    std::vector<double> vecs;
    symmetricEigen(cov, 6, vals, vecs);
    if (!(vals[1] > 1e-9 * double(count))) {
        return std::nullopt;  // more than one such line: a sphere or a cylinder, not a torus
    }
    Vec3 axis{vecs[0], vecs[6], vecs[12]};
    const Vec3 moment{vecs[18], vecs[24], vecs[30]};
    const double axisLen = axis.length();
    if (!(axisLen > 1e-9)) {
        return std::nullopt;
    }
    const Vec3 onAxis = g.centroid + axis.cross(moment) / (axisLen * axisLen) * g.size;
    axis = axis / axisLen;
    // u runs counter-clockwise about the axis.
    double turning = 0.0;
    for (int j = 0; j < kN; ++j) {
        for (int i = 0; i + 1 < kN; ++i) {
            const Vec3 p0 = g.at(i, j).p;
            const Vec3 p1 = g.at(i + 1, j).p;
            turning += axis.dot((p0 - onAxis).cross(p1 - p0));
        }
    }
    if (turning < 0.0) {
        axis = -axis;
    }
    // Kasa's circle fit of the profile (rho, h): rho^2 + h^2 = 2 R rho + 2 h0 h + C.
    const Frame basis = Frame::fromAxis(onAxis, axis);
    std::vector<double> a(9, 0.0);
    std::vector<double> b(3, 0.0);
    for (const Sample& s : g.samples) {
        const Vec3 d = (s.p - onAxis) / g.size;
        const double rho = std::hypot(d.dot(basis.x), d.dot(basis.y));
        const double h = d.dot(axis);
        const double row[3] = {2.0 * rho, 2.0 * h, 1.0};
        for (int r = 0; r < 3; ++r) {
            for (int k = 0; k < 3; ++k) {
                a[r * 3 + k] += row[r] * row[k];
            }
            b[r] += row[r] * (rho * rho + h * h);
        }
    }
    std::vector<double> x;
    if (!solve(a, b, 3, x)) {
        return std::nullopt;
    }
    const double major = x[0] * g.size;
    const double r2 = x[2] + x[0] * x[0] + x[1] * x[1];
    if (!(major > 0.0 && r2 > 0.0)) {
        return std::nullopt;
    }
    const double minor = std::sqrt(r2) * g.size;
    const Vec3 origin = onAxis + axis * (x[1] * g.size);

    Candidate c;
    c.distance = [origin, axis, major, minor](const Vec3& p) {
        const Vec3 d = p - origin;
        const double h = d.dot(axis);
        const double rho = (d - axis * h).length();
        return std::abs(std::hypot(rho - major, h) - minor);
    };
    c.normal = [origin, axis, major](const Vec3& p, Vec3& out) {
        const Vec3 d = p - origin;
        const double h = d.dot(axis);
        const Vec3 radial = unit(d - axis * h);
        if (radial.length() == 0.0) {
            return false;
        }
        const Vec3 n = radial * ((d - axis * h).length() - major) + axis * h;
        out = unit(n);
        return out.length() > 0.0;
    };
    c.hasAxis = true;
    c.axisOrigin = origin;
    c.axisDir = axis;
    c.ringRadius = major;
    const Frame frame = frameToward(origin, axis, g.at(0, kN / 2).p);
    c.build = [frame, major, minor]() { return std::make_unique<Torus>(frame, major, minor); };
    return c;
}

}  // namespace

std::unique_ptr<Surface> recognize(const NurbsSurface& surface, double tolerance) {
    try {
        if (!(tolerance > 0.0) || !std::isfinite(tolerance)) {
            return nullptr;
        }
        const std::optional<Grid> grid = sample(surface);
        if (!grid) {
            return nullptr;
        }
        const Grid& g = *grid;
        const double normalTol = 1e-6 + 100.0 * tolerance / g.size;
        using Fit = std::optional<Candidate> (*)(const NurbsSurface&, const Grid&);
        const Fit fits[] = {fitPlane, fitSphere, fitCylinder, fitCone, fitTorus};
        for (const Fit fit : fits) {
            const std::optional<Candidate> candidate = fit(surface, g);
            if (candidate && verify(g, *candidate, tolerance, normalTol)) {
                return candidate->build();
            }
        }
    } catch (...) {
        // Degenerate input must not throw: it is not any kind.
    }
    return nullptr;
}

}  // namespace hz::geo
