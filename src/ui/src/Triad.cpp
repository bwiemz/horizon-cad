#include "horizon/ui/Triad.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <numbers>

#include "horizon/math/Vec4.h"

namespace hz::ui {

using math::Vec3;

namespace {

/// Points round a ring, and the first again: its chords.
constexpr int kRingChords = 48;

/// How far @p p is from the segment @p a – @p b, on screen.
double distanceToSegment(const QPointF& p, const QPointF& a, const QPointF& b) {
    const QPointF ab = b - a;
    const double length2 = QPointF::dotProduct(ab, ab);
    double t = length2 > 0.0 ? QPointF::dotProduct(p - a, ab) / length2 : 0.0;
    t = std::clamp(t, 0.0, 1.0);
    const QPointF off = p - (a + ab * t);
    return std::hypot(off.x(), off.y());
}

}  // namespace

Triad::Triad(const Vec3& origin, const std::array<Vec3, 3>& axes, const render::Camera& camera,
             int width, int height)
    : m_origin(origin),
      m_axes(axes),
      m_viewProjection(camera.viewProjectionMatrix()),
      m_width(width),
      m_height(height) {
    // The world length that is kPixels on screen at the origin: measured
    // across the view, where nothing foreshortens it.
    const Vec3 forward = (camera.target() - camera.eye()).normalized();
    Vec3 across = forward.cross(camera.up());
    if (across.length() < 1e-12) across = forward.cross(Vec3::UnitX);
    if (across.length() < 1e-12) across = forward.cross(Vec3::UnitY);
    across = across.normalized();
    const auto a = project(m_origin);
    const auto b = project(m_origin + across);
    if (!a || !b) return;  // its middle behind the view: not visible
    m_visible = true;
    const double pixelsPerUnit = std::hypot(b->x() - a->x(), b->y() - a->y());
    m_size = pixelsPerUnit > 1e-12 ? kPixels / pixelsPerUnit : 1.0;
}

std::optional<QPointF> Triad::project(const Vec3& world) const {
    const math::Vec4 clip = m_viewProjection * math::Vec4(world, 1.0);
    if (!(clip.w > 1e-12)) return std::nullopt;  // behind the eye, as a pick sees it
    const Vec3 ndc = clip.perspectiveDivide();
    return QPointF((ndc.x + 1.0) * 0.5 * m_width, (1.0 - ndc.y) * 0.5 * m_height);
}

QPointF Triad::toScreen(const Vec3& world) const {
    return project(world).value_or(QPointF(-1e9, -1e9));  // far off: nothing is there
}

std::vector<Vec3> Triad::segments(const Handle& handle) const {
    const Vec3& a = axis(handle.axis);
    std::vector<Vec3> out;
    if (handle.kind == Kind::Arrow) {
        const Vec3 tip = m_origin + a * m_size;
        out = {m_origin, tip};
        // A head: two barbs back from the tip, across the arrow.
        const Vec3& side = axis((handle.axis + 1) % 3);
        const double barb = 0.12 * m_size;
        out.insert(out.end(), {tip, tip - a * barb + side * (barb * 0.5), tip,
                               tip - a * barb - side * (barb * 0.5)});
        return out;
    }
    // A ring about the axis: in the plane of the other two.
    const Vec3& u = axis((handle.axis + 1) % 3);
    const Vec3& v = axis((handle.axis + 2) % 3);
    const double r = kRingShare * m_size;
    Vec3 previous = m_origin + u * r;
    for (int k = 1; k <= kRingChords; ++k) {
        const double t = 2.0 * std::numbers::pi * k / kRingChords;
        const Vec3 next = m_origin + (u * std::cos(t) + v * std::sin(t)) * r;
        out.insert(out.end(), {previous, next});
        previous = next;
    }
    return out;
}

std::optional<Triad::Handle> Triad::hitTest(const QPointF& at) const {
    if (!m_visible) return std::nullopt;
    std::optional<Handle> best;
    double nearest = kTolerance;
    for (const Kind kind : {Kind::Arrow, Kind::Ring}) {
        for (int k = 0; k < 3; ++k) {
            const Handle handle{kind, k};
            const std::vector<Vec3> points = segments(handle);
            for (size_t i = 0; i + 1 < points.size(); i += 2) {
                const auto a = project(points[i]);
                const auto b = project(points[i + 1]);
                if (!a || !b) continue;  // behind the view: not there to take
                const double d = distanceToSegment(at, *a, *b);
                // Arrows are looked at first, so on a tie an arrow keeps it:
                // it is the thinner mark.
                if (d < nearest) {
                    nearest = d;
                    best = handle;
                }
            }
        }
    }
    return best;
}

QPointF Triad::handlePoint(const Handle& handle) const {
    const Vec3& a = axis(handle.axis);
    if (handle.kind == Kind::Arrow) return toScreen(m_origin + a * (0.7 * m_size));
    const Vec3& u = axis((handle.axis + 1) % 3);
    const Vec3& v = axis((handle.axis + 2) % 3);
    const double t = std::numbers::pi / 6.0;  // a third of the way to v
    return toScreen(m_origin + (u * std::cos(t) + v * std::sin(t)) * (kRingShare * m_size));
}

}  // namespace hz::ui
