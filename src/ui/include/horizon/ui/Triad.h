#pragma once

#include <QPointF>
#include <array>
#include <optional>
#include <vector>

#include "horizon/math/Mat4.h"
#include "horizon/math/Vec3.h"
#include "horizon/render/Camera.h"

namespace hz::ui {

/// A component's triad (Phase 158): an arrow along each of its axes, and a
/// ring about each, at its middle, drawn the same size on screen however
/// near or far the view. Its geometry is worked out from the camera whenever
/// it is asked for, never kept from a paint: a view that is never painted
/// (a test's) finds its handles all the same.
class Triad {
public:
    enum class Kind { Arrow, Ring };
    struct Handle {
        Kind kind = Kind::Arrow;
        int axis = 0;  ///< 0, 1, 2: the component's x, y, z
        bool operator==(const Handle&) const = default;
    };

    /// How long the arrows are on screen, in pixels; the rings' radius is
    /// kRingShare of it.
    static constexpr double kPixels = 90.0;
    static constexpr double kRingShare = 0.75;
    /// How near a press must be to a handle, in pixels.
    static constexpr double kTolerance = 7.0;

    /// A triad at @p origin along @p axes (unit, the component's), seen by
    /// @p camera in a view @p width x @p height.
    Triad(const math::Vec3& origin, const std::array<math::Vec3, 3>& axes,
          const render::Camera& camera, int width, int height);

    const math::Vec3& origin() const { return m_origin; }
    const math::Vec3& axis(int k) const { return m_axes.at(static_cast<size_t>(k)); }
    /// The arrows' length in the world, for this view.
    double size() const { return m_size; }

    /// The handle nearest @p at, within kTolerance pixels; nothing when none
    /// is.
    std::optional<Handle> hitTest(const QPointF& at) const;
    /// A point on @p handle, on screen: along an arrow, round a ring (a
    /// third of the way to the next axis). What a press on it hits.
    QPointF handlePoint(const Handle& handle) const;
    /// @p handle as line segments in the world, endpoints in pairs (x, y, z
    /// each): what is drawn.
    std::vector<math::Vec3> segments(const Handle& handle) const;

private:
    QPointF toScreen(const math::Vec3& world) const;

    math::Vec3 m_origin;
    std::array<math::Vec3, 3> m_axes;
    math::Mat4 m_viewProjection;
    int m_width;
    int m_height;
    double m_size = 1.0;
};

}  // namespace hz::ui
