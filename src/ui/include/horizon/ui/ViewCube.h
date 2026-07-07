#pragma once

#include <QPolygonF>
#include <QString>
#include <vector>

class QPainter;
class QPoint;

namespace hz::render {
class Camera;
}

namespace hz::ui {

/// A SolidWorks/Onshape-style orientation gizmo drawn in the top-right corner
/// of the viewport. It reflects the current camera orientation and, when a face
/// or the home button is clicked, reports which standard view to snap to.
///
/// Rendering and hit-testing share the same projection: paint() caches the
/// screen-space polygon of every visible region, and hitTest() consults that
/// cache. paint() runs each frame before any click is dispatched, so the cache
/// is always current.
class ViewCube {
public:
    enum class Region { None, Front, Back, Left, Right, Top, Bottom, Iso };

    /// Draw the cube and its home/iso button into @p painter (which must cover
    /// the whole viewport; @p vpW / @p vpH are its pixel size).
    void paint(QPainter& painter, int vpW, int vpH, const render::Camera& camera);

    /// Region under @p pos using the polygons cached by the last paint(), or
    /// Region::None if the point misses the gizmo.
    Region hitTest(const QPoint& pos) const;

    /// Human-readable name of the current camera orientation ("Front", "Top",
    /// "Isometric", ...) for the view-mode badge.
    static QString orientationLabel(const render::Camera& camera);

private:
    struct Hit {
        Region region;
        QPolygonF poly;
        double depth;  ///< face-centre view depth; smaller = nearer the viewer
    };
    std::vector<Hit> m_hits;  ///< rebuilt every paint(); consulted by hitTest()
};

}  // namespace hz::ui
