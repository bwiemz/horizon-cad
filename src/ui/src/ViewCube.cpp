#include "horizon/ui/ViewCube.h"

#include <QColor>
#include <QFont>
#include <QPainter>
#include <QPoint>
#include <QPointF>
#include <QRectF>
#include <algorithm>
#include <array>
#include <cmath>

#include "horizon/math/Vec3.h"
#include "horizon/render/Camera.h"

namespace hz::ui {

namespace {

constexpr double kCubeRadius = 32.0;  // half-extent of the cube silhouette, px
constexpr double kMargin = 18.0;      // gap from the top-right viewport corner, px
constexpr double kEps = 1e-6;

// Camera basis: world -> view (screen-right / screen-up / into-screen).
struct Basis {
    math::Vec3 right;
    math::Vec3 up;
    math::Vec3 fwd;
};

Basis cameraBasis(const render::Camera& cam) {
    math::Vec3 fwd = (cam.target() - cam.eye()).normalized();
    math::Vec3 right = fwd.cross(cam.up());
    if (right.lengthSquared() < 1e-9) {
        // Camera looks (nearly) along its own up vector; pick any perpendicular
        // reference so the basis stays well-defined.
        const math::Vec3 ref = std::abs(fwd.z) < 0.9 ? math::Vec3(0, 0, 1) : math::Vec3(0, 1, 0);
        right = fwd.cross(ref);
    }
    right = right.normalized();
    const math::Vec3 up = right.cross(fwd);
    return {right, up, fwd};
}

struct FaceDef {
    math::Vec3 normal;  // outward face normal (also the face centre, cube is [-1,1]^3)
    math::Vec3 uAxis;   // first in-plane axis
    math::Vec3 vAxis;   // second in-plane axis
    ViewCube::Region region;
    const char* label;
};

const std::array<FaceDef, 6> kFaces = {{
    {{-1, 0, 0}, {0, 1, 0}, {0, 0, 1}, ViewCube::Region::Left, "LEFT"},
    {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}, ViewCube::Region::Right, "RIGHT"},
    {{0, -1, 0}, {1, 0, 0}, {0, 0, 1}, ViewCube::Region::Front, "FRONT"},
    {{0, 1, 0}, {1, 0, 0}, {0, 0, 1}, ViewCube::Region::Back, "BACK"},
    {{0, 0, 1}, {1, 0, 0}, {0, 1, 0}, ViewCube::Region::Top, "TOP"},
    {{0, 0, -1}, {1, 0, 0}, {0, 1, 0}, ViewCube::Region::Bottom, "BOTTOM"},
}};

QPointF project(const math::Vec3& v, const Basis& b, const QPointF& center, double r) {
    const double sx = b.right.dot(v);
    const double sy = b.up.dot(v);
    return {center.x() + sx * r, center.y() - sy * r};  // flip Y for Qt screen coords
}

}  // namespace

void ViewCube::paint(QPainter& painter, int vpW, int vpH, const render::Camera& camera) {
    m_hits.clear();
    if (vpW <= 0 || vpH <= 0) return;

    const Basis basis = cameraBasis(camera);
    const QPointF center(vpW - kMargin - kCubeRadius, kMargin + kCubeRadius);

    painter.save();
    painter.setRenderHint(QPainter::Antialiasing, true);

    // Collect the visible faces (front-facing toward the viewer), nearest last.
    struct DrawFace {
        QPolygonF poly;
        QPointF labelPos;
        double depth;
        ViewCube::Region region;
        const char* label;
    };
    std::vector<DrawFace> faces;
    for (const FaceDef& f : kFaces) {
        // Visible if the outward normal points against the view direction.
        if (basis.fwd.dot(f.normal) > -kEps) continue;

        const math::Vec3 corners[4] = {
            f.normal + f.uAxis + f.vAxis,
            f.normal - f.uAxis + f.vAxis,
            f.normal - f.uAxis - f.vAxis,
            f.normal + f.uAxis - f.vAxis,
        };
        QPolygonF poly;
        for (const math::Vec3& c : corners) poly << project(c, basis, center, kCubeRadius);

        faces.push_back({poly, project(f.normal, basis, center, kCubeRadius),
                         basis.fwd.dot(f.normal), f.region, f.label});
    }
    // Painter's algorithm: farthest (largest depth) first so nearer faces win.
    std::sort(faces.begin(), faces.end(),
              [](const DrawFace& a, const DrawFace& b) { return a.depth > b.depth; });

    const QColor faceFill(58, 64, 76, 235);
    const QColor faceFillNear(74, 84, 100, 245);
    const QColor edge(120, 130, 146);
    const QColor textCol(214, 219, 226);

    QFont font("Arial", 8);
    font.setBold(true);
    painter.setFont(font);

    for (size_t i = 0; i < faces.size(); ++i) {
        const DrawFace& df = faces[i];
        // The last (nearest) face is tinted a touch brighter for depth cueing.
        painter.setBrush(i + 1 == faces.size() ? faceFillNear : faceFill);
        painter.setPen(QPen(edge, 1.2));
        painter.drawPolygon(df.poly);

        painter.setPen(textCol);
        QRectF lr(df.labelPos.x() - 26, df.labelPos.y() - 8, 52, 16);
        painter.drawText(lr, Qt::AlignCenter, QString::fromLatin1(df.label));

        // Cache the screen polygon for hit-testing.
        m_hits.push_back({df.region, df.poly, df.depth});
    }

    // Home / isometric button beneath the cube.
    const double bw = 46, bh = 16;
    QRectF homeRect(center.x() - bw / 2.0, center.y() + kCubeRadius + 8.0, bw, bh);
    painter.setBrush(QColor(48, 53, 63, 235));
    painter.setPen(QPen(edge, 1.0));
    painter.drawRoundedRect(homeRect, 3.0, 3.0);
    painter.setPen(textCol);
    QFont hf("Arial", 8);
    hf.setBold(true);
    painter.setFont(hf);
    painter.drawText(homeRect, Qt::AlignCenter, QStringLiteral("HOME"));
    m_hits.push_back({Region::Iso, QPolygonF(homeRect), -1e9});

    painter.restore();
}

ViewCube::Region ViewCube::hitTest(const QPoint& pos) const {
    const QPointF p(pos);
    Region best = Region::None;
    double bestDepth = 1e30;
    for (const Hit& h : m_hits) {
        if (h.poly.containsPoint(p, Qt::OddEvenFill) && h.depth < bestDepth) {
            bestDepth = h.depth;
            best = h.region;
        }
    }
    return best;
}

QString ViewCube::orientationLabel(const render::Camera& camera) {
    const math::Vec3 fwd = (camera.target() - camera.eye()).normalized();
    constexpr double t = 0.999;
    if (fwd.dot({0, 1, 0}) > t) return QStringLiteral("Front");
    if (fwd.dot({0, -1, 0}) > t) return QStringLiteral("Back");
    if (fwd.dot({0, 0, -1}) > t) return QStringLiteral("Top");
    if (fwd.dot({0, 0, 1}) > t) return QStringLiteral("Bottom");
    if (fwd.dot({-1, 0, 0}) > t) return QStringLiteral("Right");
    if (fwd.dot({1, 0, 0}) > t) return QStringLiteral("Left");
    return QStringLiteral("Isometric");
}

}  // namespace hz::ui
