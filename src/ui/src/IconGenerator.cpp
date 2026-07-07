#include "horizon/ui/IconGenerator.h"

#include <QColor>
#include <QFont>
#include <QHash>
#include <QImage>
#include <QPainter>
#include <QPainterPath>
#include <QPen>
#include <QPixmap>
#include <QPolygonF>
#include <cmath>
#include <functional>

namespace hz::ui {

// ---------------------------------------------------------------------------
// Color palette
// ---------------------------------------------------------------------------
static const QColor kPrimary(192, 192, 192);    // light gray — geometry lines
static const QColor kAccent(74, 144, 217);      // blue — points / arrows
static const QColor kSecondary(160, 160, 160);  // medium gray — dimension lines
static const QColor kDimText(200, 200, 200);    // light — dimension text
static const QColor kGreen(100, 200, 100);      // green — for positive indicators
static const QColor kRed(220, 100, 100);        // red — for trim / delete indicators

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------
static QPen primaryPen(qreal w = 1.8) {
    QPen p(kPrimary, w, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin);
    return p;
}

static QPen accentPen(qreal w = 1.8) {
    QPen p(kAccent, w, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin);
    return p;
}

static QPen secondaryPen(qreal w = 1.5) {
    QPen p(kSecondary, w, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin);
    return p;
}

static QPen dashedPen(const QColor& c, qreal w = 1.2) {
    QPen p(c, w, Qt::DashLine, Qt::RoundCap, Qt::RoundJoin);
    return p;
}

/// Prepare a transparent image for icon painting.
/// Uses QImage instead of QPixmap to avoid Qt 6.10 qpixmap_win.cpp assertion
/// ("bm.format() == QImage::Format_Mono") on Windows.
static QImage createImage(int s) {
    QImage img(s, s, QImage::Format_ARGB32_Premultiplied);
    img.fill(Qt::transparent);
    return img;
}

/// Configure standard render hints on a painter and scale its coordinate
/// system so every icon can be drawn in a fixed 24-unit design space yet
/// rasterize crisply at any requested pixel size @p s.
static void initPainter(QPainter& p, int s) {
    p.setRenderHint(QPainter::Antialiasing, true);
    p.setRenderHint(QPainter::TextAntialiasing, true);
    const qreal scale = s / 24.0;
    p.scale(scale, scale);
}

/// Draw a small filled circle (dot).
static void drawDot(QPainter& p, qreal x, qreal y, qreal r, const QColor& c) {
    p.setPen(Qt::NoPen);
    p.setBrush(c);
    p.drawEllipse(QPointF(x, y), r, r);
}

/// Draw a small arrowhead at `tip` pointing in direction `angle` (radians).
static void drawArrowhead(QPainter& p, QPointF tip, qreal angle, qreal len, const QColor& c) {
    const qreal halfAngle = 0.45;
    QPointF p1 =
        tip - QPointF(std::cos(angle - halfAngle) * len, std::sin(angle - halfAngle) * len);
    QPointF p2 =
        tip - QPointF(std::cos(angle + halfAngle) * len, std::sin(angle + halfAngle) * len);
    p.setPen(Qt::NoPen);
    p.setBrush(c);
    QPolygonF tri;
    tri << tip << p1 << p2;
    p.drawPolygon(tri);
}

// ===========================================================================
//  icon() dispatcher
// ===========================================================================
QIcon IconGenerator::icon(const QString& name, int size) {
    using DrawFn = std::function<QIcon(int)>;
    static const QHash<QString, DrawFn> dispatch = {
        // File
        {"new", drawNew},
        {"open", drawOpen},
        {"save", drawSave},
        {"undo", drawUndo},
        {"redo", drawRedo},
        // Edit
        {"copy", drawCopy},
        {"paste", drawPaste},
        {"duplicate", drawDuplicate},
        {"group", drawGroup},
        {"ungroup", drawUngroup},
        // Tools
        {"select", drawSelect},
        {"line", drawLine},
        {"circle", drawCircle},
        {"arc", drawArc},
        {"rectangle", drawRectangle},
        {"polyline", drawPolyline},
        {"ellipse", drawEllipse},
        {"spline", drawSpline},
        {"text", drawText},
        {"hatch", drawHatch},
        // Modify
        {"move", drawMove},
        {"offset", drawOffset},
        {"mirror", drawMirror},
        {"rotate", drawRotate},
        {"scale", drawScale},
        {"trim", drawTrim},
        {"fillet", drawFillet},
        {"chamfer", drawChamfer},
        {"break", drawBreak},
        {"extend", drawExtend},
        {"stretch", drawStretch},
        {"polyline-edit", drawPolylineEdit},
        {"rect-array", drawRectArray},
        {"polar-array", drawPolarArray},
        // Dimension
        {"dim-linear", drawDimLinear},
        {"dim-radial", drawDimRadial},
        {"dim-angular", drawDimAngular},
        {"leader", drawLeader},
        // Constraint
        {"cstr-coincident", drawCstrCoincident},
        {"cstr-horizontal", drawCstrHorizontal},
        {"cstr-vertical", drawCstrVertical},
        {"cstr-perpendicular", drawCstrPerpendicular},
        {"cstr-parallel", drawCstrParallel},
        {"cstr-tangent", drawCstrTangent},
        {"cstr-equal", drawCstrEqual},
        {"cstr-fixed", drawCstrFixed},
        {"cstr-distance", drawCstrDistance},
        {"cstr-angle", drawCstrAngle},
        // Measure
        {"measure-distance", drawMeasureDistance},
        {"measure-angle", drawMeasureAngle},
        {"measure-area", drawMeasureArea},
        // Block
        {"block-create", drawBlockCreate},
        {"block-insert", drawBlockInsert},
        {"block-explode", drawBlockExplode},
        // View
        {"fit-all", drawFitAll},
        // 3D — primitives
        {"box", drawBox},
        {"cylinder", drawCylinder},
        {"sphere", drawSphere},
        {"cone", drawCone},
        {"torus", drawTorus},
        // 3D — features
        {"extrude", drawExtrude},
        {"revolve", drawRevolve},
        // 3D — boolean
        {"boolean-union", drawBooleanUnion},
        {"boolean-subtract", drawBooleanSubtract},
        {"boolean-intersect", drawBooleanIntersect},
        // 3D — modify
        {"fillet-3d", drawFillet3d},
        {"chamfer-3d", drawChamfer3d},
    };

    const int s = size > 0 ? size : kRenderSize;
    auto it = dispatch.find(name);
    if (it != dispatch.end()) {
        return it.value()(s);
    }
    // Return blank icon for unrecognized name
    return QIcon(QPixmap::fromImage(createImage(s)));
}

// ===========================================================================
//  FILE icons
// ===========================================================================

QIcon IconGenerator::drawNew(int s) {
    QImage img = createImage(s);
    QPainter p(&img);
    initPainter(p, s);
    // Blank page with folded corner
    qreal l = 5, t = 2, r = 17, b = 22, fold = 5;
    QPainterPath path;
    path.moveTo(l, t);
    path.lineTo(r - fold, t);
    path.lineTo(r, t + fold);
    path.lineTo(r, b);
    path.lineTo(l, b);
    path.closeSubpath();
    p.setPen(primaryPen());
    p.setBrush(Qt::NoBrush);
    p.drawPath(path);
    // Fold line
    p.drawLine(QPointF(r - fold, t), QPointF(r - fold, t + fold));
    p.drawLine(QPointF(r - fold, t + fold), QPointF(r, t + fold));
    // Plus sign in center
    p.setPen(accentPen(2.0));
    qreal cx = 11, cy = 14;
    p.drawLine(QPointF(cx - 3, cy), QPointF(cx + 3, cy));
    p.drawLine(QPointF(cx, cy - 3), QPointF(cx, cy + 3));
    p.end();
    return QIcon(QPixmap::fromImage(img));
}

QIcon IconGenerator::drawOpen(int s) {
    QImage img = createImage(s);
    QPainter p(&img);
    initPainter(p, s);
    // Open folder shape
    QPainterPath path;
    path.moveTo(3, 8);
    path.lineTo(3, 20);
    path.lineTo(19, 20);
    path.lineTo(21, 10);
    path.lineTo(10, 10);
    path.lineTo(8, 8);
    path.closeSubpath();
    p.setPen(primaryPen());
    p.setBrush(Qt::NoBrush);
    p.drawPath(path);
    // Folder tab
    p.drawLine(QPointF(3, 8), QPointF(3, 5));
    p.drawLine(QPointF(3, 5), QPointF(9, 5));
    p.drawLine(QPointF(9, 5), QPointF(10, 8));
    // Arrow pointing up (opening)
    p.setPen(accentPen(1.8));
    p.drawLine(QPointF(12, 18), QPointF(12, 13));
    drawArrowhead(p, QPointF(12, 13), -M_PI / 2.0, 3.5, kAccent);
    p.end();
    return QIcon(QPixmap::fromImage(img));
}

QIcon IconGenerator::drawSave(int s) {
    QImage img = createImage(s);
    QPainter p(&img);
    initPainter(p, s);
    // Floppy disk outline
    QPainterPath path;
    path.moveTo(3, 3);
    path.lineTo(19, 3);
    path.lineTo(21, 5);
    path.lineTo(21, 21);
    path.lineTo(3, 21);
    path.closeSubpath();
    p.setPen(primaryPen());
    p.setBrush(Qt::NoBrush);
    p.drawPath(path);
    // Label area at top
    p.drawRect(QRectF(7, 3, 10, 6));
    // Storage area at bottom
    p.drawRect(QRectF(6, 14, 12, 7));
    // Small detail line on label
    p.setPen(accentPen(1.5));
    p.drawLine(QPointF(14, 4.5), QPointF(14, 8));
    p.end();
    return QIcon(QPixmap::fromImage(img));
}

QIcon IconGenerator::drawUndo(int s) {
    QImage img = createImage(s);
    QPainter p(&img);
    initPainter(p, s);
    // Curved arrow going left / counter-clockwise
    QPainterPath arc;
    arc.moveTo(7, 12);
    arc.cubicTo(7, 6, 12, 4, 18, 7);
    arc.cubicTo(21, 9, 21, 14, 17, 17);
    p.setPen(primaryPen(2.0));
    p.setBrush(Qt::NoBrush);
    p.drawPath(arc);
    // Arrowhead at the start going left-down
    drawArrowhead(p, QPointF(7, 12), M_PI * 0.65, 4.5, kPrimary);
    p.end();
    return QIcon(QPixmap::fromImage(img));
}

QIcon IconGenerator::drawRedo(int s) {
    QImage img = createImage(s);
    QPainter p(&img);
    initPainter(p, s);
    // Curved arrow going right / clockwise
    QPainterPath arc;
    arc.moveTo(17, 12);
    arc.cubicTo(17, 6, 12, 4, 6, 7);
    arc.cubicTo(3, 9, 3, 14, 7, 17);
    p.setPen(primaryPen(2.0));
    p.setBrush(Qt::NoBrush);
    p.drawPath(arc);
    // Arrowhead at the start going right-down
    drawArrowhead(p, QPointF(17, 12), M_PI * 0.35, 4.5, kPrimary);
    p.end();
    return QIcon(QPixmap::fromImage(img));
}

// ===========================================================================
//  EDIT icons
// ===========================================================================

QIcon IconGenerator::drawCopy(int s) {
    QImage img = createImage(s);
    QPainter p(&img);
    initPainter(p, s);
    // Two overlapping rectangles
    p.setPen(secondaryPen(1.5));
    p.setBrush(Qt::NoBrush);
    p.drawRect(QRectF(7, 2, 12, 14));
    p.setPen(primaryPen());
    p.drawRect(QRectF(3, 6, 12, 14));
    // Lines on front page
    p.setPen(QPen(kSecondary, 1.0));
    p.drawLine(QPointF(6, 11), QPointF(12, 11));
    p.drawLine(QPointF(6, 14), QPointF(12, 14));
    p.drawLine(QPointF(6, 17), QPointF(10, 17));
    p.end();
    return QIcon(QPixmap::fromImage(img));
}

QIcon IconGenerator::drawPaste(int s) {
    QImage img = createImage(s);
    QPainter p(&img);
    initPainter(p, s);
    // Clipboard body
    p.setPen(primaryPen());
    p.setBrush(Qt::NoBrush);
    p.drawRect(QRectF(4, 4, 16, 18));
    // Clipboard clip at top
    p.drawRect(QRectF(8, 2, 8, 4));
    // Lines on clipboard
    p.setPen(QPen(kSecondary, 1.0));
    p.drawLine(QPointF(7, 10), QPointF(17, 10));
    p.drawLine(QPointF(7, 13), QPointF(17, 13));
    p.drawLine(QPointF(7, 16), QPointF(14, 16));
    // Accent dot for clip
    drawDot(p, 12, 4, 1.5, kAccent);
    p.end();
    return QIcon(QPixmap::fromImage(img));
}

QIcon IconGenerator::drawDuplicate(int s) {
    QImage img = createImage(s);
    QPainter p(&img);
    initPainter(p, s);
    // Original rectangle (dashed)
    p.setPen(dashedPen(kSecondary, 1.2));
    p.setBrush(Qt::NoBrush);
    p.drawRect(QRectF(3, 6, 10, 10));
    // Duplicated rectangle offset (solid)
    p.setPen(primaryPen());
    p.drawRect(QRectF(9, 3, 10, 10));
    // Plus sign
    p.setPen(accentPen(2.0));
    p.drawLine(QPointF(17, 17), QPointF(17, 22));
    p.drawLine(QPointF(14.5, 19.5), QPointF(19.5, 19.5));
    p.end();
    return QIcon(QPixmap::fromImage(img));
}

QIcon IconGenerator::drawGroup(int s) {
    QImage img = createImage(s);
    QPainter p(&img);
    initPainter(p, s);
    // Two overlapping rounded rectangles
    p.setPen(primaryPen(1.5));
    p.setBrush(Qt::NoBrush);
    p.drawRoundedRect(QRectF(2, 6, 10, 10), 2, 2);
    p.drawRoundedRect(QRectF(12, 4, 10, 10), 2, 2);
    // Green connector line between them
    p.setPen(QPen(kGreen, 2.0, Qt::SolidLine, Qt::RoundCap));
    p.drawLine(QPointF(12, 11), QPointF(12, 11));
    // Green bracket / brace encompassing both
    p.setPen(QPen(kGreen, 1.5, Qt::SolidLine, Qt::RoundCap));
    p.drawLine(QPointF(4, 18), QPointF(20, 18));
    p.drawLine(QPointF(4, 18), QPointF(4, 16));
    p.drawLine(QPointF(20, 18), QPointF(20, 14));
    // Green dot at center of bracket
    drawDot(p, 12, 18, 2.0, kGreen);
    p.end();
    return QIcon(QPixmap::fromImage(img));
}

QIcon IconGenerator::drawUngroup(int s) {
    QImage img = createImage(s);
    QPainter p(&img);
    initPainter(p, s);
    // Two separated rectangles
    p.setPen(primaryPen(1.5));
    p.setBrush(Qt::NoBrush);
    p.drawRoundedRect(QRectF(2, 6, 8, 8), 2, 2);
    p.drawRoundedRect(QRectF(14, 6, 8, 8), 2, 2);
    // Red slash through the connection
    p.setPen(QPen(kRed, 2.0, Qt::SolidLine, Qt::RoundCap));
    p.drawLine(QPointF(8, 18), QPointF(16, 16));
    // Broken bracket
    p.setPen(QPen(kRed, 1.5, Qt::DashLine, Qt::RoundCap));
    p.drawLine(QPointF(4, 18), QPointF(20, 18));
    p.end();
    return QIcon(QPixmap::fromImage(img));
}

// ===========================================================================
//  TOOL icons
// ===========================================================================

QIcon IconGenerator::drawSelect(int s) {
    QImage img = createImage(s);
    QPainter p(&img);
    initPainter(p, s);
    // Cursor arrow pointing up-left
    QPainterPath arrow;
    arrow.moveTo(5, 3);
    arrow.lineTo(5, 19);
    arrow.lineTo(9, 15);
    arrow.lineTo(14, 21);
    arrow.lineTo(16, 19);
    arrow.lineTo(11, 13);
    arrow.lineTo(16, 13);
    arrow.closeSubpath();
    p.setPen(QPen(kPrimary, 1.2, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin));
    p.setBrush(QColor(74, 144, 217, 80));
    p.drawPath(arrow);
    p.end();
    return QIcon(QPixmap::fromImage(img));
}

QIcon IconGenerator::drawLine(int s) {
    QImage img = createImage(s);
    QPainter p(&img);
    initPainter(p, s);
    p.setPen(primaryPen(2.0));
    p.drawLine(QPointF(4, 20), QPointF(20, 4));
    drawDot(p, 4, 20, 2.0, kAccent);
    drawDot(p, 20, 4, 2.0, kAccent);
    p.end();
    return QIcon(QPixmap::fromImage(img));
}

QIcon IconGenerator::drawCircle(int s) {
    QImage img = createImage(s);
    QPainter p(&img);
    initPainter(p, s);
    p.setPen(primaryPen(2.0));
    p.setBrush(Qt::NoBrush);
    p.drawEllipse(QPointF(12, 12), 8.5, 8.5);
    drawDot(p, 12, 12, 2.0, kAccent);
    p.end();
    return QIcon(QPixmap::fromImage(img));
}

QIcon IconGenerator::drawArc(int s) {
    QImage img = createImage(s);
    QPainter p(&img);
    initPainter(p, s);
    p.setPen(primaryPen(2.0));
    p.setBrush(Qt::NoBrush);
    // Arc from ~225 to ~360+45 degrees (spanning lower-left quadrant)
    QRectF arcRect(2, 2, 20, 20);
    p.drawArc(arcRect, 30 * 16, 120 * 16);
    // Endpoint dots
    qreal cx = 12, cy = 12, r = 10;
    qreal a1 = 30.0 * M_PI / 180.0;
    qreal a2 = 150.0 * M_PI / 180.0;
    drawDot(p, cx + r * std::cos(a1), cy - r * std::sin(a1), 2.0, kAccent);
    drawDot(p, cx + r * std::cos(a2), cy - r * std::sin(a2), 2.0, kAccent);
    p.end();
    return QIcon(QPixmap::fromImage(img));
}

QIcon IconGenerator::drawRectangle(int s) {
    QImage img = createImage(s);
    QPainter p(&img);
    initPainter(p, s);
    p.setPen(primaryPen(2.0));
    p.setBrush(Qt::NoBrush);
    p.drawRect(QRectF(4, 6, 16, 12));
    drawDot(p, 4, 6, 2.0, kAccent);
    drawDot(p, 20, 18, 2.0, kAccent);
    p.end();
    return QIcon(QPixmap::fromImage(img));
}

QIcon IconGenerator::drawPolyline(int s) {
    QImage img = createImage(s);
    QPainter p(&img);
    initPainter(p, s);
    p.setPen(primaryPen(2.0));
    QPolygonF poly;
    poly << QPointF(3, 19) << QPointF(7, 8) << QPointF(14, 16) << QPointF(21, 5);
    p.drawPolyline(poly);
    for (const auto& pt : poly) {
        drawDot(p, pt.x(), pt.y(), 2.0, kAccent);
    }
    p.end();
    return QIcon(QPixmap::fromImage(img));
}

QIcon IconGenerator::drawEllipse(int s) {
    QImage img = createImage(s);
    QPainter p(&img);
    initPainter(p, s);
    p.setPen(primaryPen(2.0));
    p.setBrush(Qt::NoBrush);
    p.drawEllipse(QPointF(12, 12), 9, 6);
    drawDot(p, 12, 12, 2.0, kAccent);
    p.end();
    return QIcon(QPixmap::fromImage(img));
}

QIcon IconGenerator::drawSpline(int s) {
    QImage img = createImage(s);
    QPainter p(&img);
    initPainter(p, s);
    // S-curve spline
    QPainterPath spline;
    spline.moveTo(3, 18);
    spline.cubicTo(8, 2, 16, 22, 21, 6);
    p.setPen(primaryPen(2.0));
    p.setBrush(Qt::NoBrush);
    p.drawPath(spline);
    drawDot(p, 3, 18, 2.0, kAccent);
    drawDot(p, 21, 6, 2.0, kAccent);
    p.end();
    return QIcon(QPixmap::fromImage(img));
}

QIcon IconGenerator::drawText(int s) {
    QImage img = createImage(s);
    QPainter p(&img);
    initPainter(p, s);
    // Large "A" letter
    p.setPen(primaryPen(2.0));
    // Left leg
    p.drawLine(QPointF(5, 20), QPointF(12, 4));
    // Right leg
    p.drawLine(QPointF(12, 4), QPointF(19, 20));
    // Cross bar
    p.drawLine(QPointF(8, 14), QPointF(16, 14));
    // Text cursor line
    p.setPen(accentPen(1.5));
    p.drawLine(QPointF(21, 4), QPointF(21, 20));
    p.end();
    return QIcon(QPixmap::fromImage(img));
}

QIcon IconGenerator::drawHatch(int s) {
    QImage img = createImage(s);
    QPainter p(&img);
    initPainter(p, s);
    // Closed shape outline
    QPainterPath shape;
    shape.moveTo(4, 20);
    shape.lineTo(4, 6);
    shape.lineTo(20, 4);
    shape.lineTo(20, 18);
    shape.closeSubpath();
    p.setPen(primaryPen(1.5));
    p.setBrush(Qt::NoBrush);
    p.drawPath(shape);
    // Diagonal hatch lines inside
    p.setPen(QPen(kSecondary, 1.0, Qt::SolidLine, Qt::RoundCap));
    p.drawLine(QPointF(6, 18), QPointF(10, 7));
    p.drawLine(QPointF(10, 18), QPointF(14, 7));
    p.drawLine(QPointF(14, 19), QPointF(18, 6));
    p.end();
    return QIcon(QPixmap::fromImage(img));
}

// ===========================================================================
//  MODIFY icons
// ===========================================================================

QIcon IconGenerator::drawMove(int s) {
    QImage img = createImage(s);
    QPainter p(&img);
    initPainter(p, s);
    // Dashed original rectangle
    p.setPen(dashedPen(kSecondary, 1.2));
    p.setBrush(Qt::NoBrush);
    p.drawRect(QRectF(2, 8, 9, 9));
    // Solid moved rectangle
    p.setPen(primaryPen(1.5));
    p.drawRect(QRectF(11, 3, 9, 9));
    // Arrow from old to new
    p.setPen(accentPen(1.8));
    p.drawLine(QPointF(7, 12), QPointF(15, 7));
    drawArrowhead(p, QPointF(15, 7), std::atan2(-5.0, 8.0), 4.0, kAccent);
    p.end();
    return QIcon(QPixmap::fromImage(img));
}

QIcon IconGenerator::drawOffset(int s) {
    QImage img = createImage(s);
    QPainter p(&img);
    initPainter(p, s);
    // Inner arc
    p.setPen(primaryPen(1.8));
    p.setBrush(Qt::NoBrush);
    QRectF inner(5, 3, 14, 18);
    p.drawArc(inner, 60 * 16, 140 * 16);
    // Outer arc (offset)
    p.setPen(accentPen(1.8));
    QRectF outer(1, 0, 22, 24);
    p.drawArc(outer, 60 * 16, 140 * 16);
    // Small offset arrow between arcs
    p.setPen(QPen(kSecondary, 1.0));
    p.drawLine(QPointF(8, 14), QPointF(5, 14));
    drawArrowhead(p, QPointF(5, 14), M_PI, 3.0, kSecondary);
    p.end();
    return QIcon(QPixmap::fromImage(img));
}

QIcon IconGenerator::drawMirror(int s) {
    QImage img = createImage(s);
    QPainter p(&img);
    initPainter(p, s);
    // Mirror axis (dashed vertical center)
    p.setPen(dashedPen(kAccent, 1.5));
    p.drawLine(QPointF(12, 2), QPointF(12, 22));
    // Left triangle (original)
    p.setPen(primaryPen(1.5));
    p.setBrush(Qt::NoBrush);
    QPolygonF left;
    left << QPointF(3, 18) << QPointF(10, 12) << QPointF(3, 6);
    p.drawPolygon(left);
    // Right triangle (mirrored)
    QPolygonF right;
    right << QPointF(21, 18) << QPointF(14, 12) << QPointF(21, 6);
    p.drawPolygon(right);
    p.end();
    return QIcon(QPixmap::fromImage(img));
}

QIcon IconGenerator::drawRotate(int s) {
    QImage img = createImage(s);
    QPainter p(&img);
    initPainter(p, s);
    // Circular arc arrow
    p.setPen(primaryPen(1.8));
    p.setBrush(Qt::NoBrush);
    QRectF arcR(3, 3, 18, 18);
    p.drawArc(arcR, 45 * 16, 270 * 16);
    // Arrowhead at end of arc
    qreal endAngle = 45.0 * M_PI / 180.0;
    qreal cx = 12, cy = 12, r = 9;
    QPointF tip(cx + r * std::cos(endAngle), cy - r * std::sin(endAngle));
    drawArrowhead(p, tip, endAngle + M_PI / 2.0, 4.5, kPrimary);
    // Small rectangle being rotated
    p.setPen(accentPen(1.5));
    p.save();
    p.translate(12, 12);
    p.rotate(30);
    p.drawRect(QRectF(-3, -4, 6, 8));
    p.restore();
    p.end();
    return QIcon(QPixmap::fromImage(img));
}

QIcon IconGenerator::drawScale(int s) {
    QImage img = createImage(s);
    QPainter p(&img);
    initPainter(p, s);
    // Small inner rectangle (original)
    p.setPen(secondaryPen(1.3));
    p.setBrush(Qt::NoBrush);
    p.drawRect(QRectF(4, 10, 8, 8));
    // Larger outer rectangle (scaled)
    p.setPen(primaryPen(1.8));
    p.drawRect(QRectF(4, 4, 16, 16));
    // Diagonal scale arrow from corner to corner
    p.setPen(accentPen(1.5));
    p.drawLine(QPointF(12, 18), QPointF(18, 6));
    drawArrowhead(p, QPointF(18, 6), std::atan2(-12.0, 6.0), 4.0, kAccent);
    p.end();
    return QIcon(QPixmap::fromImage(img));
}

QIcon IconGenerator::drawTrim(int s) {
    QImage img = createImage(s);
    QPainter p(&img);
    initPainter(p, s);
    // Cutting line (horizontal)
    p.setPen(primaryPen(1.8));
    p.drawLine(QPointF(2, 12), QPointF(22, 12));
    // Line being trimmed — solid part
    p.setPen(primaryPen(1.8));
    p.drawLine(QPointF(12, 3), QPointF(12, 12));
    // Trimmed part — dashed and red
    p.setPen(QPen(kRed, 1.5, Qt::DashLine, Qt::RoundCap));
    p.drawLine(QPointF(12, 12), QPointF(12, 21));
    // Scissors / cut indicator — small X at intersection
    p.setPen(QPen(kRed, 2.0, Qt::SolidLine, Qt::RoundCap));
    p.drawLine(QPointF(10, 10), QPointF(14, 14));
    p.drawLine(QPointF(14, 10), QPointF(10, 14));
    p.end();
    return QIcon(QPixmap::fromImage(img));
}

QIcon IconGenerator::drawFillet(int s) {
    QImage img = createImage(s);
    QPainter p(&img);
    initPainter(p, s);
    // Two lines meeting at a corner
    p.setPen(primaryPen(1.8));
    p.drawLine(QPointF(4, 20), QPointF(4, 8));
    p.drawLine(QPointF(4, 20), QPointF(20, 20));
    // Fillet arc at the corner
    p.setPen(accentPen(2.0));
    p.setBrush(Qt::NoBrush);
    QPainterPath fillet;
    fillet.moveTo(4, 10);
    fillet.cubicTo(4, 16, 6, 20, 12, 20);
    p.drawPath(fillet);
    // Small radius indicator
    p.setPen(QPen(kSecondary, 1.0, Qt::DashLine, Qt::RoundCap));
    p.drawLine(QPointF(4, 20), QPointF(9, 15));
    p.end();
    return QIcon(QPixmap::fromImage(img));
}

QIcon IconGenerator::drawChamfer(int s) {
    QImage img = createImage(s);
    QPainter p(&img);
    initPainter(p, s);
    // Two lines meeting at a corner (bottom-left)
    p.setPen(primaryPen(1.8));
    p.drawLine(QPointF(4, 20), QPointF(4, 8));
    p.drawLine(QPointF(4, 20), QPointF(20, 20));
    // Chamfer bevel line (diagonal cut)
    p.setPen(accentPen(2.0));
    p.drawLine(QPointF(4, 10), QPointF(14, 20));
    // Dashed lines showing removed corner
    p.setPen(QPen(kSecondary, 1.0, Qt::DashLine, Qt::RoundCap));
    p.drawLine(QPointF(4, 10), QPointF(4, 20));
    p.drawLine(QPointF(4, 20), QPointF(14, 20));
    p.end();
    return QIcon(QPixmap::fromImage(img));
}

QIcon IconGenerator::drawBreak(int s) {
    QImage img = createImage(s);
    QPainter p(&img);
    initPainter(p, s);
    // Line with a gap in the middle (break point)
    p.setPen(primaryPen(1.8));
    p.drawLine(QPointF(3, 18), QPointF(10, 11));
    p.drawLine(QPointF(14, 7), QPointF(21, 3));
    // Break indicator — small zigzag at the gap
    p.setPen(accentPen(2.0));
    p.drawLine(QPointF(10, 11), QPointF(12, 7));
    p.drawLine(QPointF(12, 7), QPointF(14, 11));
    p.drawLine(QPointF(14, 11), QPointF(14, 7));
    p.end();
    return QIcon(QPixmap::fromImage(img));
}

QIcon IconGenerator::drawExtend(int s) {
    QImage img = createImage(s);
    QPainter p(&img);
    initPainter(p, s);
    // Boundary line (vertical)
    p.setPen(primaryPen(1.8));
    p.drawLine(QPointF(18, 3), QPointF(18, 21));
    // Original short line
    p.setPen(primaryPen(1.8));
    p.drawLine(QPointF(3, 14), QPointF(12, 14));
    // Extension (dashed)
    p.setPen(QPen(kAccent, 1.5, Qt::DashLine, Qt::RoundCap));
    p.drawLine(QPointF(12, 14), QPointF(18, 14));
    // Arrow at the extension tip
    p.setPen(accentPen(1.5));
    drawArrowhead(p, QPointF(18, 14), 0.0, 4.0, kAccent);
    p.end();
    return QIcon(QPixmap::fromImage(img));
}

QIcon IconGenerator::drawStretch(int s) {
    QImage img = createImage(s);
    QPainter p(&img);
    initPainter(p, s);
    // Original rectangle (bottom-left portion stays, top-right corner stretches)
    p.setPen(primaryPen(1.8));
    p.drawLine(QPointF(4, 20), QPointF(14, 20));  // bottom edge
    p.drawLine(QPointF(4, 20), QPointF(4, 8));    // left edge
    p.drawLine(QPointF(4, 8), QPointF(14, 8));    // top edge (original)
    p.drawLine(QPointF(14, 20), QPointF(14, 8));  // right edge (original)
    // Stretched corner displaced to top-right
    p.setPen(accentPen(1.5));
    p.drawLine(QPointF(14, 8), QPointF(20, 4));  // arrow from corner to new position
    drawArrowhead(p, QPointF(20, 4), -M_PI / 4.0, 4.0, kAccent);
    // Stretched edges (dashed to show new position)
    p.setPen(QPen(kAccent, 1.2, Qt::DashLine, Qt::RoundCap));
    p.drawLine(QPointF(4, 8), QPointF(20, 4));    // new top edge hint
    p.drawLine(QPointF(14, 20), QPointF(20, 4));  // new right edge hint
    p.end();
    return QIcon(QPixmap::fromImage(img));
}

QIcon IconGenerator::drawPolylineEdit(int s) {
    QImage img = createImage(s);
    QPainter p(&img);
    initPainter(p, s);
    // Polyline path
    p.setPen(primaryPen(1.8));
    p.drawLine(QPointF(3, 18), QPointF(8, 6));
    p.drawLine(QPointF(8, 6), QPointF(16, 14));
    p.drawLine(QPointF(16, 14), QPointF(21, 6));
    // Vertex dots
    p.setPen(Qt::NoPen);
    p.setBrush(kAccent);
    p.drawEllipse(QPointF(3, 18), 2.5, 2.5);
    p.drawEllipse(QPointF(8, 6), 2.5, 2.5);
    p.drawEllipse(QPointF(16, 14), 2.5, 2.5);
    p.drawEllipse(QPointF(21, 6), 2.5, 2.5);
    // Small "+" near middle segment to suggest add vertex
    p.setPen(QPen(kAccent, 1.5, Qt::SolidLine, Qt::RoundCap));
    p.drawLine(QPointF(12, 8), QPointF(12, 12));
    p.drawLine(QPointF(10, 10), QPointF(14, 10));
    p.end();
    return QIcon(QPixmap::fromImage(img));
}

QIcon IconGenerator::drawRectArray(int s) {
    QImage img = createImage(s);
    QPainter p(&img);
    initPainter(p, s);
    // 3x2 grid of small rectangles
    qreal w = 5, h = 5, gx = 2, gy = 2;
    qreal startX = 2, startY = 4;
    for (int row = 0; row < 2; ++row) {
        for (int col = 0; col < 3; ++col) {
            qreal x = startX + col * (w + gx);
            qreal y = startY + row * (h + gy);
            QColor c = (row == 0 && col == 0) ? kAccent : kPrimary;
            p.setPen(QPen(c, 1.5, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin));
            p.setBrush(Qt::NoBrush);
            p.drawRect(QRectF(x, y, w, h));
        }
    }
    // Arrows indicating spacing
    p.setPen(accentPen(1.0));
    p.drawLine(QPointF(5, 17), QPointF(12, 17));
    drawArrowhead(p, QPointF(12, 17), 0, 2.5, kAccent);
    p.end();
    return QIcon(QPixmap::fromImage(img));
}

QIcon IconGenerator::drawPolarArray(int s) {
    QImage img = createImage(s);
    QPainter p(&img);
    initPainter(p, s);
    // Center point
    drawDot(p, 12, 12, 2.0, kAccent);
    // Circular arrangement of small rectangles
    qreal r = 8;
    for (int i = 0; i < 6; ++i) {
        qreal angle = i * 60.0 * M_PI / 180.0 - M_PI / 2.0;
        qreal x = 12 + r * std::cos(angle);
        qreal y = 12 + r * std::sin(angle);
        QColor c = (i == 0) ? kAccent : kPrimary;
        p.setPen(QPen(c, 1.3, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin));
        p.setBrush(Qt::NoBrush);
        p.save();
        p.translate(x, y);
        p.rotate(i * 60.0);
        p.drawRect(QRectF(-2, -2, 4, 4));
        p.restore();
    }
    // Dashed circle
    p.setPen(dashedPen(kSecondary, 1.0));
    p.setBrush(Qt::NoBrush);
    p.drawEllipse(QPointF(12, 12), r, r);
    p.end();
    return QIcon(QPixmap::fromImage(img));
}

// ===========================================================================
//  DIMENSION icons
// ===========================================================================

QIcon IconGenerator::drawDimLinear(int s) {
    QImage img = createImage(s);
    QPainter p(&img);
    initPainter(p, s);
    // Two extension lines
    p.setPen(secondaryPen(1.2));
    p.drawLine(QPointF(4, 18), QPointF(4, 6));
    p.drawLine(QPointF(20, 18), QPointF(20, 6));
    // Dimension line with arrows
    p.setPen(primaryPen(1.5));
    p.drawLine(QPointF(4, 8), QPointF(20, 8));
    drawArrowhead(p, QPointF(4, 8), M_PI, 3.5, kPrimary);
    drawArrowhead(p, QPointF(20, 8), 0, 3.5, kPrimary);
    // Dimension text
    QFont f;
    f.setPixelSize(8);
    f.setBold(true);
    p.setFont(f);
    p.setPen(kDimText);
    p.drawText(QRectF(6, 2, 12, 8), Qt::AlignCenter, "25");
    p.end();
    return QIcon(QPixmap::fromImage(img));
}

QIcon IconGenerator::drawDimRadial(int s) {
    QImage img = createImage(s);
    QPainter p(&img);
    initPainter(p, s);
    // Circle
    p.setPen(secondaryPen(1.2));
    p.setBrush(Qt::NoBrush);
    p.drawEllipse(QPointF(10, 13), 8, 8);
    // Radius line from center outward
    p.setPen(primaryPen(1.5));
    p.drawLine(QPointF(10, 13), QPointF(17, 8));
    drawArrowhead(p, QPointF(17, 8), std::atan2(-5.0, 7.0), 3.5, kPrimary);
    // Center mark
    drawDot(p, 10, 13, 1.5, kAccent);
    // R text
    QFont f;
    f.setPixelSize(8);
    f.setBold(true);
    p.setFont(f);
    p.setPen(kDimText);
    p.drawText(QRectF(14, 2, 10, 8), Qt::AlignLeft, "R");
    p.end();
    return QIcon(QPixmap::fromImage(img));
}

QIcon IconGenerator::drawDimAngular(int s) {
    QImage img = createImage(s);
    QPainter p(&img);
    initPainter(p, s);
    // Two lines meeting at a vertex
    p.setPen(secondaryPen(1.2));
    p.drawLine(QPointF(4, 20), QPointF(4, 4));
    p.drawLine(QPointF(4, 20), QPointF(22, 8));
    // Arc dimension between them
    p.setPen(primaryPen(1.5));
    p.setBrush(Qt::NoBrush);
    QRectF arcR(-4, 12, 16, 16);
    p.drawArc(arcR, 25 * 16, 65 * 16);
    // Angle text
    QFont f;
    f.setPixelSize(7);
    f.setBold(true);
    p.setFont(f);
    p.setPen(kDimText);
    p.drawText(QRectF(8, 8, 14, 9), Qt::AlignCenter, "45\xC2\xB0");
    p.end();
    return QIcon(QPixmap::fromImage(img));
}

QIcon IconGenerator::drawLeader(int s) {
    QImage img = createImage(s);
    QPainter p(&img);
    initPainter(p, s);
    // Leader line: arrow -> elbow -> text
    p.setPen(primaryPen(1.5));
    p.drawLine(QPointF(4, 18), QPointF(12, 8));
    p.drawLine(QPointF(12, 8), QPointF(21, 8));
    drawArrowhead(p, QPointF(4, 18), std::atan2(10.0, -8.0), 4.0, kPrimary);
    // Text lines
    p.setPen(QPen(kDimText, 1.0));
    p.drawLine(QPointF(12, 5), QPointF(21, 5));
    p.drawLine(QPointF(12, 3), QPointF(18, 3));
    p.end();
    return QIcon(QPixmap::fromImage(img));
}

// ===========================================================================
//  CONSTRAINT icons
// ===========================================================================

QIcon IconGenerator::drawCstrCoincident(int s) {
    QImage img = createImage(s);
    QPainter p(&img);
    initPainter(p, s);
    // Two overlapping dots — coincident points
    drawDot(p, 12, 12, 5.0, QColor(74, 144, 217, 100));
    drawDot(p, 12, 12, 3.0, kAccent);
    // Small crosshair
    p.setPen(QPen(kPrimary, 1.2));
    p.drawLine(QPointF(12, 6), QPointF(12, 18));
    p.drawLine(QPointF(6, 12), QPointF(18, 12));
    p.end();
    return QIcon(QPixmap::fromImage(img));
}

QIcon IconGenerator::drawCstrHorizontal(int s) {
    QImage img = createImage(s);
    QPainter p(&img);
    initPainter(p, s);
    // Horizontal line
    p.setPen(accentPen(2.0));
    p.drawLine(QPointF(3, 12), QPointF(21, 12));
    drawDot(p, 3, 12, 2.0, kAccent);
    drawDot(p, 21, 12, 2.0, kAccent);
    // "H" label
    QFont f;
    f.setPixelSize(7);
    f.setBold(true);
    p.setFont(f);
    p.setPen(kPrimary);
    p.drawText(QRectF(8, 2, 8, 8), Qt::AlignCenter, "H");
    p.end();
    return QIcon(QPixmap::fromImage(img));
}

QIcon IconGenerator::drawCstrVertical(int s) {
    QImage img = createImage(s);
    QPainter p(&img);
    initPainter(p, s);
    // Vertical line
    p.setPen(accentPen(2.0));
    p.drawLine(QPointF(12, 3), QPointF(12, 21));
    drawDot(p, 12, 3, 2.0, kAccent);
    drawDot(p, 12, 21, 2.0, kAccent);
    // "V" label
    QFont f;
    f.setPixelSize(7);
    f.setBold(true);
    p.setFont(f);
    p.setPen(kPrimary);
    p.drawText(QRectF(15, 8, 8, 8), Qt::AlignLeft, "V");
    p.end();
    return QIcon(QPixmap::fromImage(img));
}

QIcon IconGenerator::drawCstrPerpendicular(int s) {
    QImage img = createImage(s);
    QPainter p(&img);
    initPainter(p, s);
    // Two perpendicular lines
    p.setPen(primaryPen(1.8));
    p.drawLine(QPointF(4, 20), QPointF(4, 4));    // vertical
    p.drawLine(QPointF(4, 20), QPointF(20, 20));  // horizontal
    // Right-angle square symbol
    p.setPen(accentPen(1.5));
    p.drawLine(QPointF(4, 14), QPointF(10, 14));
    p.drawLine(QPointF(10, 14), QPointF(10, 20));
    // Perpendicular symbol at top-right
    p.setPen(QPen(kPrimary, 1.5));
    p.drawLine(QPointF(16, 4), QPointF(16, 12));
    p.drawLine(QPointF(12, 12), QPointF(20, 12));
    p.end();
    return QIcon(QPixmap::fromImage(img));
}

QIcon IconGenerator::drawCstrParallel(int s) {
    QImage img = createImage(s);
    QPainter p(&img);
    initPainter(p, s);
    // Two parallel diagonal lines
    p.setPen(primaryPen(2.0));
    p.drawLine(QPointF(3, 18), QPointF(15, 4));
    p.drawLine(QPointF(9, 20), QPointF(21, 6));
    // Parallel symbol (two small vertical bars)
    p.setPen(accentPen(2.0));
    p.drawLine(QPointF(8, 8), QPointF(9, 6));
    p.drawLine(QPointF(10, 8), QPointF(11, 6));
    p.end();
    return QIcon(QPixmap::fromImage(img));
}

QIcon IconGenerator::drawCstrTangent(int s) {
    QImage img = createImage(s);
    QPainter p(&img);
    initPainter(p, s);
    // Circle
    p.setPen(primaryPen(1.8));
    p.setBrush(Qt::NoBrush);
    p.drawEllipse(QPointF(10, 13), 7, 7);
    // Tangent line touching the circle
    p.setPen(accentPen(1.8));
    p.drawLine(QPointF(2, 6), QPointF(22, 6));
    // Tangent point
    drawDot(p, 10, 6, 2.0, kAccent);
    p.end();
    return QIcon(QPixmap::fromImage(img));
}

QIcon IconGenerator::drawCstrEqual(int s) {
    QImage img = createImage(s);
    QPainter p(&img);
    initPainter(p, s);
    // Two horizontal lines (= sign) large
    p.setPen(accentPen(2.5));
    p.drawLine(QPointF(5, 9), QPointF(19, 9));
    p.drawLine(QPointF(5, 15), QPointF(19, 15));
    p.end();
    return QIcon(QPixmap::fromImage(img));
}

QIcon IconGenerator::drawCstrFixed(int s) {
    QImage img = createImage(s);
    QPainter p(&img);
    initPainter(p, s);
    // Pin / anchor symbol
    // Vertical pin shaft
    p.setPen(primaryPen(2.0));
    p.drawLine(QPointF(12, 4), QPointF(12, 14));
    // Pin head (circle)
    drawDot(p, 12, 4, 2.5, kAccent);
    // Ground hatching at bottom
    p.setPen(primaryPen(1.5));
    p.drawLine(QPointF(5, 14), QPointF(19, 14));
    p.setPen(QPen(kSecondary, 1.0));
    p.drawLine(QPointF(6, 16), QPointF(8, 14));
    p.drawLine(QPointF(9, 16), QPointF(11, 14));
    p.drawLine(QPointF(12, 16), QPointF(14, 14));
    p.drawLine(QPointF(15, 16), QPointF(17, 14));
    p.drawLine(QPointF(18, 16), QPointF(20, 14));
    // Triangle base
    p.setPen(primaryPen(1.5));
    QPolygonF tri;
    tri << QPointF(12, 14) << QPointF(7, 20) << QPointF(17, 20);
    p.drawPolygon(tri);
    p.end();
    return QIcon(QPixmap::fromImage(img));
}

QIcon IconGenerator::drawCstrDistance(int s) {
    QImage img = createImage(s);
    QPainter p(&img);
    initPainter(p, s);
    // Two points with a dimension-like distance indicator
    drawDot(p, 4, 16, 2.5, kAccent);
    drawDot(p, 20, 4, 2.5, kAccent);
    // Dashed connecting line
    p.setPen(dashedPen(kSecondary, 1.0));
    p.drawLine(QPointF(4, 16), QPointF(20, 4));
    // Dimension arrows
    p.setPen(primaryPen(1.5));
    qreal angle = std::atan2(-12.0, 16.0);
    // Offset the dimension line slightly
    qreal ox = 2 * std::cos(angle + M_PI / 2.0);
    qreal oy = 2 * std::sin(angle + M_PI / 2.0);
    p.drawLine(QPointF(5 + ox, 15 + oy), QPointF(19 + ox, 5 + oy));
    drawArrowhead(p, QPointF(5 + ox, 15 + oy), angle + M_PI, 3.5, kPrimary);
    drawArrowhead(p, QPointF(19 + ox, 5 + oy), angle, 3.5, kPrimary);
    // "d" label
    QFont f;
    f.setPixelSize(8);
    f.setBold(true);
    p.setFont(f);
    p.setPen(kDimText);
    p.drawText(QRectF(11, 12, 10, 10), Qt::AlignLeft, "d");
    p.end();
    return QIcon(QPixmap::fromImage(img));
}

QIcon IconGenerator::drawCstrAngle(int s) {
    QImage img = createImage(s);
    QPainter p(&img);
    initPainter(p, s);
    // Two lines from a vertex
    p.setPen(primaryPen(1.8));
    p.drawLine(QPointF(4, 20), QPointF(20, 4));
    p.drawLine(QPointF(4, 20), QPointF(22, 14));
    // Arc showing angle
    p.setPen(accentPen(1.5));
    p.setBrush(Qt::NoBrush);
    QRectF arcR(-5, 11, 18, 18);
    p.drawArc(arcR, 20 * 16, 45 * 16);
    // Angle symbol
    QFont f;
    f.setPixelSize(7);
    f.setBold(true);
    p.setFont(f);
    p.setPen(kDimText);
    p.drawText(QRectF(10, 10, 12, 8), Qt::AlignCenter, "\xC2\xB0");
    p.end();
    return QIcon(QPixmap::fromImage(img));
}

// ===========================================================================
//  MEASURE icons
// ===========================================================================

QIcon IconGenerator::drawMeasureDistance(int s) {
    QImage img = createImage(s);
    QPainter p(&img);
    initPainter(p, s);
    // Two endpoints
    drawDot(p, 3, 18, 2.5, kGreen);
    drawDot(p, 21, 6, 2.5, kGreen);
    // Measurement line
    p.setPen(QPen(kGreen, 1.5, Qt::SolidLine, Qt::RoundCap));
    p.drawLine(QPointF(3, 18), QPointF(21, 6));
    // Distance arrows
    drawArrowhead(p, QPointF(3, 18), std::atan2(12.0, -18.0), 3.5, kGreen);
    drawArrowhead(p, QPointF(21, 6), std::atan2(-12.0, 18.0), 3.5, kGreen);
    // Ruler tick marks
    p.setPen(QPen(kSecondary, 1.0));
    qreal dx = (21.0 - 3.0) / 6.0;
    qreal dy = (6.0 - 18.0) / 6.0;
    qreal nx = -dy, ny = dx;
    qreal len = std::sqrt(nx * nx + ny * ny);
    nx /= len;
    ny /= len;
    for (int i = 1; i < 6; ++i) {
        qreal mx = 3.0 + i * dx;
        qreal my = 18.0 + i * dy;
        p.drawLine(QPointF(mx - nx * 1.5, my - ny * 1.5), QPointF(mx + nx * 1.5, my + ny * 1.5));
    }
    p.end();
    return QIcon(QPixmap::fromImage(img));
}

QIcon IconGenerator::drawMeasureAngle(int s) {
    QImage img = createImage(s);
    QPainter p(&img);
    initPainter(p, s);
    // Two lines from vertex
    p.setPen(QPen(kGreen, 1.8, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin));
    p.drawLine(QPointF(4, 20), QPointF(4, 3));
    p.drawLine(QPointF(4, 20), QPointF(21, 10));
    // Arc
    p.setBrush(Qt::NoBrush);
    QRectF arcR(-4, 12, 16, 16);
    p.drawArc(arcR, 30 * 16, 60 * 16);
    // Vertex dot
    drawDot(p, 4, 20, 2.0, kGreen);
    // Angle value
    QFont f;
    f.setPixelSize(7);
    f.setBold(true);
    p.setFont(f);
    p.setPen(kDimText);
    p.drawText(QRectF(8, 7, 14, 9), Qt::AlignCenter, "60\xC2\xB0");
    p.end();
    return QIcon(QPixmap::fromImage(img));
}

QIcon IconGenerator::drawMeasureArea(int s) {
    QImage img = createImage(s);
    QPainter p(&img);
    initPainter(p, s);
    // Closed polygon filled with semi-transparent green
    QPolygonF poly;
    poly << QPointF(4, 20) << QPointF(4, 6) << QPointF(14, 3) << QPointF(20, 10) << QPointF(18, 20);
    p.setPen(QPen(kGreen, 1.5, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin));
    p.setBrush(QColor(100, 200, 100, 50));
    p.drawPolygon(poly);
    // Area text
    QFont f;
    f.setPixelSize(7);
    f.setBold(true);
    p.setFont(f);
    p.setPen(kDimText);
    p.drawText(QRectF(5, 9, 14, 10), Qt::AlignCenter, "A");
    p.end();
    return QIcon(QPixmap::fromImage(img));
}

// ===========================================================================
//  BLOCK icons
// ===========================================================================

QIcon IconGenerator::drawBlockCreate(int s) {
    QImage img = createImage(s);
    QPainter p(&img);
    initPainter(p, s);
    // Outer dashed rectangle — selection
    p.setPen(dashedPen(kSecondary, 1.2));
    p.setBrush(Qt::NoBrush);
    p.drawRect(QRectF(3, 3, 18, 18));
    // Inner geometry — a small house shape
    p.setPen(primaryPen(1.8));
    QPolygonF house;
    house << QPointF(7, 18) << QPointF(7, 11) << QPointF(12, 7) << QPointF(17, 11)
          << QPointF(17, 18);
    p.drawPolyline(house);
    p.drawLine(QPointF(7, 18), QPointF(17, 18));
    // Plus sign in corner
    p.setPen(accentPen(2.0));
    p.drawLine(QPointF(18, 2), QPointF(22, 2));
    p.drawLine(QPointF(20, 0), QPointF(20, 4));
    p.end();
    return QIcon(QPixmap::fromImage(img));
}

QIcon IconGenerator::drawBlockInsert(int s) {
    QImage img = createImage(s);
    QPainter p(&img);
    initPainter(p, s);
    // Block symbol — rectangle with diamond
    p.setPen(primaryPen(1.5));
    p.setBrush(Qt::NoBrush);
    p.drawRect(QRectF(3, 6, 14, 14));
    // Diamond inside
    QPolygonF diamond;
    diamond << QPointF(10, 8) << QPointF(15, 13) << QPointF(10, 18) << QPointF(5, 13);
    p.setPen(accentPen(1.5));
    p.drawPolygon(diamond);
    // Arrow pointing into position
    p.setPen(accentPen(1.8));
    p.drawLine(QPointF(22, 3), QPointF(17, 8));
    drawArrowhead(p, QPointF(17, 8), std::atan2(5.0, -5.0), 3.5, kAccent);
    p.end();
    return QIcon(QPixmap::fromImage(img));
}

QIcon IconGenerator::drawBlockExplode(int s) {
    QImage img = createImage(s);
    QPainter p(&img);
    initPainter(p, s);
    // Center point
    drawDot(p, 12, 12, 2.0, kAccent);
    // Exploding pieces — small lines radiating outward
    const qreal angles[] = {0,    M_PI / 3.0,       2.0 * M_PI / 3.0,
                            M_PI, 4.0 * M_PI / 3.0, 5.0 * M_PI / 3.0};
    for (qreal a : angles) {
        qreal x1 = 12 + 4 * std::cos(a);
        qreal y1 = 12 + 4 * std::sin(a);
        qreal x2 = 12 + 9 * std::cos(a);
        qreal y2 = 12 + 9 * std::sin(a);
        p.setPen(primaryPen(1.5));
        p.drawLine(QPointF(x1, y1), QPointF(x2, y2));
        drawArrowhead(p, QPointF(x2, y2), a, 3.0, kPrimary);
    }
    // Small rectangle fragments at tips
    p.setPen(QPen(kRed, 1.5, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin));
    p.drawLine(QPointF(12, 2), QPointF(14, 3));
    p.drawLine(QPointF(2, 12), QPointF(3, 14));
    p.drawLine(QPointF(22, 12), QPointF(21, 10));
    p.end();
    return QIcon(QPixmap::fromImage(img));
}

// ===========================================================================
//  VIEW icons
// ===========================================================================

QIcon IconGenerator::drawFitAll(int s) {
    QImage img = createImage(s);
    QPainter p(&img);
    initPainter(p, s);
    // Geometry is authored in the shared 24-unit design space (initPainter has
    // already scaled the painter to the target pixel size).
    constexpr qreal d = 24.0;
    // Four corner brackets (like a focus/fit frame)
    p.setPen(primaryPen(2.0));
    qreal m = 3, e = 7;
    // Top-left corner
    p.drawLine(QPointF(m, m + e), QPointF(m, m));
    p.drawLine(QPointF(m, m), QPointF(m + e, m));
    // Top-right corner
    p.drawLine(QPointF(d - m - e, m), QPointF(d - m, m));
    p.drawLine(QPointF(d - m, m), QPointF(d - m, m + e));
    // Bottom-left corner
    p.drawLine(QPointF(m, d - m - e), QPointF(m, d - m));
    p.drawLine(QPointF(m, d - m), QPointF(m + e, d - m));
    // Bottom-right corner
    p.drawLine(QPointF(d - m - e, d - m), QPointF(d - m, d - m));
    p.drawLine(QPointF(d - m, d - m), QPointF(d - m, d - m - e));
    // Inward arrows from each corner toward center
    p.setPen(accentPen(1.3));
    qreal a = 6.5, b = 9;
    // Top-left
    p.drawLine(QPointF(a, a), QPointF(b, b));
    drawArrowhead(p, QPointF(b, b), M_PI / 4.0, 3.0, kAccent);
    // Top-right
    p.drawLine(QPointF(d - a, a), QPointF(d - b, b));
    drawArrowhead(p, QPointF(d - b, b), 3.0 * M_PI / 4.0, 3.0, kAccent);
    // Bottom-left
    p.drawLine(QPointF(a, d - a), QPointF(b, d - b));
    drawArrowhead(p, QPointF(b, d - b), -M_PI / 4.0, 3.0, kAccent);
    // Bottom-right
    p.drawLine(QPointF(d - a, d - a), QPointF(d - b, d - b));
    drawArrowhead(p, QPointF(d - b, d - b), -3.0 * M_PI / 4.0, 3.0, kAccent);
    p.end();
    return QIcon(QPixmap::fromImage(img));
}

// ===========================================================================
//  3D icons — isometric primitives, features and modelling operations
// ===========================================================================

// Isometric cube laid out in the shared 24-unit design space. Vertices:
//   A top, B upper-right, C lower-right, D bottom, E lower-left, F upper-left,
//   G front-centre where the three visible faces meet. Draws the hexagonal
//   silhouette plus the interior "Y" (edges G-B, G-F, G-D).
static void drawIsoCube(QPainter& p, const QPen& edge) {
    const QPointF A(12, 4), B(20, 8), C(20, 16), D(12, 20), E(4, 16), F(4, 8), G(12, 12);
    p.setPen(edge);
    p.setBrush(Qt::NoBrush);
    QPolygonF hex;
    hex << A << B << C << D << E << F;
    p.drawPolygon(hex);
    p.drawLine(G, B);
    p.drawLine(G, F);
    p.drawLine(G, D);
}

QIcon IconGenerator::drawBox(int s) {
    QImage img = createImage(s);
    QPainter p(&img);
    initPainter(p, s);
    drawIsoCube(p, primaryPen(1.8));
    p.end();
    return QIcon(QPixmap::fromImage(img));
}

QIcon IconGenerator::drawCylinder(int s) {
    QImage img = createImage(s);
    QPainter p(&img);
    initPainter(p, s);
    const qreal rx = 7, ry = 2.6, l = 5, r = 19, topY = 6, botY = 18;
    p.setPen(primaryPen(1.8));
    p.setBrush(Qt::NoBrush);
    // Vertical sides.
    p.drawLine(QPointF(l, topY), QPointF(l, botY));
    p.drawLine(QPointF(r, topY), QPointF(r, botY));
    // Top ellipse (fully visible).
    p.drawEllipse(QPointF(12, topY), rx, ry);
    // Bottom: front half solid, back half dashed (hidden).
    QRectF botRect(12 - rx, botY - ry, 2 * rx, 2 * ry);
    p.drawArc(botRect, 180 * 16, 180 * 16);  // front (lower) half
    p.setPen(QPen(kSecondary, 1.0, Qt::DashLine, Qt::RoundCap));
    p.drawArc(botRect, 0, 180 * 16);  // back (upper) half
    p.end();
    return QIcon(QPixmap::fromImage(img));
}

QIcon IconGenerator::drawSphere(int s) {
    QImage img = createImage(s);
    QPainter p(&img);
    initPainter(p, s);
    p.setPen(primaryPen(1.8));
    p.setBrush(Qt::NoBrush);
    p.drawEllipse(QPointF(12, 12), 8.0, 8.0);
    // Equator (foreshortened ellipse) to read as a globe.
    p.setPen(accentPen(1.4));
    p.drawEllipse(QPointF(12, 12), 8.0, 3.0);
    // A faint meridian.
    p.setPen(QPen(kSecondary, 1.0, Qt::SolidLine, Qt::RoundCap));
    p.drawEllipse(QPointF(12, 12), 3.0, 8.0);
    p.end();
    return QIcon(QPixmap::fromImage(img));
}

QIcon IconGenerator::drawCone(int s) {
    QImage img = createImage(s);
    QPainter p(&img);
    initPainter(p, s);
    const qreal rx = 8, ry = 2.6, apexY = 4, baseY = 18;
    const QPointF apex(12, apexY);
    p.setPen(primaryPen(1.8));
    p.setBrush(Qt::NoBrush);
    // Slanted sides down to the base ellipse extremes.
    p.drawLine(apex, QPointF(12 - rx, baseY));
    p.drawLine(apex, QPointF(12 + rx, baseY));
    // Base: front half solid, back half dashed (hidden).
    QRectF baseRect(12 - rx, baseY - ry, 2 * rx, 2 * ry);
    p.drawArc(baseRect, 180 * 16, 180 * 16);
    p.setPen(QPen(kSecondary, 1.0, Qt::DashLine, Qt::RoundCap));
    p.drawArc(baseRect, 0, 180 * 16);
    p.end();
    return QIcon(QPixmap::fromImage(img));
}

QIcon IconGenerator::drawTorus(int s) {
    QImage img = createImage(s);
    QPainter p(&img);
    initPainter(p, s);
    p.setPen(primaryPen(1.8));
    p.setBrush(Qt::NoBrush);
    // Outer ring silhouette.
    p.drawEllipse(QPointF(12, 12), 9.0, 5.5);
    // Inner hole.
    p.drawEllipse(QPointF(12, 12), 3.6, 2.0);
    p.end();
    return QIcon(QPixmap::fromImage(img));
}

QIcon IconGenerator::drawExtrude(int s) {
    QImage img = createImage(s);
    QPainter p(&img);
    initPainter(p, s);
    // Sketch profile at the bottom.
    p.setPen(primaryPen(1.8));
    p.setBrush(Qt::NoBrush);
    p.drawRect(QRectF(6, 16, 10, 5));
    // Ghosted extruded prism rising above the profile.
    p.setPen(QPen(kSecondary, 1.0, Qt::DashLine, Qt::RoundCap));
    p.drawRect(QRectF(6, 6, 10, 5));
    p.drawLine(QPointF(6, 16), QPointF(6, 11));
    p.drawLine(QPointF(16, 16), QPointF(16, 11));
    // Pull-up arrow.
    p.setPen(accentPen(2.0));
    p.drawLine(QPointF(20, 18), QPointF(20, 5));
    drawArrowhead(p, QPointF(20, 5), -M_PI / 2.0, 3.5, kAccent);
    p.end();
    return QIcon(QPixmap::fromImage(img));
}

QIcon IconGenerator::drawRevolve(int s) {
    QImage img = createImage(s);
    QPainter p(&img);
    initPainter(p, s);
    // Rotation axis (dashed).
    p.setPen(QPen(kSecondary, 1.1, Qt::DashLine, Qt::RoundCap));
    p.drawLine(QPointF(6, 3), QPointF(6, 21));
    // Profile beside the axis.
    p.setPen(primaryPen(1.8));
    p.setBrush(Qt::NoBrush);
    p.drawRect(QRectF(9, 8, 6, 10));
    // Sweep arrow curving around the top to imply revolution.
    p.setPen(accentPen(1.8));
    QPainterPath sweep;
    sweep.moveTo(6, 6);
    sweep.cubicTo(14, 3, 21, 5, 20, 10);
    p.drawPath(sweep);
    drawArrowhead(p, QPointF(20, 10), M_PI * 0.62, 3.5, kAccent);
    p.end();
    return QIcon(QPixmap::fromImage(img));
}

// Shared drawing for the three boolean icons. @p regionPath is the exact
// result region (union / difference / intersection) to fill in translucent
// accent; both operand circles are always outlined so the operation reads
// against its inputs.
static void drawBoolean(QPainter& p, const QPainterPath& regionPath, bool dashSecond) {
    const QPointF c1(9.5, 12), c2(15.5, 12);
    const qreal rr = 6.0;
    p.setPen(Qt::NoPen);
    p.setBrush(QColor(74, 144, 217, 90));
    p.drawPath(regionPath);
    p.setBrush(Qt::NoBrush);
    p.setPen(primaryPen(1.6));
    p.drawEllipse(c1, rr, rr);
    p.setPen(dashSecond ? QPen(kSecondary, 1.2, Qt::DashLine, Qt::RoundCap) : primaryPen(1.6));
    p.drawEllipse(c2, rr, rr);
}

// Build the two operand circle paths used by the boolean icons.
static void booleanOperands(QPainterPath& a, QPainterPath& b) {
    a.addEllipse(QPointF(9.5, 12), 6.0, 6.0);
    b.addEllipse(QPointF(15.5, 12), 6.0, 6.0);
}

QIcon IconGenerator::drawBooleanUnion(int s) {
    QImage img = createImage(s);
    QPainter p(&img);
    initPainter(p, s);
    QPainterPath a, b;
    booleanOperands(a, b);
    drawBoolean(p, a.united(b), false);
    p.end();
    return QIcon(QPixmap::fromImage(img));
}

QIcon IconGenerator::drawBooleanSubtract(int s) {
    QImage img = createImage(s);
    QPainter p(&img);
    initPainter(p, s);
    QPainterPath a, b;
    booleanOperands(a, b);
    // Keep the first operand minus the second; show the second as "removed".
    drawBoolean(p, a.subtracted(b), true);
    p.end();
    return QIcon(QPixmap::fromImage(img));
}

QIcon IconGenerator::drawBooleanIntersect(int s) {
    QImage img = createImage(s);
    QPainter p(&img);
    initPainter(p, s);
    QPainterPath a, b;
    booleanOperands(a, b);
    drawBoolean(p, a.intersected(b), false);
    p.end();
    return QIcon(QPixmap::fromImage(img));
}

QIcon IconGenerator::drawFillet3d(int s) {
    QImage img = createImage(s);
    QPainter p(&img);
    initPainter(p, s);
    drawIsoCube(p, primaryPen(1.6));
    // Round the near top edge (A-B) and highlight it as the filleted edge.
    p.setPen(accentPen(2.2));
    p.setBrush(Qt::NoBrush);
    QPainterPath round;
    round.moveTo(8, 6);
    round.cubicTo(12, 3, 16, 5, 18, 9);
    p.drawPath(round);
    p.end();
    return QIcon(QPixmap::fromImage(img));
}

QIcon IconGenerator::drawChamfer3d(int s) {
    QImage img = createImage(s);
    QPainter p(&img);
    initPainter(p, s);
    drawIsoCube(p, primaryPen(1.6));
    // Bevel the near top vertex (A) with a straight cut and highlight it.
    p.setPen(accentPen(2.2));
    p.drawLine(QPointF(8, 6), QPointF(16, 6));
    p.end();
    return QIcon(QPixmap::fromImage(img));
}

}  // namespace hz::ui
