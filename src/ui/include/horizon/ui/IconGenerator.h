#pragma once

#include <QIcon>
#include <QString>

namespace hz::ui {

/// Static utility class that generates QIcon objects programmatically using QPainter.
/// No external asset files are required — all icons are drawn in code.
class IconGenerator {
public:
    /// Returns a QIcon for the given name. Supported names:
    ///
    /// File:       "new", "open", "save", "undo", "redo"
    /// Edit:       "copy", "paste", "duplicate"
    /// Tools:      "select", "line", "circle", "arc", "rectangle", "polyline",
    ///             "ellipse", "spline", "text", "hatch"
    /// Modify:     "move", "offset", "mirror", "rotate", "scale", "trim",
    ///             "fillet", "chamfer", "break", "extend", "stretch", "rect-array", "polar-array"
    /// Dimension:  "dim-linear", "dim-radial", "dim-angular", "leader"
    /// Constraint: "cstr-coincident", "cstr-horizontal", "cstr-vertical",
    ///             "cstr-perpendicular", "cstr-parallel", "cstr-tangent",
    ///             "cstr-equal", "cstr-fixed", "cstr-distance", "cstr-angle"
    /// Measure:    "measure-distance", "measure-angle", "measure-area"
    /// Block:      "block-create", "block-insert", "block-explode"
    /// View:       "fit-all"
    ///
    /// Returns a blank transparent icon for unrecognized names.
    static QIcon icon(const QString& name);

    IconGenerator() = delete;

private:
    static constexpr int kDefaultSize = 24;

    // File
    static QIcon drawNew(int s);
    static QIcon drawOpen(int s);
    static QIcon drawSave(int s);
    static QIcon drawUndo(int s);
    static QIcon drawRedo(int s);

    // Edit
    static QIcon drawCopy(int s);
    static QIcon drawPaste(int s);
    static QIcon drawDuplicate(int s);

    // Tools
    static QIcon drawSelect(int s);
    static QIcon drawLine(int s);
    static QIcon drawCircle(int s);
    static QIcon drawArc(int s);
    static QIcon drawRectangle(int s);
    static QIcon drawPolyline(int s);
    static QIcon drawEllipse(int s);
    static QIcon drawSpline(int s);
    static QIcon drawText(int s);
    static QIcon drawHatch(int s);

    // Modify
    static QIcon drawMove(int s);
    static QIcon drawOffset(int s);
    static QIcon drawMirror(int s);
    static QIcon drawRotate(int s);
    static QIcon drawScale(int s);
    static QIcon drawTrim(int s);
    static QIcon drawFillet(int s);
    static QIcon drawChamfer(int s);
    static QIcon drawBreak(int s);
    static QIcon drawExtend(int s);
    static QIcon drawStretch(int s);
    static QIcon drawRectArray(int s);
    static QIcon drawPolarArray(int s);

    // Dimension
    static QIcon drawDimLinear(int s);
    static QIcon drawDimRadial(int s);
    static QIcon drawDimAngular(int s);
    static QIcon drawLeader(int s);

    // Constraint
    static QIcon drawCstrCoincident(int s);
    static QIcon drawCstrHorizontal(int s);
    static QIcon drawCstrVertical(int s);
    static QIcon drawCstrPerpendicular(int s);
    static QIcon drawCstrParallel(int s);
    static QIcon drawCstrTangent(int s);
    static QIcon drawCstrEqual(int s);
    static QIcon drawCstrFixed(int s);
    static QIcon drawCstrDistance(int s);
    static QIcon drawCstrAngle(int s);

    // Measure
    static QIcon drawMeasureDistance(int s);
    static QIcon drawMeasureAngle(int s);
    static QIcon drawMeasureArea(int s);

    // Block
    static QIcon drawBlockCreate(int s);
    static QIcon drawBlockInsert(int s);
    static QIcon drawBlockExplode(int s);

    // View
    static QIcon drawFitAll(int s);
};

} // namespace hz::ui
