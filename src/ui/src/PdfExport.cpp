#include "horizon/ui/PdfExport.h"

#include <QtGui/qtguiglobal.h>

#include <QFont>
#include <QFontMetricsF>
#include <QPageSize>
#include <QPainter>
#if QT_CONFIG(pdf)
#include <QPdfWriter>
#endif
#include <QPen>
#include <QPolygonF>
#include <QSaveFile>
#include <cmath>

#include "horizon/math/Constants.h"

namespace hz::ui {

namespace {

constexpr int kDotsPerInch = 1200;
constexpr double kDotsPerMm = kDotsPerInch / 25.4;

QColor toQColor(uint32_t argb) {
    return QColor::fromRgb((argb >> 16) & 0xFFu, (argb >> 8) & 0xFFu, argb & 0xFFu);
}

}  // namespace

bool exportPdf(const QString& path, const draft::PlotScene& scene, const draft::PlotLayout& layout,
               QString* error) {
#if !QT_CONFIG(pdf)
    (void)path;
    (void)scene;
    (void)layout;
    if (error) *error = QStringLiteral("this build of Qt cannot write PDF");
    return false;
#else
    QSaveFile file(path);
    if (!file.open(QIODevice::WriteOnly)) {
        if (error) *error = file.errorString();
        return false;
    }
    {
        QPdfWriter writer(&file);
        writer.setResolution(kDotsPerInch);
        writer.setPageSize(
            QPageSize(QSizeF(layout.paperWidthMm, layout.paperHeightMm), QPageSize::Millimeter));
        writer.setPageMargins(QMarginsF(0, 0, 0, 0));
        writer.setCreator(QStringLiteral("Horizon CAD"));
        QPainter painter;
        if (!painter.begin(&writer)) {
            if (error) *error = QStringLiteral("the PDF could not be started");
            file.cancelWriting();
            return false;
        }
        painter.setRenderHint(QPainter::Antialiasing);
        const draft::PlotTransform t = draft::plotTransform(scene, layout);
        const auto dots = [&t](const math::Vec2& world) {
            const math::Vec2 p = t.toPaper(world);
            return QPointF(p.x * kDotsPerMm, p.y * kDotsPerMm);
        };

        for (const auto& stroke : scene.strokes) {
            const double widthMm = draft::plotWeightMm(stroke.width);
            QPen pen(toQColor(draft::plotColor(stroke.color, layout.monochrome)));
            pen.setWidthF(widthMm * kDotsPerMm);
            pen.setJoinStyle(Qt::RoundJoin);
            const auto dashes = draft::plotDashMm(stroke.lineType);
            if (dashes.empty()) {
                pen.setCapStyle(Qt::RoundCap);
            } else {
                // Qt measures a dash pattern in pen widths.
                QList<qreal> pattern;
                for (const double mm : dashes) pattern.append(mm / widthMm);
                pen.setCapStyle(Qt::FlatCap);
                pen.setDashPattern(pattern);
            }
            painter.setPen(pen);
            QPolygonF polygon;
            polygon.reserve(static_cast<qsizetype>(stroke.points.size()));
            for (const auto& p : stroke.points) polygon.append(dots(p));
            if (stroke.closed) {
                painter.drawPolygon(polygon);
            } else {
                painter.drawPolyline(polygon);
            }
        }

        for (const auto& text : scene.texts) {
            // The height is the capitals'; the em is about a third more.
            const double emDots = text.height * t.scale / 0.7 * kDotsPerMm;
            if (!(emDots >= 1.0)) continue;  // too small to set on this page
            QFont font(QStringLiteral("Sans Serif"));
            font.setPixelSize(static_cast<int>(std::lround(emDots)));
            painter.setFont(font);
            painter.setPen(toQColor(draft::plotColor(text.color, layout.monochrome)));
            const QString run = QString::fromStdString(text.text);
            const double advance = QFontMetricsF(font, &writer).horizontalAdvance(run);
            const double left = text.alignment == draft::TextAlignment::Left     ? 0.0
                                : text.alignment == draft::TextAlignment::Center ? -advance / 2.0
                                                                                 : -advance;
            painter.save();
            painter.translate(dots(text.position));
            painter.rotate(-text.rotation * math::kRadToDeg);  // the page's y runs down
            painter.drawText(QPointF(left, 0.0), run);
            painter.restore();
        }
        painter.end();
    }
    if (!file.commit()) {
        if (error) *error = file.errorString();
        return false;
    }
    return true;
#endif
}

}  // namespace hz::ui
