#include "horizon/ui/FilletTool.h"
#include "horizon/ui/ViewportWidget.h"
#include "horizon/document/Document.h"
#include "horizon/document/Commands.h"
#include "horizon/document/UndoStack.h"
#include "horizon/drafting/DraftLine.h"
#include "horizon/drafting/DraftArc.h"
#include "horizon/math/Constants.h"
#include "horizon/math/MathUtils.h"

#include <QMouseEvent>
#include <QKeyEvent>
#include <cmath>

namespace hz::ui {

void FilletTool::activate(ViewportWidget* viewport) {
    Tool::activate(viewport);
    m_state = State::SelectFirstLine;
    m_firstEntityId = 0;
    m_radiusInput.clear();
    m_hasPreview = false;
}

void FilletTool::deactivate() {
    cancel();
    Tool::deactivate();
}

// ---------------------------------------------------------------------------
// Fillet computation (line-line only)
// ---------------------------------------------------------------------------

bool FilletTool::computeFillet(
    uint64_t lineAId, const math::Vec2& clickA,
    uint64_t lineBId, const math::Vec2& clickB,
    math::Vec2& arcCenter, double& arcRadius,
    double& arcStartAngle, double& arcEndAngle,
    math::Vec2& trimA_start, math::Vec2& trimA_end,
    math::Vec2& trimB_start, math::Vec2& trimB_end) const {

    if (!m_viewport || !m_viewport->document()) return false;
    auto& doc = m_viewport->document()->draftDocument();

    // Find the two lines.
    const draft::DraftLine* lineA = nullptr;
    const draft::DraftLine* lineB = nullptr;
    for (const auto& e : doc.entities()) {
        if (e->id() == lineAId) lineA = dynamic_cast<const draft::DraftLine*>(e.get());
        if (e->id() == lineBId) lineB = dynamic_cast<const draft::DraftLine*>(e.get());
    }
    if (!lineA || !lineB) return false;

    // Find infinite-line intersection (not clamped to segments).
    math::Vec2 d1 = lineA->end() - lineA->start();
    math::Vec2 d2 = lineB->end() - lineB->start();
    double denom = d1.cross(d2);
    if (std::abs(denom) < 1e-10) return false;  // Parallel lines.

    math::Vec2 d3 = lineB->start() - lineA->start();
    double tA = d3.cross(d2) / denom;
    math::Vec2 corner = lineA->start() + d1 * tA;

    // Determine which side of each line the fillet goes on.
    // The fillet should be on the interior side (toward the other line).
    math::Vec2 n1 = d1.normalized().perpendicular();
    math::Vec2 n2 = d2.normalized().perpendicular();

    // Choose perpendicular directions that point toward each other.
    math::Vec2 midClick = (clickA + clickB) * 0.5;
    if ((midClick - corner).dot(n1) < 0) n1 = -n1;
    if ((midClick - corner).dot(n2) < 0) n2 = -n2;

    // Offset lines by radius R.
    math::Vec2 offA1 = lineA->start() + n1 * m_filletRadius;
    math::Vec2 offA2 = lineA->end() + n1 * m_filletRadius;
    math::Vec2 offB1 = lineB->start() + n2 * m_filletRadius;
    math::Vec2 offB2 = lineB->end() + n2 * m_filletRadius;

    // Intersect offset lines to find arc center.
    math::Vec2 offD1 = offA2 - offA1;
    math::Vec2 offD2 = offB2 - offB1;
    double offDenom = offD1.cross(offD2);
    if (std::abs(offDenom) < 1e-10) return false;

    double offT = (offB1 - offA1).cross(offD2) / offDenom;
    arcCenter = offA1 + offD1 * offT;
    arcRadius = m_filletRadius;

    // Tangent points: project arc center onto each line.
    auto projectOnLine = [](const math::Vec2& center,
                            const math::Vec2& lineStart,
                            const math::Vec2& lineDir) -> math::Vec2 {
        math::Vec2 v = center - lineStart;
        double t = v.dot(lineDir) / lineDir.dot(lineDir);
        return lineStart + lineDir * t;
    };

    math::Vec2 tangentA = projectOnLine(arcCenter, lineA->start(), d1);
    math::Vec2 tangentB = projectOnLine(arcCenter, lineB->start(), d2);

    // Compute arc angles.
    double angleA = std::atan2(tangentA.y - arcCenter.y, tangentA.x - arcCenter.x);
    double angleB = std::atan2(tangentB.y - arcCenter.y, tangentB.x - arcCenter.x);

    // Determine correct arc direction (short arc between tangent points).
    double sweep = math::normalizeAngle(angleB - angleA);
    if (sweep > math::kPi) {
        arcStartAngle = math::normalizeAngle(angleB);
        arcEndAngle = math::normalizeAngle(angleA);
    } else {
        arcStartAngle = math::normalizeAngle(angleA);
        arcEndAngle = math::normalizeAngle(angleB);
    }

    // Determine trimmed line endpoints.
    // For each line, the end closer to the corner gets moved to the tangent point.
    double distStartA = lineA->start().distanceTo(corner);
    double distEndA = lineA->end().distanceTo(corner);
    if (distStartA < distEndA) {
        trimA_start = tangentA;
        trimA_end = lineA->end();
    } else {
        trimA_start = lineA->start();
        trimA_end = tangentA;
    }

    double distStartB = lineB->start().distanceTo(corner);
    double distEndB = lineB->end().distanceTo(corner);
    if (distStartB < distEndB) {
        trimB_start = tangentB;
        trimB_end = lineB->end();
    } else {
        trimB_start = lineB->start();
        trimB_end = tangentB;
    }

    return true;
}

// ---------------------------------------------------------------------------
// Event handlers
// ---------------------------------------------------------------------------

bool FilletTool::mousePressEvent(QMouseEvent* event, const math::Vec2& worldPos) {
    if (event->button() != Qt::LeftButton) return false;
    if (!m_viewport || !m_viewport->document()) return false;

    auto& doc = m_viewport->document()->draftDocument();
    double pixelScale = m_viewport->pixelToWorldScale();
    double tolerance = std::max(10.0 * pixelScale, 0.15);

    if (m_state == State::SelectFirstLine) {
        const auto& layerMgr = m_viewport->document()->layerManager();
        for (const auto& entity : doc.entities()) {
            const auto* lp = layerMgr.getLayer(entity->layer());
            if (!lp || !lp->visible || lp->locked) continue;
            if (dynamic_cast<const draft::DraftLine*>(entity.get()) &&
                entity->hitTest(worldPos, tolerance)) {
                m_firstEntityId = entity->id();
                m_firstClickPos = worldPos;
                m_state = State::SelectSecondLine;
                return true;
            }
        }
        return false;
    }

    if (m_state == State::SelectSecondLine) {
        const auto& layerMgr = m_viewport->document()->layerManager();
        for (const auto& entity : doc.entities()) {
            if (entity->id() == m_firstEntityId) continue;
            const auto* lp = layerMgr.getLayer(entity->layer());
            if (!lp || !lp->visible || lp->locked) continue;
            if (!dynamic_cast<const draft::DraftLine*>(entity.get())) continue;
            if (!entity->hitTest(worldPos, tolerance)) continue;

            math::Vec2 arcCenter, trimA_start, trimA_end, trimB_start, trimB_end;
            double arcRadius, arcStart, arcEnd;

            if (computeFillet(m_firstEntityId, m_firstClickPos,
                              entity->id(), worldPos,
                              arcCenter, arcRadius, arcStart, arcEnd,
                              trimA_start, trimA_end, trimB_start, trimB_end)) {

                auto composite = std::make_unique<doc::CompositeCommand>("Fillet");

                // Remove original lines.
                composite->addCommand(std::make_unique<doc::RemoveEntityCommand>(doc, m_firstEntityId));
                composite->addCommand(std::make_unique<doc::RemoveEntityCommand>(doc, entity->id()));

                // Add trimmed lines.
                // Find originals for layer/color.
                const draft::DraftLine* origA = nullptr;
                const draft::DraftLine* origB = nullptr;
                for (const auto& e : doc.entities()) {
                    if (e->id() == m_firstEntityId) origA = dynamic_cast<const draft::DraftLine*>(e.get());
                    if (e->id() == entity->id()) origB = dynamic_cast<const draft::DraftLine*>(e.get());
                }

                auto newLineA = std::make_shared<draft::DraftLine>(trimA_start, trimA_end);
                if (origA) { newLineA->setLayer(origA->layer()); newLineA->setColor(origA->color()); newLineA->setLineWidth(origA->lineWidth()); }
                composite->addCommand(std::make_unique<doc::AddEntityCommand>(doc, newLineA));

                auto newLineB = std::make_shared<draft::DraftLine>(trimB_start, trimB_end);
                if (origB) { newLineB->setLayer(origB->layer()); newLineB->setColor(origB->color()); newLineB->setLineWidth(origB->lineWidth()); }
                composite->addCommand(std::make_unique<doc::AddEntityCommand>(doc, newLineB));

                // Add fillet arc.
                auto filletArc = std::make_shared<draft::DraftArc>(arcCenter, arcRadius, arcStart, arcEnd);
                if (origA) { filletArc->setLayer(origA->layer()); filletArc->setColor(origA->color()); filletArc->setLineWidth(origA->lineWidth()); }
                composite->addCommand(std::make_unique<doc::AddEntityCommand>(doc, filletArc));

                m_viewport->document()->undoStack().push(std::move(composite));
            }

            m_state = State::SelectFirstLine;
            m_firstEntityId = 0;
            m_hasPreview = false;
            return true;
        }
        return false;
    }

    return false;
}

bool FilletTool::mouseMoveEvent(QMouseEvent* /*event*/, const math::Vec2& worldPos) {
    m_currentPos = worldPos;
    m_hasPreview = false;
    return false;
}

bool FilletTool::mouseReleaseEvent(QMouseEvent* /*event*/, const math::Vec2& /*worldPos*/) {
    return false;
}

bool FilletTool::keyPressEvent(QKeyEvent* event) {
    if (event->key() == Qt::Key_Escape) {
        cancel();
        return true;
    }

    // Capture numeric input for radius.
    if (event->key() >= Qt::Key_0 && event->key() <= Qt::Key_9) {
        m_radiusInput += static_cast<char>('0' + (event->key() - Qt::Key_0));
        return true;
    }
    if (event->key() == Qt::Key_Period) {
        m_radiusInput += '.';
        return true;
    }
    if (event->key() == Qt::Key_Backspace && !m_radiusInput.empty()) {
        m_radiusInput.pop_back();
        return true;
    }
    if (event->key() == Qt::Key_Return || event->key() == Qt::Key_Enter) {
        if (!m_radiusInput.empty()) {
            try {
                double r = std::stod(m_radiusInput);
                if (r > 0.0) m_filletRadius = r;
            } catch (...) {}
            m_radiusInput.clear();
        }
        return true;
    }

    return false;
}

void FilletTool::cancel() {
    m_state = State::SelectFirstLine;
    m_firstEntityId = 0;
    m_hasPreview = false;
    m_radiusInput.clear();
}

std::vector<Tool::ArcPreview> FilletTool::getPreviewArcs() const {
    if (m_state != State::SelectSecondLine) return {};
    if (!m_viewport || !m_viewport->document()) return {};

    auto& doc = m_viewport->document()->draftDocument();
    double pixelScale = m_viewport->pixelToWorldScale();
    double tolerance = std::max(10.0 * pixelScale, 0.15);

    // Find line under cursor for preview.
    for (const auto& entity : doc.entities()) {
        if (entity->id() == m_firstEntityId) continue;
        if (!dynamic_cast<const draft::DraftLine*>(entity.get())) continue;
        if (!entity->hitTest(m_currentPos, tolerance)) continue;

        math::Vec2 arcCenter, trimA_start, trimA_end, trimB_start, trimB_end;
        double arcRadius, arcStart, arcEnd;

        if (computeFillet(m_firstEntityId, m_firstClickPos,
                          entity->id(), m_currentPos,
                          arcCenter, arcRadius, arcStart, arcEnd,
                          trimA_start, trimA_end, trimB_start, trimB_end)) {
            return {{arcCenter, arcRadius, arcStart, arcEnd}};
        }
    }
    return {};
}

std::string FilletTool::promptText() const {
    switch (m_state) {
        case State::SelectFirstLine: return "Select first line for fillet";
        case State::SelectSecondLine: return "Select second line for fillet";
    }
    return "";
}

bool FilletTool::wantsCrosshair() const { return false; }

}  // namespace hz::ui
