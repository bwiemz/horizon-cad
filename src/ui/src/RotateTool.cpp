#include "horizon/ui/RotateTool.h"
#include "horizon/ui/ViewportWidget.h"
#include "horizon/document/Document.h"
#include "horizon/document/Commands.h"
#include "horizon/document/UndoStack.h"
#include "horizon/drafting/DraftLine.h"
#include "horizon/drafting/DraftCircle.h"
#include "horizon/drafting/DraftArc.h"
#include "horizon/drafting/DraftRectangle.h"
#include "horizon/drafting/DraftPolyline.h"
#include "horizon/drafting/Intersection.h"
#include "horizon/math/Constants.h"
#include "horizon/math/MathUtils.h"

#include <QMouseEvent>
#include <QKeyEvent>
#include <algorithm>
#include <cmath>

namespace hz::ui {

static math::Vec2 rotatePoint(const math::Vec2& p,
                               const math::Vec2& center, double angle) {
    double c = std::cos(angle), s = std::sin(angle);
    math::Vec2 v = p - center;
    return {center.x + v.x * c - v.y * s, center.y + v.x * s + v.y * c};
}

double RotateTool_computeAngle(const math::Vec2& center, const math::Vec2& mouse) {
    return std::atan2(mouse.y - center.y, mouse.x - center.x);
}

bool RotateTool::mousePressEvent(QMouseEvent* event, const math::Vec2& worldPos) {
    if (event->button() != Qt::LeftButton) return false;
    if (!m_viewport || !m_viewport->document()) return false;

    auto& doc = m_viewport->document()->draftDocument();

    if (m_state == State::SelectCenter) {
        auto& sel = m_viewport->selectionManager();
        if (sel.empty()) return false;

        auto result = m_viewport->snapEngine().snap(worldPos, doc.entities());
        m_center = result.point;
        m_viewport->setLastSnapResult(result);

        m_state = State::SelectAngle;
        m_angleInput.clear();
        return true;
    }

    if (m_state == State::SelectAngle) {
        auto result = m_viewport->snapEngine().snap(worldPos, doc.entities());
        m_viewport->setLastSnapResult(result);

        double angle = std::atan2(result.point.y - m_center.y,
                                  result.point.x - m_center.x);
        if (m_center.distanceTo(result.point) < 1e-6) return false;

        auto& sel = m_viewport->selectionManager();
        auto ids = sel.selectedIds();
        std::vector<uint64_t> idVec(ids.begin(), ids.end());

        auto cmd = std::make_unique<doc::RotateEntityCommand>(doc, idVec, m_center, angle);
        auto* rawCmd = cmd.get();
        m_viewport->document()->undoStack().push(std::move(cmd));

        sel.clearSelection();
        for (uint64_t id : rawCmd->rotatedIds()) {
            sel.select(id);
        }

        m_state = State::SelectCenter;
        m_viewport->setLastSnapResult({});
        return true;
    }

    return false;
}

bool RotateTool::mouseMoveEvent(QMouseEvent* /*event*/, const math::Vec2& worldPos) {
    m_currentPos = worldPos;
    if (m_state == State::SelectAngle && m_viewport) {
        auto& doc = m_viewport->document()->draftDocument();
        auto result = m_viewport->snapEngine().snap(worldPos, doc.entities());
        m_currentPos = result.point;
        m_viewport->setLastSnapResult(result);
    }
    return (m_state == State::SelectAngle);
}

bool RotateTool::mouseReleaseEvent(QMouseEvent* /*event*/, const math::Vec2& /*worldPos*/) {
    return false;
}

bool RotateTool::keyPressEvent(QKeyEvent* event) {
    if (event->key() == Qt::Key_Escape) {
        cancel();
        return true;
    }

    if (m_state == State::SelectAngle) {
        if (event->key() >= Qt::Key_0 && event->key() <= Qt::Key_9) {
            m_angleInput += static_cast<char>('0' + (event->key() - Qt::Key_0));
            return true;
        }
        if (event->key() == Qt::Key_Period) {
            m_angleInput += '.';
            return true;
        }
        if (event->key() == Qt::Key_Minus && m_angleInput.empty()) {
            m_angleInput += '-';
            return true;
        }
        if (event->key() == Qt::Key_Backspace && !m_angleInput.empty()) {
            m_angleInput.pop_back();
            return true;
        }
        if (event->key() == Qt::Key_Return || event->key() == Qt::Key_Enter) {
            if (!m_angleInput.empty() && m_viewport && m_viewport->document()) {
                try {
                    double angleDeg = std::stod(m_angleInput);
                    double angleRad = math::degToRad(angleDeg);

                    auto& doc = m_viewport->document()->draftDocument();
                    auto& sel = m_viewport->selectionManager();
                    auto ids = sel.selectedIds();
                    std::vector<uint64_t> idVec(ids.begin(), ids.end());

                    auto cmd = std::make_unique<doc::RotateEntityCommand>(
                        doc, idVec, m_center, angleRad);
                    auto* rawCmd = cmd.get();
                    m_viewport->document()->undoStack().push(std::move(cmd));

                    sel.clearSelection();
                    for (uint64_t id : rawCmd->rotatedIds()) {
                        sel.select(id);
                    }

                    m_state = State::SelectCenter;
                    m_angleInput.clear();
                    m_viewport->setLastSnapResult({});
                } catch (...) {
                    m_angleInput.clear();
                }
            }
            return true;
        }
    }

    return false;
}

void RotateTool::cancel() {
    m_state = State::SelectCenter;
    m_angleInput.clear();
    if (m_viewport) {
        m_viewport->setLastSnapResult({});
    }
}

std::vector<std::pair<math::Vec2, math::Vec2>> RotateTool::getPreviewLines() const {
    std::vector<std::pair<math::Vec2, math::Vec2>> result;
    if (m_state != State::SelectAngle) return result;
    if (!m_viewport || !m_viewport->document()) return result;

    // Draw a line from center to mouse.
    result.emplace_back(m_center, m_currentPos);

    if (m_center.distanceTo(m_currentPos) < 1e-6) return result;

    double angle = std::atan2(m_currentPos.y - m_center.y,
                              m_currentPos.x - m_center.x);

    auto& doc = m_viewport->document()->draftDocument();
    auto& sel = m_viewport->selectionManager();
    for (const auto& entity : doc.entities()) {
        if (!sel.isSelected(entity->id())) continue;

        auto segs = draft::extractSegments(*entity);
        for (const auto& [s, e] : segs) {
            result.emplace_back(rotatePoint(s, m_center, angle),
                                rotatePoint(e, m_center, angle));
        }

        if (auto* c = dynamic_cast<const draft::DraftCircle*>(entity.get())) {
            math::Vec2 mc = rotatePoint(c->center(), m_center, angle);
            for (int i = 0; i < 32; ++i) {
                double a1 = math::kTwoPi * i / 32.0;
                double a2 = math::kTwoPi * (i + 1) / 32.0;
                math::Vec2 p1(mc.x + c->radius() * std::cos(a1),
                              mc.y + c->radius() * std::sin(a1));
                math::Vec2 p2(mc.x + c->radius() * std::cos(a2),
                              mc.y + c->radius() * std::sin(a2));
                result.emplace_back(p1, p2);
            }
        }

        if (auto* a = dynamic_cast<const draft::DraftArc*>(entity.get())) {
            auto clone = a->clone();
            clone->rotate(m_center, angle);
            auto* ma = dynamic_cast<const draft::DraftArc*>(clone.get());
            if (ma) {
                double sweep = ma->sweepAngle();
                int segsN = std::max(8, static_cast<int>(sweep * 16.0 / math::kTwoPi));
                for (int i = 0; i < segsN; ++i) {
                    double a1 = ma->startAngle() + sweep * i / segsN;
                    double a2 = ma->startAngle() + sweep * (i + 1) / segsN;
                    math::Vec2 p1(ma->center().x + ma->radius() * std::cos(a1),
                                  ma->center().y + ma->radius() * std::sin(a1));
                    math::Vec2 p2(ma->center().x + ma->radius() * std::cos(a2),
                                  ma->center().y + ma->radius() * std::sin(a2));
                    result.emplace_back(p1, p2);
                }
            }
        }
    }

    return result;
}

std::vector<std::pair<math::Vec2, double>> RotateTool::getPreviewCircles() const {
    return {};
}

std::vector<Tool::ArcPreview> RotateTool::getPreviewArcs() const {
    return {};
}

}  // namespace hz::ui
