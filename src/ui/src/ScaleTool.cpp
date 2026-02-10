#include "horizon/ui/ScaleTool.h"
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

static math::Vec2 scalePoint(const math::Vec2& p,
                              const math::Vec2& center, double factor) {
    return center + (p - center) * factor;
}

static double computeSelectionCentroidDist(const math::Vec2& basePoint,
                                           ViewportWidget* viewport) {
    auto& doc = viewport->document()->draftDocument();
    auto& sel = viewport->selectionManager();
    math::Vec2 sum{0.0, 0.0};
    int count = 0;
    for (const auto& entity : doc.entities()) {
        if (!sel.isSelected(entity->id())) continue;
        auto bbox = entity->boundingBox();
        if (bbox.isValid()) {
            auto lo = bbox.min();
            auto hi = bbox.max();
            sum.x += (lo.x + hi.x) * 0.5;
            sum.y += (lo.y + hi.y) * 0.5;
            ++count;
        }
    }
    if (count == 0) return 1.0;
    math::Vec2 centroid{sum.x / count, sum.y / count};
    double dist = basePoint.distanceTo(centroid);
    return (dist < 1e-6) ? 1.0 : dist;
}

bool ScaleTool::mousePressEvent(QMouseEvent* event, const math::Vec2& worldPos) {
    if (event->button() != Qt::LeftButton) return false;
    if (!m_viewport || !m_viewport->document()) return false;

    auto& doc = m_viewport->document()->draftDocument();

    if (m_state == State::SelectBasePoint) {
        auto& sel = m_viewport->selectionManager();
        if (sel.empty()) return false;

        auto result = m_viewport->snapEngine().snap(worldPos, doc.entities());
        m_basePoint = result.point;
        m_viewport->setLastSnapResult(result);

        m_referenceDist = computeSelectionCentroidDist(m_basePoint, m_viewport);

        m_state = State::SelectScaleFactor;
        m_factorInput.clear();
        return true;
    }

    if (m_state == State::SelectScaleFactor) {
        auto result = m_viewport->snapEngine().snap(worldPos, doc.entities());
        m_viewport->setLastSnapResult(result);

        double mouseDist = m_basePoint.distanceTo(result.point);
        double factor = mouseDist / m_referenceDist;
        if (factor < 1e-6) return false;

        auto& sel = m_viewport->selectionManager();
        auto ids = sel.selectedIds();
        std::vector<uint64_t> idVec(ids.begin(), ids.end());

        auto cmd = std::make_unique<doc::ScaleEntityCommand>(doc, idVec, m_basePoint, factor);
        auto* rawCmd = cmd.get();
        m_viewport->document()->undoStack().push(std::move(cmd));

        sel.clearSelection();
        for (uint64_t id : rawCmd->scaledIds()) {
            sel.select(id);
        }

        m_state = State::SelectBasePoint;
        m_viewport->setLastSnapResult({});
        return true;
    }

    return false;
}

bool ScaleTool::mouseMoveEvent(QMouseEvent* /*event*/, const math::Vec2& worldPos) {
    m_currentPos = worldPos;
    if (m_state == State::SelectScaleFactor && m_viewport) {
        auto& doc = m_viewport->document()->draftDocument();
        auto result = m_viewport->snapEngine().snap(worldPos, doc.entities());
        m_currentPos = result.point;
        m_viewport->setLastSnapResult(result);
    }
    return (m_state == State::SelectScaleFactor);
}

bool ScaleTool::mouseReleaseEvent(QMouseEvent* /*event*/, const math::Vec2& /*worldPos*/) {
    return false;
}

bool ScaleTool::keyPressEvent(QKeyEvent* event) {
    if (event->key() == Qt::Key_Escape) {
        cancel();
        return true;
    }

    if (m_state == State::SelectScaleFactor) {
        if (event->key() >= Qt::Key_0 && event->key() <= Qt::Key_9) {
            m_factorInput += static_cast<char>('0' + (event->key() - Qt::Key_0));
            return true;
        }
        if (event->key() == Qt::Key_Period) {
            m_factorInput += '.';
            return true;
        }
        if (event->key() == Qt::Key_Backspace && !m_factorInput.empty()) {
            m_factorInput.pop_back();
            return true;
        }
        if (event->key() == Qt::Key_Return || event->key() == Qt::Key_Enter) {
            if (!m_factorInput.empty() && m_viewport && m_viewport->document()) {
                try {
                    double factor = std::stod(m_factorInput);
                    if (factor < 1e-6) { m_factorInput.clear(); return true; }

                    auto& doc = m_viewport->document()->draftDocument();
                    auto& sel = m_viewport->selectionManager();
                    auto ids = sel.selectedIds();
                    std::vector<uint64_t> idVec(ids.begin(), ids.end());

                    auto cmd = std::make_unique<doc::ScaleEntityCommand>(
                        doc, idVec, m_basePoint, factor);
                    auto* rawCmd = cmd.get();
                    m_viewport->document()->undoStack().push(std::move(cmd));

                    sel.clearSelection();
                    for (uint64_t id : rawCmd->scaledIds()) {
                        sel.select(id);
                    }

                    m_state = State::SelectBasePoint;
                    m_factorInput.clear();
                    m_viewport->setLastSnapResult({});
                } catch (...) {
                    m_factorInput.clear();
                }
            }
            return true;
        }
    }

    return false;
}

void ScaleTool::cancel() {
    m_state = State::SelectBasePoint;
    m_factorInput.clear();
    if (m_viewport) {
        m_viewport->setLastSnapResult({});
    }
}

std::vector<std::pair<math::Vec2, math::Vec2>> ScaleTool::getPreviewLines() const {
    std::vector<std::pair<math::Vec2, math::Vec2>> result;
    if (m_state != State::SelectScaleFactor) return result;
    if (!m_viewport || !m_viewport->document()) return result;

    double mouseDist = m_basePoint.distanceTo(m_currentPos);
    double factor = mouseDist / m_referenceDist;
    if (factor < 1e-6) return result;

    auto& doc = m_viewport->document()->draftDocument();
    auto& sel = m_viewport->selectionManager();
    for (const auto& entity : doc.entities()) {
        if (!sel.isSelected(entity->id())) continue;

        auto segs = draft::extractSegments(*entity);
        for (const auto& [s, e] : segs) {
            result.emplace_back(scalePoint(s, m_basePoint, factor),
                                scalePoint(e, m_basePoint, factor));
        }

        if (auto* c = dynamic_cast<const draft::DraftCircle*>(entity.get())) {
            math::Vec2 mc = scalePoint(c->center(), m_basePoint, factor);
            double mr = c->radius() * factor;
            for (int i = 0; i < 32; ++i) {
                double a1 = math::kTwoPi * i / 32.0;
                double a2 = math::kTwoPi * (i + 1) / 32.0;
                math::Vec2 p1(mc.x + mr * std::cos(a1), mc.y + mr * std::sin(a1));
                math::Vec2 p2(mc.x + mr * std::cos(a2), mc.y + mr * std::sin(a2));
                result.emplace_back(p1, p2);
            }
        }

        if (auto* a = dynamic_cast<const draft::DraftArc*>(entity.get())) {
            math::Vec2 mc = scalePoint(a->center(), m_basePoint, factor);
            double mr = a->radius() * factor;
            double sweep = a->sweepAngle();
            int segsN = std::max(8, static_cast<int>(sweep * 16.0 / math::kTwoPi));
            for (int i = 0; i < segsN; ++i) {
                double a1 = a->startAngle() + sweep * i / segsN;
                double a2 = a->startAngle() + sweep * (i + 1) / segsN;
                math::Vec2 p1(mc.x + mr * std::cos(a1), mc.y + mr * std::sin(a1));
                math::Vec2 p2(mc.x + mr * std::cos(a2), mc.y + mr * std::sin(a2));
                result.emplace_back(p1, p2);
            }
        }
    }

    return result;
}

std::vector<std::pair<math::Vec2, double>> ScaleTool::getPreviewCircles() const {
    return {};
}

std::vector<Tool::ArcPreview> ScaleTool::getPreviewArcs() const {
    return {};
}

}  // namespace hz::ui
