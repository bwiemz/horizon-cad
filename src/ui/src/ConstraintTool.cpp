#include "horizon/ui/ConstraintTool.h"
#include "horizon/ui/ViewportWidget.h"
#include "horizon/constraint/ConstraintSystem.h"
#include "horizon/constraint/GeometryRef.h"
#include "horizon/constraint/ParameterTable.h"
#include "horizon/constraint/SketchSolver.h"
#include "horizon/document/Commands.h"
#include "horizon/document/ConstraintCommands.h"
#include "horizon/document/Document.h"
#include "horizon/drafting/DraftArc.h"
#include "horizon/drafting/DraftCircle.h"
#include "horizon/drafting/DraftLine.h"
#include "horizon/drafting/DraftPolyline.h"
#include "horizon/drafting/DraftRectangle.h"
#include "horizon/drafting/Layer.h"
#include "horizon/math/MathUtils.h"

#include <QInputDialog>
#include <QMouseEvent>
#include <cmath>

namespace hz::ui {

ConstraintTool::ConstraintTool() = default;

void ConstraintTool::activate(ViewportWidget* viewport) {
    Tool::activate(viewport);
    m_state = State::WaitingForFirst;
    m_firstRef = {};
    m_hoveredRef = {};
}

void ConstraintTool::deactivate() {
    m_state = State::WaitingForFirst;
    m_firstRef = {};
    m_hoveredRef = {};
    Tool::deactivate();
}

void ConstraintTool::setMode(Mode mode) {
    m_mode = mode;
    m_state = State::WaitingForFirst;
    m_firstRef = {};
    m_hoveredRef = {};
}

bool ConstraintTool::isSingleRefMode() const {
    return m_mode == Mode::Fixed;
}

cstr::FeatureType ConstraintTool::requiredFeatureType() const {
    switch (m_mode) {
        case Mode::Coincident:
        case Mode::Horizontal:
        case Mode::Vertical:
        case Mode::Fixed:
        case Mode::Distance:
            return cstr::FeatureType::Point;
        case Mode::Perpendicular:
        case Mode::Parallel:
        case Mode::Angle:
            return cstr::FeatureType::Line;
        case Mode::Tangent:
        case Mode::Equal:
            return cstr::FeatureType::Point;  // flexible — detect automatically
    }
    return cstr::FeatureType::Point;
}

bool ConstraintTool::isCompatibleFeature(cstr::FeatureType ft) const {
    switch (m_mode) {
        case Mode::Coincident:
        case Mode::Horizontal:
        case Mode::Vertical:
        case Mode::Fixed:
        case Mode::Distance:
            return ft == cstr::FeatureType::Point;
        case Mode::Perpendicular:
        case Mode::Parallel:
        case Mode::Angle:
            return ft == cstr::FeatureType::Line;
        case Mode::Tangent:
            // First ref can be line or circle; second must be the other
            return ft == cstr::FeatureType::Line || ft == cstr::FeatureType::Circle;
        case Mode::Equal:
            return ft == cstr::FeatureType::Line || ft == cstr::FeatureType::Circle;
    }
    return false;
}

cstr::GeometryRef ConstraintTool::detectFeature(const math::Vec2& worldPos) const {
    if (!m_viewport || !m_viewport->document()) return {};

    const auto& doc = m_viewport->document()->draftDocument();
    const auto& layerMgr = m_viewport->document()->layerManager();
    double tolerance = std::max(10.0 * m_viewport->pixelToWorldScale(), 0.15);

    cstr::GeometryRef bestRef;
    double bestDist = tolerance;

    for (const auto& entity : doc.entities()) {
        const auto* lp = layerMgr.getLayer(entity->layer());
        if (!lp || !lp->visible || lp->locked) continue;

        // Check point features
        if (isCompatibleFeature(cstr::FeatureType::Point)) {
            if (auto* line = dynamic_cast<const draft::DraftLine*>(entity.get())) {
                for (int i = 0; i < 2; ++i) {
                    math::Vec2 p = (i == 0) ? line->start() : line->end();
                    double d = p.distanceTo(worldPos);
                    if (d < bestDist) {
                        bestDist = d;
                        bestRef = {entity->id(), cstr::FeatureType::Point, i};
                        m_viewport->setLastSnapResult({p, draft::SnapType::Endpoint});
                    }
                }
            } else if (auto* circle = dynamic_cast<const draft::DraftCircle*>(entity.get())) {
                double d = circle->center().distanceTo(worldPos);
                if (d < bestDist) {
                    bestDist = d;
                    bestRef = {entity->id(), cstr::FeatureType::Point, 0};
                }
            } else if (auto* arc = dynamic_cast<const draft::DraftArc*>(entity.get())) {
                math::Vec2 pts[] = {arc->center(), arc->startPoint(), arc->endPoint()};
                for (int i = 0; i < 3; ++i) {
                    double d = pts[i].distanceTo(worldPos);
                    if (d < bestDist) {
                        bestDist = d;
                        bestRef = {entity->id(), cstr::FeatureType::Point, i};
                    }
                }
            } else if (auto* poly = dynamic_cast<const draft::DraftPolyline*>(entity.get())) {
                for (int i = 0; i < static_cast<int>(poly->points().size()); ++i) {
                    double d = poly->points()[i].distanceTo(worldPos);
                    if (d < bestDist) {
                        bestDist = d;
                        bestRef = {entity->id(), cstr::FeatureType::Point, i};
                    }
                }
            }
        }

        // Check line features
        if (isCompatibleFeature(cstr::FeatureType::Line)) {
            if (auto* line = dynamic_cast<const draft::DraftLine*>(entity.get())) {
                if (line->hitTest(worldPos, tolerance)) {
                    double d = 0.0;  // hitTest passed, approximate distance
                    // Compute actual distance
                    math::Vec2 ab = line->end() - line->start();
                    double len = ab.length();
                    if (len > 1e-12) {
                        math::Vec2 ap = worldPos - line->start();
                        double t = std::clamp(ap.dot(ab) / (len * len), 0.0, 1.0);
                        math::Vec2 closest = line->start() + ab * t;
                        d = closest.distanceTo(worldPos);
                    }
                    if (d < bestDist) {
                        bestDist = d;
                        bestRef = {entity->id(), cstr::FeatureType::Line, 0};
                    }
                }
            }
        }

        // Check circle features
        if (isCompatibleFeature(cstr::FeatureType::Circle)) {
            if (auto* circle = dynamic_cast<const draft::DraftCircle*>(entity.get())) {
                if (circle->hitTest(worldPos, tolerance)) {
                    bestRef = {entity->id(), cstr::FeatureType::Circle, 0};
                    bestDist = 0.0;
                }
            } else if (auto* arc = dynamic_cast<const draft::DraftArc*>(entity.get())) {
                if (arc->hitTest(worldPos, tolerance)) {
                    bestRef = {entity->id(), cstr::FeatureType::Circle, 0};
                    bestDist = 0.0;
                }
            }
        }
    }

    return bestRef;
}

bool ConstraintTool::mousePressEvent(QMouseEvent* event, const math::Vec2& worldPos) {
    if (event->button() != Qt::LeftButton) return false;
    if (!m_viewport || !m_viewport->document()) return false;

    cstr::GeometryRef ref = detectFeature(worldPos);
    if (!ref.isValid()) return false;

    if (m_state == State::WaitingForFirst) {
        m_firstRef = ref;
        m_firstPos = worldPos;

        if (isSingleRefMode()) {
            // Fixed constraint: commit immediately
            commitConstraint();
            m_state = State::WaitingForFirst;
        } else {
            m_state = State::WaitingForSecond;
        }
        return true;
    } else if (m_state == State::WaitingForSecond) {
        // Don't allow constraining an entity to itself for same feature
        if (ref == m_firstRef) return false;

        m_hoveredRef = ref;
        m_hoveredPos = worldPos;
        commitConstraint();
        m_state = State::WaitingForFirst;
        m_firstRef = {};
        return true;
    }

    return false;
}

bool ConstraintTool::mouseMoveEvent(QMouseEvent* /*event*/, const math::Vec2& worldPos) {
    if (!m_viewport || !m_viewport->document()) return false;

    cstr::GeometryRef ref = detectFeature(worldPos);
    m_hoveredRef = ref;
    m_hoveredPos = worldPos;
    return true;  // Always request redraw for preview
}

bool ConstraintTool::mouseReleaseEvent(QMouseEvent* /*event*/,
                                        const math::Vec2& /*worldPos*/) {
    return false;
}

bool ConstraintTool::keyPressEvent(QKeyEvent* event) {
    if (event->key() == Qt::Key_Escape) {
        cancel();
        return true;
    }
    return false;
}

void ConstraintTool::cancel() {
    m_state = State::WaitingForFirst;
    m_firstRef = {};
    m_hoveredRef = {};
}

void ConstraintTool::commitConstraint() {
    if (!m_viewport || !m_viewport->document()) return;

    auto& doc = *m_viewport->document();
    auto& csys = doc.constraintSystem();
    auto& draftDoc = doc.draftDocument();
    const auto& entities = draftDoc.entities();

    std::shared_ptr<cstr::Constraint> constraint;

    switch (m_mode) {
        case Mode::Coincident:
            constraint = std::make_shared<cstr::CoincidentConstraint>(m_firstRef, m_hoveredRef);
            break;
        case Mode::Horizontal:
            constraint = std::make_shared<cstr::HorizontalConstraint>(m_firstRef, m_hoveredRef);
            break;
        case Mode::Vertical:
            constraint = std::make_shared<cstr::VerticalConstraint>(m_firstRef, m_hoveredRef);
            break;
        case Mode::Perpendicular:
            constraint =
                std::make_shared<cstr::PerpendicularConstraint>(m_firstRef, m_hoveredRef);
            break;
        case Mode::Parallel:
            constraint = std::make_shared<cstr::ParallelConstraint>(m_firstRef, m_hoveredRef);
            break;
        case Mode::Tangent:
            constraint = std::make_shared<cstr::TangentConstraint>(m_firstRef, m_hoveredRef);
            break;
        case Mode::Equal:
            constraint = std::make_shared<cstr::EqualConstraint>(m_firstRef, m_hoveredRef);
            break;
        case Mode::Fixed: {
            // Extract current position
            const auto* entity = cstr::findEntity(m_firstRef.entityId, entities);
            if (!entity) return;
            math::Vec2 pos = cstr::extractPoint(m_firstRef, *entity);
            constraint = std::make_shared<cstr::FixedConstraint>(m_firstRef, pos);
            break;
        }
        case Mode::Distance: {
            // Measure current distance as default
            const auto* e1 = cstr::findEntity(m_firstRef.entityId, entities);
            const auto* e2 = cstr::findEntity(m_hoveredRef.entityId, entities);
            if (!e1 || !e2) return;
            math::Vec2 p1 = cstr::extractPoint(m_firstRef, *e1);
            math::Vec2 p2 = cstr::extractPoint(m_hoveredRef, *e2);
            double dist = p1.distanceTo(p2);
            bool ok = false;
            double val = QInputDialog::getDouble(m_viewport, "Distance Constraint",
                                                  "Distance:", dist, 0.0, 1e9, 4, &ok);
            if (!ok) return;
            constraint = std::make_shared<cstr::DistanceConstraint>(m_firstRef, m_hoveredRef, val);
            break;
        }
        case Mode::Angle: {
            // Measure current angle as default
            const auto* e1 = cstr::findEntity(m_firstRef.entityId, entities);
            const auto* e2 = cstr::findEntity(m_hoveredRef.entityId, entities);
            if (!e1 || !e2) return;
            auto [s1, e1p] = cstr::extractLine(m_firstRef, *e1);
            auto [s2, e2p] = cstr::extractLine(m_hoveredRef, *e2);
            math::Vec2 d1 = e1p - s1, d2 = e2p - s2;
            double angle = std::atan2(d1.cross(d2), d1.dot(d2));
            double angleDeg = math::radToDeg(angle);
            bool ok = false;
            double val = QInputDialog::getDouble(m_viewport, "Angle Constraint",
                                                  "Angle (degrees):", angleDeg, -360.0, 360.0, 2,
                                                  &ok);
            if (!ok) return;
            double angleRad = math::degToRad(val);
            constraint = std::make_shared<cstr::AngleConstraint>(m_firstRef, m_hoveredRef, angleRad);
            break;
        }
    }

    if (!constraint) return;

    // Snapshot entity states before solve
    auto refIds = constraint->referencedEntityIds();
    std::vector<doc::ApplyConstraintSolveCommand::EntitySnapshot> snapshots;
    for (uint64_t id : refIds) {
        for (const auto& entity : entities) {
            if (entity->id() == id) {
                snapshots.push_back({id, entity->clone(), nullptr});
                break;
            }
        }
    }

    // Build composite: add constraint + solve
    auto composite = std::make_unique<doc::CompositeCommand>(
        "Add " + constraint->typeName() + " Constraint");
    composite->addCommand(
        std::make_unique<doc::AddConstraintCommand>(csys, constraint));

    // Run solver
    auto paramTable = cstr::ParameterTable::buildFromEntities(entities, csys);
    cstr::SketchSolver solver;
    auto result = solver.solve(paramTable, csys);

    if (result.status == cstr::SolveStatus::Success ||
        result.status == cstr::SolveStatus::Converged ||
        result.status == cstr::SolveStatus::UnderConstrained) {
        // Apply solved positions
        // We need non-const access to entities for apply
        auto& mutableEntities = draftDoc.entities();
        paramTable.applyToEntities(mutableEntities);

        // Capture after-states
        for (auto& snap : snapshots) {
            for (const auto& entity : entities) {
                if (entity->id() == snap.entityId) {
                    snap.afterState = entity->clone();
                    break;
                }
            }
        }

        // Undo the application (the command will re-apply on execute)
        // Actually, since CompositeCommand calls execute() on push,
        // and we already applied, we need to undo and let the command redo.
        // But the apply already happened... let's just record the states.
        // We'll undo by restoring before-states on undo.
        // The ApplyConstraintSolveCommand's execute is a no-op since
        // the solver already applied the changes.
    }

    // Only add solve command if positions actually changed
    bool positionsChanged = false;
    for (auto& snap : snapshots) {
        if (snap.afterState) {
            positionsChanged = true;
            break;
        }
    }
    if (positionsChanged) {
        // For undo/redo: first undo reverts to before-states,
        // first redo applies after-states
        composite->addCommand(
            std::make_unique<doc::ApplyConstraintSolveCommand>(draftDoc, std::move(snapshots)));
    }

    // Push (execute is already done, but push records for undo)
    // Note: UndoStack::push calls execute() which will re-execute.
    // We need to handle this carefully. The AddConstraintCommand will try to add again,
    // but the constraint is already added. Let me restructure...

    // Actually, let's undo what we did manually, then let push() execute properly.
    // Remove the constraint we added manually (solver needed it in the system)
    csys.removeConstraint(constraint->id());

    // Restore entity positions to before-state
    for (const auto& snap : snapshots) {
        if (!snap.beforeState) continue;
        for (auto& entity : draftDoc.entities()) {
            if (entity->id() == snap.entityId) {
                // Copy geometry from beforeState back
                if (auto* sl = dynamic_cast<const draft::DraftLine*>(snap.beforeState.get())) {
                    if (auto* dl = dynamic_cast<draft::DraftLine*>(entity.get())) {
                        dl->setStart(sl->start());
                        dl->setEnd(sl->end());
                    }
                } else if (auto* sc = dynamic_cast<const draft::DraftCircle*>(snap.beforeState.get())) {
                    if (auto* dc = dynamic_cast<draft::DraftCircle*>(entity.get())) {
                        dc->setCenter(sc->center());
                        dc->setRadius(sc->radius());
                    }
                } else if (auto* sa = dynamic_cast<const draft::DraftArc*>(snap.beforeState.get())) {
                    if (auto* da = dynamic_cast<draft::DraftArc*>(entity.get())) {
                        da->setCenter(sa->center());
                        da->setRadius(sa->radius());
                        da->setStartAngle(sa->startAngle());
                        da->setEndAngle(sa->endAngle());
                    }
                } else if (auto* sr = dynamic_cast<const draft::DraftRectangle*>(snap.beforeState.get())) {
                    if (auto* dr = dynamic_cast<draft::DraftRectangle*>(entity.get())) {
                        dr->setCorner1(sr->corner1());
                        dr->setCorner2(sr->corner2());
                    }
                } else if (auto* sp = dynamic_cast<const draft::DraftPolyline*>(snap.beforeState.get())) {
                    if (auto* dp = dynamic_cast<draft::DraftPolyline*>(entity.get())) {
                        dp->setPoints(sp->points());
                    }
                }
                break;
            }
        }
    }

    // Now push the composite — it will execute AddConstraint + ApplyConstraintSolve
    doc.undoStack().push(std::move(composite));
    doc.setDirty(true);
}

std::vector<std::pair<math::Vec2, math::Vec2>> ConstraintTool::getPreviewLines() const {
    std::vector<std::pair<math::Vec2, math::Vec2>> lines;
    if (!m_viewport || !m_viewport->document()) return lines;

    // Show highlighted line feature
    if (m_hoveredRef.isValid() && m_hoveredRef.featureType == cstr::FeatureType::Line) {
        const auto* entity = cstr::findEntity(
            m_hoveredRef.entityId, m_viewport->document()->draftDocument().entities());
        if (entity) {
            try {
                auto [s, e] = cstr::extractLine(m_hoveredRef, *entity);
                lines.push_back({s, e});
            } catch (...) {}
        }
    }

    // Show first selected feature if it's a line
    if (m_state == State::WaitingForSecond && m_firstRef.isValid() &&
        m_firstRef.featureType == cstr::FeatureType::Line) {
        const auto* entity = cstr::findEntity(
            m_firstRef.entityId, m_viewport->document()->draftDocument().entities());
        if (entity) {
            try {
                auto [s, e] = cstr::extractLine(m_firstRef, *entity);
                lines.push_back({s, e});
            } catch (...) {}
        }
    }

    return lines;
}

std::vector<std::pair<math::Vec2, double>> ConstraintTool::getPreviewCircles() const {
    std::vector<std::pair<math::Vec2, double>> circles;
    if (!m_viewport || !m_viewport->document()) return circles;

    double ptRadius = 5.0 * m_viewport->pixelToWorldScale();

    // Show highlighted point feature
    if (m_hoveredRef.isValid() && m_hoveredRef.featureType == cstr::FeatureType::Point) {
        const auto* entity = cstr::findEntity(
            m_hoveredRef.entityId, m_viewport->document()->draftDocument().entities());
        if (entity) {
            try {
                math::Vec2 p = cstr::extractPoint(m_hoveredRef, *entity);
                circles.push_back({p, ptRadius});
            } catch (...) {}
        }
    }

    // Show first selected point
    if (m_state == State::WaitingForSecond && m_firstRef.isValid() &&
        m_firstRef.featureType == cstr::FeatureType::Point) {
        const auto* entity = cstr::findEntity(
            m_firstRef.entityId, m_viewport->document()->draftDocument().entities());
        if (entity) {
            try {
                math::Vec2 p = cstr::extractPoint(m_firstRef, *entity);
                circles.push_back({p, ptRadius});
            } catch (...) {}
        }
    }

    return circles;
}

}  // namespace hz::ui
