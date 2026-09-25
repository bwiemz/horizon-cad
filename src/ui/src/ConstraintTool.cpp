#include "horizon/ui/ConstraintTool.h"

#include <QMouseEvent>
#include <cmath>

#include "horizon/constraint/ConstraintSystem.h"
#include "horizon/constraint/GeometryRef.h"
#include "horizon/document/Commands.h"
#include "horizon/document/ConstraintCommands.h"
#include "horizon/document/ConstraintSolveHelper.h"
#include "horizon/document/Document.h"
#include "horizon/drafting/DraftArc.h"
#include "horizon/drafting/DraftCircle.h"
#include "horizon/drafting/DraftLine.h"
#include "horizon/drafting/DraftPolyline.h"
#include "horizon/drafting/Layer.h"
#include "horizon/math/MathUtils.h"
#include "horizon/ui/FeatureForm.h"
#include "horizon/ui/QuantitySpinBox.h"
#include "horizon/ui/ViewportWidget.h"

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

    const auto& doc = m_viewport->document()->activeDrawing();
    const auto& layerMgr = m_viewport->document()->layerManager();
    double tolerance = m_viewport->pickTolerance(10.0);

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

bool ConstraintTool::mouseReleaseEvent(QMouseEvent* /*event*/, const math::Vec2& /*worldPos*/) {
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
    auto& csys = doc.activeConstraints();
    auto& draftDoc = doc.activeDrawing();
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
            constraint = std::make_shared<cstr::PerpendicularConstraint>(m_firstRef, m_hoveredRef);
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
            const auto pos = entity ? cstr::pointOf(m_firstRef, *entity) : std::nullopt;
            if (!pos) return;
            constraint = std::make_shared<cstr::FixedConstraint>(m_firstRef, *pos);
            break;
        }
        case Mode::Distance: {
            // Measure current distance as default
            const auto* e1 = cstr::findEntity(m_firstRef.entityId, entities);
            const auto* e2 = cstr::findEntity(m_hoveredRef.entityId, entities);
            if (!e1 || !e2) return;
            const auto p1 = cstr::pointOf(m_firstRef, *e1);
            const auto p2 = cstr::pointOf(m_hoveredRef, *e2);
            if (!p1 || !p2) return;
            double dist = p1->distanceTo(*p2);
            // In the document's unit, or typed in another (Phase 154).
            FeatureForm form(m_viewport, QObject::tr("Distance Constraint"),
                             m_viewport->document()->lengthUnit());
            auto* field =
                form.length(QStringLiteral("value"), QObject::tr("Distance:"), dist, 0.0, 1e9, 4);
            if (!form.exec()) return;
            const double val = field->value();
            constraint = std::make_shared<cstr::DistanceConstraint>(m_firstRef, m_hoveredRef, val);
            break;
        }
        case Mode::Angle: {
            // Measure current angle as default
            const auto* e1 = cstr::findEntity(m_firstRef.entityId, entities);
            const auto* e2 = cstr::findEntity(m_hoveredRef.entityId, entities);
            if (!e1 || !e2) return;
            const auto l1 = cstr::lineOf(m_firstRef, *e1);
            const auto l2 = cstr::lineOf(m_hoveredRef, *e2);
            if (!l1 || !l2) return;
            math::Vec2 d1 = l1->second - l1->first, d2 = l2->second - l2->first;
            double angle = std::atan2(d1.cross(d2), d1.dot(d2));
            double angleDeg = math::radToDeg(angle);
            FeatureForm form(m_viewport, QObject::tr("Angle Constraint"));
            auto* field = form.angle(QStringLiteral("value"), QObject::tr("Angle:"), angleDeg,
                                     -360.0, 360.0, 2);
            if (!form.exec()) return;
            const double val = field->value();
            double angleRad = math::degToRad(val);
            constraint =
                std::make_shared<cstr::AngleConstraint>(m_firstRef, m_hoveredRef, angleRad);
            break;
        }
    }

    if (!constraint) return;

    // Build composite: add constraint + solve
    auto composite =
        std::make_unique<doc::CompositeCommand>("Add " + constraint->typeName() + " Constraint");

    // Add the constraint command to the composite.
    composite->addCommand(std::make_unique<doc::AddConstraintCommand>(csys, constraint));

    // Temporarily add constraint so solver can see it.
    csys.addConstraint(constraint);

    // Use helper to solve and create apply command.
    auto resolver = doc.variableResolver();
    auto solveCmd = doc::ConstraintSolveHelper::solveAndCreateCommand(draftDoc, csys, resolver);
    if (solveCmd) {
        // Undo the solve (push will re-execute via the command).
        solveCmd->undo();
        composite->addCommand(std::move(solveCmd));
    }

    // Remove the temporary constraint (push re-adds via AddConstraintCommand).
    csys.removeConstraint(constraint->id());

    doc.undoStack().push(std::move(composite));
    doc.setDirty(true);
}

std::vector<std::pair<math::Vec2, math::Vec2>> ConstraintTool::getPreviewLines() const {
    std::vector<std::pair<math::Vec2, math::Vec2>> lines;
    if (!m_viewport || !m_viewport->document()) return lines;
    const auto& draft = m_viewport->document()->activeDrawing();
    // A ref that no longer fits its entity (edited meanwhile) highlights
    // nothing: this is only a preview.
    const auto show = [&](const cstr::GeometryRef& ref) {
        if (!ref.isValid() || ref.featureType != cstr::FeatureType::Line) return;
        const auto* entity = draft.findEntity(ref.entityId);
        if (!entity) return;
        if (auto line = cstr::lineOf(ref, *entity)) lines.push_back(*line);
    };
    show(m_hoveredRef);                                        // the line under the cursor
    if (m_state == State::WaitingForSecond) show(m_firstRef);  // the first one picked
    return lines;
}

std::vector<std::pair<math::Vec2, double>> ConstraintTool::getPreviewCircles() const {
    std::vector<std::pair<math::Vec2, double>> circles;
    if (!m_viewport || !m_viewport->document()) return circles;
    const auto& draft = m_viewport->document()->activeDrawing();
    const double ptRadius = 5.0 * m_viewport->pixelToWorldScale();
    const auto show = [&](const cstr::GeometryRef& ref) {
        if (!ref.isValid() || ref.featureType != cstr::FeatureType::Point) return;
        const auto* entity = draft.findEntity(ref.entityId);
        if (!entity) return;
        if (auto point = cstr::pointOf(ref, *entity)) circles.push_back({*point, ptRadius});
    };
    show(m_hoveredRef);                                        // the point under the cursor
    if (m_state == State::WaitingForSecond) show(m_firstRef);  // the first one picked
    return circles;
}

std::string ConstraintTool::promptText() const {
    static const char* modeNames[] = {"Coincident", "Horizontal", "Vertical", "Perpendicular",
                                      "Parallel",   "Tangent",    "Equal",    "Fixed",
                                      "Distance",   "Angle"};
    int idx = static_cast<int>(m_mode);
    const char* modeName = (idx >= 0 && idx < 10) ? modeNames[idx] : "constraint";

    if (m_state == State::WaitingForFirst)
        return std::string("Select first entity for ") + modeName;
    return std::string("Select second entity for ") + modeName;
}

bool ConstraintTool::wantsCrosshair() const {
    return false;
}

}  // namespace hz::ui
