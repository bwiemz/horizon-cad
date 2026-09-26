#include "horizon/document/AssemblyMates.h"

#include <algorithm>

#include "horizon/document/Document.h"
#include "horizon/document/FeatureTree.h"
#include "horizon/modeling/MateGeometry.h"

namespace hz::doc {

namespace {

/// The frame of @p ref's face in its component's part (the part's own
/// coordinates); nullopt when the component, its part or the face is not
/// there, or the face has no frame (not flat, not a cylinder).
std::optional<model::MateFrame> frameOf(const AssemblyDocument& assembly,
                                        const MateReference& ref) {
    const ComponentInstance* comp = assembly.component(ref.componentId);
    if (comp == nullptr) return std::nullopt;
    // A datum of its part (Phase 160): a plane, an axis or a point, as the
    // part keeps it.
    if (ref.kind == ReferenceKind::Datum) {
        if (!comp->resolvedPart) return std::nullopt;
        const FeatureTree& tree = comp->resolvedPart->featureTree();
        for (size_t i = 0; i < tree.featureCount(); ++i) {
            const auto* datum = dynamic_cast<const DatumFeature*>(tree.feature(i));
            if (datum == nullptr || datum->featureID() != ref.faceId.tag()) continue;
            model::MateFrame frame;
            switch (datum->datumKind()) {
                case DatumFeature::DatumKind::Plane:
                    frame.kind = model::MateFrameKind::Planar;
                    frame.origin = datum->asPlane().origin;
                    frame.direction = datum->asPlane().normal;
                    break;
                case DatumFeature::DatumKind::Axis:
                    frame.kind = model::MateFrameKind::Line;
                    frame.origin = datum->asAxis().origin;
                    frame.direction = datum->asAxis().direction;
                    break;
                case DatumFeature::DatumKind::Point:
                    frame.kind = model::MateFrameKind::Point;
                    frame.origin = datum->asPoint().position;
                    break;
            }
            return frame;
        }
        return std::nullopt;
    }
    // A part's solid, or a subassembly's gathered one (Phase 159), whose
    // faces are named "c<id>/..." after its own components.
    const topo::Solid* solid = comp->solid();
    if (solid == nullptr) return std::nullopt;
    if (ref.kind == ReferenceKind::Edge) {
        return model::MateGeometry::frameForEdge(*solid, ref.faceId.tag());
    }
    const topo::Face* face = model::MateGeometry::findFace(*solid, ref.faceId);
    if (face == nullptr) return std::nullopt;
    return model::MateGeometry::frameForFace(*face);
}

}  // namespace

std::optional<AssemblyMates> AssemblyMates::gather(const AssemblyDocument& assembly,
                                                   std::string* why) {
    AssemblyMates out;
    out.m_components.reserve(assembly.components().size());
    for (const auto& comp : assembly.components()) {
        // A pattern's instance is placed from its seed (Phase 161), not
        // solved: it follows the seed after.
        if (comp.isPatternInstance()) continue;
        model::SolverComponent sc;
        sc.id = comp.id;
        sc.transform = comp.transform;
        out.m_components.push_back(sc);
    }
    out.m_mates.reserve(assembly.mates().size());
    for (const auto& mate : assembly.mates()) {
        for (const uint64_t side : {mate.a.componentId, mate.b.componentId}) {
            const ComponentInstance* comp = assembly.component(side);
            if (comp != nullptr && comp->isPatternInstance()) {
                if (why != nullptr) {
                    *why = "mate " + std::to_string(mate.id) + " is on " + comp->name +
                           ", which its pattern places: mate its seed";
                }
                return std::nullopt;
            }
        }
        model::SolverMate sm;
        sm.type = mate.type;
        sm.componentA = mate.a.componentId;
        sm.componentB = mate.b.componentId;
        sm.value = mate.value;
        sm.minimum = mate.minimum;
        sm.maximum = mate.maximum;
        if (mate.type != MateType::Fixed) {
            const auto a = frameOf(assembly, mate.a);
            const auto b = a ? frameOf(assembly, mate.b) : std::nullopt;
            if (!a || !b) {
                if (why != nullptr) {
                    *why = "mate " + std::to_string(mate.id) +
                           " references geometry that could not be resolved";
                }
                return std::nullopt;
            }
            sm.frameA = *a;
            sm.frameB = *b;
        }
        out.m_mates.push_back(sm);
    }
    return out;
}

model::AssemblySolveResult AssemblyMates::solve(const Options& options) const {
    std::vector<model::SolverComponent> components = m_components;
    if (options.hold) {
        for (auto& sc : components) {
            if (sc.id != options.hold->component) continue;
            sc.transform = options.hold->at;
            sc.grounded = true;
        }
        // What grounds the assembly without a hold grounds it still: with no
        // Fixed mate, the solver grounds the first component. Held elsewhere,
        // that would free it, and a mate the hold breaks would move it
        // instead of the component held.
        const bool fixedMate = std::any_of(m_mates.begin(), m_mates.end(), [](const auto& m) {
            return m.type == model::MateType::Fixed;
        });
        if (!fixedMate && !components.empty()) components.front().grounded = true;
    }
    model::AssemblySolver solver;
    solver.setComputeDiagnostics(options.diagnostics);
    return solver.solve(components, m_mates);
}

void AssemblyMates::place(const std::map<uint64_t, math::Mat4>& transforms) {
    for (auto& sc : m_components) {
        const auto found = transforms.find(sc.id);
        if (found != transforms.end()) sc.transform = found->second;
    }
}

}  // namespace hz::doc
