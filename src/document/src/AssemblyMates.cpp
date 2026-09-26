#include "horizon/document/AssemblyMates.h"

#include <algorithm>

#include "horizon/document/Document.h"
#include "horizon/modeling/MateGeometry.h"

namespace hz::doc {

namespace {

/// The frame of @p ref's face in its component's part (the part's own
/// coordinates); nullopt when the component, its part or the face is not
/// there, or the face has no frame (not flat, not a cylinder).
std::optional<model::MateFrame> frameOf(const AssemblyDocument& assembly,
                                        const MateReference& ref) {
    const ComponentInstance* comp = assembly.component(ref.componentId);
    // A part's solid, or a subassembly's gathered one (Phase 159), whose
    // faces are named "c<id>/..." after its own components.
    const topo::Solid* solid = comp != nullptr ? comp->solid() : nullptr;
    if (solid == nullptr) return std::nullopt;
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
        model::SolverComponent sc;
        sc.id = comp.id;
        sc.transform = comp.transform;
        out.m_components.push_back(sc);
    }
    out.m_mates.reserve(assembly.mates().size());
    for (const auto& mate : assembly.mates()) {
        model::SolverMate sm;
        sm.type = mate.type;
        sm.componentA = mate.a.componentId;
        sm.componentB = mate.b.componentId;
        sm.value = mate.value;
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
