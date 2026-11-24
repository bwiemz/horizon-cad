#include "horizon/document/MotionStudy.h"

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <numbers>
#include <utility>

#include "horizon/document/DocumentManager.h"
#include "horizon/modeling/AssemblySolver.h"
#include "horizon/modeling/InterferenceChecker.h"
#include "horizon/modeling/Pattern.h"

namespace hz::doc {

namespace {

/// The components that share material with @p input's solids placed at
/// @p at, as the Interference check measures it.
std::vector<ComponentInterference> clashesAt(const MotionInput& input,
                                             const std::map<uint64_t, math::Mat4>& at,
                                             const std::atomic<bool>* cancelled) {
    std::vector<std::unique_ptr<topo::Solid>> placed;
    std::vector<uint64_t> ids;
    for (const auto& [id, solid] : input.solids) {
        const auto where = at.find(id);
        if (where == at.end()) continue;
        placed.push_back(model::Pattern::transformed(*solid, where->second));
        ids.push_back(id);
    }
    std::vector<const topo::Solid*> solids;
    solids.reserve(placed.size());
    for (const auto& s : placed) solids.push_back(s.get());
    std::vector<ComponentInterference> clashes;
    for (const auto& pair : model::InterferenceChecker::check(solids, cancelled)) {
        ComponentInterference clash;
        clash.componentA = ids[pair.indexA];
        clash.componentB = ids[pair.indexB];
        clash.volume = pair.volume;
        clash.volumeResolved = pair.volumeResolved;
        clashes.push_back(clash);
    }
    return clashes;
}

}  // namespace

std::optional<MotionInput> prepareMotion(AssemblyDocument& assembly, DocumentManager& documents,
                                         uint64_t mate, std::string* why) {
    const auto refuse = [why](std::string reason) -> std::optional<MotionInput> {
        if (why != nullptr) *why = std::move(reason);
        return std::nullopt;
    };
    const auto& mates = assembly.mates();
    const auto driver =
        std::find_if(mates.begin(), mates.end(), [mate](const Mate& m) { return m.id == mate; });
    if (driver == mates.end()) return refuse("there is no mate " + std::to_string(mate));
    if (driver->type != MateType::Distance && driver->type != MateType::Angle) {
        return refuse("a motion is driven by a Distance or an Angle mate");
    }
    const std::size_t driverIndex = static_cast<std::size_t>(driver - mates.begin());
    const MateType driverType = driver->type;

    // As placeByMates: a mate's frames come from its components' faces, so
    // their parts are read whole, a relative path from the assembly's folder.
    const std::string dir = assembly.filePath().empty()
                                ? std::string()
                                : std::filesystem::path(assembly.filePath()).parent_path().string();
    for (auto& comp : assembly.components()) {
        if (comp.solid() == nullptr) {
            documents.resolveComponent(comp, ComponentState::Resolved, dir);
        }
    }
    auto gathered = AssemblyMates::gather(assembly, why);
    if (!gathered) return std::nullopt;

    MotionInput input;
    input.mates = std::move(*gathered);
    input.driver = driverIndex;
    input.driverType = driverType;
    for (const auto& comp : assembly.components()) {
        if (comp.isPatternInstance()) {
            if (const ComponentPattern* pattern = assembly.pattern(comp.patternId)) {
                input.followers.push_back(
                    {comp.id, comp.seedId, pattern->instanceTransform(comp.patternIndex)});
            }
        }
        if (comp.suppressed) continue;
        const topo::Solid* solid = comp.solid();  // a part's, or a subassembly's
        if (solid == nullptr) {
            input.unchecked.push_back(comp.id);
            continue;
        }
        // A copy, which the study places at each position: the part may be
        // rebuilt while the study runs on a worker.
        input.solids.emplace_back(comp.id,
                                  model::Pattern::transformed(*solid, math::Mat4::identity()));
    }
    return input;
}

MotionResult runMotion(const MotionInput& input, const MotionStudy& study,
                       const std::atomic<bool>* cancelled) {
    MotionResult out;
    out.unchecked = input.unchecked;
    if (!std::isfinite(study.from) || !std::isfinite(study.to)) {
        out.why = "its range is not a pair of numbers";
        return out;
    }
    // An Angle mate measures the angle between its faces' normals, from 0
    // to 180 degrees, unsigned: past either end it can never be held, which
    // would read as the mechanism binding.
    constexpr double kSlack = 1e-9;
    const auto outside = [](double angle) {
        return angle < -kSlack || angle > std::numbers::pi + kSlack;
    };
    if (input.driverType == MateType::Angle && (outside(study.from) || outside(study.to))) {
        out.why = "an angle mate measures from 0 to 180 degrees";
        return out;
    }
    const int steps = std::clamp(study.steps, 1, MotionStudy::kMaxSteps);
    // Driven, and placed where each step leaves it: the next is solved from
    // there, as a drag is, so the mechanism moves on rather than jumping to
    // another way of meeting its mates.
    AssemblyMates mates = input.mates;
    for (int k = 0; k <= steps; ++k) {
        if (cancelled != nullptr && cancelled->load()) {
            out.why = "cancelled";
            return out;
        }
        const double value = study.from + (study.to - study.from) * static_cast<double>(k) / steps;
        mates.drive(input.driver, value);
        AssemblyMates::Options options;
        options.diagnostics = false;
        const model::AssemblySolveResult solved = mates.solve(options);
        if (solved.status != model::AssemblySolveStatus::Success) {
            out.bound = value;
            out.why = "the mates do not hold there";
            return out;
        }
        mates.place(solved.transforms);

        MotionFrame frame;
        frame.value = value;
        for (const auto& component : mates.components()) {
            frame.placements[component.id] = component.transform;
        }
        // A pattern's instance follows its seed.
        for (const auto& follower : input.followers) {
            const auto seed = frame.placements.find(follower.seed);
            if (seed != frame.placements.end()) {
                frame.placements[follower.component] = follower.relative * seed->second;
            }
        }
        if (study.checkInterference) {
            frame.clashes = clashesAt(input, frame.placements, cancelled);
            if (!frame.clashes.empty() && !out.firstClash) out.firstClash = out.frames.size();
        }
        out.frames.push_back(std::move(frame));
    }
    out.complete = true;
    return out;
}

}  // namespace hz::doc
