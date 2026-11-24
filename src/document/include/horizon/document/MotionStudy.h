#pragma once

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "horizon/document/AssemblyDocument.h"
#include "horizon/document/AssemblyMates.h"
#include "horizon/math/Mat4.h"
#include "horizon/topology/Solid.h"

namespace hz::doc {

class DocumentManager;

/// A motion study: one Distance or Angle mate driven from one value to
/// another, in steps, and the assembly's mates solved at each, as a motor
/// turning a hinge or a slide moving a carriage drives a mechanism. It finds
/// how far the mechanism goes, where it binds, and where parts collide,
/// without moving the components (see AssemblyDocument::setPose to show it).
struct MotionStudy {
    /// The driving mate, by id: a Distance mate (a length) or an Angle mate
    /// (radians, from 0 to pi: the angle between its faces is unsigned).
    uint64_t mate = 0;
    double from = 0.0;
    double to = 0.0;
    /// Positions after the first: from, then `steps` more evenly to `to`.
    int steps = 24;
    /// Check each position for interference (shared material).
    bool checkInterference = true;

    static constexpr int kMaxSteps = 720;
};

/// One position of a motion.
struct MotionFrame {
    double value = 0.0;                          ///< the driving mate's
    std::map<uint64_t, math::Mat4> placements;   ///< every component's, by id
    std::vector<ComponentInterference> clashes;  ///< when checked
};

/// What a motion study came to.
struct MotionResult {
    /// The positions, from `from`, as far as the mates hold: none when the
    /// study cannot start.
    std::vector<MotionFrame> frames;
    /// It reached `to`.
    bool complete = false;
    /// Why not, when not: the mates bind (`bound`), or it was cancelled, or
    /// it could not start.
    std::string why;
    /// The first value the mates could not reach, when they bind there.
    std::optional<double> bound;
    /// The first position where components clash, when one does.
    std::optional<std::size_t> firstClash;
    /// Components it could not check for interference: no solid.
    std::vector<uint64_t> unchecked;
};

/// What a motion study needs of an assembly, copied (prepareMotion), so it
/// can run on a worker while the assembly is edited.
struct MotionInput {
    AssemblyMates mates;
    /// The driving mate's place among the mates, and its type.
    std::size_t driver = 0;
    MateType driverType = MateType::Distance;
    /// A pattern instance, placed from its seed: `relative * seed`.
    struct Follower {
        uint64_t component = 0;
        uint64_t seed = 0;
        math::Mat4 relative = math::Mat4::identity();
    };
    std::vector<Follower> followers;
    /// Each unsuppressed component's solid in its part, unplaced, to be
    /// placed at each position and checked.
    std::vector<std::pair<uint64_t, std::shared_ptr<const topo::Solid>>> solids;
    std::vector<uint64_t> unchecked;
};

/// Ready @p assembly for a study driven by mate @p mate: its components'
/// parts read from @p documents first (a mate's faces are found there), as
/// placeByMates does. On the GUI thread. Nullopt, and why in @p why, when it
/// cannot be driven: no such mate, not a Distance or Angle mate, or a mate
/// naming geometry that is not there.
std::optional<MotionInput> prepareMotion(AssemblyDocument& assembly, DocumentManager& documents,
                                         uint64_t mate, std::string* why = nullptr);

/// Run @p study on @p input: on any thread. Stops at the first step the
/// mates cannot reach, or once @p cancelled is set, with what it reached.
MotionResult runMotion(const MotionInput& input, const MotionStudy& study,
                       const std::atomic<bool>* cancelled = nullptr);

}  // namespace hz::doc
