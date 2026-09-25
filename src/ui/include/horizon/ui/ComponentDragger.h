#pragma once

#include <QPointF>
#include <array>
#include <cstdint>
#include <optional>

#include "horizon/math/Vec3.h"
#include "horizon/ui/Triad.h"

namespace hz::ui {

/// What moves an assembly's component under the cursor (Phase 158): the
/// viewport hands it a drag that starts on a component, and it solves the
/// mates as the component moves.
class ComponentDragger {
public:
    virtual ~ComponentDragger() = default;

    /// Where the chosen component's triad stands (Phase 158b): its middle,
    /// and its axes; nothing when no component is chosen.
    struct TriadPose {
        std::uint64_t component = 0;
        math::Vec3 origin;
        std::array<math::Vec3, 3> axes{math::Vec3::UnitX, math::Vec3::UnitY, math::Vec3::UnitZ};
    };
    virtual std::optional<TriadPose> triadPose() const = 0;

    /// A drag of @p component begun at @p at (the viewport's coordinates),
    /// the point pressed: free under the cursor, or by one of its triad's
    /// @p handle (along an arrow, round a ring). False when it cannot be
    /// dragged (no assembly shown, a component held by a Fixed mate): the
    /// dragger says why.
    virtual bool beginDrag(std::uint64_t component, const QPointF& at,
                           const std::optional<Triad::Handle>& handle) = 0;
    /// The cursor at @p at, the button still down.
    virtual void dragTo(const QPointF& at) = 0;
    /// The button released: the drag kept, as one undo step.
    virtual void endDrag() = 0;
    /// Escape, or the view lost: everything back where it was.
    virtual void cancelDrag() = 0;
};

}  // namespace hz::ui
