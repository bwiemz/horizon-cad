#pragma once

#include <QPointF>
#include <cstdint>

namespace hz::ui {

/// What moves an assembly's component under the cursor (Phase 158): the
/// viewport hands it a drag that starts on a component, and it solves the
/// mates as the component moves.
class ComponentDragger {
public:
    virtual ~ComponentDragger() = default;

    /// A drag of @p component begun at @p at (the viewport's coordinates),
    /// the point pressed. False when it cannot be dragged (no assembly
    /// shown, a component held by a Fixed mate): the dragger says why.
    virtual bool beginDrag(std::uint64_t component, const QPointF& at) = 0;
    /// The cursor at @p at, the button still down.
    virtual void dragTo(const QPointF& at) = 0;
    /// The button released: the drag kept, as one undo step.
    virtual void endDrag() = 0;
    /// Escape, or the view lost: everything back where it was.
    virtual void cancelDrag() = 0;
};

}  // namespace hz::ui
