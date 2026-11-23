#pragma once

#include <cstdint>
#include <string>

#include "horizon/math/Vec2.h"
#include "horizon/ui/Tool.h"

namespace hz::draft {
class DraftDocument;
class DraftEntity;
class LayerManager;
}  // namespace hz::draft

namespace hz::ui {

/// Match Properties (AutoCAD's MATCHPROP, SolidWorks' Format Painter): click
/// an entity, then each entity that is to look like it. Each takes the
/// first's layer, colour, line width and line type, as one step to undo.
/// With one entity selected as the tool starts, that one is the first. The
/// first may be on a locked layer, since nothing is done to it; what takes
/// its properties may not. Escape picks a new first; Escape again leaves.
class MatchPropertiesTool : public Tool {
public:
    std::string name() const override { return "Match Properties"; }

    void activate(ViewportWidget* viewport) override;
    void deactivate() override;

    bool mousePressEvent(QMouseEvent* event, const math::Vec2& worldPos) override;
    bool mouseMoveEvent(QMouseEvent* event, const math::Vec2& worldPos) override;
    bool mouseReleaseEvent(QMouseEvent* event, const math::Vec2& worldPos) override;
    bool keyPressEvent(QKeyEvent* event) override;
    void cancel() override;

    std::string promptText() const override;
    bool wantsCrosshair() const override { return true; }

    /// The entity whose properties are given, or 0 while none is picked.
    uint64_t sourceId() const { return m_sourceId; }

    /// The entity of @p drawing under @p point, within @p tolerance, on a
    /// layer that is shown (and unlocked, with @p unlockedOnly): found
    /// through the drawing's index, not by testing every entity. Null when
    /// there is none.
    static const draft::DraftEntity* entityAt(const draft::DraftDocument& drawing,
                                              const draft::LayerManager& layers,
                                              const math::Vec2& point, double tolerance,
                                              bool unlockedOnly);

private:
    /// The source, if it is still in the drawing on a shown layer.
    const draft::DraftEntity* source() const;
    /// Give @p target the source's properties, as one step; false when it
    /// has them already.
    bool matchTo(const draft::DraftEntity& target);

    uint64_t m_sourceId = 0;
    /// What the last click did, said in the prompt.
    std::string m_note;
};

}  // namespace hz::ui
