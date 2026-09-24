#pragma once

#include <memory>
#include <optional>
#include <string>

#include "horizon/drafting/DraftLinearDimension.h"
#include "horizon/math/Vec2.h"
#include "horizon/ui/Tool.h"

namespace hz::ui {

/// Dimension ▸ Continue and Baseline: more linear dimensions from a
/// horizontal or vertical one, each to the next point clicked or typed.
/// - Continue measures from where the last dimension ended, its dimension
///   line in line with the last one's.
/// - Baseline measures from where the first began, each dimension line a
///   step further out than the one before.
///
/// It starts from the last horizontal or vertical dimension in the drawing,
/// or asks for one to be picked; Enter picks another. Escape stops.
class ChainDimensionTool : public Tool {
public:
    enum class Mode { Continue, Baseline };

    explicit ChainDimensionTool(Mode mode) : m_mode(mode) {}

    std::string name() const override {
        return m_mode == Mode::Continue ? "Continue Dimension" : "Baseline Dimension";
    }

    void activate(ViewportWidget* viewport) override;
    void deactivate() override;
    bool mousePressEvent(QMouseEvent* event, const math::Vec2& worldPos) override;
    bool mouseMoveEvent(QMouseEvent* event, const math::Vec2& worldPos) override;
    bool mouseReleaseEvent(QMouseEvent* event, const math::Vec2& worldPos) override;
    bool keyPressEvent(QKeyEvent* event) override;
    void cancel() override;

    std::vector<std::pair<math::Vec2, math::Vec2>> getPreviewLines() const override;
    std::string promptText() const override;
    bool wantsCrosshair() const override { return true; }

    bool acceptsTypedPoints() const override { return m_chain.has_value(); }
    std::optional<math::Vec2> basePoint() const override;

    /// The dimension the next point makes, from the chain as it stands
    /// (null before a base is picked). Public for tests.
    std::shared_ptr<draft::DraftLinearDimension> next(const math::Vec2& point) const;

private:
    /// Where a chain stands: what the next dimension measures from, and
    /// where its dimension line goes.
    struct Chain {
        draft::DraftLinearDimension::Orientation orientation;
        math::Vec2 from;  ///< the next dimension's first point
        double lineAt;    ///< the dimension line's y (horizontal) or x (vertical)
        double outward;   ///< +1 or -1: the side of the points the line is on
        double step;      ///< Baseline: how much further out each line goes
    };

    /// A chain starting at @p base, if it is horizontal or vertical.
    std::optional<Chain> chainFrom(const draft::DraftLinearDimension& base) const;
    /// The last horizontal or vertical dimension in the drawing, on a layer
    /// that is shown and not locked.
    const draft::DraftLinearDimension* lastDimension() const;
    /// The horizontal or vertical dimension under @p point, likewise.
    const draft::DraftLinearDimension* dimensionAt(const math::Vec2& point) const;

    Mode m_mode;
    std::optional<Chain> m_chain;
    math::Vec2 m_cursor;
};

}  // namespace hz::ui
