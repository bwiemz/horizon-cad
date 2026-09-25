#pragma once

#include <cstddef>
#include <functional>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "horizon/ui/Tool.h"

namespace hz::ui {

/// Points clicked in the view for a command (Phase 149): a detail's centre
/// and radius, or a view and the place it goes to.
///
/// Each click is offered to the command, which may refuse it (a centre
/// outside every view), saying why in the prompt. The last click hands over
/// every point when its button is released. The command then ends the tool,
/// with no release left over to reach the tool that follows.
class PickTool : public Tool {
public:
    /// Whether the command takes @p point as click @p step. A refusal says
    /// why in @p why, which the prompt shows.
    using Accept = std::function<bool(std::size_t step, const math::Vec2& point, std::string& why)>;
    /// Every point, once the last is clicked.
    using Done = std::function<void(const std::vector<math::Vec2>& points)>;
    /// What to show with the pointer at @p pointer after @p picked.
    using Lines = std::function<std::vector<std::pair<math::Vec2, math::Vec2>>(
        const std::vector<math::Vec2>& picked, const math::Vec2& pointer)>;
    using Circles = std::function<std::vector<std::pair<math::Vec2, double>>(
        const std::vector<math::Vec2>& picked, const math::Vec2& pointer)>;

    /// One click for each of @p prompts, which the status bar shows in turn.
    PickTool(std::string name, std::vector<std::string> prompts, Accept accept, Done done);

    void setPreview(Lines lines, Circles circles);

    std::string name() const override { return m_name; }
    void activate(ViewportWidget* viewport) override;
    void deactivate() override;
    bool mousePressEvent(QMouseEvent* event, const math::Vec2& worldPos) override;
    bool mouseMoveEvent(QMouseEvent* event, const math::Vec2& worldPos) override;
    bool mouseReleaseEvent(QMouseEvent* event, const math::Vec2& worldPos) override;
    bool keyPressEvent(QKeyEvent* event) override;
    void cancel() override;

    bool acceptsTypedPoints() const override { return true; }
    std::optional<math::Vec2> basePoint() const override;
    std::vector<std::pair<math::Vec2, math::Vec2>> getPreviewLines() const override;
    std::vector<std::pair<math::Vec2, double>> getPreviewCircles() const override;
    std::string promptText() const override;
    bool wantsCrosshair() const override { return true; }

private:
    /// @p p snapped as the drawing tools snap it.
    math::Vec2 snapped(const math::Vec2& p);
    void reset();

    std::string m_name;
    std::vector<std::string> m_prompts;
    Accept m_accept;
    Done m_done;
    Lines m_lines;
    Circles m_circles;
    std::vector<math::Vec2> m_picked;
    math::Vec2 m_pointer;
    std::string m_refusal;
    /// The last point is taken; the points go when its button comes up.
    bool m_complete = false;
};

}  // namespace hz::ui
