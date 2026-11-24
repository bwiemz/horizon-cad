#pragma once

#include <QDialog>
#include <QString>
#include <cstddef>
#include <functional>
#include <memory>

#include "horizon/document/MotionStudy.h"

class QLabel;
class QPushButton;
class QSlider;
class QTimer;

namespace hz::ui {

/// A motion study's positions, played (Assembly ▸ Motion Study): a slider
/// over them, Play, what each is (the driving mate's value, and any clash),
/// Keep This Position, and Close. Beside the window, not over it: the view
/// can be turned while the mechanism moves.
///
/// It shows a position through @p show, and stops showing any (show(npos))
/// when it closes. Its controls are named for tests and automation:
/// "motion_position", "motion_play", "motion_keep", "motion_close",
/// "motion_label".
class MotionPlayer : public QDialog {
    Q_OBJECT

public:
    static constexpr std::size_t npos = static_cast<std::size_t>(-1);

    /// @p describe says what position @p index is; @p show shows it (npos:
    /// none); @p keep makes it where the components are.
    MotionPlayer(QWidget* parent, std::shared_ptr<const doc::MotionResult> motion,
                 std::function<QString(std::size_t)> describe,
                 std::function<void(std::size_t)> show, std::function<void(std::size_t)> keep);
    ~MotionPlayer() override;

    MotionPlayer(const MotionPlayer&) = delete;
    MotionPlayer& operator=(const MotionPlayer&) = delete;

    const doc::MotionResult& motion() const { return *m_motion; }
    /// The position shown.
    std::size_t position() const;
    void showPosition(std::size_t index);
    bool playing() const;
    /// Play from where it is, or from the start at the end; once more pauses.
    void togglePlay();

protected:
    void done(int result) override;

private:
    void step();

    std::shared_ptr<const doc::MotionResult> m_motion;
    std::function<QString(std::size_t)> m_describe;
    std::function<void(std::size_t)> m_show;
    std::function<void(std::size_t)> m_keep;
    QSlider* m_slider = nullptr;
    QLabel* m_label = nullptr;
    QPushButton* m_play = nullptr;
    QTimer* m_timer = nullptr;
    bool m_closed = false;
};

}  // namespace hz::ui
