#pragma once

#include <QFrame>
#include <QPointer>
#include <QString>
#include <vector>

class QLabel;
class QPushButton;

namespace hz::ui {

/// Help ▸ Getting Started (Phase 168c): a short tour of the window, a step
/// at a time. Each step outlines a part of the window (the ribbon, the
/// feature tree, the view...) and says what it is for, in a panel beside it
/// with Back, Next and Close (or Escape). The outline lets clicks through;
/// the window stays usable. A step whose target is not in the window (a
/// panel floated, or closed) shows its panel alone, in the middle. The tour
/// ends, and deletes itself, when closed or finished.
class GettingStartedTour : public QFrame {
    Q_OBJECT

public:
    struct Step {
        QPointer<QWidget> target;  ///< what the step points at
        QString title;
        QString text;
    };

    /// A tour of @p window (its child), through @p steps; it starts at once.
    GettingStartedTour(QWidget* window, std::vector<Step> steps);

    int step() const { return m_step; }
    int stepCount() const { return static_cast<int>(m_steps.size()); }
    /// The outline drawn around the step's target, in the window.
    QRect outlined() const;

    void next();
    void back();

signals:
    void finished();

protected:
    bool eventFilter(QObject* watched, QEvent* event) override;
    void keyPressEvent(QKeyEvent* event) override;

private:
    void showStep(int index);
    void place();
    void finish();

    QWidget* m_window;
    std::vector<Step> m_steps;
    int m_step = 0;
    QWidget* m_outline;
    QLabel* m_count;
    QLabel* m_title;
    QLabel* m_text;
    QPushButton* m_back;
    QPushButton* m_next;
};

}  // namespace hz::ui
