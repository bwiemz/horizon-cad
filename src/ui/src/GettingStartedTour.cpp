#include "horizon/ui/GettingStartedTour.h"

#include <QEvent>
#include <QHBoxLayout>
#include <QKeyEvent>
#include <QLabel>
#include <QPainter>
#include <QPushButton>
#include <QVBoxLayout>
#include <algorithm>
#include <utility>

namespace hz::ui {

namespace {

/// The outline around a step's target: drawn only, clicks go through it.
class Outline : public QWidget {
public:
    explicit Outline(QWidget* parent) : QWidget(parent) {
        setObjectName(QStringLiteral("tourOutline"));
        setAttribute(Qt::WA_TransparentForMouseEvents);
        setAttribute(Qt::WA_NoSystemBackground);
    }

protected:
    void paintEvent(QPaintEvent* /*event*/) override {
        QPainter painter(this);
        painter.setRenderHint(QPainter::Antialiasing);
        painter.setPen(QPen(palette().color(QPalette::Highlight), 3));
        painter.drawRoundedRect(QRectF(rect()).adjusted(1.5, 1.5, -1.5, -1.5), 6, 6);
    }
};

constexpr int kMargin = 6;  ///< between the outline and its target
constexpr int kPanelWidth = 340;

}  // namespace

GettingStartedTour::GettingStartedTour(QWidget* window, std::vector<Step> steps)
    : QFrame(window),
      m_window(window),
      m_steps(std::move(steps)),
      m_outline(new Outline(window)),
      m_count(new QLabel(this)),
      m_title(new QLabel(this)),
      m_text(new QLabel(this)),
      m_back(new QPushButton(tr("Back"), this)),
      m_next(new QPushButton(tr("Next"), this)) {
    setObjectName(QStringLiteral("gettingStartedTour"));
    setFrameShape(QFrame::StyledPanel);
    setAutoFillBackground(true);
    setFixedWidth(kPanelWidth);

    QFont bold = m_title->font();
    bold.setBold(true);
    bold.setPointSizeF(bold.pointSizeF() * 1.15);
    m_title->setFont(bold);
    m_title->setObjectName(QStringLiteral("tourTitle"));
    m_text->setWordWrap(true);
    m_text->setObjectName(QStringLiteral("tourText"));
    m_count->setObjectName(QStringLiteral("tourCount"));
    auto* close = new QPushButton(tr("Close"), this);
    m_next->setDefault(true);

    auto* buttons = new QHBoxLayout;
    buttons->addWidget(m_count);
    buttons->addStretch();
    buttons->addWidget(close);
    buttons->addWidget(m_back);
    buttons->addWidget(m_next);
    auto* layout = new QVBoxLayout(this);
    layout->addWidget(m_title);
    layout->addWidget(m_text);
    layout->addLayout(buttons);

    connect(m_back, &QPushButton::clicked, this, &GettingStartedTour::back);
    connect(m_next, &QPushButton::clicked, this, &GettingStartedTour::next);
    connect(close, &QPushButton::clicked, this, &GettingStartedTour::finish);
    // Kept beside its target as the window changes size.
    m_window->installEventFilter(this);

    showStep(0);
    show();
    raise();
    // Its buttons have the keys: Enter goes on, Escape closes it.
    m_next->setFocus();
}

QRect GettingStartedTour::outlined() const {
    return m_outline->isVisible() ? m_outline->geometry() : QRect();
}

void GettingStartedTour::next() {
    if (m_step + 1 >= stepCount()) {
        finish();
        return;
    }
    showStep(m_step + 1);
}

void GettingStartedTour::back() {
    if (m_step > 0) showStep(m_step - 1);
}

void GettingStartedTour::showStep(int index) {
    m_step = std::clamp(index, 0, std::max(0, stepCount() - 1));
    if (m_steps.empty()) return;
    const Step& step = m_steps[static_cast<std::size_t>(m_step)];
    m_title->setText(step.title);
    m_text->setText(step.text);
    m_count->setText(tr("%1 of %2").arg(m_step + 1).arg(stepCount()));
    m_back->setEnabled(m_step > 0);
    m_next->setText(m_step + 1 < stepCount() ? tr("Next") : tr("Done"));
    place();
}

void GettingStartedTour::place() {
    if (m_steps.empty()) return;
    const QWidget* target = m_steps[static_cast<std::size_t>(m_step)].target;
    adjustSize();
    const QRect window = m_window->rect();
    // Nothing to point at in the window (closed, or floated as a window of
    // its own): the panel alone, in the middle.
    if (target == nullptr || !target->isVisible() || target->window() != m_window) {
        m_outline->hide();
        move(window.center() - rect().center());
        return;
    }
    const QRect area(target->mapTo(m_window, QPoint(0, 0)), target->size());
    m_outline->setGeometry(area.adjusted(-kMargin, -kMargin, kMargin, kMargin).intersected(window));
    m_outline->show();
    m_outline->raise();

    // Below the target if it fits, else above, else inside it at the top.
    QPoint at(area.center().x() - width() / 2, area.bottom() + kMargin * 2);
    if (at.y() + height() > window.bottom()) at.setY(area.top() - kMargin * 2 - height());
    if (at.y() < window.top()) at.setY(area.top() + kMargin * 2);
    at.setX(std::clamp(at.x(), window.left() + kMargin,
                       std::max(window.left() + kMargin, window.right() - width() - kMargin)));
    at.setY(std::clamp(at.y(), window.top() + kMargin,
                       std::max(window.top() + kMargin, window.bottom() - height() - kMargin)));
    move(at);
    raise();
}

void GettingStartedTour::finish() {
    m_window->removeEventFilter(this);
    m_outline->hide();
    m_outline->deleteLater();
    hide();
    emit finished();
    deleteLater();
}

void GettingStartedTour::keyPressEvent(QKeyEvent* event) {
    if (event->key() == Qt::Key_Escape) {
        finish();
        return;
    }
    QFrame::keyPressEvent(event);
}

bool GettingStartedTour::eventFilter(QObject* watched, QEvent* event) {
    if (watched == m_window && event->type() == QEvent::Resize) place();
    return QFrame::eventFilter(watched, event);
}

}  // namespace hz::ui
