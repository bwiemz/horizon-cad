#include "horizon/ui/MotionPlayer.h"

#include <QHBoxLayout>
#include <QLabel>
#include <QPushButton>
#include <QSlider>
#include <QTimer>
#include <QVBoxLayout>
#include <algorithm>
#include <utility>

namespace hz::ui {

namespace {

/// The whole motion takes about three seconds to play, however many
/// positions it has, each shown for 30 to 250 ms.
constexpr int kPlayMs = 3000;
constexpr int kFastestMs = 30;
constexpr int kSlowestMs = 250;

}  // namespace

MotionPlayer::MotionPlayer(QWidget* parent, std::shared_ptr<const doc::MotionResult> motion,
                           std::function<QString(std::size_t)> describe,
                           std::function<void(std::size_t)> show,
                           std::function<void(std::size_t)> keep)
    : QDialog(parent),
      m_motion(std::move(motion)),
      m_describe(std::move(describe)),
      m_show(std::move(show)),
      m_keep(std::move(keep)) {
    setWindowTitle(tr("Motion Study"));
    setModal(false);
    setObjectName(QStringLiteral("motion_player"));

    m_slider = new QSlider(Qt::Horizontal, this);
    m_slider->setObjectName(QStringLiteral("motion_position"));
    m_slider->setAccessibleName(tr("Position"));
    m_slider->setRange(0, std::max(0, static_cast<int>(m_motion->frames.size()) - 1));
    m_slider->setMinimumWidth(320);
    m_label = new QLabel(this);
    m_label->setObjectName(QStringLiteral("motion_label"));
    m_label->setWordWrap(true);

    m_play = new QPushButton(tr("&Play"), this);
    m_play->setObjectName(QStringLiteral("motion_play"));
    auto* keepButton = new QPushButton(tr("&Keep This Position"), this);
    keepButton->setObjectName(QStringLiteral("motion_keep"));
    keepButton->setToolTip(
        tr("Leave the components where this position has them, as one undo step"));
    auto* closeButton = new QPushButton(tr("&Close"), this);
    closeButton->setObjectName(QStringLiteral("motion_close"));

    auto* buttons = new QHBoxLayout;
    buttons->addWidget(m_play);
    buttons->addStretch(1);
    buttons->addWidget(keepButton);
    buttons->addWidget(closeButton);
    auto* layout = new QVBoxLayout(this);
    layout->addWidget(m_slider);
    layout->addWidget(m_label);
    layout->addLayout(buttons);

    m_timer = new QTimer(this);
    const int count = std::max(1, static_cast<int>(m_motion->frames.size()));
    m_timer->setInterval(std::clamp(kPlayMs / count, kFastestMs, kSlowestMs));
    connect(m_timer, &QTimer::timeout, this, &MotionPlayer::step);
    connect(m_slider, &QSlider::valueChanged, this, [this](int index) {
        const auto at = static_cast<std::size_t>(index);
        m_label->setText(m_describe(at));
        m_show(at);
    });
    connect(m_play, &QPushButton::clicked, this, &MotionPlayer::togglePlay);
    connect(keepButton, &QPushButton::clicked, this, [this] {
        const std::size_t at = position();
        m_timer->stop();
        m_keep(at);
        accept();
    });
    connect(closeButton, &QPushButton::clicked, this, &QDialog::reject);

    if (!m_motion->frames.empty()) {
        m_label->setText(m_describe(0));
        m_show(0);
    }
    m_play->setEnabled(m_motion->frames.size() > 1);
    keepButton->setEnabled(!m_motion->frames.empty());
}

MotionPlayer::~MotionPlayer() {
    if (!m_closed) m_show(npos);
}

std::size_t MotionPlayer::position() const {
    return static_cast<std::size_t>(std::max(0, m_slider->value()));
}

void MotionPlayer::showPosition(std::size_t index) {
    if (m_motion->frames.empty()) return;
    m_slider->setValue(static_cast<int>(std::min(index, m_motion->frames.size() - 1)));
}

bool MotionPlayer::playing() const {
    return m_timer->isActive();
}

void MotionPlayer::togglePlay() {
    if (m_timer->isActive()) {
        m_timer->stop();
        m_play->setText(tr("&Play"));
        return;
    }
    if (m_slider->value() >= m_slider->maximum()) m_slider->setValue(0);
    m_timer->start();
    m_play->setText(tr("&Pause"));
}

void MotionPlayer::step() {
    if (m_slider->value() >= m_slider->maximum()) {
        togglePlay();  // at the end: stopped there
        return;
    }
    m_slider->setValue(m_slider->value() + 1);
}

void MotionPlayer::done(int result) {
    m_timer->stop();
    if (!m_closed) {
        m_closed = true;
        m_show(npos);
    }
    QDialog::done(result);
}

}  // namespace hz::ui
