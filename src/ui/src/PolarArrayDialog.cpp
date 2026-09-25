#include "horizon/ui/PolarArrayDialog.h"

#include <QCheckBox>
#include <QDialogButtonBox>
#include <QDoubleSpinBox>
#include <QFormLayout>
#include <QSpinBox>
#include <QVBoxLayout>

#include "horizon/ui/QuantitySpinBox.h"

namespace hz::ui {

PolarArrayDialog::PolarArrayDialog(QWidget* parent, math::LengthUnit unit) : QDialog(parent) {
    setWindowTitle(tr("Polar Array"));

    auto* layout = new QVBoxLayout(this);
    auto* form = new QFormLayout();

    m_count = new QSpinBox(this);
    m_count->setRange(2, 100);
    m_count->setValue(6);
    form->addRow(tr("Count:"), m_count);

    m_centerX = new QuantitySpinBox(QuantitySpinBox::Kind::Length, unit, 3, this);
    m_centerX->setRange(-10000.0, 10000.0);
    m_centerX->setValue(0.0);
    form->addRow(tr("Center X:"), m_centerX);

    m_centerY = new QuantitySpinBox(QuantitySpinBox::Kind::Length, unit, 3, this);
    m_centerY->setRange(-10000.0, 10000.0);
    m_centerY->setValue(0.0);
    form->addRow(tr("Center Y:"), m_centerY);

    m_totalAngle = new QuantitySpinBox(QuantitySpinBox::Kind::Angle, unit, 1, this);
    m_totalAngle->setRange(0.1, 360.0);
    m_totalAngle->setValue(360.0);
    form->addRow(tr("Total Angle:"), m_totalAngle);

    m_fillFullCircle = new QCheckBox(tr("Fill 360\u00b0"), this);
    m_fillFullCircle->setChecked(true);
    m_totalAngle->setEnabled(false);
    connect(m_fillFullCircle, &QCheckBox::checkStateChanged, this,
            &PolarArrayDialog::onFillFullCircleChanged);
    form->addRow(m_fillFullCircle);

    layout->addLayout(form);

    auto* buttons = new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel, this);
    connect(buttons, &QDialogButtonBox::accepted, this, &QDialog::accept);
    connect(buttons, &QDialogButtonBox::rejected, this, &QDialog::reject);
    layout->addWidget(buttons);
}

void PolarArrayDialog::onFillFullCircleChanged(Qt::CheckState state) {
    if (state == Qt::Checked) {
        m_totalAngle->setValue(360.0);
        m_totalAngle->setEnabled(false);
    } else {
        m_totalAngle->setEnabled(true);
    }
}

int PolarArrayDialog::count() const {
    return m_count->value();
}
double PolarArrayDialog::totalAngle() const {
    return m_totalAngle->value();
}
double PolarArrayDialog::centerX() const {
    return m_centerX->value();
}
double PolarArrayDialog::centerY() const {
    return m_centerY->value();
}

}  // namespace hz::ui
