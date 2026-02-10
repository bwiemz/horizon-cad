#include "horizon/ui/RectArrayDialog.h"

#include <QDialogButtonBox>
#include <QDoubleSpinBox>
#include <QFormLayout>
#include <QSpinBox>
#include <QVBoxLayout>

namespace hz::ui {

RectArrayDialog::RectArrayDialog(QWidget* parent)
    : QDialog(parent) {
    setWindowTitle(tr("Rectangular Array"));

    auto* layout = new QVBoxLayout(this);
    auto* form = new QFormLayout();

    m_columns = new QSpinBox(this);
    m_columns->setRange(1, 100);
    m_columns->setValue(3);
    form->addRow(tr("Columns:"), m_columns);

    m_rows = new QSpinBox(this);
    m_rows->setRange(1, 100);
    m_rows->setValue(3);
    form->addRow(tr("Rows:"), m_rows);

    m_spacingX = new QDoubleSpinBox(this);
    m_spacingX->setRange(-1000.0, 1000.0);
    m_spacingX->setDecimals(3);
    m_spacingX->setValue(2.0);
    form->addRow(tr("Spacing X:"), m_spacingX);

    m_spacingY = new QDoubleSpinBox(this);
    m_spacingY->setRange(-1000.0, 1000.0);
    m_spacingY->setDecimals(3);
    m_spacingY->setValue(2.0);
    form->addRow(tr("Spacing Y:"), m_spacingY);

    layout->addLayout(form);

    auto* buttons = new QDialogButtonBox(
        QDialogButtonBox::Ok | QDialogButtonBox::Cancel, this);
    connect(buttons, &QDialogButtonBox::accepted, this, &QDialog::accept);
    connect(buttons, &QDialogButtonBox::rejected, this, &QDialog::reject);
    layout->addWidget(buttons);
}

int RectArrayDialog::columns() const { return m_columns->value(); }
int RectArrayDialog::rows() const { return m_rows->value(); }
double RectArrayDialog::spacingX() const { return m_spacingX->value(); }
double RectArrayDialog::spacingY() const { return m_spacingY->value(); }

}  // namespace hz::ui
