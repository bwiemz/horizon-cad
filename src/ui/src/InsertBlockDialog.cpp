#include "horizon/ui/InsertBlockDialog.h"

#include <QComboBox>
#include <QDialogButtonBox>
#include <QDoubleSpinBox>
#include <QFormLayout>
#include <QVBoxLayout>

namespace hz::ui {

InsertBlockDialog::InsertBlockDialog(const std::vector<std::string>& blockNames,
                                     QWidget* parent)
    : QDialog(parent) {
    setWindowTitle(tr("Insert Block"));

    auto* layout = new QVBoxLayout(this);
    auto* form = new QFormLayout();

    m_blockCombo = new QComboBox(this);
    for (const auto& name : blockNames) {
        m_blockCombo->addItem(QString::fromStdString(name));
    }
    form->addRow(tr("Block:"), m_blockCombo);

    m_rotation = new QDoubleSpinBox(this);
    m_rotation->setRange(-360.0, 360.0);
    m_rotation->setDecimals(2);
    m_rotation->setValue(0.0);
    m_rotation->setSuffix(QString::fromUtf8("\xC2\xB0"));
    form->addRow(tr("Rotation:"), m_rotation);

    m_scale = new QDoubleSpinBox(this);
    m_scale->setRange(0.001, 1000.0);
    m_scale->setDecimals(3);
    m_scale->setValue(1.0);
    form->addRow(tr("Scale:"), m_scale);

    layout->addLayout(form);

    auto* buttons = new QDialogButtonBox(
        QDialogButtonBox::Ok | QDialogButtonBox::Cancel, this);
    connect(buttons, &QDialogButtonBox::accepted, this, &QDialog::accept);
    connect(buttons, &QDialogButtonBox::rejected, this, &QDialog::reject);
    layout->addWidget(buttons);
}

std::string InsertBlockDialog::selectedBlock() const {
    return m_blockCombo->currentText().toStdString();
}

double InsertBlockDialog::rotation() const {
    return m_rotation->value();
}

double InsertBlockDialog::scale() const {
    return m_scale->value();
}

}  // namespace hz::ui
