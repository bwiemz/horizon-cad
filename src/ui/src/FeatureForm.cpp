#include "horizon/ui/FeatureForm.h"

#include <QComboBox>
#include <QCoreApplication>
#include <QDialogButtonBox>
#include <QDoubleSpinBox>
#include <QFormLayout>
#include <QLineEdit>
#include <QListWidget>
#include <QSpinBox>

namespace hz::ui {

FeatureForm::FeatureForm(QWidget* parent, const QString& title)
    : m_dialog(parent), m_form(new QFormLayout(&m_dialog)) {
    m_dialog.setWindowTitle(title);
}

FeatureForm::~FeatureForm() = default;

QDoubleSpinBox* FeatureForm::number(const QString& name, const QString& label, double value,
                                    double min, double max, int decimals) {
    auto* spin = new QDoubleSpinBox(&m_dialog);
    spin->setObjectName(name);
    spin->setDecimals(decimals);
    spin->setRange(min, max);
    spin->setValue(value);
    m_form->addRow(label, spin);
    return spin;
}

QLineEdit* FeatureForm::text(const QString& name, const QString& label, const QString& value) {
    auto* edit = new QLineEdit(value, &m_dialog);
    edit->setObjectName(name);
    m_form->addRow(label, edit);
    return edit;
}

QSpinBox* FeatureForm::count(const QString& name, const QString& label, int value, int min,
                             int max) {
    auto* spin = new QSpinBox(&m_dialog);
    spin->setObjectName(name);
    spin->setRange(min, max);
    spin->setValue(value);
    m_form->addRow(label, spin);
    return spin;
}

QComboBox* FeatureForm::choice(const QString& name, const QString& label,
                               const QStringList& items) {
    auto* combo = new QComboBox(&m_dialog);
    combo->setObjectName(name);
    combo->addItems(items);
    m_form->addRow(label, combo);
    return combo;
}

QComboBox* FeatureForm::operationChoice(doc::BodyOperation initial) {
    auto* combo = new QComboBox(&m_dialog);
    combo->setObjectName(QStringLiteral("bodyOperation"));
    combo->addItem(QCoreApplication::translate("hz::ui::FeatureForm", "Join the part"),
                   static_cast<int>(doc::BodyOperation::Join));
    combo->addItem(QCoreApplication::translate("hz::ui::FeatureForm", "Cut from the part"),
                   static_cast<int>(doc::BodyOperation::Cut));
    combo->addItem(QCoreApplication::translate("hz::ui::FeatureForm", "Keep the intersection"),
                   static_cast<int>(doc::BodyOperation::Intersect));
    combo->addItem(QCoreApplication::translate("hz::ui::FeatureForm", "New body"),
                   static_cast<int>(doc::BodyOperation::NewBody));
    combo->setCurrentIndex(combo->findData(static_cast<int>(initial)));
    m_form->addRow(QCoreApplication::translate("hz::ui::FeatureForm", "Result:"), combo);
    return combo;
}

doc::BodyOperation FeatureForm::operation(const QComboBox* choice) {
    return static_cast<doc::BodyOperation>(choice->currentData().toInt());
}

QListWidget* FeatureForm::checklist(const QString& name, const QString& label,
                                    const std::vector<std::pair<QString, QString>>& items) {
    auto* list = new QListWidget(&m_dialog);
    list->setObjectName(name);
    for (const auto& [text, tooltip] : items) {
        auto* item = new QListWidgetItem(text, list);
        item->setFlags(item->flags() | Qt::ItemIsUserCheckable);
        item->setCheckState(Qt::Unchecked);
        item->setToolTip(tooltip);
    }
    m_form->addRow(label, list);
    return list;
}

std::vector<int> FeatureForm::checkedRows(const QListWidget* list) {
    std::vector<int> rows;
    for (int i = 0; i < list->count(); ++i) {
        if (list->item(i)->checkState() == Qt::Checked) rows.push_back(i);
    }
    return rows;
}

bool FeatureForm::exec() {
    auto* buttons =
        new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel, &m_dialog);
    QObject::connect(buttons, &QDialogButtonBox::accepted, &m_dialog, &QDialog::accept);
    QObject::connect(buttons, &QDialogButtonBox::rejected, &m_dialog, &QDialog::reject);
    m_form->addRow(buttons);
    return m_dialog.exec() == QDialog::Accepted;
}

}  // namespace hz::ui
