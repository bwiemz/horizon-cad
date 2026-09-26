#include "horizon/ui/FeatureForm.h"

#include <QComboBox>
#include <QCoreApplication>
#include <QDialogButtonBox>
#include <QDoubleSpinBox>
#include <QFormLayout>
#include <QLineEdit>
#include <QListWidget>
#include <QSpinBox>
#include <cmath>

#include "horizon/ui/QuantitySpinBox.h"

namespace hz::ui {

namespace {

/// @p label as a screen reader says it: without the "&" that marks its
/// mnemonic ("&&" is a literal "&").
QString spoken(const QString& label) {
    QString out;
    for (qsizetype i = 0; i < label.size(); ++i) {
        if (label[i] == QLatin1Char('&') && ++i == label.size()) break;
        out += label[i];
    }
    return out;
}

}  // namespace

QString formatPoint(const math::Vec3& p) {
    const auto n = [](double v) { return QString::number(std::abs(v) < 5e-10 ? 0.0 : v, 'g', 6); };
    return QStringLiteral("(%1, %2, %3)").arg(n(p.x), n(p.y), n(p.z));
}

FeatureForm::FeatureForm(QWidget* parent, const QString& title, math::LengthUnit unit)
    : m_dialog(parent), m_form(new QFormLayout(&m_dialog)), m_unit(unit) {
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

QuantitySpinBox* FeatureForm::length(const QString& name, const QString& label, double mm,
                                     double min, double max, int decimals) {
    auto* spin = new QuantitySpinBox(QuantitySpinBox::Kind::Length, m_unit, decimals, &m_dialog);
    spin->setObjectName(name);
    spin->setRange(min, max);
    spin->setValue(mm);
    m_form->addRow(label, spin);
    return spin;
}

QuantitySpinBox* FeatureForm::angle(const QString& name, const QString& label, double degrees,
                                    double min, double max, int decimals) {
    auto* spin = new QuantitySpinBox(QuantitySpinBox::Kind::Angle, m_unit, decimals, &m_dialog);
    spin->setObjectName(name);
    spin->setRange(min, max);
    spin->setValue(degrees);
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
    // A form names each field after its label, but not an item view on every
    // Qt: 6.11 does, CI's 6.9 does not. Named here, it is named on both.
    list->setAccessibleName(spoken(label));
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
