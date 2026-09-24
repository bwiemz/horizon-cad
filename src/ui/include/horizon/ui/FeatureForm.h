#pragma once

#include <QDialog>
#include <QString>
#include <QStringList>
#include <utility>
#include <vector>

#include "horizon/document/FeatureTree.h"

class QComboBox;
class QDoubleSpinBox;
class QFormLayout;
class QLineEdit;
class QListWidget;
class QSpinBox;

namespace hz::ui {

/// A modal form asking for a feature's inputs: labelled number and text
/// fields, choices and checklists above OK / Cancel. Every field carries the object
/// name it is given, so tests and automation can fill it.
class FeatureForm {
public:
    FeatureForm(QWidget* parent, const QString& title);
    ~FeatureForm();
    FeatureForm(const FeatureForm&) = delete;
    FeatureForm& operator=(const FeatureForm&) = delete;

    QDoubleSpinBox* number(const QString& name, const QString& label, double value, double min,
                           double max, int decimals = 3);
    QLineEdit* text(const QString& name, const QString& label, const QString& value = {});
    QSpinBox* count(const QString& name, const QString& label, int value, int min, int max);
    QComboBox* choice(const QString& name, const QString& label, const QStringList& items);

    /// How the feature's body combines with the part (object name
    /// "bodyOperation"), showing `initial`. Read it with `operation()`.
    QComboBox* operationChoice(doc::BodyOperation initial);
    static doc::BodyOperation operation(const QComboBox* choice);

    /// A list of checkable items, each `{text, tooltip}`, none checked.
    QListWidget* checklist(const QString& name, const QString& label,
                           const std::vector<std::pair<QString, QString>>& items);
    /// Rows of `list` that are checked, in order.
    static std::vector<int> checkedRows(const QListWidget* list);

    /// Show the form; true when accepted.
    bool exec();

    QDialog& dialog() { return m_dialog; }

private:
    QDialog m_dialog;
    QFormLayout* m_form;
};

}  // namespace hz::ui
