#pragma once

#include <QDialog>
#include <QString>
#include <QStringList>
#include <utility>
#include <vector>

#include "horizon/document/FeatureTree.h"
#include "horizon/math/Units.h"
#include "horizon/math/Vec3.h"

class QComboBox;
class QDoubleSpinBox;
class QFormLayout;
class QLineEdit;
class QListWidget;
class QSpinBox;

namespace hz::ui {

class QuantitySpinBox;

/// A point or direction as the forms and dialogs show it: "(10, 0, 2.5)",
/// with rounding noise shown as 0.
QString formatPoint(const math::Vec3& p);

/// A modal form asking for a feature's inputs: labelled number and text
/// fields, choices and checklists above OK / Cancel. Every field carries the object
/// name it is given, so tests and automation can fill it.
class FeatureForm {
public:
    /// Its lengths shown and typed in @p unit, the document's (Phase 154).
    FeatureForm(QWidget* parent, const QString& title,
                math::LengthUnit unit = math::LengthUnit::Millimetre);
    ~FeatureForm();
    FeatureForm(const FeatureForm&) = delete;
    FeatureForm& operator=(const FeatureForm&) = delete;

    /// A plain number: a factor, a ratio, a count with a fraction.
    QDoubleSpinBox* number(const QString& name, const QString& label, double value, double min,
                           double max, int decimals = 3);
    /// A length: its value() and @p mm, @p min, @p max in millimetres, shown
    /// and typed in the form's unit, or any other ("2 in").
    QuantitySpinBox* length(const QString& name, const QString& label, double mm, double min,
                            double max, int decimals = 3);
    /// An angle: its value() and @p degrees, @p min, @p max in degrees,
    /// typed in them or in radians ("0.5 rad").
    QuantitySpinBox* angle(const QString& name, const QString& label, double degrees, double min,
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
    math::LengthUnit m_unit;
};

}  // namespace hz::ui
