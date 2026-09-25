#pragma once

#include <QDialog>
#include <map>
#include <string>

#include "horizon/document/ConfigurationTable.h"
#include "horizon/math/Units.h"

class QLabel;
class QTableWidget;

namespace hz::ui {

/// Edit ▸ Variables (Phase 155): a document's variables, each a name and an
/// expression ("wall" = "3 mm", "width" = "10 * wall", "count" = "4"),
/// with the value each works out to, shown as they are typed. OK keeps them
/// only if each is a name, each expression one, and none depends on itself;
/// otherwise the dialog says why and stays.
class VariablesDialog : public QDialog {
    Q_OBJECT

public:
    /// @p definitions to start from; lengths shown in @p unit.
    VariablesDialog(const std::map<std::string, std::string>& definitions, math::LengthUnit unit,
                    QWidget* parent = nullptr);

    /// The variables as they are now in the table (the rows with a name).
    std::map<std::string, std::string> definitions() const;

    /// The document's configurations: OK is refused, too, if one of them
    /// laid over the variables as they are now could not be worked out (it
    /// overrides a variable with an expression of one that is gone).
    void setConfigurations(const doc::ConfigurationTable& configurations) {
        m_configurations = configurations;
    }

    void accept() override;

private:
    void addRow(const QString& name, const QString& expression);
    /// Each row's value, worked out from all of them as they are now.
    void showValues();
    /// The rows' names and expressions; false, and why, when two share a
    /// name or one has an expression and no name.
    bool collect(std::map<std::string, std::string>& out, QString* why) const;

    QTableWidget* m_table;
    QLabel* m_problem;
    math::LengthUnit m_unit;
    doc::ConfigurationTable m_configurations;
    bool m_showing = false;  ///< the values being written: no edit of the user's
};

}  // namespace hz::ui
