#pragma once

#include <QDialog>
#include <map>
#include <string>
#include <vector>

#include "horizon/document/ConfigurationTable.h"

class QLabel;
class QTableWidget;

namespace hz::ui {

/// Edit ▸ Configurations (Phase 156): a part's design table. A row for each
/// configuration, a column for each variable; a cell is the expression that
/// configuration gives it, or blank for the variable's own. OK keeps the
/// table only if each configuration has a name of its own, and each, laid
/// over the variables, can be worked out; otherwise it says why and stays.
class ConfigurationsDialog : public QDialog {
    Q_OBJECT

public:
    /// @p table to start from; @p variables the document's own, by name.
    ConfigurationsDialog(const doc::ConfigurationTable& table,
                         const std::map<std::string, std::string>& variables,
                         QWidget* parent = nullptr);

    /// The table as it is now. The active configuration stays active while
    /// it is in it (by name).
    doc::ConfigurationTable table() const;

    void accept() override;

private:
    /// A row; @p existing for one the table had, which, renamed, is still
    /// that configuration (the active one stays active).
    void addRow(const QString& name, const doc::ConfigurationTable::Overrides& overrides,
                bool existing);
    bool collect(doc::ConfigurationTable& out, QString* why) const;

    QTableWidget* m_table;
    QLabel* m_problem;
    std::vector<std::string> m_variables;  ///< the columns after the name
    std::map<std::string, std::string> m_own;
    std::string m_active;
};

}  // namespace hz::ui
