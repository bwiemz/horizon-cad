#include "horizon/ui/ConfigurationsDialog.h"

#include <QDialogButtonBox>
#include <QHBoxLayout>
#include <QHeaderView>
#include <QLabel>
#include <QPushButton>
#include <QTableWidget>
#include <QVBoxLayout>
#include <set>

#include "horizon/document/ParameterRegistry.h"

namespace hz::ui {

ConfigurationsDialog::ConfigurationsDialog(const doc::ConfigurationTable& table,
                                           const std::map<std::string, std::string>& variables,
                                           QWidget* parent)
    : QDialog(parent),
      m_table(new QTableWidget(this)),
      m_problem(new QLabel(this)),
      m_own(variables),
      m_active(table.active()) {
    setWindowTitle(tr("Configurations"));
    // A column for each variable: the document's own, and any a
    // configuration gives that it has not.
    std::set<std::string> names;
    for (const auto& [name, expression] : variables) names.insert(name);
    for (const std::string& configuration : table.configurationNames()) {
        for (const auto& [name, expression] : table.overrides(configuration)) names.insert(name);
    }
    m_variables.assign(names.begin(), names.end());

    auto* layout = new QVBoxLayout(this);
    auto* note = new QLabel(tr("Each row is a configuration of the part: the values it gives "
                               "its variables. A blank cell keeps the variable's own."),
                            this);
    note->setWordWrap(true);
    layout->addWidget(note);

    m_table->setObjectName(QStringLiteral("configurations"));
    m_table->setColumnCount(static_cast<int>(m_variables.size()) + 1);
    QStringList headers{tr("Configuration")};
    for (const std::string& name : m_variables) {
        const auto own = variables.find(name);
        headers << (own != variables.end()
                        ? QStringLiteral("%1 (%2)").arg(QString::fromStdString(name),
                                                        QString::fromStdString(own->second))
                        : QString::fromStdString(name));
    }
    m_table->setHorizontalHeaderLabels(headers);
    m_table->verticalHeader()->hide();
    m_table->horizontalHeader()->setSectionResizeMode(QHeaderView::Stretch);
    for (const std::string& configuration : table.configurationNames()) {
        addRow(QString::fromStdString(configuration), table.overrides(configuration));
    }
    layout->addWidget(m_table);

    auto* rowButtons = new QHBoxLayout();
    auto* add = new QPushButton(tr("Add"), this);
    add->setObjectName(QStringLiteral("add"));
    auto* remove = new QPushButton(tr("Remove"), this);
    remove->setObjectName(QStringLiteral("remove"));
    rowButtons->addWidget(add);
    rowButtons->addWidget(remove);
    rowButtons->addStretch();
    layout->addLayout(rowButtons);
    connect(add, &QPushButton::clicked, this,
            [this] { addRow(tr("Configuration %1").arg(m_table->rowCount() + 1), {}); });
    connect(remove, &QPushButton::clicked, this, [this] {
        const int row = m_table->currentRow();
        if (row >= 0) m_table->removeRow(row);
    });

    m_problem->setObjectName(QStringLiteral("problem"));
    m_problem->setWordWrap(true);
    m_problem->setStyleSheet(QStringLiteral("color: #b00020;"));
    m_problem->hide();
    layout->addWidget(m_problem);

    auto* buttons = new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel, this);
    connect(buttons, &QDialogButtonBox::accepted, this, &ConfigurationsDialog::accept);
    connect(buttons, &QDialogButtonBox::rejected, this, &QDialog::reject);
    layout->addWidget(buttons);
    resize(640, 320);
}

void ConfigurationsDialog::addRow(const QString& name,
                                  const doc::ConfigurationTable::Overrides& overrides) {
    const int row = m_table->rowCount();
    m_table->insertRow(row);
    m_table->setItem(row, 0, new QTableWidgetItem(name));
    for (std::size_t k = 0; k < m_variables.size(); ++k) {
        const auto given = overrides.find(m_variables[k]);
        m_table->setItem(
            row, static_cast<int>(k) + 1,
            new QTableWidgetItem(given != overrides.end() ? QString::fromStdString(given->second)
                                                          : QString()));
    }
}

bool ConfigurationsDialog::collect(doc::ConfigurationTable& out, QString* why) const {
    out = {};
    for (int row = 0; row < m_table->rowCount(); ++row) {
        const QString name = m_table->item(row, 0)->text().trimmed();
        if (name.isEmpty()) {
            if (why != nullptr) *why = tr("Row %1 has no name.").arg(row + 1);
            return false;
        }
        if (out.hasConfiguration(name.toStdString())) {
            if (why != nullptr) *why = tr("Two configurations are named %1.").arg(name);
            return false;
        }
        doc::ConfigurationTable::Overrides overrides;
        for (std::size_t k = 0; k < m_variables.size(); ++k) {
            const QString text = m_table->item(row, static_cast<int>(k) + 1)->text().trimmed();
            if (!text.isEmpty()) overrides[m_variables[k]] = text.toStdString();
        }
        out.setConfiguration(name.toStdString(), overrides);
    }
    out.setActive(m_active);  // false, and none active, once it is gone
    return true;
}

doc::ConfigurationTable ConfigurationsDialog::table() const {
    doc::ConfigurationTable out;
    collect(out, nullptr);
    return out;
}

void ConfigurationsDialog::accept() {
    doc::ConfigurationTable collected;
    QString why;
    if (!collect(collected, &why)) {
        m_problem->setText(why);
        m_problem->show();
        return;
    }
    // Each, laid over the variables, must be worked out as they are.
    for (const std::string& name : collected.configurationNames()) {
        std::string reason;
        if (!doc::ParameterRegistry::check(collected.overlay(m_own, name), &reason)) {
            m_problem->setText(QString::fromStdString(name) + QStringLiteral(": ") +
                               QString::fromStdString(reason));
            m_problem->show();
            return;
        }
    }
    QDialog::accept();
}

}  // namespace hz::ui
