#include "horizon/ui/VariablesDialog.h"

#include <QDialogButtonBox>
#include <QHBoxLayout>
#include <QHeaderView>
#include <QLabel>
#include <QPushButton>
#include <QTableWidget>
#include <QVBoxLayout>

#include "horizon/document/ParameterRegistry.h"
#include "horizon/ui/Preferences.h"

namespace hz::ui {

namespace {

enum Column { kName = 0, kExpression = 1, kValue = 2 };

}  // namespace

VariablesDialog::VariablesDialog(const std::map<std::string, std::string>& definitions,
                                 math::LengthUnit unit, QWidget* parent)
    : QDialog(parent),
      m_table(new QTableWidget(0, 3, this)),
      m_problem(new QLabel(this)),
      m_unit(unit) {
    setWindowTitle(tr("Variables"));
    auto* layout = new QVBoxLayout(this);
    auto* note = new QLabel(
        tr("Each variable is an expression: \"3 mm\", \"10 * wall\", \"4\". A number without a "
           "unit is a plain number. A feature's size can then be \"=wall * 2\"."),
        this);
    note->setWordWrap(true);
    layout->addWidget(note);

    m_table->setObjectName(QStringLiteral("variables"));
    m_table->setHorizontalHeaderLabels({tr("Name"), tr("Expression"), tr("Value")});
    m_table->horizontalHeader()->setSectionResizeMode(kExpression, QHeaderView::Stretch);
    m_table->verticalHeader()->hide();
    layout->addWidget(m_table);
    for (const auto& [name, expression] : definitions) {
        addRow(QString::fromStdString(name), QString::fromStdString(expression));
    }

    auto* rowButtons = new QHBoxLayout();
    auto* add = new QPushButton(tr("Add"), this);
    add->setObjectName(QStringLiteral("add"));
    auto* remove = new QPushButton(tr("Remove"), this);
    remove->setObjectName(QStringLiteral("remove"));
    rowButtons->addWidget(add);
    rowButtons->addWidget(remove);
    rowButtons->addStretch();
    layout->addLayout(rowButtons);
    connect(add, &QPushButton::clicked, this, [this] {
        addRow(QString(), QString());
        m_table->editItem(m_table->item(m_table->rowCount() - 1, kName));
    });
    connect(remove, &QPushButton::clicked, this, [this] {
        const int row = m_table->currentRow();
        if (row >= 0) m_table->removeRow(row);
        showValues();
    });

    m_problem->setObjectName(QStringLiteral("problem"));
    m_problem->setWordWrap(true);
    m_problem->setStyleSheet(QStringLiteral("color: #b00020;"));
    m_problem->hide();
    layout->addWidget(m_problem);

    auto* buttons = new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel, this);
    connect(buttons, &QDialogButtonBox::accepted, this, &VariablesDialog::accept);
    connect(buttons, &QDialogButtonBox::rejected, this, &QDialog::reject);
    layout->addWidget(buttons);

    connect(m_table, &QTableWidget::itemChanged, this, [this](QTableWidgetItem* item) {
        if (!m_showing && item->column() != kValue) showValues();
    });
    showValues();
    resize(520, 360);
}

void VariablesDialog::addRow(const QString& name, const QString& expression) {
    const bool showing = m_showing;
    m_showing = true;  // values follow, once the row is whole
    const int row = m_table->rowCount();
    m_table->insertRow(row);
    m_table->setItem(row, kName, new QTableWidgetItem(name));
    m_table->setItem(row, kExpression, new QTableWidgetItem(expression));
    auto* value = new QTableWidgetItem();
    value->setFlags(value->flags() & ~Qt::ItemIsEditable);
    m_table->setItem(row, kValue, value);
    m_showing = showing;
}

bool VariablesDialog::collect(std::map<std::string, std::string>& out, QString* why) const {
    for (int row = 0; row < m_table->rowCount(); ++row) {
        const QString name = m_table->item(row, kName)->text().trimmed();
        const QString expression = m_table->item(row, kExpression)->text().trimmed();
        if (name.isEmpty()) {
            if (expression.isEmpty()) continue;  // a row left blank
            if (why != nullptr) *why = tr("Row %1 has an expression and no name.").arg(row + 1);
            return false;
        }
        if (!out.emplace(name.toStdString(), expression.toStdString()).second) {
            if (why != nullptr) *why = tr("Two variables are named %1.").arg(name);
            return false;
        }
    }
    return true;
}

std::map<std::string, std::string> VariablesDialog::definitions() const {
    std::map<std::string, std::string> out;
    collect(out, nullptr);
    return out;
}

void VariablesDialog::showValues() {
    std::map<std::string, std::string> defined;
    collect(defined, nullptr);
    doc::ParameterRegistry trial;
    trial.setDefinitions(defined);
    std::map<std::string, std::string> errors;
    const auto values = trial.quantities(&errors);
    const Preferences& prefs = Preferences::current();

    m_showing = true;
    for (int row = 0; row < m_table->rowCount(); ++row) {
        const std::string name = m_table->item(row, kName)->text().trimmed().toStdString();
        QTableWidgetItem* cell = m_table->item(row, kValue);
        const auto value = values.find(name);
        const auto error = errors.find(name);
        if (value != values.end()) {
            const math::Quantity& q = value->second;
            QString text;
            if (q.pure()) {
                text = QString::number(q.value, 'g', 10);
            } else if (q.length == 1 && q.angle == 0) {
                text = prefs.formatLength(q.value, m_unit);
            } else if (q.length == 0 && q.angle == 1) {
                text = prefs.formatAngle(q.value);
            } else if (q.length == 2 && q.angle == 0) {
                text = prefs.formatArea(q.value, m_unit);
            } else {
                text = QString::number(q.value, 'g', 10) + QStringLiteral(" (") +
                       QString::fromStdString(math::measureName(q)) + QStringLiteral(")");
            }
            cell->setText(text);
            cell->setToolTip(QString());
        } else {
            cell->setText(name.empty() ? QString() : tr("?"));
            cell->setToolTip(error != errors.end() ? QString::fromStdString(error->second)
                                                   : QString());
        }
    }
    m_showing = false;
}

void VariablesDialog::accept() {
    std::map<std::string, std::string> defined;
    QString why;
    std::string reason;
    if (!collect(defined, &why)) {
        m_problem->setText(why);
        m_problem->show();
        return;
    }
    if (!doc::ParameterRegistry::check(defined, &reason)) {
        m_problem->setText(QString::fromStdString(reason));
        m_problem->show();
        return;
    }
    QDialog::accept();
}

}  // namespace hz::ui
