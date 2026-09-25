#include "horizon/ui/PreferencesDialog.h"

#include <QComboBox>
#include <QDialogButtonBox>
#include <QDoubleSpinBox>
#include <QFormLayout>
#include <QGroupBox>
#include <QLabel>
#include <QLocale>
#include <QSpinBox>
#include <QVBoxLayout>
#include <algorithm>

namespace hz::ui {

PreferencesDialog::PreferencesDialog(const Preferences& prefs, const QStringList& languages,
                                     QWidget* parent)
    : QDialog(parent), m_base(prefs) {
    setWindowTitle(tr("Preferences"));
    setObjectName(QStringLiteral("preferencesDialog"));

    auto* general = new QGroupBox(tr("General"), this);
    auto* generalForm = new QFormLayout(general);
    m_autosaveMinutes = new QSpinBox(general);
    m_autosaveMinutes->setObjectName(QStringLiteral("autosaveMinutes"));
    m_autosaveMinutes->setRange(0, 120);
    m_autosaveMinutes->setSpecialValueText(tr("Off"));
    m_autosaveMinutes->setSuffix(tr(" min"));
    m_autosaveMinutes->setValue((prefs.autosaveSeconds + 59) / 60);
    generalForm->addRow(tr("Autosave every:"), m_autosaveMinutes);

    m_undoLimit = new QSpinBox(general);
    m_undoLimit->setObjectName(QStringLiteral("undoLimit"));
    m_undoLimit->setRange(0, 100000);
    m_undoLimit->setSpecialValueText(tr("No limit"));
    m_undoLimit->setValue(prefs.undoLimit);
    m_undoLimit->setToolTip(tr("How many steps each document keeps to undo; the oldest go first"));
    generalForm->addRow(tr("Undo steps:"), m_undoLimit);

    m_language = new QComboBox(general);
    m_language->setObjectName(QStringLiteral("language"));
    m_language->addItem(tr("System default"), QString());
    m_language->addItem(QStringLiteral("English"), QStringLiteral("en"));
    for (const QString& code : languages) {
        if (code == QLatin1String("en")) continue;
        const QLocale locale(code);
        m_language->addItem(
            locale.nativeLanguageName().isEmpty() ? code : locale.nativeLanguageName(), code);
    }
    const int current = m_language->findData(prefs.language);
    m_language->setCurrentIndex(current < 0 ? 0 : current);
    generalForm->addRow(tr("Language:"), m_language);
    auto* restartNote =
        new QLabel(tr("A new language takes effect when Horizon CAD restarts."), general);
    restartNote->setWordWrap(true);
    generalForm->addRow(QString(), restartNote);

    auto* drafting = new QGroupBox(tr("Drawing"), this);
    auto* draftingForm = new QFormLayout(drafting);
    m_gridSpacing = new QDoubleSpinBox(drafting);
    m_gridSpacing->setObjectName(QStringLiteral("gridSpacing"));
    m_gridSpacing->setRange(0.001, 10000.0);
    m_gridSpacing->setDecimals(3);
    m_gridSpacing->setSuffix(tr(" mm"));
    m_gridSpacing->setValue(prefs.gridSpacing);
    draftingForm->addRow(tr("Grid snap spacing:"), m_gridSpacing);

    m_snapPixels = new QSpinBox(drafting);
    m_snapPixels->setObjectName(QStringLiteral("snapPixels"));
    m_snapPixels->setRange(2, 100);
    m_snapPixels->setSuffix(tr(" px"));
    m_snapPixels->setValue(prefs.snapPixels);
    draftingForm->addRow(tr("Snap reach:"), m_snapPixels);

    m_lengthUnit = new QComboBox(drafting);
    m_lengthUnit->setObjectName(QStringLiteral("lengthUnit"));
    for (const QString& unit : Preferences::lengthUnits()) m_lengthUnit->addItem(unit, unit);
    m_lengthUnit->setCurrentIndex(std::max(0, m_lengthUnit->findData(prefs.lengthUnit)));
    draftingForm->addRow(tr("Show lengths in:"), m_lengthUnit);

    m_decimals = new QSpinBox(drafting);
    m_decimals->setObjectName(QStringLiteral("decimals"));
    m_decimals->setRange(0, 8);
    m_decimals->setValue(prefs.decimals);
    draftingForm->addRow(tr("Decimal places:"), m_decimals);
    auto* unitNote =
        new QLabel(tr("The model is always in millimetres; this changes only how lengths are "
                      "shown in coordinates and measurements."),
                   drafting);
    unitNote->setWordWrap(true);
    draftingForm->addRow(QString(), unitNote);

    auto* buttons = new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel, this);
    connect(buttons, &QDialogButtonBox::accepted, this, &QDialog::accept);
    connect(buttons, &QDialogButtonBox::rejected, this, &QDialog::reject);

    auto* layout = new QVBoxLayout(this);
    layout->addWidget(general);
    layout->addWidget(drafting);
    layout->addWidget(buttons);
}

Preferences PreferencesDialog::preferences() const {
    Preferences p = m_base;
    p.autosaveSeconds = m_autosaveMinutes->value() * 60;
    p.undoLimit = m_undoLimit->value();
    p.language = m_language->currentData().toString();
    p.gridSpacing = m_gridSpacing->value();
    p.snapPixels = m_snapPixels->value();
    p.lengthUnit = m_lengthUnit->currentData().toString();
    p.decimals = m_decimals->value();
    return p;
}

}  // namespace hz::ui
