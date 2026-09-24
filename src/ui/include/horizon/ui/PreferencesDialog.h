#pragma once

#include <QDialog>

#include "horizon/ui/Preferences.h"

class QComboBox;
class QDoubleSpinBox;
class QSpinBox;

namespace hz::ui {

/// Edit ▸ Preferences: autosave, language, the grid and snap, and the unit
/// lengths are shown in.
class PreferencesDialog : public QDialog {
    Q_OBJECT

public:
    PreferencesDialog(const Preferences& prefs, const QStringList& languages,
                      QWidget* parent = nullptr);

    /// What the dialog shows now.
    Preferences preferences() const;

private:
    QSpinBox* m_autosaveMinutes = nullptr;
    QComboBox* m_language = nullptr;
    QDoubleSpinBox* m_gridSpacing = nullptr;
    QSpinBox* m_snapPixels = nullptr;
    QComboBox* m_lengthUnit = nullptr;
    QSpinBox* m_decimals = nullptr;
    Preferences m_base;
};

}  // namespace hz::ui
