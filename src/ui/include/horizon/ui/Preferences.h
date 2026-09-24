#pragma once

#include <QString>
#include <QStringList>

namespace hz::ui {

/// What the user can set in Edit ▸ Preferences, kept in the application
/// settings. Lengths in the model are millimetres; the display unit only
/// changes how they are shown.
struct Preferences {
    int autosaveSeconds = 120;                  ///< 0: autosave is off
    QString language;                           ///< "" follows the system; applies on restart
    double gridSpacing = 1.0;                   ///< the grid snap's spacing, in millimetres
    int snapPixels = 10;                        ///< how far on screen a snap reaches
    QString lengthUnit = QStringLiteral("mm");  ///< mm, cm, m, in or ft
    int decimals = 3;

    /// The saved preferences, each clamped to a sensible range.
    static Preferences load();
    void save() const;

    /// The preferences in force: loaded once, then kept up to date by save().
    static const Preferences& current();

    /// The display units offered, and how many millimetres make one.
    static QStringList lengthUnits();
    static double millimetresPer(const QString& unit);

    /// A length or an area, given in millimetres, in the display unit:
    /// "25.400 mm", "1.000 in²".
    QString formatLength(double mm) const;
    QString formatArea(double mm2) const;
};

}  // namespace hz::ui
