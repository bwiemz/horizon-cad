#pragma once

#include <QString>
#include <QStringList>

#include "horizon/math/Units.h"

namespace hz::ui {

/// What the user can set in Edit ▸ Preferences, kept in the application
/// settings. Lengths in the model are millimetres. Each document has its own
/// unit (Phase 154); the one here is the unit new documents are made in.
struct Preferences {
    int autosaveSeconds = 120;                  ///< 0: autosave is off
    int undoLimit = 1000;                       ///< steps kept to undo; 0: no limit
    QString language;                           ///< "" follows the system; applies on restart
    double gridSpacing = 1.0;                   ///< the grid snap's spacing, in millimetres
    int snapPixels = 10;                        ///< how far on screen a snap reaches
    bool objectSnap = true;                     ///< snap to points on entities (F3)
    bool gridSnap = true;                       ///< snap to the grid (F9)
    bool ortho = false;                         ///< ortho (F8); never with polarTracking
    bool polarTracking = false;                 ///< polar tracking (F10)
    double polarAngle = 15.0;                   ///< polar tracking's step, in degrees
    QString lengthUnit = QStringLiteral("mm");  ///< for new documents: mm, cm, m, in or ft
    int decimals = 3;                           ///< places lengths and angles are shown to

    /// The saved preferences, each clamped to a sensible range.
    static Preferences load();
    void save() const;

    /// The preferences in force: loaded once, then kept up to date by save().
    static const Preferences& current();

    /// The units offered, by their symbols.
    static QStringList lengthUnits();
    /// The unit new documents are made in.
    math::LengthUnit newDocumentUnit() const;

    /// A length, an area or a volume, given in millimetres, in @p unit to
    /// the decimals set: "25.400 mm", "1.000 in²", "2.000 cm³".
    QString formatLength(double mm, math::LengthUnit unit) const;
    QString formatArea(double mm2, math::LengthUnit unit) const;
    QString formatVolume(double mm3, math::LengthUnit unit) const;
    /// An angle, given in radians, in degrees to the decimals set: "30.000°".
    QString formatAngle(double radians) const;
};

}  // namespace hz::ui
