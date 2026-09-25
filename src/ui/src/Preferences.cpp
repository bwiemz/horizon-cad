#include "horizon/ui/Preferences.h"

#include <QSettings>
#include <algorithm>
#include <cmath>
#include <string_view>

#include "horizon/math/Constants.h"

namespace hz::ui {

namespace {

Preferences& cache() {
    static Preferences prefs = Preferences::load();
    return prefs;
}

}  // namespace

Preferences Preferences::load() {
    const QSettings settings;
    Preferences p;
    p.autosaveSeconds = std::clamp(
        settings.value(QStringLiteral("autosave/intervalSeconds"), 120).toInt(), 0, 24 * 3600);
    p.undoLimit =
        std::clamp(settings.value(QStringLiteral("edit/undoLimit"), 1000).toInt(), 0, 100000);
    p.language = settings.value(QStringLiteral("ui/language")).toString();
    const double grid = settings.value(QStringLiteral("drafting/gridSpacing"), 1.0).toDouble();
    p.gridSpacing = std::isfinite(grid) && grid > 0.0 ? grid : 1.0;
    p.snapPixels =
        std::clamp(settings.value(QStringLiteral("drafting/snapPixels"), 10).toInt(), 2, 100);
    p.objectSnap = settings.value(QStringLiteral("drafting/objectSnap"), true).toBool();
    p.gridSnap = settings.value(QStringLiteral("drafting/gridSnap"), true).toBool();
    p.ortho = settings.value(QStringLiteral("drafting/ortho"), false).toBool();
    p.polarTracking =
        !p.ortho && settings.value(QStringLiteral("drafting/polarTracking"), false).toBool();
    const double polar = settings.value(QStringLiteral("drafting/polarAngle"), 15.0).toDouble();
    p.polarAngle = std::isfinite(polar) ? std::clamp(polar, 1.0, 90.0) : 15.0;
    const QString unit = settings.value(QStringLiteral("units/length"), "mm").toString();
    p.lengthUnit = lengthUnits().contains(unit) ? unit : QStringLiteral("mm");
    p.decimals = std::clamp(settings.value(QStringLiteral("units/decimals"), 3).toInt(), 0, 8);
    return p;
}

void Preferences::save() const {
    QSettings settings;
    settings.setValue(QStringLiteral("autosave/intervalSeconds"), autosaveSeconds);
    settings.setValue(QStringLiteral("edit/undoLimit"), undoLimit);
    // No language stored means "follow the system" (see main.cpp).
    if (language.isEmpty()) {
        settings.remove(QStringLiteral("ui/language"));
    } else {
        settings.setValue(QStringLiteral("ui/language"), language);
    }
    settings.setValue(QStringLiteral("drafting/gridSpacing"), gridSpacing);
    settings.setValue(QStringLiteral("drafting/snapPixels"), snapPixels);
    settings.setValue(QStringLiteral("drafting/objectSnap"), objectSnap);
    settings.setValue(QStringLiteral("drafting/gridSnap"), gridSnap);
    settings.setValue(QStringLiteral("drafting/ortho"), ortho);
    settings.setValue(QStringLiteral("drafting/polarTracking"), polarTracking);
    settings.setValue(QStringLiteral("drafting/polarAngle"), polarAngle);
    settings.setValue(QStringLiteral("units/length"), lengthUnit);
    settings.setValue(QStringLiteral("units/decimals"), decimals);
    cache() = *this;
}

const Preferences& Preferences::current() {
    return cache();
}

QStringList Preferences::lengthUnits() {
    QStringList names;
    for (const math::LengthUnit unit : math::kLengthUnits) {
        const std::string_view symbol = math::symbolOf(unit);
        names << QString::fromLatin1(symbol.data(), static_cast<qsizetype>(symbol.size()));
    }
    return names;
}

math::LengthUnit Preferences::newDocumentUnit() const {
    return math::lengthUnitFrom(lengthUnit.toStdString()).value_or(math::LengthUnit::Millimetre);
}

namespace {

QString symbol(math::LengthUnit unit) {
    const std::string_view text = math::symbolOf(unit);
    return QString::fromLatin1(text.data(), static_cast<qsizetype>(text.size()));
}

/// @p value to @p decimals places, without "-0.000".
QString fixed(double value, int decimals) {
    if (std::abs(value) < 0.5 * std::pow(10.0, -decimals)) value = 0.0;
    return QString::number(value, 'f', decimals);
}

}  // namespace

QString Preferences::formatLength(double mm, math::LengthUnit unit) const {
    return QStringLiteral("%1 %2").arg(fixed(mm / math::millimetresPer(unit), decimals),
                                       symbol(unit));
}

QString Preferences::formatArea(double mm2, math::LengthUnit unit) const {
    const double per = math::millimetresPer(unit);
    return QStringLiteral("%1 %2\u00B2").arg(fixed(mm2 / (per * per), decimals), symbol(unit));
}

QString Preferences::formatVolume(double mm3, math::LengthUnit unit) const {
    const double per = math::millimetresPer(unit);
    return QStringLiteral("%1 %2\u00B3")
        .arg(fixed(mm3 / (per * per * per), decimals), symbol(unit));
}

QString Preferences::formatAngle(double radians) const {
    return QStringLiteral("%1\u00B0").arg(fixed(radians * math::kRadToDeg, decimals));
}

}  // namespace hz::ui
