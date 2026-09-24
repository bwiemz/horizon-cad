#include "horizon/ui/Preferences.h"

#include <QSettings>
#include <algorithm>
#include <cmath>

namespace hz::ui {

namespace {

struct Unit {
    const char* name;
    double mm;
};

constexpr Unit kUnits[] = {
    {"mm", 1.0}, {"cm", 10.0}, {"m", 1000.0}, {"in", 25.4}, {"ft", 304.8},
};

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
    p.language = settings.value(QStringLiteral("ui/language")).toString();
    const double grid = settings.value(QStringLiteral("drafting/gridSpacing"), 1.0).toDouble();
    p.gridSpacing = std::isfinite(grid) && grid > 0.0 ? grid : 1.0;
    p.snapPixels =
        std::clamp(settings.value(QStringLiteral("drafting/snapPixels"), 10).toInt(), 2, 100);
    const QString unit = settings.value(QStringLiteral("units/length"), "mm").toString();
    p.lengthUnit = lengthUnits().contains(unit) ? unit : QStringLiteral("mm");
    p.decimals = std::clamp(settings.value(QStringLiteral("units/decimals"), 3).toInt(), 0, 8);
    return p;
}

void Preferences::save() const {
    QSettings settings;
    settings.setValue(QStringLiteral("autosave/intervalSeconds"), autosaveSeconds);
    // No language stored means "follow the system" (see main.cpp).
    if (language.isEmpty()) {
        settings.remove(QStringLiteral("ui/language"));
    } else {
        settings.setValue(QStringLiteral("ui/language"), language);
    }
    settings.setValue(QStringLiteral("drafting/gridSpacing"), gridSpacing);
    settings.setValue(QStringLiteral("drafting/snapPixels"), snapPixels);
    settings.setValue(QStringLiteral("units/length"), lengthUnit);
    settings.setValue(QStringLiteral("units/decimals"), decimals);
    cache() = *this;
}

const Preferences& Preferences::current() {
    return cache();
}

QStringList Preferences::lengthUnits() {
    QStringList names;
    for (const Unit& u : kUnits) names << QString::fromLatin1(u.name);
    return names;
}

double Preferences::millimetresPer(const QString& unit) {
    for (const Unit& u : kUnits) {
        if (unit == QLatin1String(u.name)) return u.mm;
    }
    return 1.0;
}

QString Preferences::formatLength(double mm) const {
    return QStringLiteral("%1 %2")
        .arg(mm / millimetresPer(lengthUnit), 0, 'f', decimals)
        .arg(lengthUnit);
}

QString Preferences::formatArea(double mm2) const {
    const double per = millimetresPer(lengthUnit);
    return QStringLiteral("%1 %2²").arg(mm2 / (per * per), 0, 'f', decimals).arg(lengthUnit);
}

}  // namespace hz::ui
