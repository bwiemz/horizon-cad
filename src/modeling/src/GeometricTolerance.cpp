#include "horizon/modeling/GeometricTolerance.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <iomanip>
#include <locale>
#include <sstream>
#include <utility>
#include <vector>

namespace hz::model {

std::string Gdt::symbol(GeometricCharacteristic c) {
    switch (c) {
        case GeometricCharacteristic::Flatness:
            return "⏥";  // ⏥
        case GeometricCharacteristic::Straightness:
            return "⏤";  // ⏤
        case GeometricCharacteristic::Circularity:
            return "○";  // ○
        case GeometricCharacteristic::Cylindricity:
            return "⌭";  // ⌭
        case GeometricCharacteristic::Perpendicularity:
            return "⟂";  // ⟂
        case GeometricCharacteristic::Parallelism:
            return "∥";  // ∥
        case GeometricCharacteristic::Angularity:
            return "∠";  // ∠
        case GeometricCharacteristic::Position:
            return "⌖";  // ⌖
        case GeometricCharacteristic::Concentricity:
            return "◎";  // ◎
        case GeometricCharacteristic::Symmetry:
            return "⌯";  // ⌯
        case GeometricCharacteristic::Runout:
            return "↗";  // ↗
        case GeometricCharacteristic::Profile:
            return "⌓";  // ⌓
    }
    return "?";
}

std::string Gdt::name(GeometricCharacteristic c) {
    switch (c) {
        case GeometricCharacteristic::Flatness:
            return "Flatness";
        case GeometricCharacteristic::Straightness:
            return "Straightness";
        case GeometricCharacteristic::Circularity:
            return "Circularity";
        case GeometricCharacteristic::Cylindricity:
            return "Cylindricity";
        case GeometricCharacteristic::Perpendicularity:
            return "Perpendicularity";
        case GeometricCharacteristic::Parallelism:
            return "Parallelism";
        case GeometricCharacteristic::Angularity:
            return "Angularity";
        case GeometricCharacteristic::Position:
            return "Position";
        case GeometricCharacteristic::Concentricity:
            return "Concentricity";
        case GeometricCharacteristic::Symmetry:
            return "Symmetry";
        case GeometricCharacteristic::Runout:
            return "Runout";
        case GeometricCharacteristic::Profile:
            return "Profile";
    }
    return "Unknown";
}

bool Gdt::requiresDatum(GeometricCharacteristic c) {
    switch (c) {
        // Form tolerances reference no datum.
        case GeometricCharacteristic::Flatness:
        case GeometricCharacteristic::Straightness:
        case GeometricCharacteristic::Circularity:
        case GeometricCharacteristic::Cylindricity:
        // Profile may be used with or without a datum.
        case GeometricCharacteristic::Profile:
            return false;
        // Orientation, location, and runout are datum-referenced.
        case GeometricCharacteristic::Perpendicularity:
        case GeometricCharacteristic::Parallelism:
        case GeometricCharacteristic::Angularity:
        case GeometricCharacteristic::Position:
        case GeometricCharacteristic::Concentricity:
        case GeometricCharacteristic::Symmetry:
        case GeometricCharacteristic::Runout:
            return true;
    }
    return false;
}

std::string Gdt::modifierSymbol(MaterialCondition m) {
    switch (m) {
        case MaterialCondition::None:
            return "";  // regardless of feature size — no symbol
        case MaterialCondition::MMC:
            return "Ⓜ";  // maximum material condition
        case MaterialCondition::LMC:
            return "Ⓛ";  // least material condition
    }
    return "";
}

std::string Gdt::datumSymbol(const DatumFeature& d) {
    return "[" + d.label + "]";
}

std::string Gdt::format(const FeatureControlFrame& fcf) {
    std::ostringstream oss;
    oss << symbol(fcf.characteristic) << ' ' << (fcf.diameterZone ? "\xE2\x8C\x80" : "")
        << formatValue(fcf.tolerance);
    const std::string mod = modifierSymbol(fcf.modifier);
    if (!mod.empty()) {
        oss << ' ' << mod;
    }
    for (const std::string& datum : fcf.datumRefs) {
        oss << " | " << datum;
    }
    return oss.str();
}

namespace {

const std::vector<std::pair<GeometricCharacteristic, const char*>>& keys() {
    static const std::vector<std::pair<GeometricCharacteristic, const char*>> all = {
        {GeometricCharacteristic::Flatness, "flatness"},
        {GeometricCharacteristic::Straightness, "straightness"},
        {GeometricCharacteristic::Circularity, "circularity"},
        {GeometricCharacteristic::Cylindricity, "cylindricity"},
        {GeometricCharacteristic::Perpendicularity, "perpendicularity"},
        {GeometricCharacteristic::Parallelism, "parallelism"},
        {GeometricCharacteristic::Angularity, "angularity"},
        {GeometricCharacteristic::Position, "position"},
        {GeometricCharacteristic::Concentricity, "concentricity"},
        {GeometricCharacteristic::Symmetry, "symmetry"},
        {GeometricCharacteristic::Runout, "runout"},
        {GeometricCharacteristic::Profile, "profile"},
    };
    return all;
}

/// The letters a datum is named with: A to Z but I, O and Q.
bool datumLetter(char c) {
    return c >= 'A' && c <= 'Z' && c != 'I' && c != 'O' && c != 'Q';
}

}  // namespace

std::string Gdt::key(GeometricCharacteristic c) {
    for (const auto& [known, name] : keys()) {
        if (known == c) return name;
    }
    return "position";
}

std::optional<GeometricCharacteristic> Gdt::characteristicNamed(const std::string& key) {
    for (const auto& [known, name] : keys()) {
        if (key == name) return known;
    }
    return std::nullopt;
}

std::string Gdt::conditionKey(MaterialCondition m) {
    switch (m) {
        case MaterialCondition::MMC:
            return "mmc";
        case MaterialCondition::LMC:
            return "lmc";
        case MaterialCondition::None:
            break;
    }
    return "none";
}

std::optional<MaterialCondition> Gdt::conditionNamed(const std::string& key) {
    if (key == "none") return MaterialCondition::None;
    if (key == "mmc") return MaterialCondition::MMC;
    if (key == "lmc") return MaterialCondition::LMC;
    return std::nullopt;
}

bool Gdt::takesDatums(GeometricCharacteristic c) {
    switch (c) {
        case GeometricCharacteristic::Flatness:
        case GeometricCharacteristic::Straightness:
        case GeometricCharacteristic::Circularity:
        case GeometricCharacteristic::Cylindricity:
            return false;
        default:
            return true;
    }
}

bool Gdt::takesMaterialCondition(GeometricCharacteristic c) {
    switch (c) {
        case GeometricCharacteristic::Straightness:
        case GeometricCharacteristic::Perpendicularity:
        case GeometricCharacteristic::Parallelism:
        case GeometricCharacteristic::Angularity:
        case GeometricCharacteristic::Position:
            return true;
        default:
            return false;
    }
}

bool Gdt::takesDiameterZone(GeometricCharacteristic c) {
    return takesMaterialCondition(c) || c == GeometricCharacteristic::Concentricity;
}

bool Gdt::usableDatumLabel(const std::string& label) {
    return (label.size() == 1 || label.size() == 2) &&
           std::all_of(label.begin(), label.end(), datumLetter);
}

std::string Gdt::nextDatumLabel(const std::vector<std::string>& used) {
    const auto free = [&used](const std::string& label) {
        return std::find(used.begin(), used.end(), label) == used.end();
    };
    for (char c = 'A'; c <= 'Z'; ++c) {
        if (datumLetter(c) && free(std::string(1, c))) return std::string(1, c);
    }
    for (char first = 'A'; first <= 'Z'; ++first) {
        if (!datumLetter(first)) continue;
        for (char second = 'A'; second <= 'Z'; ++second) {
            const std::string label{first, second};
            if (datumLetter(second) && free(label)) return label;
        }
    }
    return {};  // 23 + 529 datums on one sheet: there are no more
}

std::string Gdt::check(const FeatureControlFrame& frame) {
    const std::string what = name(frame.characteristic);
    if (!(frame.tolerance > 0.0) || !(frame.tolerance <= kMaxTolerance)) {
        return "its tolerance must be more than 0 and at most " + formatValue(kMaxTolerance) +
               " mm";
    }
    if (frame.tolerance < kMinTolerance) {
        return "its tolerance must be at least " + formatValue(kMinTolerance) +
               " mm: no gauge measures less";
    }
    if (!takesDatums(frame.characteristic) && !frame.datumRefs.empty()) {
        return what + " is a form tolerance: it refers to no datum";
    }
    if (requiresDatum(frame.characteristic) && frame.datumRefs.empty()) {
        return what + " needs a datum to refer to";
    }
    if (frame.datumRefs.size() > 3) {
        return "a frame refers to at most three datums: primary, secondary and tertiary";
    }
    for (std::size_t i = 0; i < frame.datumRefs.size(); ++i) {
        const std::string& label = frame.datumRefs[i];
        if (!usableDatumLabel(label)) {
            return "\"" + label + "\" is not a datum's letter: one or two capitals, not I, O or Q";
        }
        if (std::find(frame.datumRefs.begin(),
                      frame.datumRefs.begin() + static_cast<std::ptrdiff_t>(i),
                      label) != frame.datumRefs.begin() + static_cast<std::ptrdiff_t>(i)) {
            return "datum " + label + " is referred to twice";
        }
    }
    if (frame.modifier != MaterialCondition::None &&
        !takesMaterialCondition(frame.characteristic)) {
        return what +
               " takes no material condition (MMC or LMC): it applies regardless of "
               "feature size";
    }
    if (frame.diameterZone && !takesDiameterZone(frame.characteristic)) {
        return what + " has no cylindrical (\xE2\x8C\x80) zone";
    }
    return {};
}

std::string Gdt::formatValue(double value) {
    std::ostringstream out;
    out.imbue(std::locale::classic());
    // Four decimals, or as many as a smaller one needs: a tolerance written
    // "0" would say another thing (a zero tolerance at MMC).
    const bool small = value != 0.0 && std::abs(value) < 0.00005;
    out << std::fixed << std::setprecision(small ? 9 : 4) << value;
    std::string text = out.str();
    if (text.find('.') != std::string::npos) {
        while (!text.empty() && text.back() == '0') text.pop_back();
        if (!text.empty() && text.back() == '.') text.pop_back();
    }
    return text;
}

}  // namespace hz::model
