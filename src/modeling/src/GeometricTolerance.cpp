#include "horizon/modeling/GeometricTolerance.h"

#include <sstream>

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

std::string Gdt::format(const FeatureControlFrame& fcf) {
    std::ostringstream oss;
    oss << symbol(fcf.characteristic) << ' ' << fcf.tolerance;
    for (const std::string& datum : fcf.datumRefs) {
        oss << " | " << datum;
    }
    return oss.str();
}

}  // namespace hz::model
