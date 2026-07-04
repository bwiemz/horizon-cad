#pragma once

#include <string>
#include <vector>

#include "horizon/topology/TopologyID.h"

namespace hz::model {

/// The ASME Y14.5 / ISO 1101 geometric characteristics.
enum class GeometricCharacteristic {
    Flatness,
    Straightness,
    Circularity,
    Cylindricity,
    Perpendicularity,
    Parallelism,
    Angularity,
    Position,
    Concentricity,
    Symmetry,
    Runout,
    Profile,
};

/// A GD&T feature control frame: a geometric tolerance applied to a model
/// feature (identified by TopologyID) with an optional list of datum references.
///
/// Like the drawing dimensions, the frame is anchored to a TopologyID (stable
/// across rebuilds), so it stays attached to the same feature as the model
/// changes — the associative basis for GD&T on regenerated drawings.
struct FeatureControlFrame {
    GeometricCharacteristic characteristic = GeometricCharacteristic::Position;
    double tolerance = 0.0;              ///< tolerance-zone size
    std::vector<std::string> datumRefs;  ///< datum letters, e.g. {"A", "B", "C"}
    topo::TopologyID feature;            ///< the toleranced model feature
};

/// GD&T symbol/formatting utilities.
class Gdt {
public:
    /// The characteristic's ASME symbol (UTF-8).
    static std::string symbol(GeometricCharacteristic c);

    /// The characteristic's human-readable name (e.g. "Perpendicularity").
    static std::string name(GeometricCharacteristic c);

    /// Whether the characteristic requires at least one datum reference.
    /// Form tolerances (flatness, straightness, circularity, cylindricity) and
    /// profile do not; orientation, location, and runout do.
    static bool requiresDatum(GeometricCharacteristic c);

    /// A compact one-line frame, e.g. "⊥ 0.05 | A | B" — the symbol, the
    /// tolerance, then each datum reference separated by " | ".
    static std::string format(const FeatureControlFrame& fcf);
};

}  // namespace hz::model
