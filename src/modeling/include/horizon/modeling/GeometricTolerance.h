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

/// Material-condition modifier for a tolerance zone (ASME Y14.5).
/// RFS (regardless of feature size) is the default and prints no symbol.
enum class MaterialCondition {
    None,  ///< regardless of feature size (RFS) — no modifier symbol
    MMC,   ///< maximum material condition — Ⓜ
    LMC,   ///< least material condition — Ⓛ
};

/// A GD&T feature control frame: a geometric tolerance applied to a model
/// feature (identified by TopologyID) with an optional list of datum references.
///
/// Like the drawing dimensions, the frame is anchored to a TopologyID (stable
/// across rebuilds), so it stays attached to the same feature as the model
/// changes — the associative basis for GD&T on regenerated drawings.
struct FeatureControlFrame {
    GeometricCharacteristic characteristic = GeometricCharacteristic::Position;
    double tolerance = 0.0;                                ///< tolerance-zone size
    MaterialCondition modifier = MaterialCondition::None;  ///< applied to the zone
    std::vector<std::string> datumRefs;                    ///< datum letters, e.g. {"A", "B", "C"}
    topo::TopologyID feature;                              ///< the toleranced model feature
};

/// A datum feature: the physical model feature (identified by TopologyID) that
/// establishes a datum, labelled with a single reference letter ("A", "B", …).
///
/// A drawing renders it as a datum feature symbol ([A]) attached to the feature.
/// Anchoring to a TopologyID keeps the datum bound to the same feature across
/// rebuilds, matching the associative behaviour of dimensions and frames.
struct DatumFeature {
    std::string label;         ///< datum reference letter, e.g. "A"
    topo::TopologyID feature;  ///< the model feature establishing the datum
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

    /// The material-condition modifier symbol (UTF-8), or "" for None (RFS).
    static std::string modifierSymbol(MaterialCondition m);

    /// A datum feature symbol, the boxed reference letter, e.g. "[A]".
    static std::string datumSymbol(const DatumFeature& d);

    /// A compact one-line frame, e.g. "⟂ 0.05 Ⓜ | A | B" — the characteristic
    /// symbol, the tolerance, an optional material-condition modifier, then each
    /// datum reference separated by " | ".
    static std::string format(const FeatureControlFrame& fcf);
};

}  // namespace hz::model
