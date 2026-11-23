#pragma once

#include <optional>
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
    /// The zone is a cylinder, its tolerance a diameter: "⌀" before it, as a
    /// hole's position or an axis's perpendicularity is given.
    bool diameterZone = false;
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
    /// symbol, the tolerance (after "⌀" for a cylindrical zone), an optional
    /// material-condition modifier, then each datum reference separated by
    /// " | ".
    static std::string format(const FeatureControlFrame& fcf);

    /// The characteristic's name in files and in the AI tools: "position",
    /// "perpendicularity"... Read back by characteristicNamed().
    static std::string key(GeometricCharacteristic c);
    static std::optional<GeometricCharacteristic> characteristicNamed(const std::string& key);
    /// "none", "mmc" or "lmc"; read back by conditionNamed().
    static std::string conditionKey(MaterialCondition m);
    static std::optional<MaterialCondition> conditionNamed(const std::string& key);

    /// Whether a frame of @p c may refer to datums: a form tolerance
    /// (flatness, straightness, circularity, cylindricity) never does.
    static bool takesDatums(GeometricCharacteristic c);
    /// Whether a frame of @p c may hold its zone at MMC or LMC: those that
    /// control a feature of size's axis or plane (straightness, orientation,
    /// position). The others apply regardless of feature size.
    static bool takesMaterialCondition(GeometricCharacteristic c);
    /// Whether a frame of @p c may have a cylindrical (⌀) zone: those that
    /// can control an axis.
    static bool takesDiameterZone(GeometricCharacteristic c);

    /// Whether @p label can name a datum: one or two capital letters, none of
    /// them I, O or Q (ASME Y14.5 leaves those out: they read as 1, 0 and O).
    static bool usableDatumLabel(const std::string& label);
    /// The first datum letter not in @p used: A, B, C... past I, O and Q,
    /// then AA, AB...
    static std::string nextDatumLabel(const std::vector<std::string>& used);

    /// Why @p frame is not one the standard allows ("Flatness is a form
    /// tolerance: it refers to no datum"); empty when it is. Its tolerance is
    /// more than 0 and at most kMaxTolerance; it refers to datums when its
    /// characteristic needs one, never when it takes none, to at most three,
    /// each once and by a usable label; its modifier and zone are ones its
    /// characteristic takes.
    static std::string check(const FeatureControlFrame& frame);
    /// No tolerance zone larger than this, in millimetres.
    static constexpr double kMaxTolerance = 1000.0;

    /// A tolerance as a frame states it: to at most four decimals, without
    /// the zeros after the last digit that counts ("0.05", "1"), with a point
    /// whatever the locale.
    static std::string formatValue(double value);
};

}  // namespace hz::model
