#include <gtest/gtest.h>

#include <set>
#include <vector>

#include "horizon/modeling/GeometricTolerance.h"
#include "horizon/topology/TopologyID.h"

using hz::model::FeatureControlFrame;
using hz::model::Gdt;
using hz::model::GeometricCharacteristic;
using hz::topo::TopologyID;

namespace {
const std::vector<GeometricCharacteristic> kAll = {
    GeometricCharacteristic::Flatness,         GeometricCharacteristic::Straightness,
    GeometricCharacteristic::Circularity,      GeometricCharacteristic::Cylindricity,
    GeometricCharacteristic::Perpendicularity, GeometricCharacteristic::Parallelism,
    GeometricCharacteristic::Angularity,       GeometricCharacteristic::Position,
    GeometricCharacteristic::Concentricity,    GeometricCharacteristic::Symmetry,
    GeometricCharacteristic::Runout,           GeometricCharacteristic::Profile,
};
}  // namespace

// Every characteristic has a distinct, non-empty symbol and name.
TEST(GeometricToleranceTest, SymbolsAndNamesAreDistinct) {
    std::set<std::string> symbols;
    std::set<std::string> names;
    for (auto c : kAll) {
        EXPECT_FALSE(Gdt::symbol(c).empty());
        EXPECT_FALSE(Gdt::name(c).empty());
        symbols.insert(Gdt::symbol(c));
        names.insert(Gdt::name(c));
    }
    EXPECT_EQ(symbols.size(), kAll.size());  // all symbols distinct
    EXPECT_EQ(names.size(), kAll.size());    // all names distinct
}

// Form tolerances and profile take no datum; orientation/location/runout do.
TEST(GeometricToleranceTest, DatumRequirement) {
    EXPECT_FALSE(Gdt::requiresDatum(GeometricCharacteristic::Flatness));
    EXPECT_FALSE(Gdt::requiresDatum(GeometricCharacteristic::Circularity));
    EXPECT_FALSE(Gdt::requiresDatum(GeometricCharacteristic::Profile));
    EXPECT_TRUE(Gdt::requiresDatum(GeometricCharacteristic::Perpendicularity));
    EXPECT_TRUE(Gdt::requiresDatum(GeometricCharacteristic::Position));
    EXPECT_TRUE(Gdt::requiresDatum(GeometricCharacteristic::Runout));
}

// A frame formats as symbol, tolerance, then datum references.
TEST(GeometricToleranceTest, FormatsFrame) {
    FeatureControlFrame fcf;
    fcf.characteristic = GeometricCharacteristic::Perpendicularity;
    fcf.tolerance = 0.05;
    fcf.datumRefs = {"A", "B"};
    fcf.feature = TopologyID::make("box", "face_right");

    const std::string s = Gdt::format(fcf);
    EXPECT_EQ(s, Gdt::symbol(GeometricCharacteristic::Perpendicularity) + " 0.05 | A | B");
    EXPECT_TRUE(fcf.feature.isValid());  // frame is anchored to a model feature
}

TEST(GeometricToleranceTest, FormatsFrameWithoutDatums) {
    FeatureControlFrame fcf;
    fcf.characteristic = GeometricCharacteristic::Flatness;
    fcf.tolerance = 0.1;
    const std::string s = Gdt::format(fcf);
    EXPECT_EQ(s, Gdt::symbol(GeometricCharacteristic::Flatness) + " 0.1");
}
