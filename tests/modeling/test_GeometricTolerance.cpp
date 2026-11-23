#include <gtest/gtest.h>

#include <cmath>
#include <set>
#include <string>
#include <utility>
#include <vector>

#include "horizon/modeling/GeometricTolerance.h"
#include "horizon/topology/TopologyID.h"

using hz::model::DatumFeature;
using hz::model::FeatureControlFrame;
using hz::model::Gdt;
using hz::model::GeometricCharacteristic;
using hz::model::MaterialCondition;
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

// RFS (None) prints no modifier; MMC and LMC print distinct, non-empty symbols.
TEST(GeometricToleranceTest, ModifierSymbols) {
    EXPECT_TRUE(Gdt::modifierSymbol(MaterialCondition::None).empty());
    EXPECT_FALSE(Gdt::modifierSymbol(MaterialCondition::MMC).empty());
    EXPECT_FALSE(Gdt::modifierSymbol(MaterialCondition::LMC).empty());
    EXPECT_NE(Gdt::modifierSymbol(MaterialCondition::MMC),
              Gdt::modifierSymbol(MaterialCondition::LMC));
}

// A material-condition modifier follows the tolerance, before the datums.
TEST(GeometricToleranceTest, FormatsFrameWithModifier) {
    FeatureControlFrame fcf;
    fcf.characteristic = GeometricCharacteristic::Position;
    fcf.tolerance = 0.25;
    fcf.modifier = MaterialCondition::MMC;
    fcf.datumRefs = {"A"};

    const std::string s = Gdt::format(fcf);
    EXPECT_EQ(s, Gdt::symbol(GeometricCharacteristic::Position) + " 0.25 " +
                     Gdt::modifierSymbol(MaterialCondition::MMC) + " | A");
}

// The default modifier (None/RFS) leaves the frame unchanged.
TEST(GeometricToleranceTest, DefaultModifierAddsNothing) {
    FeatureControlFrame fcf;
    fcf.characteristic = GeometricCharacteristic::Position;
    fcf.tolerance = 0.25;
    fcf.datumRefs = {"A"};
    EXPECT_EQ(fcf.modifier, MaterialCondition::None);
    EXPECT_EQ(Gdt::format(fcf), Gdt::symbol(GeometricCharacteristic::Position) + " 0.25 | A");
}

// A datum feature renders as a boxed reference letter and stays anchored.
TEST(GeometricToleranceTest, DatumFeatureSymbol) {
    DatumFeature d;
    d.label = "A";
    d.feature = TopologyID::make("box", "face_bottom");
    EXPECT_EQ(Gdt::datumSymbol(d), "[A]");
    EXPECT_TRUE(d.feature.isValid());
}

// Every characteristic and condition has a name a file or the AI gives, and
// reads back from it; a name no characteristic has reads as none.
TEST(GeometricToleranceTest, NamesReadBack) {
    std::set<std::string> keys;
    for (GeometricCharacteristic c : kAll) {
        const std::string key = Gdt::key(c);
        EXPECT_TRUE(keys.insert(key).second) << key;
        EXPECT_EQ(Gdt::characteristicNamed(key), c);
    }
    EXPECT_EQ(Gdt::key(GeometricCharacteristic::Position), "position");
    EXPECT_FALSE(Gdt::characteristicNamed("Position").has_value());
    EXPECT_FALSE(Gdt::characteristicNamed("").has_value());
    for (MaterialCondition m :
         {MaterialCondition::None, MaterialCondition::MMC, MaterialCondition::LMC}) {
        EXPECT_EQ(Gdt::conditionNamed(Gdt::conditionKey(m)), m);
    }
    EXPECT_FALSE(Gdt::conditionNamed("rfs").has_value());
}

// A datum is named by one or two capitals, never I, O or Q; the next free
// letter skips those, then doubles.
TEST(GeometricToleranceTest, DatumLetters) {
    for (const char* good : {"A", "B", "Z", "AA", "AB", "ZZ"}) {
        EXPECT_TRUE(Gdt::usableDatumLabel(good)) << good;
    }
    for (const char* bad : {"", "a", "I", "O", "Q", "AI", "ABC", "A1", "-", "\xC3\x84"}) {
        EXPECT_FALSE(Gdt::usableDatumLabel(bad)) << bad;
    }
    EXPECT_EQ(Gdt::nextDatumLabel({}), "A");
    EXPECT_EQ(Gdt::nextDatumLabel({"A", "C"}), "B");
    EXPECT_EQ(Gdt::nextDatumLabel({"A", "B", "C", "D", "E", "F", "G", "H"}), "J");
    std::vector<std::string> singles;
    for (char c = 'A'; c <= 'Z'; ++c) {
        if (c != 'I' && c != 'O' && c != 'Q') singles.emplace_back(1, c);
    }
    EXPECT_EQ(Gdt::nextDatumLabel(singles), "AA");
}

// What the standard allows of a frame, and what it does not, each with why.
TEST(GeometricToleranceTest, ChecksFrames) {
    const auto frame = [](GeometricCharacteristic c, double tolerance,
                          std::vector<std::string> datums) {
        FeatureControlFrame f;
        f.characteristic = c;
        f.tolerance = tolerance;
        f.datumRefs = std::move(datums);
        return f;
    };
    FeatureControlFrame position = frame(GeometricCharacteristic::Position, 0.2, {"A", "B", "C"});
    position.diameterZone = true;
    position.modifier = MaterialCondition::MMC;
    EXPECT_EQ(Gdt::check(position), "");
    EXPECT_EQ(Gdt::check(frame(GeometricCharacteristic::Flatness, 0.05, {})), "");
    EXPECT_EQ(Gdt::check(frame(GeometricCharacteristic::Profile, 0.1, {})), "");
    EXPECT_EQ(Gdt::check(frame(GeometricCharacteristic::Profile, 0.1, {"A"})), "");

    EXPECT_NE(Gdt::check(frame(GeometricCharacteristic::Flatness, 0.05, {"A"})).find("form"),
              std::string::npos);
    EXPECT_NE(Gdt::check(frame(GeometricCharacteristic::Position, 0.2, {})).find("needs a datum"),
              std::string::npos);
    EXPECT_NE(Gdt::check(frame(GeometricCharacteristic::Position, 0.2, {"A", "B", "C", "D"}))
                  .find("at most three"),
              std::string::npos);
    EXPECT_NE(
        Gdt::check(frame(GeometricCharacteristic::Position, 0.2, {"A", "B", "A"})).find("twice"),
        std::string::npos);
    EXPECT_NE(Gdt::check(frame(GeometricCharacteristic::Position, 0.2, {"I"})).find("letter"),
              std::string::npos);
    for (double bad : {0.0, -0.1, 1000.5, std::nan(""), HUGE_VAL}) {
        EXPECT_NE(Gdt::check(frame(GeometricCharacteristic::Flatness, bad, {})), "") << bad;
    }
    FeatureControlFrame flatAtMmc = frame(GeometricCharacteristic::Flatness, 0.05, {});
    flatAtMmc.modifier = MaterialCondition::MMC;
    EXPECT_NE(Gdt::check(flatAtMmc).find("material condition"), std::string::npos);
    FeatureControlFrame flatDiameter = frame(GeometricCharacteristic::Flatness, 0.05, {});
    flatDiameter.diameterZone = true;
    EXPECT_NE(Gdt::check(flatDiameter).find("cylindrical"), std::string::npos);
}

// A cylindrical zone puts ⌀ before the tolerance; a tolerance is stated to
// at most four decimals, without trailing zeros, with a point.
TEST(GeometricToleranceTest, FormatsDiameterZoneAndValues) {
    FeatureControlFrame fcf;
    fcf.characteristic = GeometricCharacteristic::Position;
    fcf.tolerance = 0.2;
    fcf.diameterZone = true;
    fcf.modifier = MaterialCondition::MMC;
    fcf.datumRefs = {"A"};
    EXPECT_EQ(Gdt::format(fcf), Gdt::symbol(GeometricCharacteristic::Position) +
                                    " \xE2\x8C\x80"
                                    "0.2 " +
                                    Gdt::modifierSymbol(MaterialCondition::MMC) + " | A");
    EXPECT_EQ(Gdt::formatValue(0.05), "0.05");
    EXPECT_EQ(Gdt::formatValue(0.0005), "0.0005");
    EXPECT_EQ(Gdt::formatValue(1.0), "1");
    EXPECT_EQ(Gdt::formatValue(10.0), "10");
    EXPECT_EQ(Gdt::formatValue(2.5), "2.5");
}
