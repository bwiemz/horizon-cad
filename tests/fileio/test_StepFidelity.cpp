// STEP read as the file means it (Phase 109): faces with holes, the file's
// length unit, a file with one bad solid among good ones, and reals that
// round-trip exactly.

#include <gtest/gtest.h>

#include <cmath>
#include <filesystem>
#include <fstream>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include "horizon/fileio/ImportReport.h"
#include "horizon/fileio/StepFormat.h"
#include "horizon/math/Constants.h"
#include "horizon/math/Mat4.h"
#include "horizon/modeling/BooleanOp.h"
#include "horizon/modeling/MassProperties.h"
#include "horizon/modeling/Pattern.h"
#include "horizon/modeling/PrimitiveFactory.h"
#include "horizon/modeling/SolidTessellator.h"
#include "horizon/topology/GeometryValidator.h"
#include "horizon/topology/Solid.h"

namespace fs = std::filesystem;
using hz::io::ImportReport;
using hz::io::StepFormat;
using hz::math::Vec3;
using hz::model::MassPropertiesCalculator;

namespace {

std::string fixture(const std::string& name) {
    std::ifstream in(fs::path(HZ_STEP_FIXTURE_DIR) / "import_ok" / name, std::ios::binary);
    std::stringstream text;
    text << in.rdbuf();
    return text.str();
}

std::string replaced(std::string text, const std::string& from, const std::string& to) {
    const size_t at = text.find(from);
    EXPECT_NE(at, std::string::npos) << from;
    if (at != std::string::npos) text.replace(at, from.size(), to);
    return text;
}

double volumeOf(const hz::topo::Solid& solid) {
    return MassPropertiesCalculator::compute(solid).volume;
}

/// The area of the solid's display mesh in the plane z: the top or bottom
/// face as it is drawn.
double drawnArea(const hz::topo::Solid& solid, double z) {
    // Coarse: only the flat faces are measured, and they do not depend on it.
    const auto mesh = hz::model::SolidTessellator::tessellate(solid, 1.0);
    double area = 0.0;
    for (size_t t = 0; t + 2 < mesh.indices.size(); t += 3) {
        Vec3 p[3];
        bool flat = true;
        for (size_t k = 0; k < 3; ++k) {
            const size_t i = static_cast<size_t>(mesh.indices[t + k]) * 3;
            p[k] = Vec3(mesh.positions[i], mesh.positions[i + 1], mesh.positions[i + 2]);
            flat = flat && std::abs(p[k].z - z) < 1e-4;
        }
        if (flat) area += 0.5 * (p[1] - p[0]).cross(p[2] - p[0]).length();
    }
    return area;
}

}  // namespace

// ---------------------------------------------------------------------------
// Faces with holes
// ---------------------------------------------------------------------------

TEST(StepFidelityTest, AFaceWithASquareHoleImportsWithTheHole) {
    const auto solids = StepFormat::fromString(fixture("plate_with_square_hole.step"));
    ASSERT_EQ(solids.size(), 1u) << StepFormat::lastError();
    const auto& plate = *solids[0];
    EXPECT_TRUE(plate.isValid()) << plate.validationReport();
    EXPECT_TRUE(hz::topo::GeometryValidator::isGeometricallyValid(plate))
        << hz::topo::GeometryValidator::report(plate);

    // 10 x 10 x 2 less the 4 x 4 hole. The top and bottom faces used to
    // count the hole as solid, and their area with it.
    const auto props = MassPropertiesCalculator::compute(plate);
    EXPECT_NEAR(props.volume, 200.0 - 32.0, 1e-9);
    EXPECT_NEAR(props.surfaceArea, 2 * (100.0 - 16.0) + 4 * 10 * 2 + 4 * 4 * 2, 1e-9);
    // Drawn without the hole: the top and bottom faces used to be drawn over it.
    EXPECT_NEAR(drawnArea(plate, 2.0), 84.0, 1e-3);
    EXPECT_NEAR(drawnArea(plate, 0.0), 84.0, 1e-3);
}

TEST(StepFidelityTest, AFaceWithARoundHoleIsDrawnWithTheHole) {
    const auto solids = StepFormat::fromString(fixture("plate_with_round_hole.step"));
    ASSERT_EQ(solids.size(), 1u) << StepFormat::lastError();
    const auto& plate = *solids[0];
    EXPECT_TRUE(plate.isValid()) << plate.validationReport();
    // The hole's two arcs are followed, not cut across: the faces are drawn
    // without it, to within the arcs' sampling.
    const double face = 100.0 - hz::math::kPi * 4.0;
    EXPECT_NEAR(drawnArea(plate, 2.0), face, 0.1);
    EXPECT_NEAR(drawnArea(plate, 0.0), face, 0.1);
}

TEST(StepFidelityTest, APlateWithAHoleRoundTripsAndTakesABoolean) {
    const auto imported = StepFormat::fromString(fixture("plate_with_square_hole.step"));
    ASSERT_EQ(imported.size(), 1u) << StepFormat::lastError();

    // Written with its hole (FACE_BOUND) and read back the same.
    const auto again = StepFormat::fromString(StepFormat::toString({imported[0].get()}));
    ASSERT_EQ(again.size(), 1u) << StepFormat::lastError();
    EXPECT_NEAR(volumeOf(*again[0]), 168.0, 1e-9);

    // A Boolean reads the faces as they are, hole and all: cutting a corner
    // takes its volume and no more.
    const auto corner = hz::model::PrimitiveFactory::makeBox(2, 2, 4);
    const auto moved =
        hz::model::Pattern::transformed(*corner, hz::math::Mat4::translation(Vec3(-1, -1, -1)));
    std::string reason;
    const auto cut = hz::model::BooleanOp::execute(*imported[0], *moved,
                                                   hz::model::BooleanType::Subtract, &reason);
    ASSERT_NE(cut, nullptr) << reason;
    EXPECT_NEAR(volumeOf(*cut), 168.0 - 1 * 1 * 2, 1e-6);
}

// ---------------------------------------------------------------------------
// Units
// ---------------------------------------------------------------------------

namespace {

const char* const kMillimetres = "(LENGTH_UNIT()NAMED_UNIT(*)SI_UNIT(.MILLI.,.METRE.))";

bool contains(const std::vector<std::string>& lines, const std::string& part) {
    for (const auto& line : lines) {
        if (line.find(part) != std::string::npos) return true;
    }
    return false;
}

Vec3 farCorner(const hz::topo::Solid& solid) {
    Vec3 far(-1e300, -1e300, -1e300);
    for (const auto& v : solid.vertices()) {
        far = Vec3(std::max(far.x, v.point.x), std::max(far.y, v.point.y),
                   std::max(far.z, v.point.z));
    }
    return far;
}

}  // namespace

TEST(StepFidelityTest, AFileInMetresOrCentimetresIsScaledIntoMillimetres) {
    const std::string plate = fixture("plate_with_square_hole.step");
    struct Case {
        const char* unit;
        double mm;
        const char* says;
    };
    for (const Case& c : {Case{"(LENGTH_UNIT()NAMED_UNIT(*)SI_UNIT($,.METRE.))", 1000.0,
                               "1 solid drawn in metres, scaled by 1000 into millimetres"},
                          Case{"(LENGTH_UNIT()NAMED_UNIT(*)SI_UNIT(.CENTI.,.METRE.))", 10.0,
                               "1 solid drawn in centimetres, scaled by 10 into millimetres"}}) {
        ImportReport report;
        const auto solids = StepFormat::fromString(replaced(plate, kMillimetres, c.unit), &report);
        ASSERT_EQ(solids.size(), 1u) << StepFormat::lastError();
        EXPECT_NEAR(volumeOf(*solids[0]), 168.0 * c.mm * c.mm * c.mm,
                    168.0 * c.mm * c.mm * c.mm * 1e-12)
            << c.unit;
        const Vec3 far = farCorner(*solids[0]);
        EXPECT_NEAR(far.x, 10.0 * c.mm, 1e-9 * c.mm);
        EXPECT_TRUE(contains(report.converted, c.says)) << c.unit;
        EXPECT_TRUE(report.empty()) << "a conversion loses nothing";
    }
}

TEST(StepFidelityTest, AFileInInchesIsScaledIntoMillimetres) {
    // An inch is a conversion-based unit: 25.4 of the millimetre unit.
    std::string inches =
        replaced(fixture("plate_with_square_hole.step"), kMillimetres,
                 "(CONVERSION_BASED_UNIT('INCH',#9001)LENGTH_UNIT()NAMED_UNIT(#9003))");
    inches = replaced(inches, "\nENDSEC;\nEND-ISO-10303-21;",
                      "\n#9001=LENGTH_MEASURE_WITH_UNIT(LENGTH_MEASURE(25.4),#9002);\n"
                      "#9002=" +
                          std::string(kMillimetres) +
                          ";\n"
                          "#9003=DIMENSIONAL_EXPONENTS(1.,0.,0.,0.,0.,0.,0.);\n"
                          "ENDSEC;\nEND-ISO-10303-21;");
    ImportReport report;
    const auto solids = StepFormat::fromString(inches, &report);
    ASSERT_EQ(solids.size(), 1u) << StepFormat::lastError();
    const double cube = 25.4 * 25.4 * 25.4;
    EXPECT_NEAR(volumeOf(*solids[0]), 168.0 * cube, 168.0 * cube * 1e-12);
    EXPECT_NEAR(farCorner(*solids[0]).x, 254.0, 1e-9);
    EXPECT_TRUE(
        contains(report.converted, "1 solid drawn in inches, scaled by 25.4 into millimetres"));
}

TEST(StepFidelityTest, AUnitThatCannotBeReadIsTakenAsMillimetresAndSaidSo) {
    ImportReport report;
    const auto solids =
        StepFormat::fromString(replaced(fixture("plate_with_square_hole.step"), kMillimetres,
                                        "(LENGTH_UNIT()NAMED_UNIT(*)SI_UNIT(.SOMETHING.,.METRE.))"),
                               &report);
    ASSERT_EQ(solids.size(), 1u) << StepFormat::lastError();
    EXPECT_NEAR(volumeOf(*solids[0]), 168.0, 1e-9);
    EXPECT_TRUE(contains(report.approximated, "the length unit could not be read"));
}

// ---------------------------------------------------------------------------
// A bad solid among good ones
// ---------------------------------------------------------------------------

TEST(StepFidelityTest, ASolidThatCannotBeReadDoesNotTakeTheOthersWithIt) {
    const auto small = hz::model::PrimitiveFactory::makeBox(1, 2, 3);
    const auto large = hz::model::PrimitiveFactory::makeBox(4, 5, 6);
    std::string text = StepFormat::toString({small.get(), large.get()});

    // The second solid's shell reference points at nothing. The whole file
    // used to be refused for it.
    const size_t second = text.find("MANIFOLD_SOLID_BREP(", text.find("MANIFOLD_SOLID_BREP(") + 1);
    ASSERT_NE(second, std::string::npos);
    const size_t ref = text.find(",#", second);
    const size_t close = text.find(')', ref);
    text.replace(ref, close - ref, ",#999999");

    ImportReport report;
    const auto solids = StepFormat::fromString(text, &report);
    ASSERT_EQ(solids.size(), 1u) << StepFormat::lastError();
    EXPECT_NEAR(volumeOf(*solids[0]), 6.0, 1e-9);
    EXPECT_TRUE(StepFormat::lastError().empty()) << "reading the file succeeded";
    ASSERT_EQ(report.skipped.size(), 1u);
    EXPECT_NE(report.skipped[0].find("solid 2"), std::string::npos) << report.skipped[0];
}
