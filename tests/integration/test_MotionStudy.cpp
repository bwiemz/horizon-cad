// Motion studies: a Distance or Angle mate driven through a range, the
// assembly's mates solved at each step (MotionStudy.h).
//
// The mechanisms are two 20 x 10 x 2 plates: a base held by a Fixed mate,
// and an arm mated to it.

#include <gtest/gtest.h>

#include <atomic>
#include <cmath>
#include <filesystem>
#include <memory>
#include <numbers>
#include <string>
#include <vector>

#include "horizon/document/AssemblyDocument.h"
#include "horizon/document/Document.h"
#include "horizon/document/DocumentManager.h"
#include "horizon/document/FeatureTree.h"
#include "horizon/document/MotionStudy.h"
#include "horizon/document/Sketch.h"
#include "horizon/drafting/DraftLine.h"
#include "horizon/fileio/NativeFormat.h"
#include "horizon/modeling/MateGeometry.h"

using namespace hz::doc;
using hz::io::NativeFormat;
using hz::math::Mat4;
using hz::math::Vec2;
using hz::math::Vec3;
using hz::topo::TopologyID;

namespace fs = std::filesystem;

namespace {

/// A mechanism of two plates, the base fixed: its parts saved, its assembly
/// built but its arm not yet mated.
class Mechanism {
public:
    Mechanism() {
        static int made = 0;
        m_dir = fs::temp_directory_path() / ("hz_motion_" + std::to_string(++made));
        fs::create_directories(m_dir);
        const std::string part = (m_dir / "plate.hzpart").string();
        {
            Document plate;
            plate.setType(DocumentType::Part);
            auto sketch = std::make_shared<Sketch>();
            sketch->addEntity(std::make_shared<hz::draft::DraftLine>(Vec2(0, 0), Vec2(20, 0)));
            sketch->addEntity(std::make_shared<hz::draft::DraftLine>(Vec2(20, 0), Vec2(20, 10)));
            sketch->addEntity(std::make_shared<hz::draft::DraftLine>(Vec2(20, 10), Vec2(0, 10)));
            sketch->addEntity(std::make_shared<hz::draft::DraftLine>(Vec2(0, 10), Vec2(0, 0)));
            plate.addSketch(sketch);
            auto extrude = std::make_unique<ExtrudeFeature>(sketch, Vec3(0, 0, 1), 2.0);
            m_feature = extrude->featureID();
            plate.featureTree().addFeature(std::move(extrude));
            EXPECT_TRUE(plate.rebuildModel());
            // The side facing +x, found by where it faces: its name depends
            // on how the sketch's lines are named.
            for (const auto& face : plate.solid()->faces()) {
                const auto frame = hz::model::MateGeometry::frameForFace(face);
                if (frame && frame->kind == hz::model::MateFrameKind::Planar &&
                    frame->direction.x > 0.99) {
                    m_side = face.topoId;
                }
            }
            EXPECT_TRUE(NativeFormat::save(part, plate));
        }
        m_documents.setPartLoader(
            [](const std::string& path, Document& doc) { return NativeFormat::load(path, doc); });
        m_documents.setMeshLoader(
            [](const std::string& path) { return NativeFormat::loadPartMesh(path); });

        ComponentInstance base;
        base.name = "Base";
        base.partPath = part;
        m_base = m_assembly.addComponent(base);
        ComponentInstance arm;
        arm.name = "Arm";
        arm.partPath = part;
        arm.transform = Mat4::translation(Vec3(0, 0, 12));
        m_arm = m_assembly.addComponent(arm);
        Mate ground;
        ground.type = MateType::Fixed;
        ground.a.componentId = m_base;
        m_assembly.addMate(ground);
    }
    ~Mechanism() {
        std::error_code ec;
        fs::remove_all(m_dir, ec);
    }
    Mechanism(const Mechanism&) = delete;
    Mechanism& operator=(const Mechanism&) = delete;

    /// A mate between the base's face @p onBase and the arm's @p onArm.
    uint64_t mate(MateType type, const TopologyID& onBase, const TopologyID& onArm,
                  double value = 0.0) {
        Mate m;
        m.type = type;
        m.a = {m_base, onBase};
        m.b = {m_arm, onArm};
        m.value = value;
        return m_assembly.addMate(m);
    }
    TopologyID top() const { return TopologyID::make(m_feature, "cap_top"); }
    TopologyID bottom() const { return TopologyID::make(m_feature, "cap_bottom"); }
    const TopologyID& side() const { return m_side; }

    MotionResult run(uint64_t driver, double from, double to, int steps, bool clashes = true) {
        std::string why;
        const auto input = prepareMotion(m_assembly, m_documents, driver, &why);
        if (!input) {
            ADD_FAILURE() << why;
            return {};
        }
        MotionStudy study;
        study.mate = driver;
        study.from = from;
        study.to = to;
        study.steps = steps;
        study.checkInterference = clashes;
        return runMotion(*input, study);
    }

    AssemblyDocument& assembly() { return m_assembly; }
    DocumentManager& documents() { return m_documents; }
    uint64_t base() const { return m_base; }
    uint64_t arm() const { return m_arm; }

private:
    fs::path m_dir;
    std::string m_feature;
    TopologyID m_side;
    DocumentManager m_documents;
    AssemblyDocument m_assembly;
    uint64_t m_base = 0;
    uint64_t m_arm = 0;
};

double heightOf(const Mat4& placement) {
    return placement.at(2, 3);
}

}  // namespace

TEST(MotionStudyTest, ASlideMovesThroughItsRangeAndLeavesTheAssemblyBe) {
    Mechanism m;
    const uint64_t slide = m.mate(MateType::Distance, m.top(), m.bottom(), 10.0);
    const MotionResult motion = m.run(slide, 10.0, 4.0, 6);
    ASSERT_TRUE(motion.complete) << motion.why;
    ASSERT_EQ(motion.frames.size(), 7u) << "the start and six steps";
    for (std::size_t k = 0; k < motion.frames.size(); ++k) {
        const MotionFrame& frame = motion.frames[k];
        EXPECT_DOUBLE_EQ(frame.value, 10.0 - static_cast<double>(k));
        // The arm's underside is the driven distance above the base's top.
        EXPECT_NEAR(heightOf(frame.placements.at(m.arm())), 2.0 + frame.value, 1e-6) << k;
        EXPECT_NEAR(heightOf(frame.placements.at(m.base())), 0.0, 1e-9) << "the base is fixed";
        EXPECT_TRUE(frame.clashes.empty()) << k;
    }
    EXPECT_FALSE(motion.firstClash.has_value());
    EXPECT_DOUBLE_EQ(heightOf(m.assembly().component(m.arm())->transform), 12.0)
        << "a study moves nothing";
}

TEST(MotionStudyTest, ASlideIntoTheBaseClashesWhereItEntersIt) {
    Mechanism m;
    const uint64_t slide = m.mate(MateType::Distance, m.top(), m.bottom(), 4.0);
    const MotionResult motion = m.run(slide, 4.0, -1.0, 5);
    ASSERT_TRUE(motion.complete) << motion.why;
    ASSERT_EQ(motion.frames.size(), 6u);
    // Resting on it at 0 is touching, not a clash; 1 into it is.
    ASSERT_TRUE(motion.firstClash.has_value());
    EXPECT_EQ(*motion.firstClash, 5u);
    const auto& clash = motion.frames[5].clashes;
    ASSERT_EQ(clash.size(), 1u);
    EXPECT_EQ(std::min(clash[0].componentA, clash[0].componentB), m.base());
    EXPECT_EQ(std::max(clash[0].componentA, clash[0].componentB), m.arm());
    EXPECT_NEAR(clash[0].volume, 20.0 * 10.0 * 1.0, 1e-6);

    const MotionResult unchecked = m.run(slide, 4.0, -1.0, 5, /*clashes=*/false);
    EXPECT_FALSE(unchecked.firstClash.has_value()) << "not looked for";
}

TEST(MotionStudyTest, WhereTheMatesBindTheMotionStops) {
    Mechanism m;
    const uint64_t slide = m.mate(MateType::Distance, m.top(), m.bottom(), 3.0);
    // Another mate holds the arm 3 above the base: the slide can go nowhere.
    m.mate(MateType::Distance, m.top(), m.bottom(), 3.0);
    const MotionResult motion = m.run(slide, 3.0, 6.0, 3);
    EXPECT_FALSE(motion.complete);
    ASSERT_EQ(motion.frames.size(), 1u) << "the start holds";
    ASSERT_TRUE(motion.bound.has_value());
    EXPECT_DOUBLE_EQ(*motion.bound, 4.0);
    EXPECT_FALSE(motion.why.empty());
}

TEST(MotionStudyTest, AHingeTurnsThroughItsAngle) {
    Mechanism m;
    m.mate(MateType::Coincident, m.top(), m.bottom());
    const uint64_t hinge = m.mate(MateType::Angle, m.side(), m.side(), 0.0);
    const double quarter = std::numbers::pi / 2.0;
    const MotionResult motion = m.run(hinge, 0.0, quarter, 6, /*clashes=*/false);
    ASSERT_TRUE(motion.complete) << motion.why;
    ASSERT_EQ(motion.frames.size(), 7u);
    for (const MotionFrame& frame : motion.frames) {
        const Mat4& arm = frame.placements.at(m.arm());
        // Turned about the vertical by the driven angle: its own x along
        // (cos, sin) of it, one way or the other.
        EXPECT_NEAR(std::abs(arm.at(0, 0)), std::abs(std::cos(frame.value)), 1e-6);
        EXPECT_NEAR(std::abs(arm.at(2, 2)), 1.0, 1e-6) << "still flat on the base";
        EXPECT_NEAR(heightOf(arm), 2.0, 1e-6);
    }
}

TEST(MotionStudyTest, OnlyADistanceOrAnAngleMateDrives) {
    Mechanism m;
    const uint64_t rest = m.mate(MateType::Coincident, m.top(), m.bottom());
    std::string why;
    EXPECT_FALSE(prepareMotion(m.assembly(), m.documents(), rest, &why).has_value());
    EXPECT_EQ(why, "a motion is driven by a Distance or an Angle mate");
    EXPECT_FALSE(prepareMotion(m.assembly(), m.documents(), 999, &why).has_value());
    EXPECT_EQ(why, "there is no mate 999");
}

TEST(MotionStudyTest, ACancelledStudyStopsWithWhatItReached) {
    Mechanism m;
    const uint64_t slide = m.mate(MateType::Distance, m.top(), m.bottom(), 10.0);
    std::string why;
    const auto input = prepareMotion(m.assembly(), m.documents(), slide, &why);
    ASSERT_TRUE(input.has_value()) << why;
    MotionStudy study;
    study.mate = slide;
    study.from = 10.0;
    study.to = 0.0;
    const std::atomic<bool> cancelled{true};
    const MotionResult motion = runMotion(*input, study, &cancelled);
    EXPECT_FALSE(motion.complete);
    EXPECT_TRUE(motion.frames.empty());
    EXPECT_EQ(motion.why, "cancelled");
}

TEST(MotionStudyTest, ARangeThatIsNotNumbersIsRefused) {
    Mechanism m;
    const uint64_t slide = m.mate(MateType::Distance, m.top(), m.bottom(), 10.0);
    const MotionResult motion = m.run(slide, 0.0, std::nan(""), 4);
    EXPECT_FALSE(motion.complete);
    EXPECT_TRUE(motion.frames.empty());
}

TEST(MotionStudyTest, APoseIsDrawnInPlaceOfThePlacementsAndIsNoEdit) {
    Mechanism m;
    AssemblyDocument& assembly = m.assembly();
    assembly.setDirty(false);
    const ComponentInstance& arm = *assembly.component(m.arm());
    assembly.setPose({{m.arm(), Mat4::translation(Vec3(0, 0, 30))}});
    EXPECT_DOUBLE_EQ(heightOf(assembly.displayTransform(arm)), 30.0);
    EXPECT_DOUBLE_EQ(heightOf(arm.transform), 12.0) << "where it is placed is unchanged";
    EXPECT_DOUBLE_EQ(heightOf(assembly.displayTransform(*assembly.component(m.base()))), 0.0)
        << "a component the pose leaves out is drawn where it is";
    EXPECT_FALSE(assembly.isDirty());
    assembly.setPose({});
    EXPECT_DOUBLE_EQ(heightOf(assembly.displayTransform(arm)), 12.0);
}
