#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <map>
#include <memory>
#include <set>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "horizon/geometry/Frame.h"
#include "horizon/geometry/surfaces/Surface.h"
#include "horizon/math/Constants.h"
#include "horizon/math/Vec3.h"
#include "horizon/modeling/BoundaryMesh.h"
#include "horizon/modeling/ExactPrimitives.h"
#include "horizon/modeling/MassProperties.h"
#include "horizon/modeling/PrimitiveFactory.h"
#include "horizon/modeling/SolidTessellator.h"
#include "horizon/topology/HalfEdge.h"
#include "horizon/topology/Queries.h"
#include "horizon/topology/Solid.h"

using namespace hz::model;
using hz::math::Vec3;

// ---------------------------------------------------------------------------
// Basic tessellation tests
// ---------------------------------------------------------------------------

TEST(SolidTessellatorTest, BoxTessellation) {
    auto solid = PrimitiveFactory::makeBox(10.0, 5.0, 3.0);
    auto mesh = SolidTessellator::tessellate(*solid, 0.1);
    EXPECT_FALSE(mesh.positions.empty());
    EXPECT_FALSE(mesh.normals.empty());
    EXPECT_FALSE(mesh.indices.empty());
    EXPECT_EQ(mesh.positions.size(), mesh.normals.size());
    EXPECT_EQ(mesh.indices.size() % 3, 0u);
}

TEST(SolidTessellatorTest, AllIndicesInRange) {
    auto solid = PrimitiveFactory::makeBox(10.0, 5.0, 3.0);
    auto mesh = SolidTessellator::tessellate(*solid, 0.1);
    size_t vertexCount = mesh.positions.size() / 3;
    for (uint32_t idx : mesh.indices) {
        EXPECT_LT(idx, vertexCount);
    }
}

TEST(SolidTessellatorTest, CylinderTessellation) {
    auto solid = PrimitiveFactory::makeCylinder(5.0, 10.0);
    auto mesh = SolidTessellator::tessellate(*solid, 0.1);
    EXPECT_FALSE(mesh.positions.empty());
    EXPECT_EQ(mesh.indices.size() % 3, 0u);
}

TEST(SolidTessellatorTest, SphereTessellation) {
    auto solid = PrimitiveFactory::makeSphere(5.0);
    auto mesh = SolidTessellator::tessellate(*solid, 0.1);
    EXPECT_FALSE(mesh.positions.empty());
}

TEST(SolidTessellatorTest, NormalsAreUnitLength) {
    auto solid = PrimitiveFactory::makeBox(10.0, 10.0, 10.0);
    auto mesh = SolidTessellator::tessellate(*solid, 0.5);
    EXPECT_FALSE(mesh.normals.empty());
    // Every normal should be approximately unit length.
    for (size_t i = 0; i < mesh.normals.size(); i += 3) {
        Vec3 n(mesh.normals[i], mesh.normals[i + 1], mesh.normals[i + 2]);
        double len = n.length();
        EXPECT_NEAR(len, 1.0, 0.01) << "Normal at vertex " << (i / 3) << " has length " << len;
    }
}

TEST(SolidTessellatorTest, ConeTessellation) {
    auto solid = PrimitiveFactory::makeCone(3.0, 1.0, 5.0);
    auto mesh = SolidTessellator::tessellate(*solid, 0.1);
    EXPECT_FALSE(mesh.positions.empty());
    EXPECT_EQ(mesh.indices.size() % 3, 0u);
    size_t vertexCount = mesh.positions.size() / 3;
    for (uint32_t idx : mesh.indices) {
        EXPECT_LT(idx, vertexCount);
    }
}

TEST(SolidTessellatorTest, TorusTessellation) {
    auto solid = PrimitiveFactory::makeTorus(5.0, 1.5);
    auto mesh = SolidTessellator::tessellate(*solid, 0.1);
    EXPECT_FALSE(mesh.positions.empty());
    EXPECT_EQ(mesh.indices.size() % 3, 0u);
    size_t vertexCount = mesh.positions.size() / 3;
    for (uint32_t idx : mesh.indices) {
        EXPECT_LT(idx, vertexCount);
    }
}

TEST(SolidTessellatorTest, EmptySolidProducesEmptyMesh) {
    hz::topo::Solid empty;
    auto mesh = SolidTessellator::tessellate(empty, 0.1);
    EXPECT_TRUE(mesh.positions.empty());
    EXPECT_TRUE(mesh.normals.empty());
    EXPECT_TRUE(mesh.indices.empty());
}

// ---------------------------------------------------------------------------
// Display geometry matches the solid, once per face.
//
// SolidTessellator emits a face's whole *untrimmed* carrier when that carrier
// is curved.  With the old box-topology primitives that meant every face on a
// shared surface re-emitted the entire surface: a sphere bound one sphere
// patch to six faces and drew it six times — 480,000 triangles of overlapping
// geometry for a shape that needs a couple of thousand, and a mesh whose
// enclosed volume was three times the solid's.
//
// Faceted primitives carry planar patches that match their loops, so every
// face takes the loop path and is emitted exactly once.  These bounds pin
// that: a regression to shared curved carriers would blow straight past them.
// ---------------------------------------------------------------------------

TEST(SolidTessellatorTest, CurvedPrimitivesEmitEachFaceExactlyOnce) {
    struct Case {
        const char* name;
        std::unique_ptr<hz::topo::Solid> solid;
    };
    std::vector<Case> cases;
    cases.push_back({"cylinder", PrimitiveFactory::makeCylinder(5.0, 10.0)});
    cases.push_back({"sphere", PrimitiveFactory::makeSphere(5.0)});
    cases.push_back({"cone", PrimitiveFactory::makeCone(5.0, 2.0, 10.0)});
    cases.push_back({"torus", PrimitiveFactory::makeTorus(10.0, 3.0)});

    for (const auto& c : cases) {
        SCOPED_TRACE(c.name);
        ASSERT_NE(c.solid, nullptr);
        const auto mesh = SolidTessellator::tessellate(*c.solid, 0.01);
        const size_t tris = mesh.indices.size() / 3;

        // A loop of k vertices triangulates to exactly k-2 triangles, so one
        // pass over the faces gives the count the mesh must have.
        size_t expected = 0;
        for (const auto& f : c.solid->faces()) {
            expected += static_cast<size_t>(hz::topo::loopSize(f.outerLoop)) - 2;
        }
        EXPECT_EQ(tris, expected) << "each face must be emitted once — " << tris
                                  << " triangles for " << c.solid->faceCount() << " faces";
    }
}

TEST(SolidTessellatorTest, TessellatedMeshEnclosesTheSolidsVolume) {
    // The display mesh and the mass-properties integrator must agree; when
    // curved carriers were re-emitted per face they did not (a cylinder's mesh
    // enclosed 2388 against the solid's 785).
    auto cyl = PrimitiveFactory::makeCylinder(5.0, 10.0);
    ASSERT_NE(cyl, nullptr);
    const auto mesh = SolidTessellator::tessellate(*cyl, 0.01);

    double vol6 = 0.0;
    for (size_t i = 0; i + 2 < mesh.indices.size(); i += 3) {
        Vec3 p[3];
        for (int k = 0; k < 3; ++k) {
            const size_t base = 3 * static_cast<size_t>(mesh.indices[i + static_cast<size_t>(k)]);
            p[k] = Vec3(mesh.positions[base], mesh.positions[base + 1], mesh.positions[base + 2]);
        }
        vol6 += p[0].dot(p[1].cross(p[2]));
    }
    const double meshVolume = std::abs(vol6) / 6.0;
    const double solidVolume = MassPropertiesCalculator::compute(*cyl).volume;
    EXPECT_NEAR(meshVolume, solidVolume, 1e-6 * solidVolume);
}

// -- What the mesh says of the solid (Phase 132) ------------------------------

// Every triangle names its face, and the edges are the box's twelve.
TEST(SolidTessellatorTest, TrianglesNameTheirFacesAndTheEdgesAreThePartsOwn) {
    const auto box = hz::model::PrimitiveFactory::makeBox(2.0, 3.0, 4.0);
    const auto mesh = hz::model::SolidTessellator::tessellate(*box, 0.1);
    ASSERT_TRUE(mesh.hasFaces());
    EXPECT_EQ(mesh.faceTags.size(), 6u);
    EXPECT_NE(std::find(mesh.faceTags.begin(), mesh.faceTags.end(), "box/top"),
              mesh.faceTags.end());
    EXPECT_EQ(mesh.edges.size(), 12u);
    for (const auto& edge : mesh.edges) {
        EXPECT_EQ(edge.points.size(), 6u) << "a straight edge is a line";
        EXPECT_FALSE(edge.tag.empty());
    }
}

// A faceted cylinder shows its two rims, not a seam between every facet.
TEST(SolidTessellatorTest, ACylindersFacetsDoNotShowAsEdges) {
    const auto cylinder = hz::model::PrimitiveFactory::makeCylinder(1.0, 2.0, 16);
    const auto mesh = hz::model::SolidTessellator::tessellate(*cylinder, 0.1);
    ASSERT_FALSE(mesh.edges.empty());
    for (const auto& edge : mesh.edges) {
        const float z0 = edge.points[2];
        const float z1 = edge.points[edge.points.size() - 1];
        EXPECT_FLOAT_EQ(z0, z1) << edge.tag << " runs up the side: a seam between facets";
    }
    EXPECT_EQ(mesh.edges.size(), 32u) << "sixteen chords round each rim";
}

// -- A solid with exact faces (stage 2) ---------------------------------------

namespace {

/// A cylinder of radius 1 and height 2 about +z, of whole exact faces: a side with its seam, and
/// two caps.
std::unique_ptr<hz::topo::Solid> exactCylinder() {
    return hz::model::exact::makeCylinder(hz::geo::Frame::fromAxis({0, 0, 0}, {0, 0, 1}), 1.0, 2.0);
}

/// Vertex @p i of @p mesh, and its normal.
Vec3 positionOf(const hz::geo::MeshData& mesh, std::size_t i) {
    return {mesh.positions[3 * i], mesh.positions[3 * i + 1], mesh.positions[3 * i + 2]};
}
Vec3 normalOf(const hz::geo::MeshData& mesh, std::size_t i) {
    return {mesh.normals[3 * i], mesh.normals[3 * i + 1], mesh.normals[3 * i + 2]};
}

/// The tag of @p solid's face on a surface of @p kind.
std::string faceOfKind(const hz::topo::Solid& solid, hz::geo::SurfaceKind kind) {
    for (const auto& face : solid.faces()) {
        if (face.exactSurface && face.exactSurface->kind() == kind) return face.topoId.tag();
    }
    return {};
}

}  // namespace

// A solid with exact faces is meshed at the read point to the tolerance asked, and named by face:
// one entry of faceTags for each face, not one for each of its triangles, so a pick names the
// face. Its triangles are shaded by the surface's normals (radial on the side, which looks
// round), its corners lie on the surfaces, and it encloses the cylinder within its tolerance.
// Its seam (the side on both sides of it) is not drawn; its rims are, round their circles.
TEST(SolidTessellatorTest, OneTagPerFace) {
    const auto cylinder = exactCylinder();
    ASSERT_NE(cylinder, nullptr);
    constexpr double kTolerance = 0.01;
    const auto mesh = SolidTessellator::tessellate(*cylinder, kTolerance);
    ASSERT_TRUE(mesh.hasFaces());
    std::set<std::string> faces;
    for (const auto& face : cylinder->faces()) faces.insert(face.topoId.tag());
    EXPECT_EQ(mesh.faceTags.size(), 3u);
    EXPECT_EQ(std::set<std::string>(mesh.faceTags.begin(), mesh.faceTags.end()), faces);
    std::vector<std::size_t> triangles(mesh.faceTags.size(), 0);
    for (const std::uint32_t face : mesh.triangleFaces) {
        ASSERT_LT(face, triangles.size());
        ++triangles[face];
    }
    for (std::size_t f = 0; f < triangles.size(); ++f) {
        EXPECT_GT(triangles[f], 0u) << mesh.faceTags[f];
    }

    const std::string side = faceOfKind(*cylinder, hz::geo::SurfaceKind::Cylinder);
    ASSERT_FALSE(side.empty());
    double vol6 = 0.0;
    double area = 0.0;
    std::size_t offSide = 0;  // side corners off the surface, or not shaded radially
    std::size_t offCaps = 0;  // cap corners outside the rim, or not shaded along the axis
    for (std::size_t t = 0; t < mesh.indices.size() / 3; ++t) {
        const bool onSide = mesh.faceTags[mesh.triangleFaces[t]] == side;
        Vec3 p[3];
        for (std::size_t k = 0; k < 3; ++k) {
            const std::size_t i = mesh.indices[3 * t + k];
            p[k] = positionOf(mesh, i);
            const Vec3 n = normalOf(mesh, i);
            const double r = std::hypot(p[k].x, p[k].y);
            if (onSide) {
                const Vec3 radial(p[k].x / r, p[k].y / r, 0.0);
                offSide += std::abs(r - 1.0) > 1e-6 || (n - radial).length() > 1e-5 ? 1 : 0;
            } else {
                const double out = p[k].z > 1.0 ? 1.0 : -1.0;
                offCaps += r > 1.0 + 1e-6 || std::abs(n.z - out) > 1e-6 ? 1 : 0;
            }
        }
        vol6 += p[0].dot(p[1].cross(p[2]));
        area += 0.5 * (p[1] - p[0]).cross(p[2] - p[0]).length();
    }
    EXPECT_EQ(offSide, 0u) << "the side's corners on it, each shaded by its normal there: radial";
    EXPECT_EQ(offCaps, 0u) << "the caps' corners within their rims, shaded facing out";
    EXPECT_NEAR(vol6 / 6.0, 2.0 * hz::math::kPi, area * kTolerance) << "wound outward";

    ASSERT_EQ(mesh.edges.size(), 2u) << "the two rims, not the seam";
    for (const auto& edge : mesh.edges) {
        ASSERT_GE(edge.points.size(), 3u * 18u) << "round its circle, not a line";
        const float z = edge.points[2];
        for (std::size_t k = 0; k + 2 < edge.points.size(); k += 3) {
            EXPECT_NEAR(std::hypot(edge.points[k], edge.points[k + 1]), 1.0, 1e-6);
            EXPECT_FLOAT_EQ(edge.points[k + 2], z) << "a rim; the seam runs up the side";
        }
    }
}

// A face the read point cannot mesh is said, never thrown for: the faces that can be meshed are,
// and that one is left out.
TEST(SolidTessellatorTest, AFaceThatCannotBeMeshedIsSaidNotThrown) {
    auto cylinder = exactCylinder();
    ASSERT_NE(cylinder, nullptr);
    const std::string side = faceOfKind(*cylinder, hz::geo::SurfaceKind::Cylinder);
    for (auto& face : cylinder->faces()) {
        if (face.topoId.tag() == side) face.outerLoop->halfEdge->pcurve = nullptr;
    }
    hz::geo::MeshData mesh;
    EXPECT_NO_THROW(mesh = SolidTessellator::tessellate(*cylinder, 0.01));
    EXPECT_EQ(mesh.faceTags.size(), 2u) << "the caps";
    EXPECT_EQ(std::find(mesh.faceTags.begin(), mesh.faceTags.end(), side), mesh.faceTags.end());

    std::string problem;
    const auto said = SolidTessellator::tessellate(*cylinder, 0.01, &problem);
    EXPECT_EQ(problem.rfind(side + ": ", 0), 0u) << problem;
    EXPECT_EQ(said.faceTags, mesh.faceTags);

    std::string none = "not cleared";
    (void)SolidTessellator::tessellate(*exactCylinder(), 0.01, &none);
    EXPECT_TRUE(none.empty()) << none;
}

// -- The display's density -------------------------------------------------------

namespace {

/// How many distinct points of @p mesh lie on the circle of radius @p r about the z axis in the
/// plane z = @p z: a rim as drawn, a point for each of its segments.
std::size_t rimPoints(const hz::geo::MeshData& mesh, double r, double z) {
    std::set<std::array<float, 3>> points;
    for (std::size_t i = 0; i < mesh.positions.size() / 3; ++i) {
        const Vec3 p = positionOf(mesh, i);
        if (std::abs(p.z - z) <= 1e-4 * r && std::abs(std::hypot(p.x, p.y) - r) <= 1e-4 * r) {
            points.insert(
                {mesh.positions[3 * i], mesh.positions[3 * i + 1], mesh.positions[3 * i + 2]});
        }
    }
    return points.size();
}

/// An exact cylinder of radius @p r and height @p h, its bottom rim at z = 0.
std::unique_ptr<hz::topo::Solid> exactCylinder(double r, double h) {
    return hz::model::exact::makeCylinder(hz::geo::Frame::fromAxis({0, 0, 0}, {0, 0, 1}), r, h);
}

}  // namespace

// An exact part is drawn at the display's angle, coarser than the read point's at the same
// tolerance: a small cylinder, whose rims the angle samples (the tolerance would allow fewer
// chords), is drawn with fewer triangles than the read point gives it, and each rim with chords
// that turn by up to half the display's angle (26 a turn, where the read point's makes 36).
TEST(SolidTessellatorTest, AnExactPartIsDrawnAtTheDisplaysAngle) {
    const auto cylinder = exactCylinder(2.0, 4.0);
    ASSERT_NE(cylinder, nullptr);
    const auto mesh = SolidTessellator::tessellate(*cylinder);
    const auto read =
        BoundaryMesh::extractFacePolygons(*cylinder, SolidTessellator::kDisplayTolerance);
    EXPECT_LT(mesh.indices.size() / 3, read.size());
    const auto segments = static_cast<std::size_t>(
        std::ceil(2.0 * hz::math::kPi / (0.5 * SolidTessellator::kDisplayMaxAngle)));
    EXPECT_EQ(rimPoints(mesh, 2.0, 0.0), segments);
    EXPECT_EQ(rimPoints(mesh, 2.0, 4.0), segments);
}

// However coarse the display's angle, a large round is drawn round: a cylinder of radius 50 has
// at least 13 segments round each rim, and so has one of radius 5.
TEST(SolidTessellatorTest, ALargeCylinderIsDrawnRound) {
    for (const double r : {5.0, 50.0}) {
        SCOPED_TRACE(r);
        const auto cylinder = exactCylinder(r, 20.0);
        ASSERT_NE(cylinder, nullptr);
        std::string problem;
        const auto mesh =
            SolidTessellator::tessellate(*cylinder, SolidTessellator::kDisplayTolerance, &problem);
        EXPECT_TRUE(problem.empty()) << problem;
        EXPECT_GE(rimPoints(mesh, r, 0.0), 13u);
        EXPECT_GE(rimPoints(mesh, r, 20.0), 13u);
    }
}

// The display's mesh of an exact part is watertight across the faces' shared edges: each side of
// a triangle, by its corners' positions, is met once the other way round by another (an edge's
// samples are both its faces'), along a seam too. A plate with a round hole, and a cylinder.
TEST(SolidTessellatorTest, AnExactPartsMeshIsWatertight) {
    std::vector<std::unique_ptr<hz::topo::Solid>> parts;
    parts.push_back(hz::model::exact::makePlateWithHole({0, 0, 0}, {40, 30, 5}, {20, 15}, 4.0));
    parts.push_back(exactCylinder(50.0, 20.0));
    for (const auto& part : parts) {
        ASSERT_NE(part, nullptr);
        std::string problem;
        const auto mesh =
            SolidTessellator::tessellate(*part, SolidTessellator::kDisplayTolerance, &problem);
        ASSERT_TRUE(problem.empty()) << problem;
        ASSERT_FALSE(mesh.indices.empty());
        using Corner = std::array<float, 3>;
        const auto corner = [&mesh](std::uint32_t i) {
            return Corner{mesh.positions[3 * i], mesh.positions[3 * i + 1],
                          mesh.positions[3 * i + 2]};
        };
        std::map<std::pair<Corner, Corner>, int> sides;
        for (std::size_t t = 0; t + 2 < mesh.indices.size(); t += 3) {
            for (std::size_t k = 0; k < 3; ++k) {
                ++sides[{corner(mesh.indices[t + k]), corner(mesh.indices[t + (k + 1) % 3])}];
            }
        }
        std::size_t open = 0;
        for (const auto& [side, uses] : sides) {
            const auto back = sides.find({side.second, side.first});
            open += uses == 1 && back != sides.end() && back->second == 1 ? 0 : 1;
        }
        EXPECT_EQ(open, 0u) << "sides not met once the other way round, of " << sides.size();
    }
}
