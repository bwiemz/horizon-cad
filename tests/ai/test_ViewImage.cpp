// The pictures a model is shown of the part: drawn in software, so each is
// the same every time, and checked here pixel by pixel.

#include <gtest/gtest.h>
#include <zlib.h>

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "horizon/ai/CadTools.h"
#include "horizon/ai/ViewImage.h"
#include "horizon/document/Document.h"
#include "horizon/geometry/MeshData.h"
#include "horizon/modeling/SolidTessellator.h"

using namespace hz;
using ai::ViewImage;
using ai::ViewMesh;
using nlohmann::json;

namespace {

struct Pixel {
    int r = 0;
    int g = 0;
    int b = 0;
    bool operator==(const Pixel&) const = default;
};

Pixel at(const ViewImage& image, int x, int y) {
    const std::size_t i = (static_cast<std::size_t>(y) * static_cast<std::size_t>(image.width) +
                           static_cast<std::size_t>(x)) *
                          3;
    return {image.rgb[i], image.rgb[i + 1], image.rgb[i + 2]};
}

const Pixel kBackground{250, 250, 252};
const Pixel kEdge{35, 40, 52};

bool shaded(const Pixel& p) {
    return !(p == kBackground) && !(p == kEdge) && p.b > p.r;  // the part's steel blue
}

/// A part built with the tools, as a mesh to draw.
struct Part {
    doc::Document document;
    ai::ToolRegistry tools;

    Part() { ai::registerCadTools(tools, document); }
    json run(const std::string& name, const json& in) {
        const ai::ToolResult result = tools.run(name, in);
        EXPECT_FALSE(result.isError) << name << ": " << result.content;
        return json::parse(result.content, nullptr, false);
    }
    std::vector<ViewMesh> meshes() const {
        if (document.solid() == nullptr) return {};
        return {{std::make_shared<const geo::MeshData>(
                     model::SolidTessellator::tessellate(*document.solid())),
                 math::Mat4::identity()}};
    }
};

/// How many columns of row @p y show the part, and how many rows of column
/// @p x do.
int shadedAcross(const ViewImage& image, int y) {
    int n = 0;
    for (int x = 0; x < image.width; ++x) n += at(image, x, y) == kBackground ? 0 : 1;
    return n;
}
int shadedDown(const ViewImage& image, int x) {
    int n = 0;
    for (int y = 0; y < image.height; ++y) n += at(image, x, y) == kBackground ? 0 : 1;
    return n;
}

}  // namespace

TEST(ViewImage, EveryViewShowsTheBoxInTheMiddle) {
    Part part;
    part.run(
        "add_primitive",
        {{"shape", "box"}, {"size_x", 40}, {"size_y", 20}, {"size_z", 10}, {"centered", true}});
    for (const auto& view : ai::viewNames()) {
        const auto image = ai::renderView(part.meshes(), view, 400, 300);
        ASSERT_TRUE(image.has_value()) << view;
        EXPECT_EQ(image->rgb.size(), 400u * 300u * 3u);
        EXPECT_TRUE(shaded(at(*image, 200, 150))) << view;
        EXPECT_EQ(at(*image, 395, 5), kBackground) << view;
    }
    EXPECT_FALSE(ai::renderView(part.meshes(), "sideways", 400, 300).has_value());
    EXPECT_FALSE(ai::renderView({}, "iso", 400, 300).has_value());
}

TEST(ViewImage, AViewIsSeenTheWayItIsNamed) {
    Part part;
    part.run(
        "add_primitive",
        {{"shape", "box"}, {"size_x", 40}, {"size_y", 20}, {"size_z", 10}, {"centered", true}});
    // From the top, 40 across and 20 down; from the front, 40 across and 10
    // down; from the right, 20 across and 10 down: fitted, the same scale
    // both ways.
    const auto top = ai::renderView(part.meshes(), "top", 400, 400);
    ASSERT_TRUE(top.has_value());
    EXPECT_NEAR(static_cast<double>(shadedAcross(*top, 200)) / shadedDown(*top, 200), 2.0, 0.1);
    const auto front = ai::renderView(part.meshes(), "front", 400, 400);
    ASSERT_TRUE(front.has_value());
    EXPECT_NEAR(static_cast<double>(shadedAcross(*front, 200)) / shadedDown(*front, 200), 4.0, 0.2);
    const auto right = ai::renderView(part.meshes(), "right", 400, 400);
    ASSERT_TRUE(right.has_value());
    EXPECT_NEAR(static_cast<double>(shadedAcross(*right, 200)) / shadedDown(*right, 200), 2.0, 0.1);
}

TEST(ViewImage, AHoleIsSeenThroughAndEdgesAreDrawn) {
    Part part;
    part.run(
        "add_primitive",
        {{"shape", "box"}, {"size_x", 40}, {"size_y", 40}, {"size_z", 10}, {"centered", true}});
    part.run("add_hole",
             {{"face", "top"}, {"position", {0, 0, 5}}, {"diameter", 16}, {"through_all", true}});
    const auto top = ai::renderView(part.meshes(), "top", 400, 400);
    ASSERT_TRUE(top.has_value());
    EXPECT_EQ(at(*top, 200, 200), kBackground) << "through the hole";
    // Along the middle row: background, an edge, the plate, the hole's edge,
    // the hole, and back.
    int edges = 0;
    bool inEdge = false;
    for (int x = 0; x < 400; ++x) {
        const bool edge = at(*top, x, 200) == kEdge;
        if (edge && !inEdge) ++edges;
        inEdge = edge;
    }
    EXPECT_EQ(edges, 4) << "the outline twice, the hole's rim twice";
}

TEST(ViewImage, ACurvedFaceIsOneFaceNotItsFacets) {
    Part part;
    part.run("add_primitive", {{"shape", "cylinder"}, {"radius", 10}, {"height", 30}});
    const auto front = ai::renderView(part.meshes(), "front", 400, 400);
    ASSERT_TRUE(front.has_value());
    int edges = 0;
    bool inEdge = false;
    for (int x = 0; x < 400; ++x) {
        const bool edge = at(*front, x, 200) == kEdge;
        if (edge && !inEdge) ++edges;
        inEdge = edge;
    }
    EXPECT_EQ(edges, 2) << "its outline, not a line between each facet";
}

TEST(ViewImage, TheAxesAreMarkedInTheLowerLeft) {
    Part part;
    part.run("add_primitive", {{"shape", "box"}, {"size_x", 10}, {"size_y", 10}, {"size_z", 10}});
    const auto iso = ai::renderView(part.meshes(), "iso", 400, 300);
    ASSERT_TRUE(iso.has_value());
    bool red = false;
    bool green = false;
    bool blue = false;
    for (int y = 200; y < 300; ++y) {
        for (int x = 0; x < 100; ++x) {
            const Pixel p = at(*iso, x, y);
            red = red || (p.r > 200 && p.g < 80);
            green = green || (p.g > 140 && p.r < 60);
            blue = blue || (p.b > 200 && p.r < 60 && p.g < 100);
        }
    }
    EXPECT_TRUE(red && green && blue);
}

TEST(ViewImage, ThePngHoldsThePixels) {
    ViewImage image;
    image.width = 3;
    image.height = 2;
    image.rgb = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18};
    const std::vector<std::uint8_t> png = image.png();
    ASSERT_GT(png.size(), 33u);
    EXPECT_EQ(std::vector<std::uint8_t>(png.begin(), png.begin() + 8),
              (std::vector<std::uint8_t>{0x89, 'P', 'N', 'G', '\r', '\n', 0x1A, '\n'}));
    EXPECT_EQ(png[19], 3);  // IHDR width, big-endian, after length and type
    EXPECT_EQ(png[23], 2);  // height
    EXPECT_EQ(png[24], 8);  // bits
    EXPECT_EQ(png[25], 2);  // RGB
    // IDAT: its length at 33, then its type, then the zlib stream.
    const std::uint32_t length = (static_cast<std::uint32_t>(png[33]) << 24U) |
                                 (static_cast<std::uint32_t>(png[34]) << 16U) |
                                 (static_cast<std::uint32_t>(png[35]) << 8U) | png[36];
    ASSERT_EQ(std::string(png.begin() + 37, png.begin() + 41), "IDAT");
    std::vector<std::uint8_t> raw(2 * (1 + 9));
    uLongf rawSize = static_cast<uLongf>(raw.size());
    ASSERT_EQ(uncompress(raw.data(), &rawSize, png.data() + 41, length), Z_OK);
    EXPECT_EQ(raw, (std::vector<std::uint8_t>{0, 1,  2,  3,  4,  5,  6,  7,  8,  9,
                                              0, 10, 11, 12, 13, 14, 15, 16, 17, 18}));
    EXPECT_EQ(std::string(png.end() - 8, png.end() - 4), "IEND");

    ViewImage wrong;
    wrong.width = 2;
    wrong.height = 2;
    wrong.rgb = {1, 2, 3};
    EXPECT_TRUE(wrong.png().empty());
}

TEST(ViewImage, Base64) {
    const auto b64 = [](const std::string& s) {
        return ai::base64(std::vector<std::uint8_t>(s.begin(), s.end()));
    };
    EXPECT_EQ(b64(""), "");
    EXPECT_EQ(b64("f"), "Zg==");
    EXPECT_EQ(b64("fo"), "Zm8=");
    EXPECT_EQ(b64("foo"), "Zm9v");
    EXPECT_EQ(b64("foobar"), "Zm9vYmFy");
}

TEST(ViewImage, TheToolShowsThePart) {
    Part part;
    ai::registerViewTool(part.tools, part.document);
    const ai::ToolResult empty = part.tools.run("get_view_image", json::object());
    EXPECT_TRUE(empty.isError);
    EXPECT_NE(empty.content.find("no solid"), std::string::npos);

    part.run("add_primitive", {{"shape", "box"}, {"size_x", 10}, {"size_y", 10}, {"size_z", 10}});
    const ai::ToolResult shown =
        part.tools.run("get_view_image", {{"view", "top"}, {"width", 300}});
    ASSERT_FALSE(shown.isError) << shown.content;
    const json text = json::parse(shown.content);
    EXPECT_EQ(text["view"], "top");
    EXPECT_EQ(text["width"], 300);
    EXPECT_EQ(text["height"], 225);
    ASSERT_EQ(shown.images.size(), 1u);
    EXPECT_EQ(shown.images[0].mediaType, "image/png");
    EXPECT_EQ(shown.images[0].base64.rfind("iVBORw0KGgo", 0), 0u);  // a PNG's signature
    EXPECT_EQ(shown.undoSteps, 0);

    EXPECT_TRUE(part.tools.run("get_view_image", {{"width", 100}}).isError);
    EXPECT_TRUE(part.tools.run("get_view_image", {{"view", "under"}}).isError);
}

// Holes cut a flat face into pieces that lie flat together: no line is drawn
// between them (one ran across the plate from hole to hole).
TEST(ViewImage, PiecesOfAFlatFaceShowNoEdge) {
    Part part;
    part.run("add_primitive", {{"shape", "box"}, {"size_x", 60}, {"size_y", 40}, {"size_z", 6}});
    for (const auto& [x, y] : {std::pair{8, 8}, {52, 8}, {8, 32}, {52, 32}}) {
        part.run(
            "add_hole",
            {{"face", "top"}, {"position", {x, y, 6}}, {"diameter", 5}, {"through_all", true}});
    }
    const auto top = ai::renderView(part.meshes(), "top", 600, 400);
    ASSERT_TRUE(top.has_value());
    const auto edgesAlong = [&](bool row, int line) {
        int edges = 0;
        bool inEdge = false;
        for (int i = 0; i < (row ? top->width : top->height); ++i) {
            const bool edge = (row ? at(*top, i, line) : at(*top, line, i)) == kEdge;
            if (edge && !inEdge) ++edges;
            inEdge = edge;
        }
        return edges;
    };
    EXPECT_EQ(edgesAlong(true, 200), 2) << "between the holes, the outline alone";
    EXPECT_EQ(edgesAlong(false, 300), 2);
}
