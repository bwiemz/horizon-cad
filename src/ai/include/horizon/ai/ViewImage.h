#pragma once

#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "horizon/ai/ToolRegistry.h"
#include "horizon/math/Mat4.h"

namespace hz::geo {
struct MeshData;
}

namespace hz::doc {
class AssemblyDocument;
class Document;
}  // namespace hz::doc

namespace hz::ai {

/// A mesh placed in the scene a view shows.
struct ViewMesh {
    std::shared_ptr<const geo::MeshData> mesh;
    math::Mat4 transform = math::Mat4::identity();
};

/// The views a model may ask for: iso (from the front, right and top at
/// once), front (looking along +Y), back, left, right, top (looking down -Z)
/// and bottom.
const std::vector<std::string>& viewNames();

/// A view drawn: RGB pixels, row by row from the top, 3 bytes each.
struct ViewImage {
    int width = 0;
    int height = 0;
    std::vector<std::uint8_t> rgb;

    /// The image as a PNG file's bytes.
    std::vector<std::uint8_t> png() const;
};

/// Draw @p meshes as seen from @p view (one of viewNames()), in parallel
/// projection, fitted to a @p width by @p height image with a margin: their
/// faces shaded (a curved face's facets as one), the edges between faces and
/// the outline drawn dark, and the world axes marked in the lower left (X
/// red, Y green, Z blue). Software only, so the same scene always gives the
/// same image, with no GPU and no window. Nullopt when there is nothing to
/// draw or @p view is not a view.
std::optional<ViewImage> renderView(const std::vector<ViewMesh>& meshes, const std::string& view,
                                    int width, int height);

/// @p bytes in base64 (RFC 4648, with padding).
std::string base64(const std::vector<std::uint8_t>& bytes);

/// Add get_view_image, which shows a model the part of @p document, to
/// @p registry. Offered only to a model that can see images: a host adds it
/// when the user's settings say so. @p document must outlive the registry's
/// use.
void registerViewTool(ToolRegistry& registry, const doc::Document& document);
/// The same for an assembly: its unsuppressed components where they are
/// shown (an exploded view as it is drawn).
void registerViewTool(ToolRegistry& registry, const doc::AssemblyDocument& assembly);

}  // namespace hz::ai
