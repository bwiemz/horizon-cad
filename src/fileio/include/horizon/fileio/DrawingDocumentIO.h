#pragma once

#include <string>

#include "horizon/modeling/DrawingView.h"

namespace hz::io {

/// A drawing document's persisted specification: which part it draws and how its
/// views are spaced. The drawing geometry itself is *not* stored — it is
/// regenerated from the referenced part on load, so the drawing stays associated
/// with (and updates from) the 3D model.
struct DrawingDocumentSpec {
    std::string partPath;  ///< path to the source .hzpart
    double gap = 10.0;     ///< inter-view spacing on the sheet
};

/// Reads/writes `.hzdwg` drawing documents.
///
/// A `.hzdwg` is a small JSON file that references a part file. Loading it opens
/// and rebuilds that part, then regenerates the standard four-view drawing — so
/// the drawing reflects the current state of the model rather than a stale
/// snapshot.
class DrawingDocumentIO {
public:
    /// Write a `.hzdwg` describing @p spec. Returns false on I/O failure.
    static bool save(const std::string& path, const DrawingDocumentSpec& spec);

    /// Load a `.hzdwg`: read the spec, open and rebuild the referenced part, and
    /// regenerate its standard four-view drawing into @p outDrawing. @p outSpec
    /// receives the parsed spec. Returns false on I/O failure, a missing/broken
    /// part reference, or a rebuild that produces no solid.
    static bool load(const std::string& path, DrawingDocumentSpec& outSpec,
                     model::Drawing& outDrawing);
};

}  // namespace hz::io
