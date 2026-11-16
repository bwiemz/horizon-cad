#pragma once

#include <cstdint>
#include <memory>
#include <vector>

#include "horizon/drafting/DraftEntity.h"

namespace hz::draft {

/// The entities of @p entities that repeat one before them (AutoCAD's
/// OVERKILL, "delete duplicate objects"): the same kind, on the same layer,
/// in the same colour, width and line type, and the same shape to within
/// @p tolerance, whichever way round it was drawn (a line from either end,
/// an open polyline either way). The first of each kept, their ids are
/// returned in drawing order. Lines, circles, arcs, polylines, rectangles,
/// ellipses, splines and texts are compared; anything else (a dimension, a
/// hatch, a block reference) is never taken for a copy.
std::vector<uint64_t> duplicateEntities(const std::vector<std::shared_ptr<DraftEntity>>& entities,
                                        double tolerance);

}  // namespace hz::draft
