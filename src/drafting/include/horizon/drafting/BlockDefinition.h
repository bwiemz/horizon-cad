#pragma once

#include "DraftEntity.h"
#include "horizon/math/Vec2.h"
#include <memory>
#include <string>
#include <vector>

namespace hz::draft {

/// A named block definition: a base point and a set of entity templates.
struct BlockDefinition {
    std::string name;
    math::Vec2 basePoint;
    std::vector<std::shared_ptr<DraftEntity>> entities;
};

}  // namespace hz::draft
