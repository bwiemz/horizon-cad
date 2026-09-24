#include "horizon/drafting/DraftEntity.h"

namespace hz::draft {

math::IdCounter<uint64_t> DraftEntity::s_nextId{1};

DraftEntity::DraftEntity()
    : m_id(s_nextId.next()),
      m_layer("0"),
      m_color(0x00000000),
      m_lineWidth(0.0),
      m_lineType(0),
      m_groupId(0) {}

std::vector<SnapPoint> DraftEntity::typedSnapPoints() const {
    const auto points = snapPoints();
    std::vector<SnapPoint> out;
    out.reserve(points.size());
    for (const auto& p : points) out.push_back({p, SnapType::Endpoint});
    return out;
}

}  // namespace hz::draft
