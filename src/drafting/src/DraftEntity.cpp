#include "horizon/drafting/DraftEntity.h"

namespace hz::draft {

uint64_t DraftEntity::s_nextId = 1;

DraftEntity::DraftEntity()
    : m_id(s_nextId++)
    , m_layer("0")
    , m_color(0x00000000)
    , m_lineWidth(0.0)
    , m_lineType(0)
    , m_groupId(0) {}

}  // namespace hz::draft
