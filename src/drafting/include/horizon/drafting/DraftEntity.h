#pragma once

#include "horizon/math/Vec2.h"
#include "horizon/math/BoundingBox.h"
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace hz::draft {

class DraftEntity {
public:
    DraftEntity();
    virtual ~DraftEntity() = default;

    uint64_t id() const { return m_id; }

    const std::string& layer() const { return m_layer; }
    void setLayer(const std::string& layer) { m_layer = layer; }

    uint32_t color() const { return m_color; }
    void setColor(uint32_t argb) { m_color = argb; }

    virtual math::BoundingBox boundingBox() const = 0;
    virtual bool hitTest(const math::Vec2& point, double tolerance) const = 0;
    virtual std::vector<math::Vec2> snapPoints() const = 0;
    virtual void translate(const math::Vec2& delta) = 0;
    virtual std::shared_ptr<DraftEntity> clone() const = 0;
    virtual void mirror(const math::Vec2& axisP1, const math::Vec2& axisP2) = 0;
    virtual void rotate(const math::Vec2& center, double angle) = 0;
    virtual void scale(const math::Vec2& center, double factor) = 0;

private:
    uint64_t m_id;
    std::string m_layer;
    uint32_t m_color;

    static uint64_t s_nextId;
};

}  // namespace hz::draft
