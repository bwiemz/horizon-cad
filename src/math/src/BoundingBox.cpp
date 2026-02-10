#include "horizon/math/BoundingBox.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace hz::math {

BoundingBox::BoundingBox()
    : m_min({std::numeric_limits<double>::max(), std::numeric_limits<double>::max(),
             std::numeric_limits<double>::max()}),
      m_max({-std::numeric_limits<double>::max(), -std::numeric_limits<double>::max(),
             -std::numeric_limits<double>::max()}),
      m_valid(false) {}

BoundingBox::BoundingBox(const Vec3& min, const Vec3& max) : m_min(min), m_max(max), m_valid(true) {}

void BoundingBox::expand(const Vec3& point) {
    m_min.x = std::min(m_min.x, point.x);
    m_min.y = std::min(m_min.y, point.y);
    m_min.z = std::min(m_min.z, point.z);
    m_max.x = std::max(m_max.x, point.x);
    m_max.y = std::max(m_max.y, point.y);
    m_max.z = std::max(m_max.z, point.z);
    m_valid = true;
}

void BoundingBox::expand(const BoundingBox& other) {
    if (!other.m_valid) return;
    expand(other.m_min);
    expand(other.m_max);
}

void BoundingBox::reset() {
    m_min = {std::numeric_limits<double>::max(), std::numeric_limits<double>::max(),
             std::numeric_limits<double>::max()};
    m_max = {-std::numeric_limits<double>::max(), -std::numeric_limits<double>::max(),
             -std::numeric_limits<double>::max()};
    m_valid = false;
}

bool BoundingBox::contains(const Vec3& point) const {
    if (!m_valid) return false;
    return point.x >= m_min.x && point.x <= m_max.x && point.y >= m_min.y &&
           point.y <= m_max.y && point.z >= m_min.z && point.z <= m_max.z;
}

bool BoundingBox::intersects(const BoundingBox& other) const {
    if (!m_valid || !other.m_valid) return false;
    return m_min.x <= other.m_max.x && m_max.x >= other.m_min.x &&
           m_min.y <= other.m_max.y && m_max.y >= other.m_min.y &&
           m_min.z <= other.m_max.z && m_max.z >= other.m_min.z;
}

bool BoundingBox::isValid() const { return m_valid; }

Vec3 BoundingBox::center() const {
    return {(m_min.x + m_max.x) / 2.0, (m_min.y + m_max.y) / 2.0, (m_min.z + m_max.z) / 2.0};
}

Vec3 BoundingBox::size() const {
    return {m_max.x - m_min.x, m_max.y - m_min.y, m_max.z - m_min.z};
}

double BoundingBox::diagonal() const { return size().length(); }

}  // namespace hz::math
