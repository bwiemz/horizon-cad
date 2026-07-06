#include "horizon/cam/Toolpath.h"

#include <cmath>

namespace hz::cam {

namespace {

double distance(const math::Vec3& a, const math::Vec3& b) {
    const double dx = a.x - b.x, dy = a.y - b.y, dz = a.z - b.z;
    return std::sqrt(dx * dx + dy * dy + dz * dz);
}

double lengthOf(const Toolpath& path, MoveType type) {
    double total = 0.0;
    for (std::size_t i = 1; i < path.moves.size(); ++i) {
        if (path.moves[i].type == type) {
            total += distance(path.moves[i].target, path.moves[i - 1].target);
        }
    }
    return total;
}

}  // namespace

double Toolpath::cuttingLength() const {
    return lengthOf(*this, MoveType::Feed);
}

double Toolpath::rapidLength() const {
    return lengthOf(*this, MoveType::Rapid);
}

Toolpath CamGenerator::contour(const std::vector<math::Vec2>& profile, double cutDepth,
                               double safeZ, double feed, bool closed) {
    Toolpath path;
    if (profile.empty()) return path;

    const math::Vec2& p0 = profile.front();
    path.moves.push_back({MoveType::Rapid, {p0.x, p0.y, safeZ}, 0.0});
    path.moves.push_back({MoveType::Feed, {p0.x, p0.y, cutDepth}, feed});  // plunge
    for (std::size_t i = 1; i < profile.size(); ++i) {
        path.moves.push_back({MoveType::Feed, {profile[i].x, profile[i].y, cutDepth}, feed});
    }
    if (closed && profile.size() > 1) {
        path.moves.push_back({MoveType::Feed, {p0.x, p0.y, cutDepth}, feed});
    }
    const math::Vec2& last = closed ? p0 : profile.back();
    path.moves.push_back({MoveType::Rapid, {last.x, last.y, safeZ}, 0.0});  // retract
    return path;
}

Toolpath CamGenerator::drill(const std::vector<math::Vec2>& holes, double cutDepth, double safeZ,
                             double feed) {
    Toolpath path;
    for (const math::Vec2& h : holes) {
        path.moves.push_back({MoveType::Rapid, {h.x, h.y, safeZ}, 0.0});
        path.moves.push_back({MoveType::Feed, {h.x, h.y, cutDepth}, feed});  // plunge
        path.moves.push_back({MoveType::Rapid, {h.x, h.y, safeZ}, 0.0});     // retract
    }
    return path;
}

}  // namespace hz::cam
