#include "horizon/geometry/curves/Curve2D.h"
#include "horizon/math/Constants.h"
#include "horizon/math/MathUtils.h"
#include <cmath>
#include <limits>

namespace hz::geo {

double Curve2D::length() const {
    // Numerical integration using the composite Simpson's rule
    const int n = 128;
    double tA = tMin();
    double tB = tMax();
    double h = (tB - tA) / n;
    double sum = derivative(tA).length() + derivative(tB).length();

    for (int i = 1; i < n; i += 2) {
        double t = tA + i * h;
        sum += 4.0 * derivative(t).length();
    }
    for (int i = 2; i < n - 1; i += 2) {
        double t = tA + i * h;
        sum += 2.0 * derivative(t).length();
    }

    return sum * h / 3.0;
}

double Curve2D::project(const math::Vec2& point) const {
    // Brute-force search along the curve for the closest parameter
    const int samples = 256;
    double tA = tMin();
    double tB = tMax();
    double step = (tB - tA) / samples;
    double bestT = tA;
    double bestDistSq = std::numeric_limits<double>::max();

    for (int i = 0; i <= samples; ++i) {
        double t = tA + i * step;
        math::Vec2 p = evaluate(t);
        double dSq = (p - point).lengthSquared();
        if (dSq < bestDistSq) {
            bestDistSq = dSq;
            bestT = t;
        }
    }

    // Refine with a few Newton iterations
    for (int iter = 0; iter < 5; ++iter) {
        math::Vec2 c = evaluate(bestT);
        math::Vec2 d = derivative(bestT);
        math::Vec2 diff = c - point;
        double num = diff.dot(d);
        double den = d.dot(d);
        if (std::abs(den) < math::kEpsilon) break;
        double newT = bestT - num / den;
        bestT = math::clamp(newT, tA, tB);
    }

    return bestT;
}

std::vector<math::Vec2> Curve2D::tessellate(int segments) const {
    std::vector<math::Vec2> points;
    points.reserve(segments + 1);
    double tA = tMin();
    double tB = tMax();
    for (int i = 0; i <= segments; ++i) {
        double t = tA + (tB - tA) * static_cast<double>(i) / segments;
        points.push_back(evaluate(t));
    }
    return points;
}

}  // namespace hz::geo
