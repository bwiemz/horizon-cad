#include "horizon/cam/GcodeWriter.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <limits>
#include <sstream>

#include "horizon/cam/Toolpath.h"

namespace hz::cam {

namespace {

std::string fmt(double value, int decimals) {
    char buf[64];
    std::snprintf(buf, sizeof(buf), "%.*f", decimals, value);
    return buf;
}

bool finite(const math::Vec3& p) {
    return std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z);
}

}  // namespace

std::string GcodeWriter::validate(const Toolpath& path, const GcodeOptions& options) {
    if (options.toolNumber < 1 || options.toolNumber > 9999) {
        return "the tool number must be 1 to 9999";
    }
    if (!std::isfinite(options.spindleRpm) || options.spindleRpm <= 0.0) {
        return "the spindle speed must be positive";
    }
    if (options.decimals < 0 || options.decimals > 6) return "decimals must be 0 to 6";
    if (path.moves.empty()) return "the toolpath is empty";
    if (path.moves.front().type != MoveType::Rapid) {
        return "the toolpath must start with a rapid to a safe height";
    }

    double highestCut = -std::numeric_limits<double>::infinity();
    for (const Move& m : path.moves) {
        if (!finite(m.target)) return "a move's position is not a finite number";
        if (m.type == MoveType::Feed) {
            if (!std::isfinite(m.feed) || m.feed <= 0.0)
                return "a cutting move's feed is not positive";
            highestCut = std::max(highestCut, m.target.z);
        }
    }
    for (const Move& m : path.moves) {
        if (m.type == MoveType::Rapid && m.target.z <= highestCut) {
            return "a rapid at Z" + fmt(m.target.z, options.decimals) +
                   " is not above the highest cut, Z" + fmt(highestCut, options.decimals) +
                   ", so it could move through material";
        }
    }
    return {};
}

std::optional<std::string> GcodeWriter::toGcode(const Toolpath& path, const GcodeOptions& options,
                                                std::string* error) {
    const std::string problem = validate(path, options);
    if (!problem.empty()) {
        if (error) *error = problem;
        return std::nullopt;
    }
    const int d = options.decimals;

    std::ostringstream out;
    out << "G21 G90 G94 G17 G40 G49 G80\n";  // a known modal state
    out << "T" << options.toolNumber << " M6\n";
    out << "S" << fmt(options.spindleRpm, 0) << " M3\n";
    if (options.coolant) out << "M8\n";

    // The first rapid: from wherever the tool is, climb (applying the tool's
    // length offset) before moving across.
    const Move& first = path.moves.front();
    out << "G0 G43 H" << options.toolNumber << " Z" << fmt(first.target.z, d) << "\n";
    out << "G0 X" << fmt(first.target.x, d) << " Y" << fmt(first.target.y, d) << "\n";
    math::Vec3 at = first.target;

    double lastFeed = -1.0;
    for (std::size_t i = 1; i < path.moves.size(); ++i) {
        const Move& m = path.moves[i];
        const std::string x = " X" + fmt(m.target.x, d);
        const std::string y = " Y" + fmt(m.target.y, d);
        const std::string z = " Z" + fmt(m.target.z, d);
        if (m.type == MoveType::Rapid) {
            const bool movesAcross = m.target.x != at.x || m.target.y != at.y;
            if (!movesAcross) {
                out << "G0" << z << "\n";
            } else if (m.target.z >= at.z) {
                // Climb, then cross.
                if (m.target.z != at.z) out << "G0" << z << "\n";
                out << "G0" << x << y << "\n";
            } else {
                // Cross, then descend.
                out << "G0" << x << y << "\n";
                out << "G0" << z << "\n";
            }
        } else {
            out << "G1" << x << y << z;
            // The feed word only when it changes (RS-274 modal F).
            if (m.feed != lastFeed) {
                out << " F" << fmt(m.feed, d);
                lastFeed = m.feed;
            }
            out << "\n";
        }
        at = m.target;
    }

    if (options.coolant) out << "M9\n";
    out << "M5\n";   // spindle stop
    out << "M30\n";  // end of program, rewind
    return out.str();
}

}  // namespace hz::cam
