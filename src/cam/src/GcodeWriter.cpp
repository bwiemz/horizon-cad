#include "horizon/cam/GcodeWriter.h"

#include <cstdio>
#include <sstream>

#include "horizon/cam/Toolpath.h"

namespace hz::cam {

namespace {

std::string fmt(double value, int decimals) {
    char buf[64];
    std::snprintf(buf, sizeof(buf), "%.*f", decimals, value);
    return buf;
}

}  // namespace

std::string GcodeWriter::toGcode(const Toolpath& path, int decimals) {
    std::ostringstream out;
    out << "G21\n";  // millimetres
    out << "G90\n";  // absolute coordinates

    double lastFeed = -1.0;
    for (const Move& m : path.moves) {
        if (m.type == MoveType::Rapid) {
            out << "G0";
        } else {
            out << "G1";
        }
        out << " X" << fmt(m.target.x, decimals) << " Y" << fmt(m.target.y, decimals) << " Z"
            << fmt(m.target.z, decimals);
        // Emit the feed word only when it changes (RS-274 modal F).
        if (m.type == MoveType::Feed && m.feed != lastFeed) {
            out << " F" << fmt(m.feed, decimals);
            lastFeed = m.feed;
        }
        out << "\n";
    }

    out << "M2\n";  // end of program
    return out.str();
}

}  // namespace hz::cam
