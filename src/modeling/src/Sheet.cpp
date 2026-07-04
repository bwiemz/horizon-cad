#include "horizon/modeling/Sheet.h"

namespace hz::model {

namespace {

/// Landscape (long edge horizontal) dimensions in millimetres.
void landscapeDimensions(PaperSize size, double& longEdge, double& shortEdge) {
    switch (size) {
        case PaperSize::A0:
            longEdge = 1189.0;
            shortEdge = 841.0;
            return;
        case PaperSize::A1:
            longEdge = 841.0;
            shortEdge = 594.0;
            return;
        case PaperSize::A2:
            longEdge = 594.0;
            shortEdge = 420.0;
            return;
        case PaperSize::A3:
            longEdge = 420.0;
            shortEdge = 297.0;
            return;
        case PaperSize::A4:
            longEdge = 297.0;
            shortEdge = 210.0;
            return;
        case PaperSize::AnsiA:  // 11 x 8.5 in
            longEdge = 279.4;
            shortEdge = 215.9;
            return;
        case PaperSize::AnsiB:  // 17 x 11 in
            longEdge = 431.8;
            shortEdge = 279.4;
            return;
        case PaperSize::AnsiC:  // 22 x 17 in
            longEdge = 558.8;
            shortEdge = 431.8;
            return;
        case PaperSize::AnsiD:  // 34 x 22 in
            longEdge = 863.6;
            shortEdge = 558.8;
            return;
    }
    longEdge = 420.0;
    shortEdge = 297.0;
}

}  // namespace

double Sheet::widthMm() const {
    double longEdge = 0.0, shortEdge = 0.0;
    landscapeDimensions(size, longEdge, shortEdge);
    return orientation == Orientation::Landscape ? longEdge : shortEdge;
}

double Sheet::heightMm() const {
    double longEdge = 0.0, shortEdge = 0.0;
    landscapeDimensions(size, longEdge, shortEdge);
    return orientation == Orientation::Landscape ? shortEdge : longEdge;
}

std::string paperSizeName(PaperSize size) {
    switch (size) {
        case PaperSize::A0:
            return "A0";
        case PaperSize::A1:
            return "A1";
        case PaperSize::A2:
            return "A2";
        case PaperSize::A3:
            return "A3";
        case PaperSize::A4:
            return "A4";
        case PaperSize::AnsiA:
            return "ANSI A";
        case PaperSize::AnsiB:
            return "ANSI B";
        case PaperSize::AnsiC:
            return "ANSI C";
        case PaperSize::AnsiD:
            return "ANSI D";
    }
    return "?";
}

}  // namespace hz::model
