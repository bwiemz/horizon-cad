#include "horizon/fileio/TitleBlockRenderer.h"

#include <algorithm>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "horizon/drafting/DraftLine.h"
#include "horizon/drafting/DraftText.h"
#include "horizon/modeling/Sheet.h"
#include "horizon/modeling/TitleBlock.h"

namespace hz::io {

namespace {

std::shared_ptr<draft::DraftLine> line(const math::Vec2& a, const math::Vec2& b,
                                       const char* layer) {
    auto l = std::make_shared<draft::DraftLine>(a, b);
    l->setLayer(layer);
    return l;
}

std::shared_ptr<draft::DraftText> text(const math::Vec2& pos, const std::string& s, double height,
                                       const char* layer) {
    auto t = std::make_shared<draft::DraftText>(pos, s, height);
    t->setAlignment(draft::TextAlignment::Left);
    t->setLayer(layer);
    return t;
}

void addRect(std::vector<std::shared_ptr<draft::DraftEntity>>& out, double x0, double y0, double x1,
             double y1, const char* layer) {
    out.push_back(line({x0, y0}, {x1, y0}, layer));
    out.push_back(line({x1, y0}, {x1, y1}, layer));
    out.push_back(line({x1, y1}, {x0, y1}, layer));
    out.push_back(line({x0, y1}, {x0, y0}, layer));
}

}  // namespace

std::vector<std::shared_ptr<draft::DraftEntity>> TitleBlockRenderer::renderBorder(
    const model::Sheet& sheet) {
    const double W = sheet.widthMm();
    const double H = sheet.heightMm();
    const double m = sheet.margin;

    std::vector<std::shared_ptr<draft::DraftEntity>> out;
    addRect(out, m, m, W - m, H - m, kBorderLayer);
    return out;
}

std::vector<std::shared_ptr<draft::DraftEntity>> TitleBlockRenderer::renderTitleBlock(
    const model::Sheet& sheet, const model::TitleBlock& block) {
    const double W = sheet.widthMm();
    const double m = sheet.margin;

    // Clamp the panel to the space inside the border.
    const double panelW = std::min(block.width, W - 2.0 * m);
    const double panelH = block.height;

    // Lower-right corner of the panel sits at the inside corner of the border.
    const double x1 = W - m;
    const double x0 = x1 - panelW;
    const double y0 = m;
    const double y1 = y0 + panelH;

    std::vector<std::shared_ptr<draft::DraftEntity>> out;
    addRect(out, x0, y0, x1, y1, kTitleBlockLayer);

    // Split off a title row across the top of the panel.
    const double titleRowH = panelH * 0.4;
    const double divY = y1 - titleRowH;
    out.push_back(line({x0, divY}, {x1, divY}, kTitleBlockLayer));

    // Title centred-ish in the top row (left-anchored with a small inset).
    if (!block.title.empty()) {
        out.push_back(
            text({x0 + 3.0, divY + titleRowH * 0.35}, block.title, 5.0, kTitleBlockLayer));
    }

    // Field lines in the lower area (skip empty values).
    const std::vector<std::pair<std::string, std::string>> fields = {
        {"PART", block.partNumber},   {"REV", block.revision},  {"SCALE", block.scale},
        {"SHEET", block.sheetNumber}, {"DRAWN", block.drawnBy}, {"DATE", block.date},
        {"MATL", block.material},     {"CO", block.company},
    };

    const double rowH = 4.5;
    double ty = divY - rowH;
    for (const auto& [label, value] : fields) {
        if (value.empty()) continue;
        out.push_back(text({x0 + 3.0, ty}, label + ": " + value, 2.5, kTitleBlockLayer));
        ty -= rowH;
        if (ty < y0 + 1.0) break;  // panel full
    }

    return out;
}

}  // namespace hz::io
