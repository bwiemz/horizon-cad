#pragma once

#include <memory>
#include <vector>

namespace hz::draft {
class DraftEntity;
}  // namespace hz::draft

namespace hz::model {
struct Sheet;
struct TitleBlock;
}  // namespace hz::model

namespace hz::io {

/// Renders a drawing sheet frame and its title block as draftable entities.
///
/// Produces the sheet border rectangle (inset by the sheet margin) on a "Border"
/// layer, and the title-block panel — outer box, dividers, and populated field
/// text — in the lower-right corner on a "TitleBlock" layer. Entities carry their
/// layer, so the caller only needs to add them to the document.
class TitleBlockRenderer {
public:
    static constexpr const char* kBorderLayer = "Border";
    static constexpr const char* kTitleBlockLayer = "TitleBlock";

    /// The sheet border for @p sheet (a rectangle inset by the sheet margin).
    static std::vector<std::shared_ptr<draft::DraftEntity>> renderBorder(const model::Sheet& sheet);

    /// The title-block panel for @p block, placed in the lower-right corner of
    /// @p sheet inside the border. Empty fields are skipped.
    static std::vector<std::shared_ptr<draft::DraftEntity>> renderTitleBlock(
        const model::Sheet& sheet, const model::TitleBlock& block);
};

}  // namespace hz::io
