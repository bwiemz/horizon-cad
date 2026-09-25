#pragma once

#include <string>
#include <vector>

namespace hz::model {

/// One part in a parts list: its item number (as its balloons show it), its
/// name and how many the assembly has.
struct PartsListRow {
    int item = 0;
    std::string name;
    int quantity = 0;
};

/// A parts list on an assembly drawing (Phase 150): the bill of materials on
/// the sheet, just above the title block and as wide as it, its header next
/// to the title block and the items numbered upward from it (ISO 7573).
struct PartsList {
    std::vector<PartsListRow> rows;

    static constexpr double kRowHeight = 7.0;       ///< mm on the sheet
    static constexpr double kItemWidth = 15.0;      ///< the ITEM column, mm
    static constexpr double kQuantityWidth = 15.0;  ///< the QTY column, mm

    /// Its height on the sheet: the header, and a row for each part.
    double height() const { return kRowHeight * static_cast<double>(rows.size() + 1); }
};

}  // namespace hz::model
