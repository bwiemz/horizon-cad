#pragma once

#include <string>

namespace hz::model {

/// A drawing title block: the parametric information panel placed in the
/// lower-right corner of a sheet. Fields are free text so any drafting standard
/// (ISO, ANSI, DIN) can populate them; empty fields are simply left blank.
struct TitleBlock {
    std::string title;              ///< drawing / part title
    std::string partNumber;         ///< part or document number
    std::string revision = "A";     ///< revision letter
    std::string drawnBy;            ///< author initials/name
    std::string date;               ///< date drawn (free-form)
    std::string scale = "1:1";      ///< drawing scale
    std::string sheetNumber = "1";  ///< sheet index, e.g. "1" or "1/3"
    std::string material;           ///< material spec
    std::string company;            ///< organisation name

    double width = 180.0;  ///< title-block panel width, in mm
    double height = 40.0;  ///< title-block panel height, in mm
};

}  // namespace hz::model
