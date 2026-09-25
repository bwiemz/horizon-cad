// Phase 154: a document's unit is saved with it, and read back; a file from
// before it, or one naming no unit it knows, reads as millimetres.

#include <gtest/gtest.h>

#include <chrono>
#include <filesystem>
#include <memory>
#include <string>

#include "horizon/document/AssemblyDocument.h"
#include "horizon/document/Document.h"
#include "horizon/document/ModelCommands.h"
#include "horizon/document/UndoStack.h"
#include "horizon/fileio/DrawingDocumentIO.h"
#include "horizon/fileio/NativeFormat.h"
#include "horizon/math/Units.h"

namespace fs = std::filesystem;
using hz::io::NativeFormat;
using hz::math::LengthUnit;

namespace {

/// @p json with its "units" object replaced by @p units (removed if empty).
std::string withUnits(std::string json, const std::string& units) {
    const auto at = json.find("\"units\":");
    EXPECT_NE(at, std::string::npos);
    const auto end = json.find('}', at);
    const std::string field = units.empty() ? std::string() : "\"units\":" + units;
    json.replace(at, end + 1 - at, field.empty() ? "\"unused\":0" : field);
    return json;
}

}  // namespace

TEST(DocumentUnitsTest, APartsUnitIsSavedAndReadBack) {
    hz::doc::Document part;
    part.setType(hz::doc::DocumentType::Part);
    part.setLengthUnit(LengthUnit::Inch);
    const std::string json = NativeFormat::documentToJson(part, false);
    hz::doc::Document read;
    ASSERT_TRUE(NativeFormat::documentFromJson(json, read));
    EXPECT_EQ(read.lengthUnit(), LengthUnit::Inch);

    // Before Phase 154, and a unit it does not know: millimetres, whatever
    // the document held before.
    for (const std::string& units : {std::string(), std::string(R"({"length":"furlong"})"),
                                     std::string(R"({"length":7})"), std::string("[]")}) {
        hz::doc::Document old;
        old.setLengthUnit(LengthUnit::Foot);
        ASSERT_TRUE(NativeFormat::documentFromJson(withUnits(json, units), old)) << units;
        EXPECT_EQ(old.lengthUnit(), LengthUnit::Millimetre) << units;
    }
}

TEST(DocumentUnitsTest, AnAssemblysUnitIsSavedAndReadBack) {
    hz::doc::AssemblyDocument assembly;
    assembly.setLengthUnit(LengthUnit::Centimetre);
    const std::string json = NativeFormat::assemblyToJson(assembly, {});
    hz::doc::AssemblyDocument read;
    ASSERT_TRUE(NativeFormat::assemblyFromJson(json, read, {}));
    EXPECT_EQ(read.lengthUnit(), LengthUnit::Centimetre);
    read.setLengthUnit(LengthUnit::Foot);
    ASSERT_TRUE(NativeFormat::assemblyFromJson(withUnits(json, {}), read, {}));
    EXPECT_EQ(read.lengthUnit(), LengthUnit::Millimetre);
}

TEST(DocumentUnitsTest, ASheetsUnitIsSavedAndReadBack) {
    const fs::path dir =
        fs::temp_directory_path() /
        ("hz_units_" + std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()));
    fs::create_directories(dir);
    hz::io::DrawingDocumentSpec spec;
    spec.partPath = (dir / "part.hzpart").string();
    spec.lengthUnit = LengthUnit::Inch;
    const std::string path = (dir / "sheet.hzdwg").string();
    ASSERT_TRUE(hz::io::DrawingDocumentIO::save(path, spec));
    hz::io::DrawingDocumentSpec read;
    std::string error;
    ASSERT_TRUE(hz::io::DrawingDocumentIO::readSpec(path, read, &error)) << error;
    EXPECT_EQ(read.lengthUnit, LengthUnit::Inch);
    fs::remove_all(dir);
}

// A change of unit is one step to undo, and leaves the model alone.
TEST(DocumentUnitsTest, AChangeOfUnitIsUndone) {
    hz::doc::Document backing;
    hz::doc::AssemblyDocument assembly;
    backing.undoStack().push(
        std::make_unique<hz::doc::SetLengthUnitCommand>(backing, LengthUnit::Inch, &assembly));
    EXPECT_EQ(backing.lengthUnit(), LengthUnit::Inch);
    EXPECT_EQ(assembly.lengthUnit(), LengthUnit::Inch);
    EXPECT_TRUE(backing.isDirty());
    backing.undoStack().undo();
    EXPECT_EQ(backing.lengthUnit(), LengthUnit::Millimetre);
    EXPECT_EQ(assembly.lengthUnit(), LengthUnit::Millimetre);
    EXPECT_FALSE(backing.isDirty());
}
