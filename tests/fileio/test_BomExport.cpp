#include <gtest/gtest.h>

#include <cstdio>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <string>

#include "horizon/document/AssemblyDocument.h"
#include "horizon/document/BillOfMaterials.h"
#include "horizon/fileio/BomExport.h"

using hz::doc::AssemblyDocument;
using hz::doc::BillOfMaterials;
using hz::doc::BomGenerator;
using hz::doc::ComponentInstance;
using hz::io::BomExport;

namespace {
std::string tempPath(const std::string& name) {
    return (std::filesystem::temp_directory_path() / name).string();
}

std::string readFile(const std::string& path) {
    std::ifstream in(path, std::ios::binary);
    std::ostringstream ss;
    ss << in.rdbuf();
    return ss.str();
}

ComponentInstance part(const std::string& name, const std::string& path) {
    ComponentInstance c;
    c.name = name;
    c.partPath = path;
    return c;
}
}  // namespace

// A BOM exports to CSV with a header and one row per line.
TEST(BomExportTest, WritesHeaderAndRows) {
    AssemblyDocument asmDoc;
    asmDoc.addComponent(part("bolt-1", "bolt.hzpart"));
    asmDoc.addComponent(part("bolt-2", "bolt.hzpart"));
    asmDoc.addComponent(part("bracket-1", "bracket.hzpart"));
    BillOfMaterials bom = BomGenerator::generate(asmDoc);

    const std::string path = tempPath("hz_test_bom.csv");
    ASSERT_TRUE(BomExport::toCsv(path, bom));

    const std::string content = readFile(path);
    EXPECT_NE(content.find("Item,Part,Quantity,Path\r\n"), std::string::npos);
    EXPECT_NE(content.find("1,bolt,2,bolt.hzpart\r\n"), std::string::npos);
    EXPECT_NE(content.find("2,bracket,1,bracket.hzpart\r\n"), std::string::npos);

    std::remove(path.c_str());
}

// Fields containing a comma are quoted per RFC 4180 so the CSV round-trips.
TEST(BomExportTest, QuotesFieldsWithCommas) {
    BillOfMaterials bom;
    hz::doc::BomLine line;
    line.item = 1;
    line.partName = "Bracket, Left";  // contains a comma
    line.partPath = "parts/bracket.hzpart";
    line.quantity = 2;
    bom.lines.push_back(line);

    const std::string path = tempPath("hz_test_bom_quoted.csv");
    ASSERT_TRUE(BomExport::toCsv(path, bom));

    const std::string content = readFile(path);
    EXPECT_NE(content.find("1,\"Bracket, Left\",2,parts/bracket.hzpart\r\n"), std::string::npos);

    std::remove(path.c_str());
}
