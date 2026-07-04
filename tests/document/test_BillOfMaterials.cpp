#include <gtest/gtest.h>

#include "horizon/document/AssemblyDocument.h"
#include "horizon/document/BillOfMaterials.h"

using hz::doc::AssemblyDocument;
using hz::doc::BillOfMaterials;
using hz::doc::BomGenerator;
using hz::doc::ComponentInstance;

namespace {
ComponentInstance part(const std::string& name, const std::string& path) {
    ComponentInstance c;
    c.name = name;
    c.partPath = path;
    return c;
}
}  // namespace

// Repeated instances of the same part collapse into one line whose quantity is
// the occurrence count; distinct parts get distinct, first-appearance-ordered
// item numbers.
TEST(BillOfMaterialsTest, GroupsByPartAndCountsQuantity) {
    AssemblyDocument asmDoc;
    asmDoc.addComponent(part("bolt-1", "hardware/bolt.hzpart"));
    asmDoc.addComponent(part("bracket-1", "bracket.hzpart"));
    asmDoc.addComponent(part("bolt-2", "hardware/bolt.hzpart"));
    asmDoc.addComponent(part("bolt-3", "hardware/bolt.hzpart"));

    BillOfMaterials bom = BomGenerator::generate(asmDoc);

    ASSERT_EQ(bom.lines.size(), 2u);
    // Bolt appears first, so it is item 1 with quantity 3.
    EXPECT_EQ(bom.lines[0].item, 1);
    EXPECT_EQ(bom.lines[0].partName, "bolt");  // file stem, dir + extension stripped
    EXPECT_EQ(bom.lines[0].partPath, "hardware/bolt.hzpart");
    EXPECT_EQ(bom.lines[0].quantity, 3);
    // Bracket is item 2 with quantity 1.
    EXPECT_EQ(bom.lines[1].item, 2);
    EXPECT_EQ(bom.lines[1].partName, "bracket");
    EXPECT_EQ(bom.lines[1].quantity, 1);

    EXPECT_EQ(bom.totalQuantity(), 4);
}

// Suppressed components are excluded from the roll-up.
TEST(BillOfMaterialsTest, ExcludesSuppressedComponents) {
    AssemblyDocument asmDoc;
    asmDoc.addComponent(part("bolt-1", "bolt.hzpart"));
    ComponentInstance suppressed = part("bolt-2", "bolt.hzpart");
    suppressed.suppressed = true;
    asmDoc.addComponent(suppressed);

    BillOfMaterials bom = BomGenerator::generate(asmDoc);
    ASSERT_EQ(bom.lines.size(), 1u);
    EXPECT_EQ(bom.lines[0].quantity, 1);  // suppressed occurrence not counted
    EXPECT_EQ(bom.totalQuantity(), 1);
}

// Parts without a saved file path key on the instance name, so distinct unsaved
// parts stay on separate lines and use the instance name as the display name.
TEST(BillOfMaterialsTest, UnsavedPartsKeyOnName) {
    AssemblyDocument asmDoc;
    asmDoc.addComponent(part("WidgetA", ""));
    asmDoc.addComponent(part("WidgetB", ""));

    BillOfMaterials bom = BomGenerator::generate(asmDoc);
    ASSERT_EQ(bom.lines.size(), 2u);
    EXPECT_EQ(bom.lines[0].partName, "WidgetA");
    EXPECT_EQ(bom.lines[1].partName, "WidgetB");
}

// An empty assembly produces an empty bill of materials.
TEST(BillOfMaterialsTest, EmptyAssemblyIsEmpty) {
    AssemblyDocument asmDoc;
    BillOfMaterials bom = BomGenerator::generate(asmDoc);
    EXPECT_TRUE(bom.lines.empty());
    EXPECT_EQ(bom.totalQuantity(), 0);
}
