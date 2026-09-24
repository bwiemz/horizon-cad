// Loaders report *why* a file could not be opened, and never throw.

#include <gtest/gtest.h>

#include <filesystem>
#include <fstream>
#include <random>
#include <string>

#include "horizon/document/AssemblyDocument.h"
#include "horizon/document/Document.h"
#include "horizon/fileio/DxfFormat.h"
#include "horizon/fileio/NativeFormat.h"

namespace fs = std::filesystem;
using hz::io::DxfFormat;
using hz::io::NativeFormat;

namespace {

class TempFile {
public:
    explicit TempFile(const std::string& extension) {
        std::random_device rd;
        m_path = fs::temp_directory_path() / ("hz_loaderr_" + std::to_string(rd()) + extension);
    }
    ~TempFile() {
        std::error_code ec;
        fs::remove_all(m_path, ec);
    }
    void write(const std::string& text) const {
        std::ofstream out(m_path, std::ios::binary);
        out << text;
    }
    std::string path() const { return m_path.string(); }

private:
    fs::path m_path;
};

bool contains(const std::string& haystack, const std::string& needle) {
    return haystack.find(needle) != std::string::npos;
}

}  // namespace

TEST(LoadErrorsTest, MissingFileSaysSo) {
    TempFile file(".hcad");
    hz::doc::Document doc;
    std::string error;
    EXPECT_FALSE(NativeFormat::load(file.path(), doc, &error));
    EXPECT_TRUE(contains(error, "does not exist")) << error;
}

TEST(LoadErrorsTest, FolderIsNotAFile) {
    TempFile dir(".hcad");
    fs::create_directory(dir.path());
    hz::doc::Document doc;
    std::string error;
    EXPECT_FALSE(NativeFormat::load(dir.path(), doc, &error));
    EXPECT_TRUE(contains(error, "folder")) << error;
}

TEST(LoadErrorsTest, InvalidJsonNamesTheLineAndColumn) {
    TempFile file(".hcad");
    file.write("{\n  \"version\": 16,\n  \"entities\": [ oops ]\n}\n");
    hz::doc::Document doc;
    std::string error;
    EXPECT_FALSE(NativeFormat::load(file.path(), doc, &error));
    EXPECT_TRUE(contains(error, "line 3")) << error;
    EXPECT_FALSE(contains(error, "[json.exception")) << "internal tag stripped: " << error;
}

TEST(LoadErrorsTest, ForeignJsonIsNotAHorizonDocument) {
    TempFile file(".hcad");
    file.write("{\"name\": \"package\", \"dependencies\": {}}");
    hz::doc::Document doc;
    std::string error;
    EXPECT_FALSE(NativeFormat::load(file.path(), doc, &error));
    EXPECT_TRUE(contains(error, "not a Horizon document")) << error;
}

TEST(LoadErrorsTest, WrongTypeInsideTheDocumentIsAFailureNotACrash) {
    // A number where the current layer's name belongs used to throw
    // json::type_error out of load() — and from there out of the Qt event loop.
    TempFile file(".hcad");
    file.write(R"({"version":16,"entities":[],"layers":[],"currentLayer":7})");
    hz::doc::Document doc;
    std::string error;
    bool loaded = true;
    ASSERT_NO_THROW(loaded = NativeFormat::load(file.path(), doc, &error));
    EXPECT_FALSE(loaded);
    EXPECT_TRUE(contains(error, "damaged")) << error;
}

TEST(LoadErrorsTest, InMemoryEnvelopeReportsToo) {
    hz::doc::Document doc;
    std::string error;
    EXPECT_FALSE(NativeFormat::documentFromJson("not json at all", doc, &error));
    EXPECT_FALSE(error.empty());
}

TEST(LoadErrorsTest, AssemblyOfTheWrongTypeSaysSo) {
    TempFile file(".hzasm");
    file.write(R"({"version":16,"type":"hcad","entities":[]})");
    hz::doc::AssemblyDocument asmDoc;
    std::string error;
    EXPECT_FALSE(NativeFormat::loadAssembly(file.path(), asmDoc, &error));
    EXPECT_TRUE(contains(error, "not a Horizon assembly")) << error;
}

TEST(LoadErrorsTest, ErrorIsOptional) {
    TempFile file(".hcad");
    hz::doc::Document doc;
    EXPECT_FALSE(NativeFormat::load(file.path(), doc));  // no out-parameter: still just false
}

TEST(LoadErrorsTest, DxfMissingFileSaysSo) {
    TempFile file(".dxf");
    hz::doc::Document doc;
    std::string error;
    EXPECT_FALSE(DxfFormat::load(file.path(), doc, &error));
    EXPECT_TRUE(contains(error, "does not exist")) << error;
}

TEST(LoadErrorsTest, DxfWithoutSectionsIsNotADxf) {
    TempFile file(".dxf");
    file.write("this is a text file\nwith two lines\n");
    hz::doc::Document doc;
    std::string error;
    EXPECT_FALSE(DxfFormat::load(file.path(), doc, &error));
    EXPECT_TRUE(contains(error, "not a DXF file")) << error;
}

TEST(LoadErrorsTest, SaveReportsWhyItFailed) {
    TempFile dir(".hcad");
    fs::create_directory(dir.path());  // a folder where the file should go
    hz::doc::Document doc;
    std::string error;
    EXPECT_FALSE(NativeFormat::save(dir.path(), doc, &error));
    EXPECT_FALSE(error.empty());
    EXPECT_FALSE(DxfFormat::save(dir.path(), doc, &error));
    EXPECT_FALSE(error.empty());
}
