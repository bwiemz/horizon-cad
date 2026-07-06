#include <gtest/gtest.h>

#include <QCoreApplication>
#include <QDir>
#include <QFile>
#include <QTemporaryDir>
#include <algorithm>
#include <utility>
#include <vector>

#include "horizon/ui/LocaleManager.h"

using hz::ui::LocaleManager;

namespace {

/// One QCoreApplication for the whole binary (headless-safe: no GUI).
QCoreApplication* app() {
    static int argc = 1;
    static char arg0[] = "hz_ui_tests";
    static char* argv[] = {arg0, nullptr};
    static QCoreApplication instance(argc, argv);
    return &instance;
}

void putU32(QByteArray& out, quint32 v) {
    out.append(char(v >> 24));
    out.append(char((v >> 16) & 0xff));
    out.append(char((v >> 8) & 0xff));
    out.append(char(v & 0xff));
}

/// ELF hash as used by Qt's .qm reader/writer (hash of source + comment).
quint32 elfHash(const QByteArray& bytes) {
    quint32 h = 0;
    for (const char ch : bytes) {
        h = (h << 4) + static_cast<unsigned char>(ch);
        const quint32 g = h & 0xf0000000;
        if (g) h ^= g >> 24;
        h &= ~g;
    }
    return h ? h : 1;
}

/// Emit a minimal but valid Qt .qm catalog: Hashes + Messages blocks, each
/// message carrying Translation (UTF-16BE), SourceText, Context, End tags.
/// This is the documented stable format lrelease produces — building it here
/// lets the tests exercise QTranslator's real load path without qttools.
QByteArray makeQm(const QByteArray& context,
                  const std::vector<std::pair<QByteArray, QString>>& messages) {
    QByteArray msgBlock;
    std::vector<std::pair<quint32, quint32>> hashes;
    for (const auto& [source, translation] : messages) {
        hashes.emplace_back(elfHash(source), static_cast<quint32>(msgBlock.size()));
        msgBlock.append(char(0x03));  // Tag_Translation
        putU32(msgBlock, static_cast<quint32>(translation.size()) * 2);
        for (const QChar ch : translation) {
            msgBlock.append(char(ch.unicode() >> 8));
            msgBlock.append(char(ch.unicode() & 0xff));
        }
        msgBlock.append(char(0x06));  // Tag_SourceText
        putU32(msgBlock, static_cast<quint32>(source.size()));
        msgBlock.append(source);
        msgBlock.append(char(0x07));  // Tag_Context
        putU32(msgBlock, static_cast<quint32>(context.size()));
        msgBlock.append(context);
        msgBlock.append(char(0x01));  // Tag_End
    }
    std::sort(hashes.begin(), hashes.end());

    QByteArray hashBlock;
    for (const auto& [hash, offset] : hashes) {
        putU32(hashBlock, hash);
        putU32(hashBlock, offset);
    }

    static const unsigned char kMagic[16] = {0x3c, 0xb8, 0x64, 0x18, 0xca, 0xef, 0x9c, 0x95,
                                             0xcd, 0x21, 0x1c, 0xbf, 0x60, 0xa1, 0xbd, 0xdd};
    QByteArray out(reinterpret_cast<const char*>(kMagic), 16);
    out.append(char(0x42));  // Hashes
    putU32(out, static_cast<quint32>(hashBlock.size()));
    out.append(hashBlock);
    out.append(char(0x69));  // Messages
    putU32(out, static_cast<quint32>(msgBlock.size()));
    out.append(msgBlock);
    return out;
}

void writeFile(const QString& path, const QByteArray& bytes) {
    QFile f(path);
    ASSERT_TRUE(f.open(QIODevice::WriteOnly));
    f.write(bytes);
}

QByteArray germanCatalog() {
    return makeQm("hz::ui::MainWindow",
                  {{"&File", QString::fromUtf8("&Datei")},
                   {"%1 selected", QString::fromUtf8("%1 ausgew\xC3\xA4hlt")}});
}

QString translated(const char* source) {
    return QCoreApplication::translate("hz::ui::MainWindow", source);
}

}  // namespace

TEST(LocaleManagerTest, ResolvePrefersExactThenLanguageFallback) {
    QTemporaryDir tmp;
    writeFile(tmp.filePath("horizon_de.qm"), "stub");
    writeFile(tmp.filePath("horizon_de_AT.qm"), "stub");

    const QString exact = LocaleManager::resolveTranslationFile(tmp.path(), "de_AT");
    EXPECT_TRUE(exact.endsWith("horizon_de_AT.qm"));
    const QString fallback = LocaleManager::resolveTranslationFile(tmp.path(), "de_DE");
    EXPECT_TRUE(fallback.endsWith("horizon_de.qm"));
    // BCP-47 hyphens behave like underscores.
    const QString hyphen = LocaleManager::resolveTranslationFile(tmp.path(), "de-DE");
    EXPECT_TRUE(hyphen.endsWith("horizon_de.qm"));
    EXPECT_TRUE(LocaleManager::resolveTranslationFile(tmp.path(), "fr").isEmpty());
}

TEST(LocaleManagerTest, AvailableLocalesListsCatalogs) {
    QTemporaryDir tmp;
    writeFile(tmp.filePath("horizon_de.qm"), "stub");
    writeFile(tmp.filePath("horizon_ja.qm"), "stub");
    writeFile(tmp.filePath("unrelated.qm"), "stub");
    writeFile(tmp.filePath("horizon_fr.txt"), "stub");

    const QStringList locales = LocaleManager::availableLocales(tmp.path());
    EXPECT_EQ(locales, QStringList({"de", "ja"}));
}

TEST(LocaleManagerTest, ApplyInstallsARealCatalog) {
    app();
    QTemporaryDir tmp;
    writeFile(tmp.filePath("horizon_de.qm"), germanCatalog());

    LocaleManager mgr;
    ASSERT_TRUE(mgr.apply(tmp.path(), "de_DE"));
    EXPECT_EQ(mgr.currentLocale(), "de_DE");
    EXPECT_EQ(translated("&File"), QString::fromUtf8("&Datei"));
    EXPECT_EQ(translated("%1 selected").arg(3), QString::fromUtf8("3 ausgew\xC3\xA4hlt"));
    // Strings missing from the catalog fall back to the English source.
    EXPECT_EQ(translated("&Quit"), QString::fromUtf8("&Quit"));

    // English restores source strings.
    ASSERT_TRUE(mgr.apply(tmp.path(), "en"));
    EXPECT_TRUE(mgr.currentLocale().isEmpty());
    EXPECT_EQ(translated("&File"), QString::fromUtf8("&File"));
}

TEST(LocaleManagerTest, FailedApplyKeepsThePreviousCatalog) {
    app();
    QTemporaryDir tmp;
    writeFile(tmp.filePath("horizon_de.qm"), germanCatalog());

    LocaleManager mgr;
    ASSERT_TRUE(mgr.apply(tmp.path(), "de"));
    EXPECT_FALSE(mgr.apply(tmp.path(), "fr"));  // no such catalog
    EXPECT_EQ(mgr.currentLocale(), "de");
    EXPECT_EQ(translated("&File"), QString::fromUtf8("&Datei"));

    // Corrupt catalogs fail to load and also keep the working one.
    writeFile(tmp.filePath("horizon_ja.qm"), "not a qm file");
    EXPECT_FALSE(mgr.apply(tmp.path(), "ja"));
    EXPECT_EQ(translated("&File"), QString::fromUtf8("&Datei"));

    ASSERT_TRUE(mgr.apply(tmp.path(), "en"));
}

TEST(LocaleManagerTest, DestructorUninstallsItsCatalog) {
    app();
    QTemporaryDir tmp;
    writeFile(tmp.filePath("horizon_de.qm"), germanCatalog());
    {
        LocaleManager mgr;
        ASSERT_TRUE(mgr.apply(tmp.path(), "de"));
        EXPECT_EQ(translated("&File"), QString::fromUtf8("&Datei"));
    }
    EXPECT_EQ(translated("&File"), QString::fromUtf8("&File"));
}

TEST(LocaleManagerTest, ShippedCatalogSourcesCoverAllSpecLanguages) {
    // The repo ships starter .ts sources for the six non-English languages
    // from roadmap §7.13 (DE, JA, ZH, ES, FR, KO).
    const QDir dir(QStringLiteral(HZ_TRANSLATIONS_SRC_DIR));
    ASSERT_TRUE(dir.exists());
    const QStringList entries = dir.entryList({"horizon_*.ts"}, QDir::Files, QDir::Name);
    EXPECT_EQ(entries, QStringList({"horizon_de.ts", "horizon_es.ts", "horizon_fr.ts",
                                    "horizon_ja.ts", "horizon_ko.ts", "horizon_zh.ts"}));
    for (const QString& entry : entries) {
        QFile f(dir.filePath(entry));
        ASSERT_TRUE(f.open(QIODevice::ReadOnly));
        const QByteArray xml = f.readAll();
        EXPECT_TRUE(xml.contains("<context>")) << entry.toStdString();
        EXPECT_TRUE(xml.contains("hz::ui::MainWindow")) << entry.toStdString();
        EXPECT_TRUE(xml.contains("<translation>")) << entry.toStdString();
    }
}
