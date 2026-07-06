#pragma once

#include <QString>
#include <QStringList>
#include <memory>

class QTranslator;

namespace hz::ui {

/// UI translation loading (Phase 77, roadmap §7.13 localization).
///
/// Catalogs are Qt `.qm` files named `horizon_<locale>.qm` (e.g.
/// `horizon_de.qm`, `horizon_ja.qm`), normally living in a `translations/`
/// directory next to the executable. Source strings are English; asking for
/// English (or an unknown locale) simply removes any installed catalog.
class LocaleManager {
public:
    LocaleManager();
    ~LocaleManager();

    LocaleManager(const LocaleManager&) = delete;
    LocaleManager& operator=(const LocaleManager&) = delete;

    /// Resolve @p locale to a catalog file inside @p dir with BCP-47-style
    /// fallback: "de_DE" tries horizon_de_DE.qm then horizon_de.qm.
    /// Hyphens are treated as underscores. Returns an empty string when no
    /// catalog matches.
    static QString resolveTranslationFile(const QString& dir, const QString& locale);

    /// Locales with a catalog present in @p dir (sorted, e.g. {"de", "ja"}).
    static QStringList availableLocales(const QString& dir);

    /// Install the catalog for @p locale from @p dir on the running
    /// QCoreApplication, replacing any catalog installed earlier by this
    /// manager. "en"/empty uninstalls (English is the source language) and
    /// succeeds. Returns false when no application is running, no catalog
    /// resolves, or the file fails to load — the previous catalog is kept
    /// in that case.
    bool apply(const QString& dir, const QString& locale);

    /// Locale of the currently installed catalog ("" = source English).
    const QString& currentLocale() const { return m_locale; }

private:
    std::unique_ptr<QTranslator> m_translator;
    QString m_locale;
};

}  // namespace hz::ui
