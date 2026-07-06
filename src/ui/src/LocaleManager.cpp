#include "horizon/ui/LocaleManager.h"

#include <QCoreApplication>
#include <QDir>
#include <QFileInfo>
#include <QTranslator>

namespace hz::ui {

namespace {
const QString kPrefix = QStringLiteral("horizon_");
const QString kSuffix = QStringLiteral(".qm");

bool isSourceLanguage(const QString& locale) {
    return locale.isEmpty() || locale == QLatin1String("en") ||
           locale.startsWith(QLatin1String("en_")) || locale.startsWith(QLatin1String("en-"));
}
}  // namespace

LocaleManager::LocaleManager() = default;

LocaleManager::~LocaleManager() {
    // Uninstall before the translator dies so the app never holds a
    // dangling translator pointer.
    if (m_translator && QCoreApplication::instance()) {
        QCoreApplication::removeTranslator(m_translator.get());
    }
}

QString LocaleManager::resolveTranslationFile(const QString& dir, const QString& locale) {
    QString name = locale;
    name.replace(QLatin1Char('-'), QLatin1Char('_'));

    // "de_DE_formal" → "de_DE" → "de": chop one _segment per attempt.
    while (!name.isEmpty()) {
        const QFileInfo candidate(QDir(dir), kPrefix + name + kSuffix);
        if (candidate.exists() && candidate.isFile()) return candidate.absoluteFilePath();
        const qsizetype cut = name.lastIndexOf(QLatin1Char('_'));
        if (cut < 0) break;
        name.truncate(cut);
    }
    return {};
}

QStringList LocaleManager::availableLocales(const QString& dir) {
    QStringList locales;
    const QStringList entries =
        QDir(dir).entryList({kPrefix + QStringLiteral("*") + kSuffix}, QDir::Files, QDir::Name);
    for (const QString& entry : entries) {
        locales << entry.mid(kPrefix.size(), entry.size() - kPrefix.size() - kSuffix.size());
    }
    return locales;
}

bool LocaleManager::apply(const QString& dir, const QString& locale) {
    QCoreApplication* app = QCoreApplication::instance();
    if (!app) return false;

    if (isSourceLanguage(locale)) {
        if (m_translator) {
            QCoreApplication::removeTranslator(m_translator.get());
            m_translator.reset();
        }
        m_locale.clear();
        return true;
    }

    const QString file = resolveTranslationFile(dir, locale);
    if (file.isEmpty()) return false;

    auto fresh = std::make_unique<QTranslator>();
    if (!fresh->load(file)) return false;

    if (m_translator) QCoreApplication::removeTranslator(m_translator.get());
    QCoreApplication::installTranslator(fresh.get());
    m_translator = std::move(fresh);
    m_locale = locale;
    return true;
}

}  // namespace hz::ui
