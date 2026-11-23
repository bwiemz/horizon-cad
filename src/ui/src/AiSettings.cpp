#include "horizon/ui/AiSettings.h"

#include <QDir>
#include <QFile>
#include <QFileInfo>
#include <QJsonDocument>
#include <QJsonObject>
#include <QSaveFile>
#include <QSettings>
#include <QStandardPaths>
#include <QUrl>
#include <algorithm>

namespace hz::ui {

namespace {

QString group(ai::ProviderKind kind) {
    return QStringLiteral("ai/") + QString::fromLatin1(ai::providerName(kind));
}

/// The saved keys; false when the file is there but cannot be read as keys
/// (then it is not written over: the other providers' keys are in it).
bool readKeys(QJsonObject& keys) {
    keys = {};
    QFile file(AiKeyStore::filePath());
    if (!file.exists()) return true;
    if (!file.open(QIODevice::ReadOnly)) return false;
    const QJsonDocument document = QJsonDocument::fromJson(file.readAll());
    if (!document.isObject()) return false;
    keys = document.object();
    return true;
}

}  // namespace

AiSettings AiSettings::defaults() {
    AiSettings settings;
    for (const ai::ProviderKind kind : ai::kAllProviders) {
        settings.configs[kind] = ai::defaultConfig(kind);
    }
    return settings;
}

AiSettings AiSettings::load() {
    AiSettings settings = defaults();
    const QSettings store;
    const QString provider = store.value(QStringLiteral("ai/provider")).toString();
    if (const auto kind = ai::providerFromName(provider.toStdString())) settings.provider = *kind;
    settings.maxSteps = std::clamp(store.value(QStringLiteral("ai/maxSteps"), 24).toInt(), 1, 200);
    for (auto& [kind, config] : settings.configs) {
        const QString g = group(kind) + QLatin1Char('/');
        config.baseUrl =
            store.value(g + QStringLiteral("baseUrl"), QString::fromStdString(config.baseUrl))
                .toString()
                .toStdString();
        config.model =
            store.value(g + QStringLiteral("model"), QString::fromStdString(config.model))
                .toString()
                .toStdString();
        config.maxTokens = std::clamp(
            store.value(g + QStringLiteral("maxTokens"), config.maxTokens).toInt(), 256, 128000);
        config.effort =
            store.value(g + QStringLiteral("effort"), QString::fromStdString(config.effort))
                .toString()
                .toStdString();
        config.serverFallback =
            store.value(g + QStringLiteral("serverFallback"), config.serverFallback).toBool();
        config.apiKey.clear();
    }
    return settings;
}

void AiSettings::save() const {
    QSettings store;
    store.setValue(QStringLiteral("ai/provider"), QString::fromLatin1(ai::providerName(provider)));
    store.setValue(QStringLiteral("ai/maxSteps"), maxSteps);
    for (const auto& [kind, config] : configs) {
        const QString g = group(kind) + QLatin1Char('/');
        store.setValue(g + QStringLiteral("baseUrl"), QString::fromStdString(config.baseUrl));
        store.setValue(g + QStringLiteral("model"), QString::fromStdString(config.model));
        store.setValue(g + QStringLiteral("maxTokens"), config.maxTokens);
        store.setValue(g + QStringLiteral("effort"), QString::fromStdString(config.effort));
        store.setValue(g + QStringLiteral("serverFallback"), config.serverFallback);
        // Never a key: those are AiKeyStore's.
    }
}

ai::ProviderConfig AiSettings::config(ai::ProviderKind kind) const {
    const auto it = configs.find(kind);
    ai::ProviderConfig config = it != configs.end() ? it->second : ai::defaultConfig(kind);
    config.kind = kind;
    config.apiKey = AiKeyStore::key(kind);
    return config;
}

QString AiSettings::host(ai::ProviderKind kind) const {
    const auto it = configs.find(kind);
    return QUrl(QString::fromStdString(it != configs.end() ? it->second.baseUrl : ""))
        .host()
        .toLower();
}

bool AiSettings::isRemote(ai::ProviderKind kind) const {
    if (kind != ai::ProviderKind::OpenAICompatible) return true;
    const QString server = host(kind);
    return !(server == QLatin1String("localhost") || server == QLatin1String("127.0.0.1") ||
             server == QLatin1String("::1"));
}

bool AiSettings::consented(ai::ProviderKind kind) const {
    const QString agreed =
        QSettings().value(group(kind) + QStringLiteral("/consentedHost")).toString();
    return !agreed.isEmpty() && agreed == host(kind);
}

void AiSettings::setConsented(ai::ProviderKind kind, bool consented) const {
    QSettings store;
    if (consented) {
        store.setValue(group(kind) + QStringLiteral("/consentedHost"), host(kind));
    } else {
        store.remove(group(kind) + QStringLiteral("/consentedHost"));
    }
}

// --- AiKeyStore ---------------------------------------------------------------

QString AiKeyStore::filePath() {
    return QStandardPaths::writableLocation(QStandardPaths::AppConfigLocation) +
           QStringLiteral("/ai-keys.json");
}

std::optional<std::string> AiKeyStore::environmentKey(ai::ProviderKind kind, QString* variable) {
    for (const auto& name : ai::apiKeyEnvironment(kind)) {
        const QString value = qEnvironmentVariable(name.c_str()).trimmed();
        if (value.isEmpty()) continue;
        if (variable) *variable = QString::fromStdString(name);
        return value.toStdString();
    }
    return std::nullopt;
}

std::string AiKeyStore::savedKey(ai::ProviderKind kind) {
    QJsonObject keys;
    readKeys(keys);
    return keys.value(QString::fromLatin1(ai::providerName(kind))).toString().toStdString();
}

std::string AiKeyStore::key(ai::ProviderKind kind) {
    if (const auto fromEnvironment = environmentKey(kind)) return *fromEnvironment;
    return savedKey(kind);
}

bool AiKeyStore::saveKey(ai::ProviderKind kind, const std::string& key, QString* error) {
    QJsonObject keys;
    if (!readKeys(keys)) {
        if (error) {
            *error = QStringLiteral("%1 cannot be read as saved keys; fix or delete it first")
                         .arg(filePath());
        }
        return false;
    }
    const QString name = QString::fromLatin1(ai::providerName(kind));
    const QString trimmed = QString::fromStdString(key).trimmed();
    if (trimmed.isEmpty()) {
        keys.remove(name);
    } else {
        keys.insert(name, trimmed);
    }
    const QString path = filePath();
    if (!QDir().mkpath(QFileInfo(path).absolutePath())) {
        if (error) *error = QStringLiteral("could not make the folder for %1").arg(path);
        return false;
    }
    QSaveFile file(path);
    if (!file.open(QIODevice::WriteOnly)) {
        if (error) *error = file.errorString();
        return false;
    }
    // Only its user may read it, from before anything is written to it.
    file.setPermissions(QFileDevice::ReadOwner | QFileDevice::WriteOwner);
    file.write(QJsonDocument(keys).toJson(QJsonDocument::Indented));
    if (!file.commit()) {
        if (error) *error = file.errorString();
        return false;
    }
    QFile::setPermissions(path, QFileDevice::ReadOwner | QFileDevice::WriteOwner);
    return true;
}

}  // namespace hz::ui
