#pragma once

#include <QString>
#include <map>
#include <optional>
#include <string>

#include "horizon/ai/Provider.h"

namespace hz::ui {

/// The AI assistant's settings: the provider chosen, and for each provider
/// its server, model and options, kept in QSettings under "ai/". API keys are
/// not among them: see AiKeyStore.
struct AiSettings {
    ai::ProviderKind provider = ai::ProviderKind::Anthropic;
    std::map<ai::ProviderKind, ai::ProviderConfig> configs;  ///< their apiKey always empty
    int maxSteps = 24;

    /// Every provider at its defaults.
    static AiSettings defaults();
    static AiSettings load();
    void save() const;

    /// @p kind's settings, its key filled in from AiKeyStore.
    ai::ProviderConfig config(ai::ProviderKind kind) const;
    ai::ProviderConfig activeConfig() const { return config(provider); }

    /// Whether a request to @p kind's server leaves this computer: any but a
    /// local OpenAI-compatible server (localhost, 127.0.0.1, ::1).
    bool isRemote(ai::ProviderKind kind) const;
    /// Whether the user has agreed to send to @p kind (asked once per
    /// provider, for a remote one).
    static bool consented(ai::ProviderKind kind);
    static void setConsented(ai::ProviderKind kind, bool consented);
};

/// API keys. An environment variable (ANTHROPIC_API_KEY, OPENAI_API_KEY,
/// GEMINI_API_KEY or GOOGLE_API_KEY) wins; otherwise the key saved in a file
/// of its own, readable only by its user, never in QSettings, a document, a
/// log or a crash report. A local server needs none.
class AiKeyStore {
public:
    /// Where saved keys are kept.
    static QString filePath();

    /// @p kind's key from the environment, and the variable's name.
    static std::optional<std::string> environmentKey(ai::ProviderKind kind,
                                                     QString* variable = nullptr);
    /// @p kind's saved key, or "".
    static std::string savedKey(ai::ProviderKind kind);
    /// The key used: the environment's, else the saved one.
    static std::string key(ai::ProviderKind kind);

    /// Save @p key for @p kind ("" forgets it). False, and why in @p error,
    /// when the file cannot be written.
    static bool saveKey(ai::ProviderKind kind, const std::string& key, QString* error = nullptr);
};

}  // namespace hz::ui
