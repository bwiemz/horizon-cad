#pragma once

#include <QDialog>
#include <map>
#include <memory>
#include <optional>
#include <string>

#include "horizon/ai/Http.h"
#include "horizon/ui/AiSettings.h"

class QCheckBox;
class QComboBox;
class QLabel;
class QLineEdit;
class QPushButton;
class QSpinBox;

namespace hz::ui {

class QtHttpTransport;

/// The assistant's settings: provider, server, model (typed, or chosen from
/// the server's list), API key, and options. Each provider keeps its own
/// while another is shown. A key typed here is saved by AiKeyStore when the
/// dialog is accepted; a saved one is never shown.
class AiSettingsDialog : public QDialog {
    Q_OBJECT
public:
    explicit AiSettingsDialog(const AiSettings& settings, QWidget* parent = nullptr);
    ~AiSettingsDialog() override;

    /// Take what the fields show, then close.
    void accept() override;

    /// The settings as edited (no keys).
    AiSettings settings() const;
    /// The keys typed, by provider ("" to forget one); providers left alone
    /// are absent.
    const std::map<ai::ProviderKind, std::string>& typedKeys() const { return m_keys; }

    /// Use @p transport for "Fetch models" (a test's); it must outlive the dialog.
    void setTransport(ai::HttpTransport* transport) { m_transport = transport; }

private:
    void showProvider(ai::ProviderKind kind);
    void takeEdits();
    void fetchModels();
    ai::ProviderKind shownProvider() const;

    AiSettings m_settings;
    std::map<ai::ProviderKind, std::string> m_keys;
    ai::ProviderKind m_shown = ai::ProviderKind::Anthropic;
    std::unique_ptr<QtHttpTransport> m_network;
    ai::HttpTransport* m_transport = nullptr;
    std::optional<ai::HttpTransport::RequestId> m_fetch;  ///< in flight
    std::uint64_t m_fetchSerial = 0;
    std::uint64_t m_answeredSerial = 0;

    QComboBox* m_provider = nullptr;
    QLineEdit* m_baseUrl = nullptr;
    QComboBox* m_model = nullptr;
    QPushButton* m_fetchButton = nullptr;
    QLineEdit* m_key = nullptr;
    QLabel* m_keyNote = nullptr;
    QComboBox* m_effort = nullptr;
    QCheckBox* m_fallback = nullptr;
    QCheckBox* m_images = nullptr;
    QCheckBox* m_stream = nullptr;
    QSpinBox* m_maxTokens = nullptr;
    QSpinBox* m_maxSteps = nullptr;
    QLabel* m_hint = nullptr;
    QLabel* m_status = nullptr;
    QWidget* m_anthropicOnly = nullptr;
};

}  // namespace hz::ui
