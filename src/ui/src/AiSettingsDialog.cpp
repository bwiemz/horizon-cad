#include "horizon/ui/AiSettingsDialog.h"

#include <QCheckBox>
#include <QComboBox>
#include <QDialogButtonBox>
#include <QFormLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QPushButton>
#include <QSpinBox>
#include <QVBoxLayout>

#include "horizon/ui/QtHttpTransport.h"

namespace hz::ui {

AiSettingsDialog::AiSettingsDialog(const AiSettings& settings, QWidget* parent)
    : QDialog(parent), m_settings(settings), m_network(std::make_unique<QtHttpTransport>()) {
    m_transport = m_network.get();
    setWindowTitle(tr("AI Assistant Settings"));
    setObjectName(QStringLiteral("AiSettingsDialog"));

    auto* layout = new QVBoxLayout(this);
    auto* form = new QFormLayout;
    layout->addLayout(form);

    m_provider = new QComboBox(this);
    m_provider->setObjectName(QStringLiteral("aiProvider"));
    m_provider->setAccessibleName(tr("Provider"));
    for (const ai::ProviderKind kind : ai::kAllProviders) {
        m_provider->addItem(QString::fromLatin1(ai::providerTitle(kind)), static_cast<int>(kind));
    }
    form->addRow(tr("Provider:"), m_provider);

    m_baseUrl = new QLineEdit(this);
    m_baseUrl->setObjectName(QStringLiteral("aiBaseUrl"));
    m_baseUrl->setAccessibleName(tr("Server address"));
    form->addRow(tr("Server:"), m_baseUrl);

    m_model = new QComboBox(this);
    m_model->setObjectName(QStringLiteral("aiModel"));
    m_model->setAccessibleName(tr("Model"));
    m_model->setEditable(true);
    m_model->setInsertPolicy(QComboBox::NoInsert);
    m_model->setMinimumContentsLength(24);
    m_fetchButton = new QPushButton(tr("Fetch Models"), this);
    m_fetchButton->setObjectName(QStringLiteral("aiFetchModels"));
    m_fetchButton->setToolTip(tr("List the models this server and key can use"));
    auto* modelRow = new QHBoxLayout;
    modelRow->addWidget(m_model, 1);
    modelRow->addWidget(m_fetchButton);
    form->addRow(tr("Model:"), modelRow);

    m_key = new QLineEdit(this);
    m_key->setObjectName(QStringLiteral("aiApiKey"));
    m_key->setAccessibleName(tr("API key"));
    m_key->setEchoMode(QLineEdit::Password);
    form->addRow(tr("API key:"), m_key);
    m_keyNote = new QLabel(this);
    m_keyNote->setWordWrap(true);
    m_keyNote->setStyleSheet(QStringLiteral("color: palette(mid);"));
    form->addRow(QString(), m_keyNote);

    // Anthropic's own options.
    m_anthropicOnly = new QWidget(this);
    auto* anthropicForm = new QFormLayout(m_anthropicOnly);
    anthropicForm->setContentsMargins(0, 0, 0, 0);
    m_effort = new QComboBox(m_anthropicOnly);
    m_effort->setObjectName(QStringLiteral("aiEffort"));
    m_effort->setAccessibleName(tr("Effort"));
    m_effort->addItem(tr("Model default"), QString());
    m_effort->addItem(tr("Low (quick)"), QStringLiteral("low"));
    m_effort->addItem(tr("Medium"), QStringLiteral("medium"));
    m_effort->addItem(tr("High"), QStringLiteral("high"));
    m_effort->addItem(tr("Extra high"), QStringLiteral("xhigh"));
    m_effort->addItem(tr("Max (slow)"), QStringLiteral("max"));
    anthropicForm->addRow(tr("Effort:"), m_effort);
    m_fallback = new QCheckBox(tr("Retry a declined request on Anthropic's fallback model"),
                               m_anthropicOnly);
    m_fallback->setObjectName(QStringLiteral("aiFallback"));
    m_fallback->setToolTip(
        tr("When a safety classifier declines a request, Anthropic re-runs it on its recommended "
           "fallback model (for the models that offer it)."));
    anthropicForm->addRow(QString(), m_fallback);
    form->addRow(m_anthropicOnly);

    m_maxTokens = new QSpinBox(this);
    m_maxTokens->setObjectName(QStringLiteral("aiMaxTokens"));
    m_maxTokens->setAccessibleName(tr("Longest reply"));
    m_maxTokens->setRange(256, 128000);
    m_maxTokens->setSingleStep(1024);
    m_maxTokens->setSuffix(tr(" tokens"));
    form->addRow(tr("Longest reply:"), m_maxTokens);

    m_images = new QCheckBox(tr("Let the AI see pictures of the part"), this);
    m_images->setObjectName(QStringLiteral("aiImages"));
    m_images->setToolTip(
        tr("The assistant can look at a drawn view of the part to check its work. The AI "
           "model must read images: the cloud providers' models do; a local model may not."));
    form->addRow(QString(), m_images);
    m_stream = new QCheckBox(tr("Show replies as they are written"), this);
    m_stream->setObjectName(QStringLiteral("aiStream"));
    m_stream->setToolTip(
        tr("Ask for each reply as it is written, so its text appears as it comes. Turn it off "
           "for a server that does not stream replies well."));
    form->addRow(QString(), m_stream);

    m_maxSteps = new QSpinBox(this);
    m_maxSteps->setObjectName(QStringLiteral("aiMaxSteps"));
    m_maxSteps->setAccessibleName(tr("Steps per request"));
    m_maxSteps->setRange(1, 200);
    m_maxSteps->setValue(m_settings.maxSteps);
    m_maxSteps->setToolTip(tr("How many rounds of tool calls one request may take"));
    form->addRow(tr("Steps per request:"), m_maxSteps);

    m_hint = new QLabel(this);
    m_hint->setWordWrap(true);
    m_hint->setTextFormat(Qt::PlainText);
    layout->addWidget(m_hint);
    m_status = new QLabel(this);
    m_status->setObjectName(QStringLiteral("aiSettingsStatus"));
    m_status->setWordWrap(true);
    layout->addWidget(m_status);

    auto* buttons = new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel, this);
    connect(buttons, &QDialogButtonBox::accepted, this, &AiSettingsDialog::accept);
    connect(buttons, &QDialogButtonBox::rejected, this, &QDialog::reject);
    layout->addWidget(buttons);

    // The provider chosen is shown before the combo is watched: a change
    // takes the fields shown into the provider shown, which they are not yet.
    m_provider->setCurrentIndex(m_provider->findData(static_cast<int>(m_settings.provider)));
    showProvider(m_settings.provider);
    connect(m_provider, &QComboBox::currentIndexChanged, this, [this](int) {
        takeEdits();
        showProvider(shownProvider());
    });
    connect(m_fetchButton, &QPushButton::clicked, this, &AiSettingsDialog::fetchModels);
}

void AiSettingsDialog::accept() {
    // However it is accepted (OK, Enter), what the fields show is taken.
    takeEdits();
    QDialog::accept();
}

AiSettingsDialog::~AiSettingsDialog() {
    if (m_fetch && m_transport) m_transport->cancel(*m_fetch);
}

ai::ProviderKind AiSettingsDialog::shownProvider() const {
    return static_cast<ai::ProviderKind>(m_provider->currentData().toInt());
}

void AiSettingsDialog::showProvider(ai::ProviderKind kind) {
    m_shown = kind;
    const ai::ProviderConfig& config = m_settings.configs[kind];
    m_baseUrl->setText(QString::fromStdString(config.baseUrl));
    m_model->clear();
    m_model->setEditText(QString::fromStdString(config.model));
    m_maxTokens->setValue(config.maxTokens);
    m_effort->setCurrentIndex(
        std::max(0, m_effort->findData(QString::fromStdString(config.effort))));
    m_fallback->setChecked(config.serverFallback);
    m_images->setChecked(config.images);
    m_stream->setChecked(config.stream);
    m_anthropicOnly->setVisible(kind == ai::ProviderKind::Anthropic);

    m_key->clear();
    QString variable;
    const bool fromEnvironment = AiKeyStore::environmentKey(kind, &variable).has_value();
    const auto typed = m_keys.find(kind);
    if (typed != m_keys.end()) m_key->setText(QString::fromStdString(typed->second));
    m_key->setEnabled(!fromEnvironment);
    if (fromEnvironment) {
        m_key->setPlaceholderText(tr("Set by %1").arg(variable));
        m_keyNote->setText(tr("The key comes from the environment variable %1.").arg(variable));
    } else if (kind == ai::ProviderKind::OpenAICompatible) {
        m_key->setPlaceholderText(tr("Not needed for a local server"));
        m_keyNote->setText(tr("Saved only on this computer, readable only by you."));
    } else {
        m_key->setPlaceholderText(AiKeyStore::savedKey(kind).empty()
                                      ? tr("Paste your API key")
                                      : tr("Saved (type to replace)"));
        m_keyNote->setText(tr("Saved only on this computer, readable only by you, in %1. Or set "
                              "%2 in the environment.")
                               .arg(AiKeyStore::filePath(),
                                    QString::fromStdString(ai::apiKeyEnvironment(kind).front())));
    }

    switch (kind) {
        case ai::ProviderKind::Anthropic:
            m_hint->setText(
                tr("Claude models from Anthropic. Get a key at console.anthropic.com."));
            break;
        case ai::ProviderKind::OpenAI:
            m_hint->setText(tr("GPT models from OpenAI. Get a key at platform.openai.com."));
            break;
        case ai::ProviderKind::Gemini:
            m_hint->setText(tr("Gemini models from Google. Get a key at aistudio.google.com."));
            break;
        case ai::ProviderKind::OpenAICompatible:
            m_hint->setText(
                tr("A model on this computer, or any server with OpenAI's chat API. Ollama: "
                   "http://localhost:11434/v1. LM Studio: http://localhost:1234/v1. llama.cpp: "
                   "http://localhost:8080/v1. Choose a model that can call tools (functions)."));
            break;
    }
    m_status->clear();
}

void AiSettingsDialog::takeEdits() {
    ai::ProviderConfig& config = m_settings.configs[m_shown];
    config.kind = m_shown;
    config.baseUrl = m_baseUrl->text().trimmed().toStdString();
    config.model = m_model->currentText().trimmed().toStdString();
    config.maxTokens = m_maxTokens->value();
    config.effort = m_effort->currentData().toString().toStdString();
    config.serverFallback = m_fallback->isChecked();
    config.images = m_images->isChecked();
    config.stream = m_stream->isChecked();
    if (m_key->isEnabled()) {
        // A key typed is saved; one typed and erased again is not (an
        // untouched field leaves the saved key as it is).
        const QString typed = m_key->text().trimmed();
        if (typed.isEmpty()) {
            m_keys.erase(m_shown);
        } else {
            m_keys[m_shown] = typed.toStdString();
        }
    }
    m_settings.maxSteps = m_maxSteps->value();
    m_settings.provider = shownProvider();
}

AiSettings AiSettingsDialog::settings() const {
    return m_settings;
}

void AiSettingsDialog::fetchModels() {
    if (!m_transport) return;
    takeEdits();
    if (m_fetch) m_transport->cancel(*m_fetch);
    ai::ProviderConfig config = m_settings.config(m_shown);
    if (const auto typed = m_keys.find(m_shown); typed != m_keys.end())
        config.apiKey = typed->second;
    const auto adapter = ai::makeAdapter(m_shown);
    m_status->setText(tr("Asking the server for its models…"));
    m_fetchButton->setEnabled(false);
    const ai::ProviderKind kind = m_shown;
    const std::uint64_t serial = ++m_fetchSerial;
    const auto id = m_transport->send(
        adapter->buildListModels(config), [this, kind, serial](ai::HttpResponse reply) {
            m_answeredSerial = serial;
            m_fetch.reset();
            m_fetchButton->setEnabled(true);
            if (kind != m_shown) return;  // another provider is shown now
            std::string error;
            const auto models = ai::makeAdapter(kind)->parseListModels(reply, &error);
            if (models.empty()) {
                m_status->setText(QString::fromStdString(error));
                return;
            }
            const QString current = m_model->currentText();
            m_model->clear();
            for (const auto& model : models) m_model->addItem(QString::fromStdString(model));
            m_model->setEditText(current.isEmpty() ? m_model->itemText(0) : current);
            m_status->setText(tr("Models listed: %n", "", static_cast<int>(models.size())));
        });
    // A transport may answer before send() returns: then nothing is in flight.
    if (m_answeredSerial != serial) m_fetch = id;
}

}  // namespace hz::ui
