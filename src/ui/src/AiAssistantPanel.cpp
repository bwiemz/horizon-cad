#include "horizon/ui/AiAssistantPanel.h"

#include <QCoreApplication>
#include <QEvent>
#include <QHBoxLayout>
#include <QKeyEvent>
#include <QLabel>
#include <QLocale>
#include <QMessageBox>
#include <QPlainTextEdit>
#include <QProgressBar>
#include <QPushButton>
#include <QScrollBar>
#include <QTextBrowser>
#include <QTextCharFormat>
#include <QTextCursor>
#include <QTextDocument>
#include <QToolButton>
#include <QUrl>
#include <QVBoxLayout>
#include <algorithm>

#include "horizon/ai/AgentSession.h"
#include "horizon/ai/CadTools.h"
#include "horizon/ai/SystemPrompt.h"
#include "horizon/document/Document.h"
#include "horizon/document/UndoStack.h"
#include "horizon/ui/AiSettingsDialog.h"
#include "horizon/ui/QtHttpTransport.h"

namespace hz::ui {

namespace {

using nlohmann::json;

/// A tool call's arguments in a line: "sketch_id 12, distance 5".
QString summariseInput(const json& input) {
    if (!input.is_object() || input.empty()) return {};
    QStringList parts;
    for (auto it = input.begin(); it != input.end(); ++it) {
        QString value;
        const json& v = it.value();
        if (v.is_string()) {
            value = QString::fromStdString(v.get<std::string>());
        } else if (v.is_array() && v.size() <= 3 &&
                   std::all_of(v.begin(), v.end(), [](const json& x) { return x.is_number(); })) {
            value = QString::fromStdString(v.dump());
        } else if (v.is_array()) {
            value = QCoreApplication::translate("hz::ui::AiAssistantPanel", "list of %n", "",
                                                static_cast<int>(v.size()));
        } else if (v.is_object()) {
            value = QStringLiteral("{…}");
        } else {
            value = QString::fromStdString(v.dump());
        }
        if (value.size() > 40) value = value.left(39) + QStringLiteral("…");
        parts << QString::fromStdString(it.key()) + QStringLiteral(": ") + value;
    }
    QString line = parts.join(QStringLiteral(", "));
    if (line.size() > 120) line = line.left(119) + QStringLiteral("…");
    return line;
}

/// What a tool's result says, in a few words. (Spelled out with its
/// context, as lupdate reads it.)
QString summariseResult(const ai::ToolResult& result) {
    if (result.isError) {
        QString text = QString::fromStdString(result.content).simplified();
        if (text.size() > 240) text = text.left(239) + QStringLiteral("…");
        return text;
    }
    const json value = json::parse(result.content, nullptr, false);
    if (!value.is_object()) {
        return QCoreApplication::translate("hz::ui::AiAssistantPanel", "Done");
    }
    // A count is read before the translate() that takes it: lupdate 6.4 never
    // finishes reading a translate() whose count is a call given a string, as
    // count("ids") is, and CI's catalog check ran out of time on it.
    const auto count = [&value](const char* key) { return static_cast<int>(value[key].size()); };
    const auto has = [&value](const char* key, json::value_t type) {
        return value.contains(key) && value[key].type() == type;
    };
    QString summary;
    if (has("added", json::value_t::string)) {
        summary = QCoreApplication::translate("hz::ui::AiAssistantPanel", "Added %1")
                      .arg(QString::fromStdString(value["added"].get<std::string>()));
    } else if (value.contains("sketch_id")) {
        summary = QCoreApplication::translate("hz::ui::AiAssistantPanel", "Made a sketch");
    } else if (value.contains("edited_feature")) {
        summary = QCoreApplication::translate("hz::ui::AiAssistantPanel", "Edited a feature");
    } else if (has("deleted", json::value_t::string)) {
        summary = QCoreApplication::translate("hz::ui::AiAssistantPanel", "Deleted %1")
                      .arg(QString::fromStdString(value["deleted"].get<std::string>()));
    } else if (has("ids", json::value_t::array)) {
        const int drawn = count("ids");
        summary =
            QCoreApplication::translate("hz::ui::AiAssistantPanel", "Shapes drawn: %n", "", drawn);
    } else if (result.changedModel && value.contains("variables")) {
        summary = QCoreApplication::translate("hz::ui::AiAssistantPanel", "Set the variables");
    } else if (has("edges", json::value_t::array)) {
        const int edges = count("edges");
        summary =
            QCoreApplication::translate("hz::ui::AiAssistantPanel", "Edges read: %n", "", edges);
    } else if (has("faces", json::value_t::array)) {
        const int faces = count("faces");
        summary =
            QCoreApplication::translate("hz::ui::AiAssistantPanel", "Faces read: %n", "", faces);
    } else if (has("features", json::value_t::array)) {
        const int features = count("features");
        summary = QCoreApplication::translate("hz::ui::AiAssistantPanel", "Features read: %n", "",
                                              features);
    } else if (value.contains("mass_g")) {
        summary = QCoreApplication::translate("hz::ui::AiAssistantPanel", "Measured the mass");
    } else if (result.changedModel) {
        summary = QCoreApplication::translate("hz::ui::AiAssistantPanel", "Changed the document");
    } else {
        summary = QCoreApplication::translate("hz::ui::AiAssistantPanel", "Looked at the document");
    }
    if (has("build_error", json::value_t::object)) {
        const json& failure = value["build_error"];
        const int index =
            failure.contains("feature_index") && failure["feature_index"].is_number_integer()
                ? failure["feature_index"].get<int>()
                : -1;
        summary +=
            QStringLiteral(" — ") +
            QCoreApplication::translate("hz::ui::AiAssistantPanel", "feature %1 fails").arg(index);
    }
    return summary;
}

}  // namespace

/// The session's events, shown in the panel.
class AiAssistantPanel::Observer : public ai::AgentObserver {
public:
    explicit Observer(AiAssistantPanel& panel) : m_panel(panel) {}

    void onAssistantText(const std::string& text) override {
        m_panel.appendAssistant(QString::fromStdString(text));
    }
    void onToolCall(const std::string& name, const json& input) override {
        m_panel.appendToolCall(QString::fromStdString(name), summariseInput(input));
    }
    void onToolResult(const std::string& /*name*/, const ai::ToolResult& result) override {
        m_panel.appendToolResult(summariseResult(result), result.isError);
    }
    void onError(const std::string& message) override {
        m_panel.appendNote(QString::fromStdString(message), true);
    }
    void onFinished(const ai::Usage& usage) override {
        if (usage.inputTokens > 0 || usage.outputTokens > 0) {
            const QLocale locale;
            m_panel.m_status->setText(
                AiAssistantPanel::tr("%1 tokens in, %2 out")
                    .arg(locale.toString(usage.inputTokens), locale.toString(usage.outputTokens)));
        } else {
            m_panel.m_status->clear();
        }
        m_panel.updateState();
        emit m_panel.busyChanged(false);
    }

private:
    AiAssistantPanel& m_panel;
};

AiAssistantPanel::AiAssistantPanel(QWidget* parent)
    : QDockWidget(tr("AI Assistant"), parent),
      m_settings(AiSettings::load()),
      m_network(std::make_unique<QtHttpTransport>()),
      m_observer(std::make_unique<Observer>(*this)) {
    setObjectName(QStringLiteral("AiAssistantPanel"));
    m_transport = m_network.get();

    auto* body = new QWidget(this);
    auto* layout = new QVBoxLayout(body);
    layout->setContentsMargins(6, 6, 6, 6);

    auto* headerRow = new QHBoxLayout;
    m_header = new QLabel(body);
    m_header->setObjectName(QStringLiteral("aiHeader"));
    m_header->setTextFormat(Qt::PlainText);
    m_header->setWordWrap(true);
    headerRow->addWidget(m_header, 1);
    m_newButton = new QToolButton(body);
    m_newButton->setObjectName(QStringLiteral("aiNewConversation"));
    m_newButton->setText(tr("New"));
    m_newButton->setToolTip(tr("Start a new conversation"));
    m_newButton->setAccessibleName(tr("New conversation"));
    headerRow->addWidget(m_newButton);
    m_settingsButton = new QToolButton(body);
    m_settingsButton->setObjectName(QStringLiteral("aiSettings"));
    m_settingsButton->setText(tr("Settings…"));
    m_settingsButton->setToolTip(tr("Choose the provider, model and API key"));
    m_settingsButton->setAccessibleName(tr("Assistant settings"));
    headerRow->addWidget(m_settingsButton);
    layout->addLayout(headerRow);

    m_emptyHint = new QLabel(body);
    m_emptyHint->setObjectName(QStringLiteral("aiEmptyHint"));
    m_emptyHint->setWordWrap(true);
    m_emptyHint->setTextFormat(Qt::PlainText);
    m_emptyHint->setText(
        tr("Describe a part or a change, and the assistant builds it from ordinary features you "
           "can edit and undo. For example:\n\n"
           "• A 60 × 40 × 5 mm plate with a 5 mm hole 8 mm in from each corner\n"
           "• Fillet the top edges of the part by 1 mm\n"
           "• Make the plate's thickness a variable called wall\n"
           "• What does this part weigh in aluminium?\n"
           "• Draw a 100 × 70 mm border on a layer called FRAME"));
    layout->addWidget(m_emptyHint);

    m_transcript = new QTextBrowser(body);
    m_transcript->setObjectName(QStringLiteral("aiTranscript"));
    m_transcript->setAccessibleName(tr("Conversation"));
    m_transcript->setOpenLinks(false);
    m_transcript->setOpenExternalLinks(false);
    layout->addWidget(m_transcript, 1);

    auto* statusRow = new QHBoxLayout;
    m_busy = new QProgressBar(body);
    m_busy->setObjectName(QStringLiteral("aiBusy"));
    m_busy->setAccessibleName(tr("Working"));
    m_busy->setRange(0, 0);
    m_busy->setMaximumHeight(8);
    m_busy->setTextVisible(false);
    statusRow->addWidget(m_busy, 1);
    m_status = new QLabel(body);
    m_status->setObjectName(QStringLiteral("aiStatus"));
    m_status->setTextFormat(Qt::PlainText);
    statusRow->addWidget(m_status);
    layout->addLayout(statusRow);

    m_input = new QPlainTextEdit(body);
    m_input->setObjectName(QStringLiteral("aiInput"));
    m_input->setAccessibleName(tr("Request to the assistant"));
    m_input->setPlaceholderText(
        tr("Describe a part or a change… (Enter sends, Shift+Enter: new line)"));
    m_input->setMaximumHeight(90);
    m_input->installEventFilter(this);
    layout->addWidget(m_input);

    auto* buttons = new QHBoxLayout;
    m_undo = new QPushButton(tr("Undo This Response"), body);
    m_undo->setObjectName(QStringLiteral("aiUndoResponse"));
    m_undo->setToolTip(tr("Undo every change the last request made"));
    buttons->addWidget(m_undo);
    buttons->addStretch(1);
    m_stop = new QPushButton(tr("Stop"), body);
    m_stop->setObjectName(QStringLiteral("aiStop"));
    buttons->addWidget(m_stop);
    m_send = new QPushButton(tr("Send"), body);
    m_send->setObjectName(QStringLiteral("aiSend"));
    m_send->setDefault(true);
    buttons->addWidget(m_send);
    layout->addLayout(buttons);
    setWidget(body);

    connect(m_send, &QPushButton::clicked, this, [this] {
        if (submit(m_input->toPlainText())) m_input->clear();
    });
    connect(m_stop, &QPushButton::clicked, this, &AiAssistantPanel::stop);
    connect(m_undo, &QPushButton::clicked, this, &AiAssistantPanel::undoLastResponse);
    connect(m_newButton, &QToolButton::clicked, this, &AiAssistantPanel::newConversation);
    connect(m_settingsButton, &QToolButton::clicked, this, &AiAssistantPanel::openSettings);

    setTransport(m_network.get());
}

AiAssistantPanel::~AiAssistantPanel() {
    // The session first: it holds the transport, the tools and the observer.
    m_session.reset();
}

void AiAssistantPanel::setTransport(ai::HttpTransport* transport) {
    if (m_session) m_session->cancel();
    m_transport = transport != nullptr ? transport : m_network.get();
    m_session = std::make_unique<ai::AgentSession>(*m_transport, m_tools, *m_observer);
    m_session->setSystemPrompt(ai::defaultSystemPrompt());
    ai::UndoHooks hooks;
    hooks.undoCount = [this]() -> std::size_t {
        return m_document ? m_document->undoStack().undoCount() : 0;
    };
    hooks.revision = [this]() -> std::uint64_t {
        return m_document ? m_document->undoStack().revision() : 0;
    };
    hooks.undo = [this] {
        if (m_host.undo) {
            m_host.undo();
        } else if (m_document) {
            m_document->undoStack().undo();
            m_document->rebuildModel();
        }
    };
    m_session->setUndoHooks(std::move(hooks));
    applyConfig();
    m_transcript->clear();
    updateState();
}

void AiAssistantPanel::setSettings(const AiSettings& settings) {
    // A request under way would carry on with the new provider or server,
    // which the user has not agreed to send to: it stops first.
    const ai::ProviderConfig before = m_settings.activeConfig();
    const ai::ProviderConfig after = settings.activeConfig();
    if (busy() && (before.kind != after.kind || before.baseUrl != after.baseUrl ||
                   before.model != after.model)) {
        m_session->cancel();
        appendNote(tr("Stopped: the provider or server changed. Send the request again."));
    }
    m_settings = settings;
    applyConfig();
    updateState();
}

void AiAssistantPanel::applyConfig() {
    m_session->setConfig(m_settings.activeConfig());
    m_session->setMaxSteps(m_settings.maxSteps);
    updateHeader();
}

void AiAssistantPanel::setDocument(std::shared_ptr<doc::Document> document, const QString& name,
                                   const QString& why) {
    if (document == m_document && name == m_documentName && why == m_unavailable) return;
    // The conversation was about another document (even with an assembly,
    // and so no document, shown between).
    const bool switched = document != nullptr && m_conversationDocument != nullptr &&
                          document.get() != m_conversationDocument;
    if (m_session->busy()) {
        m_session->cancel();
        appendNote(tr("Stopped: the document changed."));
    }
    m_document = std::move(document);
    if (m_document) m_conversationDocument = m_document.get();
    m_documentName = name;
    m_unavailable = why;
    rebindTools();
    // Its undo steps were made to the other document.
    m_session->forgetLastResponse();
    if (switched && !m_session->messages().empty()) {
        m_session->addNote("(The user switched to another document, \"" + name.toStdString() +
                           "\". Look at it before changing it.)");
    }
    updateHeader();
    updateState();
}

void AiAssistantPanel::rebindTools() {
    m_tools.clear();
    if (!m_document) return;
    ai::CadToolHooks hooks;
    hooks.modelChanged = [this] {
        if (m_host.modelChanged) m_host.modelChanged();
    };
    hooks.drawingChanged = [this] {
        if (m_host.drawingChanged) m_host.drawingChanged();
    };
    ai::registerCadTools(m_tools, *m_document, std::move(hooks));
    if (m_host.selection) {
        m_tools.add({"get_selection",
                     "What the user has selected in the window: faces and edges of the part (ids "
                     "and names as list_faces and list_edges give them) and drawing entities "
                     "(ids). Call it when the user says \"this\", \"these\", \"here\" or \"the "
                     "selected...\".",
                     json{{"type", "object"}, {"properties", json::object()}}},
                    ai::ToolAccess::Read, [this](const json&) {
                        return ai::ToolResult::ok(m_host.selection ? m_host.selection() : json());
                    });
    }
}

bool AiAssistantPanel::busy() const {
    return m_session && m_session->busy();
}

void AiAssistantPanel::updateHeader() {
    const ai::ProviderConfig config = m_settings.activeConfig();
    QString text = QString::fromLatin1(ai::providerTitle(config.kind));
    if (!config.model.empty()) text += QStringLiteral(" · ") + QString::fromStdString(config.model);
    if (config.model.empty()) {
        text += QStringLiteral(" — ") + tr("choose a model in Settings");
    } else if (config.apiKey.empty() && config.kind != ai::ProviderKind::OpenAICompatible) {
        text += QStringLiteral(" — ") + tr("add an API key in Settings");
    }
    m_header->setText(text);
}

void AiAssistantPanel::updateState() {
    const bool working = busy();
    m_busy->setVisible(working);
    m_send->setEnabled(!working && m_document != nullptr);
    m_input->setEnabled(m_document != nullptr);
    m_stop->setEnabled(working);
    m_undo->setEnabled(!working && m_session->canUndoLastResponse());
    const bool empty = m_transcript->document()->isEmpty();
    m_emptyHint->setVisible(empty);
    m_transcript->setVisible(!empty);
    if (!m_document) {
        m_input->setPlaceholderText(m_unavailable.isEmpty() ? tr("Open a part or a drawing")
                                                            : m_unavailable);
    } else {
        m_input->setPlaceholderText(
            tr("Describe a part or a change… (Enter sends, Shift+Enter: new line)"));
    }
}

bool AiAssistantPanel::askConsent() {
    const ai::ProviderConfig config = m_settings.activeConfig();
    const QString host = QUrl(QString::fromStdString(config.baseUrl)).host();
    const auto answer = QMessageBox::question(
        this, tr("Send to %1?").arg(QString::fromLatin1(ai::providerTitle(config.kind))),
        tr("The assistant sends your requests, and what it reads of the open document (its "
           "features, sketches, faces, edges and drawing), to %1. That provider's terms and "
           "privacy "
           "policy apply to what it receives.\n\nSend requests there?")
            .arg(host.isEmpty() ? QString::fromStdString(config.baseUrl) : host));
    if (answer != QMessageBox::Yes) return false;
    m_settings.setConsented(config.kind, true);
    return true;
}

bool AiAssistantPanel::submit(const QString& text) {
    const QString request = text.trimmed();
    if (request.isEmpty() || busy()) return false;
    if (!m_document) {
        appendNote(m_unavailable.isEmpty() ? tr("Open a part or a drawing first.") : m_unavailable,
                   true);
        updateState();
        return false;
    }
    const ai::ProviderConfig config = m_settings.activeConfig();
    if (config.model.empty()) {
        appendNote(tr("Choose a model in the assistant's Settings first."), true);
        updateState();
        return false;
    }
    if (config.apiKey.empty() && config.kind != ai::ProviderKind::OpenAICompatible) {
        appendNote(tr("Add your %1 API key in the assistant's Settings first.")
                       .arg(QString::fromLatin1(ai::providerTitle(config.kind))),
                   true);
        updateState();
        return false;
    }
    if (m_settings.isRemote(config.kind) && !m_settings.consented(config.kind)) {
        if (!askConsent()) return false;
        // The question ran an event loop: the document, or a request, may
        // have changed under it.
        if (!m_document || busy()) {
            updateState();
            return false;
        }
    }
    m_session->setConfig(config);
    m_session->setMaxSteps(m_settings.maxSteps);
    appendUser(request);
    m_status->setText(tr("Working…"));
    const bool sent = m_session->send(request.toStdString());
    updateState();
    if (busy()) emit busyChanged(true);
    return sent;
}

void AiAssistantPanel::stop() {
    if (!busy()) return;
    m_session->cancel();
    appendNote(tr("Stopped. Changes already made stay; Undo This Response takes them back."));
    updateState();
}

bool AiAssistantPanel::undoLastResponse() {
    if (!m_session->undoLastResponse()) {
        appendNote(tr("The last response's changes cannot be undone here: the document has "
                      "changed since. Use Edit ▸ Undo."),
                   true);
        updateState();
        return false;
    }
    appendNote(tr("Undid the last response's changes."));
    updateState();
    return true;
}

void AiAssistantPanel::newConversation() {
    if (busy()) m_session->cancel();
    m_session->reset();
    m_transcript->clear();
    m_status->clear();
    updateState();
}

QString AiAssistantPanel::transcriptText() const {
    return m_transcript->toPlainText();
}

void AiAssistantPanel::focusInput() {
    m_input->setFocus(Qt::ShortcutFocusReason);
}

void AiAssistantPanel::openSettings() {
    AiSettingsDialog dialog(m_settings, this);
    if (dialog.exec() != QDialog::Accepted) return;
    for (const auto& [kind, key] : dialog.typedKeys()) {
        QString error;
        if (!AiKeyStore::saveKey(kind, key, &error)) {
            appendNote(tr("The API key could not be saved: %1").arg(error), true);
        }
    }
    const AiSettings chosen = dialog.settings();
    chosen.save();
    setSettings(chosen);
}

bool AiAssistantPanel::eventFilter(QObject* watched, QEvent* event) {
    if (watched == m_input && event->type() == QEvent::KeyPress) {
        const auto* key = static_cast<QKeyEvent*>(event);
        if ((key->key() == Qt::Key_Return || key->key() == Qt::Key_Enter) &&
            !(key->modifiers() & Qt::ShiftModifier)) {
            if (submit(m_input->toPlainText())) m_input->clear();
            return true;
        }
    }
    return QDockWidget::eventFilter(watched, event);
}

// --- The transcript -----------------------------------------------------------

namespace {

QTextCursor endOf(QTextBrowser* view) {
    QTextCursor cursor(view->document());
    cursor.movePosition(QTextCursor::End);
    if (!view->document()->isEmpty()) cursor.insertBlock();
    return cursor;
}

void scrollToEnd(QTextBrowser* view) {
    view->verticalScrollBar()->setValue(view->verticalScrollBar()->maximum());
}

QTextCharFormat formatOf(const QColor& colour, bool bold = false, bool italic = false) {
    QTextCharFormat format;
    if (colour.isValid()) format.setForeground(colour);
    if (bold) format.setFontWeight(QFont::Bold);
    format.setFontItalic(italic);
    return format;
}

}  // namespace

void AiAssistantPanel::appendUser(const QString& text) {
    QTextCursor cursor = endOf(m_transcript);
    cursor.insertText(tr("You"), formatOf({}, true));
    cursor.insertBlock();
    cursor.insertText(text, QTextCharFormat());
    scrollToEnd(m_transcript);
    updateState();
}

void AiAssistantPanel::appendAssistant(const QString& markdown) {
    QTextCursor cursor = endOf(m_transcript);
    cursor.insertText(tr("Assistant"), formatOf(QColor(74, 144, 217), true));
    cursor.insertBlock();
    cursor.setCharFormat(QTextCharFormat());
    // No HTML from a model: its text is Markdown, shown, never loaded.
    cursor.insertMarkdown(markdown,
                          QTextDocument::MarkdownFeatures(QTextDocument::MarkdownDialectGitHub |
                                                          QTextDocument::MarkdownNoHTML));
    scrollToEnd(m_transcript);
    updateState();
}

void AiAssistantPanel::appendToolCall(const QString& name, const QString& summary) {
    QTextCursor cursor = endOf(m_transcript);
    cursor.insertText(QStringLiteral("▸ ") + name +
                          (summary.isEmpty() ? QString() : QStringLiteral("  ") + summary),
                      formatOf(QColor(140, 140, 140)));
    scrollToEnd(m_transcript);
}

void AiAssistantPanel::appendToolResult(const QString& summary, bool error) {
    QTextCursor cursor = endOf(m_transcript);
    cursor.insertText((error ? QStringLiteral("    ✗ ") : QStringLiteral("    ✓ ")) + summary,
                      formatOf(error ? QColor(220, 100, 100) : QColor(140, 140, 140)));
    scrollToEnd(m_transcript);
}

void AiAssistantPanel::appendNote(const QString& text, bool error) {
    QTextCursor cursor = endOf(m_transcript);
    cursor.insertText(
        text, formatOf(error ? QColor(220, 100, 100) : QColor(140, 140, 140), false, !error));
    scrollToEnd(m_transcript);
    updateState();
}

}  // namespace hz::ui
