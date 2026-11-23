#pragma once

#include <QDockWidget>
#include <functional>
#include <memory>
#include <nlohmann/json.hpp>

#include "horizon/ai/ToolRegistry.h"
#include "horizon/ui/AiSettings.h"

class QLabel;
class QPlainTextEdit;
class QProgressBar;
class QPushButton;
class QTextBrowser;
class QToolButton;

namespace hz::ai {
class AgentSession;
class HttpTransport;
}  // namespace hz::ai

namespace hz::doc {
class AssemblyDocument;
class Document;
class DocumentManager;
}  // namespace hz::doc

namespace hz::ui {

class QtHttpTransport;

/// The AI assistant: a chat with a model of the user's choice (Claude, GPT,
/// Gemini, or one on their own computer) that builds and changes the active
/// part or drawing through the CAD tools (hz::ai::registerCadTools). Every
/// change is an ordinary step of the document's undo stack; "Undo This
/// Response" takes back all of one request's at once.
class AiAssistantPanel : public QDockWidget {
    Q_OBJECT
public:
    explicit AiAssistantPanel(QWidget* parent = nullptr);
    ~AiAssistantPanel() override;

    /// What the window does for the panel: show a changed part (already
    /// rebuilt) or drawing, and undo one step as its Undo does.
    struct Host {
        std::function<void()> modelChanged;
        std::function<void()> drawingChanged;
        std::function<void()> undo;
        /// What the user has selected, as the tools name it (get_selection):
        /// {"faces": [...], "edges": [...], "drawing_entities": [...]}. Unset:
        /// no such tool.
        std::function<nlohmann::json()> selection;
        /// An assembly tool changed the assembly (its mates solved): the
        /// view and the assembly tree are shown again.
        std::function<void()> assemblyChanged;
        /// An assembly tool is about to change the assembly: a drag in the
        /// view is put back first.
        std::function<void()> beforeAssemblyChange;
    };
    void setHost(Host host) { m_host = std::move(host); }

    /// Work on @p document, called @p name; null (an assembly, say) for none,
    /// with @p why shown. A request under way is stopped.
    void setDocument(std::shared_ptr<doc::Document> document, const QString& name,
                     const QString& why = {});
    /// Work on the assembly @p assembly, called @p name: its tab's document
    /// @p backing carries its undo steps, and @p documents reads its parts.
    /// The assembly tools take the part and drawing tools' place.
    void setAssembly(std::shared_ptr<doc::Document> backing,
                     std::shared_ptr<doc::AssemblyDocument> assembly,
                     doc::DocumentManager& documents, const QString& name);

    /// Talk through @p transport (a test's) instead of the network. Starts a
    /// new conversation.
    void setTransport(ai::HttpTransport* transport);
    void setSettings(const AiSettings& settings);
    const AiSettings& settings() const { return m_settings; }

    /// Send @p text as the user's request. False when it was not sent (busy,
    /// no document, no key, no consent).
    bool submit(const QString& text);
    void stop();
    bool undoLastResponse();
    void newConversation();
    bool busy() const;
    /// The conversation as plain text, for tests and accessibility.
    QString transcriptText() const;
    /// Put the keyboard focus in the request box.
    void focusInput();
    /// The document changed (its undo stack moved): bring the buttons up to date.
    void refreshState() { updateState(); }

signals:
    void busyChanged(bool busy);

protected:
    bool eventFilter(QObject* watched, QEvent* event) override;

private:
    class Observer;
    friend class Observer;

    void openSettings();
    /// What setDocument and setAssembly share: @p assembly null for a part
    /// or a drawing.
    void bind(std::shared_ptr<doc::Document> document,
              std::shared_ptr<doc::AssemblyDocument> assembly, doc::DocumentManager* documents,
              const QString& name, const QString& why);
    void rebindTools();
    void applyConfig();
    void updateState();
    void updateHeader();
    bool askConsent();

    void appendUser(const QString& text);
    void appendAssistant(const QString& markdown);
    void appendToolCall(const QString& name, const QString& summary);
    void appendToolResult(const QString& summary, bool error);
    void appendNote(const QString& text, bool error = false);

    Host m_host;
    AiSettings m_settings;
    std::shared_ptr<doc::Document> m_document;
    /// The assembly worked on, when the tab shows one (m_document then its
    /// tab's document), and what reads its parts.
    std::shared_ptr<doc::AssemblyDocument> m_assembly;
    doc::DocumentManager* m_documents = nullptr;
    /// The document the conversation last worked on: compared, never used.
    const doc::Document* m_conversationDocument = nullptr;
    QString m_documentName;
    QString m_unavailable;

    std::unique_ptr<QtHttpTransport> m_network;
    ai::HttpTransport* m_transport = nullptr;
    ai::ToolRegistry m_tools;
    std::unique_ptr<Observer> m_observer;
    std::unique_ptr<ai::AgentSession> m_session;  ///< last: it holds the others

    QLabel* m_header = nullptr;
    QToolButton* m_settingsButton = nullptr;
    QToolButton* m_newButton = nullptr;
    QTextBrowser* m_transcript = nullptr;
    QLabel* m_emptyHint = nullptr;
    QProgressBar* m_busy = nullptr;
    QLabel* m_status = nullptr;
    QPlainTextEdit* m_input = nullptr;
    QPushButton* m_send = nullptr;
    QPushButton* m_stop = nullptr;
    QPushButton* m_undo = nullptr;
};

}  // namespace hz::ui
