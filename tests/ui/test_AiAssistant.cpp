// The AI assistant in the window: its panel driven by a scripted transport,
// its settings and key store, its dialog, and the Qt transport against a
// local HTTP server. Runs on Qt's offscreen platform.

#include <gtest/gtest.h>

#include <QAction>
#include <QApplication>
#include <QComboBox>
#include <QElapsedTimer>
#include <QFile>
#include <QImage>
#include <QLineEdit>
#include <QMessageBox>
#include <QSettings>
#include <QStringList>
#include <QTabBar>
#include <QTcpServer>
#include <QTcpSocket>
#include <QTemporaryDir>
#include <QTextBrowser>
#include <QTextDocument>
#include <QTimer>
#include <QUrl>
#include <optional>

#include "../ai/FakeTransport.h"
#include "UiTestSupport.h"
#include "horizon/document/AssemblyDocument.h"
#include "horizon/document/Document.h"
#include "horizon/document/FeatureTree.h"
#include "horizon/document/UndoStack.h"
#include "horizon/drafting/DraftText.h"
#include "horizon/fileio/NativeFormat.h"
#include "horizon/modeling/Naming.h"
#include "horizon/topology/Solid.h"
#include "horizon/ui/AiAssistantPanel.h"
#include "horizon/ui/AiSettings.h"
#include "horizon/ui/AiSettingsDialog.h"
#include "horizon/ui/MainWindow.h"
#include "horizon/ui/QtHttpTransport.h"
#include "horizon/ui/ViewportWidget.h"

// LeakSanitizer is told of Qt's process-lifetime singletons once for all the
// window tests, in test_MainWindowDocuments.cpp: a second definition here
// failed the AddressSanitizer build's link ("multiple definition of
// __lsan_default_options").

using hz::ai::HttpResponse;
using hz::ai::ProviderKind;
using hz::ai::test::FakeTransport;
using hz::test::DialogResponder;
using hz::ui::AiAssistantPanel;
using hz::ui::AiKeyStore;
using hz::ui::AiSettings;
using hz::ui::MainWindow;
using nlohmann::json;

namespace {

/// A reply in OpenAI's format: one tool call.
HttpResponse toolCall(const std::string& id, const std::string& name, const json& args) {
    const json body = {{"choices",
                        {{{"message",
                           {{"role", "assistant"},
                            {"content", nullptr},
                            {"tool_calls",
                             {{{"id", id},
                               {"type", "function"},
                               {"function", {{"name", name}, {"arguments", args.dump()}}}}}}}},
                          {"finish_reason", "tool_calls"}}}},
                       {"usage", {{"prompt_tokens", 100}, {"completion_tokens", 20}}}};
    return {200, body.dump(), {}};
}

HttpResponse text(const std::string& words) {
    const json body = {
        {"choices",
         {{{"message", {{"role", "assistant"}, {"content", words}}}, {"finish_reason", "stop"}}}}};
    return {200, body.dump(), {}};
}

const json kBox = {{"shape", "box"}, {"size_x", 10}, {"size_y", 10}, {"size_z", 10}};

/// The panel of @p window, talking to @p transport, on a local model that
/// needs no key and no consent.
AiAssistantPanel* localPanel(MainWindow& window, FakeTransport& transport) {
    auto* panel = window.findChild<AiAssistantPanel*>(QStringLiteral("AiAssistantPanel"));
    EXPECT_NE(panel, nullptr);
    if (!panel) return nullptr;
    panel->setTransport(&transport);
    AiSettings settings = AiSettings::defaults();
    settings.provider = ProviderKind::OpenAICompatible;
    settings.configs[ProviderKind::OpenAICompatible].model = "test-model";
    panel->setSettings(settings);
    return panel;
}

/// Run the event loop until @p done or @p ms pass.
bool waitFor(const std::function<bool()>& done, int ms = 5000) {
    QElapsedTimer clock;
    clock.start();
    while (!done() && clock.elapsed() < ms) {
        QCoreApplication::processEvents(QEventLoop::AllEvents, 20);
    }
    return done();
}

}  // namespace

TEST(AiAssistantTest, BuildsAPartThroughTheWindowAndUndoesIt) {
    MainWindow window;
    FakeTransport transport;
    AiAssistantPanel* panel = localPanel(window, transport);
    ASSERT_NE(panel, nullptr);
    transport.replies.push_back(toolCall("c1", "add_primitive", kBox));
    transport.replies.push_back(text("A 10 mm cube."));

    ASSERT_TRUE(panel->submit(QStringLiteral("a 10 mm cube")));
    EXPECT_FALSE(panel->busy());
    hz::doc::Document* document = window.activeDocument();
    ASSERT_NE(document, nullptr);
    EXPECT_EQ(document->featureTree().featureCount(), 1u);
    ASSERT_NE(document->solid(), nullptr);
    EXPECT_EQ(document->type(), hz::doc::DocumentType::Part);
    const QString shown = panel->transcriptText();
    EXPECT_TRUE(shown.contains(QStringLiteral("a 10 mm cube"))) << shown.toStdString();
    EXPECT_TRUE(shown.contains(QStringLiteral("Added Primitive (box)"))) << shown.toStdString();
    EXPECT_TRUE(shown.contains(QStringLiteral("A 10 mm cube."))) << shown.toStdString();
    // The request reached a local server's chat API, with the tools.
    ASSERT_EQ(transport.requests.size(), 2u);
    EXPECT_EQ(transport.requests[0].url, "http://localhost:11434/v1/chat/completions");
    EXPECT_GE(transport.body(0)["tools"].size(), 25u);

    ASSERT_TRUE(panel->undoLastResponse());
    EXPECT_EQ(document->featureTree().featureCount(), 0u);
    EXPECT_EQ(document->type(), hz::doc::DocumentType::Drawing);
    EXPECT_FALSE(panel->undoLastResponse());  // once
}

TEST(AiAssistantTest, NeedsAKeyForACloudProvider) {
    qunsetenv("ANTHROPIC_API_KEY");
    AiKeyStore::saveKey(ProviderKind::Anthropic, "");
    MainWindow window;
    FakeTransport transport;
    auto* panel = window.findChild<AiAssistantPanel*>(QStringLiteral("AiAssistantPanel"));
    ASSERT_NE(panel, nullptr);
    panel->setTransport(&transport);
    panel->setSettings(AiSettings::defaults());  // Anthropic, no key
    EXPECT_FALSE(panel->submit(QStringLiteral("a box")));
    EXPECT_TRUE(transport.requests.empty());
    EXPECT_TRUE(panel->transcriptText().contains(QStringLiteral("API key")));
}

TEST(AiAssistantTest, AsksBeforeSendingToARemoteServer) {
    MainWindow window;
    FakeTransport transport;
    AiAssistantPanel* panel = localPanel(window, transport);
    ASSERT_NE(panel, nullptr);
    AiSettings settings = panel->settings();
    settings.configs[ProviderKind::OpenAICompatible].baseUrl = "http://models.example.com/v1";
    panel->setSettings(settings);
    panel->settings().setConsented(ProviderKind::OpenAICompatible, false);
    {
        DialogResponder no(QMessageBox::No);
        EXPECT_FALSE(panel->submit(QStringLiteral("hello")));
        EXPECT_TRUE(no.seen());
    }
    EXPECT_TRUE(transport.requests.empty());
    transport.replies.push_back(text("Hi."));
    {
        DialogResponder yes(QMessageBox::Yes);
        EXPECT_TRUE(panel->submit(QStringLiteral("hello")));
        EXPECT_TRUE(yes.seen());
    }
    EXPECT_EQ(transport.requests.size(), 1u);
    EXPECT_TRUE(panel->settings().consented(ProviderKind::OpenAICompatible));
    // Not asked again.
    transport.replies.push_back(text("Again."));
    EXPECT_TRUE(panel->submit(QStringLiteral("hello again")));
    // Another host is asked about again.
    settings.configs[ProviderKind::OpenAICompatible].baseUrl = "http://other.example.org/v1";
    panel->setSettings(settings);
    EXPECT_FALSE(panel->settings().consented(ProviderKind::OpenAICompatible));
    {
        DialogResponder no(QMessageBox::No);
        EXPECT_FALSE(panel->submit(QStringLiteral("hello there")));
        EXPECT_TRUE(no.seen());
    }
    panel->settings().setConsented(ProviderKind::OpenAICompatible, false);
}

TEST(AiSettingsTest, AnUnreadableKeyFileIsNotWrittenOver) {
    qunsetenv("GEMINI_API_KEY");
    qunsetenv("GOOGLE_API_KEY");
    ASSERT_TRUE(AiKeyStore::saveKey(ProviderKind::Gemini, "g-kept"));
    {
        QFile file(AiKeyStore::filePath());
        ASSERT_TRUE(file.open(QIODevice::WriteOnly));
        file.write("{ not json");
    }
    QString error;
    EXPECT_FALSE(AiKeyStore::saveKey(ProviderKind::OpenAI, "sk-new", &error));
    EXPECT_FALSE(error.isEmpty());
    QFile::remove(AiKeyStore::filePath());
}

TEST(AiAssistantTest, FollowsTheActiveTab) {
    MainWindow window;
    FakeTransport transport;
    AiAssistantPanel* panel = localPanel(window, transport);
    ASSERT_NE(panel, nullptr);
    hz::doc::Document* first = window.activeDocument();
    auto* newPart = window.findChild<QAction*>(QStringLiteral("action_new_part"));
    ASSERT_NE(newPart, nullptr);
    newPart->trigger();
    hz::doc::Document* second = window.activeDocument();
    ASSERT_NE(first, second);
    transport.replies.push_back(toolCall("c1", "add_primitive", kBox));
    transport.replies.push_back(text("Done."));
    ASSERT_TRUE(panel->submit(QStringLiteral("a box")));
    EXPECT_EQ(second->featureTree().featureCount(), 1u);
    EXPECT_EQ(first->featureTree().featureCount(), 0u);
}

// In an assembly's tab the assistant has the assembly tools, not the part's:
// it places a part, as one undo step "Undo this response" takes back.
TEST(AiAssistantTest, WorksOnAnAssembly) {
    QTemporaryDir dir;
    const QString block = dir.filePath(QStringLiteral("block.hzpart"));
    {
        hz::doc::Document part;
        part.setType(hz::doc::DocumentType::Part);
        part.featureTree().addFeature(hz::doc::PrimitiveFeature::makeBox(10, 10, 10));
        ASSERT_TRUE(part.rebuildModel());
        ASSERT_TRUE(hz::io::NativeFormat::save(block.toStdString(), part));
    }
    MainWindow window;
    FakeTransport transport;
    AiAssistantPanel* panel = localPanel(window, transport);
    ASSERT_NE(panel, nullptr);
    auto* newAssembly = window.findChild<QAction*>(QStringLiteral("action_new_assembly"));
    ASSERT_NE(newAssembly, nullptr);
    newAssembly->trigger();
    hz::doc::AssemblyDocument* assembly = window.activeAssembly();
    ASSERT_NE(assembly, nullptr);
    // Saved beside its part: the assistant inserts parts from its folder.
    assembly->setFilePath(dir.filePath(QStringLiteral("rig.hzasm")).toStdString());

    transport.replies.push_back(
        toolCall("c1", "insert_component",
                 {{"file", block.toStdString()}, {"name", "Block"}, {"position", {0, 0, 5}}}));
    transport.replies.push_back(text("Placed the block."));
    ASSERT_TRUE(panel->submit(QStringLiteral("put the block in")));
    ASSERT_EQ(assembly->components().size(), 1u) << panel->transcriptText().toStdString();
    EXPECT_EQ(assembly->components().front().name, "Block");
    EXPECT_DOUBLE_EQ(assembly->components().front().transform.at(2, 3), 5.0);

    std::vector<std::string> names;
    const json request = transport.body(0);  // held: the loop is over a part of it
    for (const auto& tool : request["tools"]) {
        names.push_back(tool["function"]["name"].get<std::string>());
    }
    const auto offered = [&names](const char* name) {
        return std::find(names.begin(), names.end(), name) != names.end();
    };
    EXPECT_TRUE(offered("add_mate"));
    EXPECT_TRUE(offered("get_selection"));
    EXPECT_FALSE(offered("add_primitive"));  // a part's, not an assembly's

    ASSERT_TRUE(panel->undoLastResponse());
    EXPECT_TRUE(assembly->components().empty());
}

// With pictures on, the model can look at the part, the picture is sent as
// the provider takes images, and the user sees it in the transcript. Off (a
// compatible server's default: its model may read no images), there is no
// such tool.
TEST(AiAssistantTest, MakesADrawingOfThePartBehindItsTab) {
    QTemporaryDir dir;
    ASSERT_TRUE(dir.isValid());
    const QString path = dir.filePath(QStringLiteral("plate.hzpart"));
    {
        hz::doc::Document part;
        part.setType(hz::doc::DocumentType::Part);
        part.featureTree().addFeature(hz::doc::PrimitiveFeature::makeBox(80, 40, 6));
        ASSERT_TRUE(part.rebuildModel());
        ASSERT_TRUE(hz::io::NativeFormat::save(path.toStdString(), part));
    }
    MainWindow window;
    ASSERT_TRUE(window.openPath(path));
    ASSERT_TRUE(waitFor([&window] { return !window.backgroundWorkRunning(); }, 10000));
    hz::doc::Document* part = window.activeDocument();
    ASSERT_NE(part, nullptr);
    FakeTransport transport;
    AiAssistantPanel* panel = localPanel(window, transport);
    ASSERT_NE(panel, nullptr);
    transport.replies.push_back(toolCall(
        "c1", "make_drawing", {{"title", "Mounting plate"}, {"paper", "a4"}, {"scale", "1:2"}}));
    transport.replies.push_back(text("The drawing is in a tab of its own."));

    ASSERT_TRUE(panel->submit(QStringLiteral("make a drawing of it")));
    EXPECT_FALSE(panel->busy());
    // Behind the part's tab: a switch would have stopped the request.
    EXPECT_EQ(window.activeDocument(), part);
    const QString shown = panel->transcriptText();
    EXPECT_FALSE(shown.contains(QStringLiteral("Stopped"))) << shown.toStdString();
    EXPECT_TRUE(shown.contains(QStringLiteral("The drawing is in a tab of its own.")))
        << shown.toStdString();
    // The model was told what was made.
    ASSERT_EQ(transport.requests.size(), 2u);
    // Held: a range over a member of a temporary would read freed memory.
    const json second = transport.body(1);
    std::string told;
    for (const json& message : second["messages"]) {
        if (message.value("role", "") == "tool") told = message.value("content", "");
    }
    EXPECT_NE(told.find("Drawing of plate"), std::string::npos) << told;
    EXPECT_NE(told.find("\"scale\":\"1:2\""), std::string::npos) << told;
    EXPECT_NE(told.find("isometric"), std::string::npos) << told;

    // The sheet: unsaved, its views drawn, its title block as asked.
    auto* tabs = window.findChild<QTabBar*>(QStringLiteral("documentTabs"));
    ASSERT_NE(tabs, nullptr);
    int sheetTab = -1;
    for (int i = 0; i < tabs->count(); ++i) {
        if (tabs->tabText(i) == QStringLiteral("Drawing of plate")) sheetTab = i;
    }
    ASSERT_GE(sheetTab, 0);
    tabs->setCurrentIndex(sheetTab);
    hz::doc::Document* sheet = window.activeDocument();
    ASSERT_NE(sheet, part);
    EXPECT_TRUE(sheet->isDirty());
    QStringList titleBlock;
    for (const auto& entity : sheet->draftDocument().entities()) {
        if (entity->layer() != "TitleBlock") continue;
        if (const auto* words = dynamic_cast<const hz::draft::DraftText*>(entity.get())) {
            titleBlock << QString::fromStdString(words->text());
        }
    }
    EXPECT_TRUE(titleBlock.contains(QStringLiteral("Mounting plate")))
        << titleBlock.join(QLatin1Char('|')).toStdString();
    EXPECT_GT(sheet->draftDocument().entities().size(), 20u);
}

TEST(AiAssistantTest, TheModelSeesWhatItMade) {
    MainWindow window;
    FakeTransport transport;
    AiAssistantPanel* panel = localPanel(window, transport);
    ASSERT_NE(panel, nullptr);
    const auto offered = [&transport](std::size_t request) {
        const json body = transport.body(request);  // held: the loop is over a part of it
        for (const auto& tool : body["tools"]) {
            if (tool["function"]["name"] == "get_view_image") return true;
        }
        return false;
    };
    transport.replies.push_back(text("Hello."));
    ASSERT_TRUE(panel->submit(QStringLiteral("hi")));
    EXPECT_FALSE(offered(0));

    AiSettings settings = panel->settings();
    settings.configs[ProviderKind::OpenAICompatible].images = true;
    panel->setSettings(settings);
    transport.replies.push_back(toolCall("c1", "add_primitive", kBox));
    transport.replies.push_back(toolCall("c2", "get_view_image", {{"view", "iso"}}));
    transport.replies.push_back(text("It is a cube."));
    ASSERT_TRUE(panel->submit(QStringLiteral("a cube, and check it")));
    ASSERT_EQ(transport.requests.size(), 4u);
    EXPECT_TRUE(offered(1));
    const QString shown = panel->transcriptText();
    EXPECT_TRUE(shown.contains(QStringLiteral("Looked at the iso view"))) << shown.toStdString();
    // Sent after the tool's result, as a compatible server takes an image.
    const json last = transport.body(3)["messages"].back();
    EXPECT_EQ(last["role"], "user");
    ASSERT_TRUE(last["content"].is_array());
    EXPECT_EQ(last["content"][1]["image_url"]["url"].get<std::string>().rfind(
                  "data:image/png;base64,iVBORw0KGgo", 0),
              0u);
    // And in the transcript, for the user.
    auto* transcript = panel->findChild<QTextBrowser*>();
    ASSERT_NE(transcript, nullptr);
    const QVariant picture =
        transcript->document()->resource(QTextDocument::ImageResource, QUrl("hzview://1"));
    ASSERT_TRUE(picture.isValid());
    EXPECT_LE(picture.value<QImage>().width(), 320);
    // A new conversation lets the pictures go with the text.
    panel->newConversation();
    EXPECT_FALSE(transcript->document()
                     ->resource(QTextDocument::ImageResource, QUrl("hzview://1"))
                     .isValid());
}

TEST(AiAssistantTest, StopLeavesThePanelReady) {
    MainWindow window;
    FakeTransport transport;
    AiAssistantPanel* panel = localPanel(window, transport);
    ASSERT_NE(panel, nullptr);
    transport.hold = true;
    transport.replies.push_back(text("late"));
    ASSERT_TRUE(panel->submit(QStringLiteral("think hard")));
    EXPECT_TRUE(panel->busy());
    panel->stop();
    EXPECT_FALSE(panel->busy());
    EXPECT_TRUE(panel->transcriptText().contains(QStringLiteral("Stopped")));
    EXPECT_EQ(transport.pending(), 0u);
}

TEST(AiAssistantTest, ChangingTheServerStopsARequest) {
    MainWindow window;
    FakeTransport transport;
    AiAssistantPanel* panel = localPanel(window, transport);
    ASSERT_NE(panel, nullptr);
    transport.hold = true;
    transport.replies.push_back(toolCall("c1", "add_primitive", kBox));
    ASSERT_TRUE(panel->submit(QStringLiteral("a box")));
    AiSettings moved = panel->settings();
    moved.configs[ProviderKind::OpenAICompatible].baseUrl = "http://elsewhere.example.com/v1";
    panel->setSettings(moved);
    EXPECT_FALSE(panel->busy());
    EXPECT_TRUE(panel->transcriptText().contains(QStringLiteral("Stopped")));
    EXPECT_EQ(transport.pending(), 0u);
}

TEST(AiAssistantTest, AFailedToolIsShownAndTheModelCarriesOn) {
    MainWindow window;
    FakeTransport transport;
    AiAssistantPanel* panel = localPanel(window, transport);
    ASSERT_NE(panel, nullptr);
    transport.replies.push_back(toolCall("c1", "fillet", {{"edges", {"nope"}}, {"radius", 1}}));
    transport.replies.push_back(text("There is no part to fillet yet."));
    ASSERT_TRUE(panel->submit(QStringLiteral("fillet it")));
    const QString shown = panel->transcriptText();
    EXPECT_TRUE(shown.contains(QStringLiteral("no solid"))) << shown.toStdString();
    EXPECT_TRUE(shown.contains(QStringLiteral("There is no part to fillet yet.")));
    // The error went back to the model as a tool result.
    EXPECT_EQ(transport.body(1)["messages"].back()["role"], "tool");
}

TEST(AiAssistantTest, TheModelCanAskWhatIsSelected) {
    MainWindow window;
    FakeTransport transport;
    AiAssistantPanel* panel = localPanel(window, transport);
    ASSERT_NE(panel, nullptr);
    transport.replies.push_back(toolCall("c1", "add_primitive", kBox));
    transport.replies.push_back(text("Box."));
    ASSERT_TRUE(panel->submit(QStringLiteral("a box")));
    const hz::topo::Solid* solid = window.activeDocument()->solid();
    ASSERT_NE(solid, nullptr);
    const std::string tag = solid->faces().front().topoId.tag();
    auto* viewport = window.findChild<hz::ui::ViewportWidget*>();
    ASSERT_NE(viewport, nullptr);
    viewport->chooseModel(hz::ui::ViewportWidget::ModelPick{0, tag, false}, false);

    transport.replies.push_back(toolCall("c2", "get_selection", json::object()));
    transport.replies.push_back(text("That face."));
    ASSERT_TRUE(panel->submit(QStringLiteral("what is this?")));
    const json sent = transport.body(transport.requests.size() - 1);
    const json result = json::parse(sent["messages"].back()["content"].get<std::string>());
    ASSERT_EQ(result["faces"].size(), 1u);
    EXPECT_EQ(result["faces"][0]["id"], hz::model::logicalFace(tag));
    EXPECT_TRUE(result["edges"].empty());
    EXPECT_TRUE(result.contains("drawing_entities"));
}

TEST(AiAssistantTest, TheMenuShowsThePanel) {
    MainWindow window;
    auto* panel = window.findChild<AiAssistantPanel*>(QStringLiteral("AiAssistantPanel"));
    ASSERT_NE(panel, nullptr);
    panel->hide();
    auto* action = window.findChild<QAction*>(QStringLiteral("action_assistant"));
    ASSERT_NE(action, nullptr);
    EXPECT_EQ(action->shortcut(), QKeySequence(Qt::CTRL | Qt::SHIFT | Qt::Key_A));
    action->trigger();
    EXPECT_FALSE(panel->isHidden());
}

// --- Settings and keys --------------------------------------------------------

TEST(AiSettingsTest, RoundTrip) {
    AiSettings settings = AiSettings::defaults();
    settings.provider = ProviderKind::Gemini;
    settings.maxSteps = 9;
    settings.configs[ProviderKind::Gemini].model = "gemini-test";
    settings.configs[ProviderKind::Anthropic].effort = "high";
    settings.configs[ProviderKind::Anthropic].serverFallback = false;
    settings.configs[ProviderKind::OpenAICompatible].baseUrl = "http://localhost:1234/v1";
    EXPECT_FALSE(settings.configs[ProviderKind::OpenAICompatible].images);  // may read none
    EXPECT_TRUE(settings.configs[ProviderKind::Gemini].images);
    settings.configs[ProviderKind::OpenAICompatible].images = true;
    settings.configs[ProviderKind::Gemini].images = false;
    settings.save();
    const AiSettings read = AiSettings::load();
    EXPECT_EQ(read.provider, ProviderKind::Gemini);
    EXPECT_EQ(read.maxSteps, 9);
    EXPECT_EQ(read.configs.at(ProviderKind::Gemini).model, "gemini-test");
    EXPECT_EQ(read.configs.at(ProviderKind::Anthropic).effort, "high");
    EXPECT_FALSE(read.configs.at(ProviderKind::Anthropic).serverFallback);
    EXPECT_EQ(read.configs.at(ProviderKind::OpenAICompatible).baseUrl, "http://localhost:1234/v1");
    EXPECT_TRUE(read.configs.at(ProviderKind::OpenAICompatible).images);
    EXPECT_FALSE(read.configs.at(ProviderKind::Gemini).images);
    EXPECT_FALSE(read.isRemote(ProviderKind::OpenAICompatible));
    EXPECT_TRUE(read.isRemote(ProviderKind::Gemini));
    AiSettings::defaults().save();
}

TEST(AiSettingsTest, KeysAreKeptApartAndOwnerOnly) {
    qunsetenv("OPENAI_API_KEY");
    ASSERT_TRUE(AiKeyStore::saveKey(ProviderKind::OpenAI, "sk-secret-123"));
    EXPECT_EQ(AiKeyStore::key(ProviderKind::OpenAI), "sk-secret-123");
    AiSettings::defaults().save();
    EXPECT_EQ(AiSettings::load().config(ProviderKind::OpenAI).apiKey, "sk-secret-123");
    // Never in QSettings.
    QSettings store;
    for (const QString& name : store.allKeys()) {
        EXPECT_FALSE(store.value(name).toString().contains(QStringLiteral("sk-secret")))
            << name.toStdString();
    }
#ifndef _WIN32
    const QFile::Permissions permissions = QFile::permissions(AiKeyStore::filePath());
    EXPECT_TRUE(permissions & QFileDevice::ReadOwner);
    EXPECT_FALSE(permissions & (QFileDevice::ReadGroup | QFileDevice::ReadOther));
#endif
    // The environment wins.
    qputenv("OPENAI_API_KEY", "sk-from-environment");
    EXPECT_EQ(AiKeyStore::key(ProviderKind::OpenAI), "sk-from-environment");
    qunsetenv("OPENAI_API_KEY");
    ASSERT_TRUE(AiKeyStore::saveKey(ProviderKind::OpenAI, ""));
    EXPECT_EQ(AiKeyStore::key(ProviderKind::OpenAI), "");
}

TEST(AiSettingsDialogTest, EachProviderKeepsItsEditsAndTypedKeys) {
    qunsetenv("ANTHROPIC_API_KEY");
    AiSettings settings = AiSettings::defaults();
    hz::ui::AiSettingsDialog dialog(settings);
    auto* provider = dialog.findChild<QComboBox*>(QStringLiteral("aiProvider"));
    auto* model = dialog.findChild<QComboBox*>(QStringLiteral("aiModel"));
    auto* key = dialog.findChild<QLineEdit*>(QStringLiteral("aiApiKey"));
    ASSERT_TRUE(provider && model && key);
    EXPECT_EQ(model->currentText(), QStringLiteral("claude-opus-5-5"));
    model->setEditText(QStringLiteral("claude-haiku-5-5"));
    key->setText(QStringLiteral("sk-ant-typed"));
    provider->setCurrentIndex(provider->findData(static_cast<int>(ProviderKind::Gemini)));
    EXPECT_EQ(model->currentText(), QStringLiteral("gemini-4-argon"));
    EXPECT_TRUE(key->text().isEmpty());
    provider->setCurrentIndex(provider->findData(static_cast<int>(ProviderKind::Anthropic)));
    EXPECT_EQ(model->currentText(), QStringLiteral("claude-haiku-5-5"));
    EXPECT_EQ(key->text(), QStringLiteral("sk-ant-typed"));
    dialog.accept();
    EXPECT_EQ(dialog.settings().configs.at(ProviderKind::Anthropic).model, "claude-haiku-5-5");
    EXPECT_EQ(dialog.typedKeys().at(ProviderKind::Anthropic), "sk-ant-typed");
}

TEST(AiSettingsDialogTest, AKeyTypedAndErasedIsNotSaved) {
    qunsetenv("OPENAI_API_KEY");
    hz::ui::AiSettingsDialog dialog(AiSettings::defaults());
    auto* provider = dialog.findChild<QComboBox*>(QStringLiteral("aiProvider"));
    auto* key = dialog.findChild<QLineEdit*>(QStringLiteral("aiApiKey"));
    ASSERT_TRUE(provider && key);
    provider->setCurrentIndex(provider->findData(static_cast<int>(ProviderKind::OpenAI)));
    key->setText(QStringLiteral("sk-oops"));
    provider->setCurrentIndex(provider->findData(static_cast<int>(ProviderKind::Gemini)));
    provider->setCurrentIndex(provider->findData(static_cast<int>(ProviderKind::OpenAI)));
    EXPECT_EQ(key->text(), QStringLiteral("sk-oops"));
    key->clear();
    dialog.accept();
    EXPECT_EQ(dialog.typedKeys().count(ProviderKind::OpenAI), 0u);
}

TEST(AiSettingsDialogTest, FetchesTheServersModels) {
    AiSettings settings = AiSettings::defaults();
    settings.provider = ProviderKind::OpenAICompatible;
    FakeTransport transport;  // outlives the dialog that uses it
    hz::ui::AiSettingsDialog dialog(settings);
    transport.replies.push_back(
        {200, json{{"data", {{{"id", "qwen3:8b"}}, {{"id", "llama3.1"}}}}}.dump(), {}});
    dialog.setTransport(&transport);
    auto* fetch = dialog.findChild<QPushButton*>(QStringLiteral("aiFetchModels"));
    auto* model = dialog.findChild<QComboBox*>(QStringLiteral("aiModel"));
    ASSERT_TRUE(fetch && model);
    fetch->click();
    ASSERT_EQ(transport.requests.size(), 1u);
    EXPECT_EQ(transport.requests[0].url, "http://localhost:11434/v1/models");
    EXPECT_EQ(model->count(), 2);
    EXPECT_EQ(model->currentText(), QStringLiteral("llama3.1"));
}

// --- The Qt transport ---------------------------------------------------------

namespace {

/// An HTTP server on a free local port that answers every request with
/// @p status and @p body, and keeps what it was sent.
class OneShotServer {
public:
    OneShotServer(int status, QByteArray body) : m_status(status), m_body(std::move(body)) {
        m_server.listen(QHostAddress::LocalHost, 0);
        QObject::connect(&m_server, &QTcpServer::newConnection, [this] {
            QTcpSocket* socket = m_server.nextPendingConnection();
            QObject::connect(socket, &QTcpSocket::readyRead, [this, socket] {
                received += socket->readAll();
                const qsizetype end = received.indexOf("\r\n\r\n");
                if (end < 0) return;
                const qsizetype lengthAt = received.toLower().indexOf("content-length:");
                qsizetype length = 0;
                if (lengthAt >= 0) {
                    const qsizetype lineEnd = received.indexOf("\r\n", lengthAt);
                    length =
                        received.mid(lengthAt + 15, lineEnd - lengthAt - 15).trimmed().toLongLong();
                }
                if (received.size() < end + 4 + length) return;
                if (m_holdReply) return;
                if (!rawReply.isEmpty()) {
                    socket->write(rawReply);
                    socket->disconnectFromHost();
                    return;
                }
                socket->write("HTTP/1.1 " + QByteArray::number(m_status) +
                              " OK\r\nContent-Type: application/json\r\nContent-Length: " +
                              QByteArray::number(m_body.size()) + "\r\nConnection: close\r\n\r\n" +
                              m_body);
                socket->disconnectFromHost();
            });
        });
    }
    QString url() const {
        return QStringLiteral("http://127.0.0.1:%1/v1/chat/completions").arg(m_server.serverPort());
    }
    void holdReply() { m_holdReply = true; }
    quint16 port() const { return m_server.serverPort(); }
    QByteArray received;
    QByteArray rawReply;  ///< sent as it is, when set

private:
    QTcpServer m_server;
    int m_status;
    QByteArray m_body;
    bool m_holdReply = false;
};

}  // namespace

TEST(QtHttpTransportTest, PostsAndReadsTheReply) {
    OneShotServer server(200, R"({"ok":true})");
    hz::ui::QtHttpTransport transport;
    hz::ai::HttpRequest request;
    request.url = server.url().toStdString();
    request.headers = {{"content-type", "application/json"}, {"x-test", "yes"}};
    request.body = R"({"hello":"world"})";
    std::optional<HttpResponse> reply;
    transport.send(request, [&reply](HttpResponse r) { reply = std::move(r); });
    ASSERT_TRUE(waitFor([&reply] { return reply.has_value(); }));
    EXPECT_EQ(reply->status, 200);
    EXPECT_EQ(reply->body, R"({"ok":true})");
    EXPECT_TRUE(reply->networkError.empty());
    EXPECT_TRUE(server.received.startsWith("POST /v1/chat/completions"));
    EXPECT_TRUE(server.received.toLower().contains("x-test: yes"));
    EXPECT_TRUE(server.received.endsWith(R"({"hello":"world"})"));
}

TEST(QtHttpTransportTest, ReportsAnHttpError) {
    OneShotServer server(401, R"({"error":{"message":"bad key"}})");
    hz::ui::QtHttpTransport transport;
    hz::ai::HttpRequest request;
    request.url = server.url().toStdString();
    std::optional<HttpResponse> reply;
    transport.send(request, [&reply](HttpResponse r) { reply = std::move(r); });
    ASSERT_TRUE(waitFor([&reply] { return reply.has_value(); }));
    EXPECT_EQ(reply->status, 401);
    EXPECT_NE(reply->body.find("bad key"), std::string::npos);
}

TEST(QtHttpTransportTest, ReportsConnectionRefused) {
    QTcpServer probe;
    ASSERT_TRUE(probe.listen(QHostAddress::LocalHost, 0));
    const quint16 port = probe.serverPort();
    probe.close();  // nothing listens there now
    hz::ui::QtHttpTransport transport;
    hz::ai::HttpRequest request;
    request.url = QStringLiteral("http://127.0.0.1:%1/v1/models").arg(port).toStdString();
    request.method = "GET";
    std::optional<HttpResponse> reply;
    transport.send(request, [&reply](HttpResponse r) { reply = std::move(r); });
    ASSERT_TRUE(waitFor([&reply] { return reply.has_value(); }));
    EXPECT_EQ(reply->status, 0);
    EXPECT_FALSE(reply->networkError.empty());
}

TEST(QtHttpTransportTest, ACancelledRequestNeverCallsBack) {
    OneShotServer server(200, R"({})");
    server.holdReply();
    hz::ui::QtHttpTransport transport;
    hz::ai::HttpRequest request;
    request.url = server.url().toStdString();
    bool called = false;
    const auto id = transport.send(request, [&called](HttpResponse) { called = true; });
    waitFor([&server] { return !server.received.isEmpty(); }, 2000);
    transport.cancel(id);
    waitFor([] { return false; }, 200);
    EXPECT_FALSE(called);

    // A refused URL is answered later, unless cancelled first.
    hz::ai::HttpRequest bad;
    bad.url = "ftp://example.com";
    bool refused = false;
    const auto badId = transport.send(bad, [&refused](HttpResponse) { refused = true; });
    transport.cancel(badId);
    waitFor([] { return false; }, 100);
    EXPECT_FALSE(refused);
    std::optional<HttpResponse> reply;
    transport.send(bad, [&reply](HttpResponse r) { reply = std::move(r); });
    ASSERT_TRUE(waitFor([&reply] { return reply.has_value(); }));
    EXPECT_NE(reply->networkError.find("http"), std::string::npos);
}

TEST(QtHttpTransportTest, NoKeyFollowsARedirectToAnotherHost) {
    OneShotServer elsewhere(200, R"({"stolen":true})");
    OneShotServer api(200, R"({})");
    api.rawReply =
        "HTTP/1.1 302 Found\r\nLocation: http://localhost:" + QByteArray::number(elsewhere.port()) +
        "/collect\r\nContent-Length: 0\r\nConnection: close\r\n\r\n";
    hz::ui::QtHttpTransport transport;
    hz::ai::HttpRequest request;
    request.url = api.url().toStdString();
    request.headers = {{"x-api-key", "sk-must-not-leak"}};
    std::optional<HttpResponse> reply;
    transport.send(request, [&reply](HttpResponse r) { reply = std::move(r); });
    ASSERT_TRUE(waitFor([&reply] { return reply.has_value(); }));
    waitFor([] { return false; }, 200);
    EXPECT_TRUE(elsewhere.received.isEmpty()) << elsewhere.received.toStdString();
    EXPECT_NE(reply->status, 200);
}
