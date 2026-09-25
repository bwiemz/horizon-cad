#pragma once

#include <QObject>
#include <QString>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "horizon/document/AssemblyDocument.h"
#include "horizon/fileio/StepFormat.h"
#include "horizon/ui/BackgroundTask.h"

class QComboBox;

namespace hz::ui {

class AssemblyTreePanel;
class FeatureForm;
class WorkbenchHost;

/// The assembly commands (Phase 146): placing components, mates,
/// interference, the bill of materials, and keeping assemblies up to date
/// with their parts. They were a thousand lines of MainWindow; here they
/// reach the window through WorkbenchHost alone. The window builds the
/// Assembly menu and the assembly tree's dock, and calls these.
///
/// Each command acts on the active tab's assembly, and says so in the status
/// bar when the tab shows none. Every edit is one undo step, with the mates
/// solved again after it.
class AssemblyWorkbench : public QObject {
    Q_OBJECT

public:
    /// @p tree is the assembly tree's dock: its requests are wired to these
    /// commands, and it follows the view's selection.
    AssemblyWorkbench(WorkbenchHost& host, AssemblyTreePanel& tree, QObject* parent = nullptr);
    /// Stops an interference check still running.
    ~AssemblyWorkbench() override;

    AssemblyWorkbench(const AssemblyWorkbench&) = delete;
    AssemblyWorkbench& operator=(const AssemblyWorkbench&) = delete;

    /// An interference check of at least this many faces runs on a worker in
    /// the Auto rebuild mode.
    static constexpr std::size_t kWorkerInterferenceFaces = 2000;

    // --- The Assembly menu ---
    void onInsertComponent();
    /// On the component clicked or current in the tree, or chosen in a form.
    void onMoveComponent();
    void onRotateComponent();
    void onRemoveComponent();
    void onSuppressComponent();
    void onRenameComponent();
    void onOpenPart();
    void onAddMate();
    void onEditMate();
    void onRemoveMate();
    void onCheckInterference();

    /// What a STEP export of the active assembly writes (Phase 153): each
    /// part once, from its file, and each unsuppressed component placed; the
    /// components whose part could not be read, by name. The parts'
    /// solids are the resolved components' own: valid while the assembly
    /// is not changed.
    struct StepExport {
        std::vector<io::StepWritePart> parts;
        std::vector<io::StepOccurrence> occurrences;
        std::vector<std::string> unread;
    };
    StepExport stepExport();
    void onBillOfMaterials();

    // --- For the window ---

    /// Place @p assembly by its mates. A failure is said in the status bar,
    /// and a success too unless @p reportSuccess is false.
    bool solveAssemblyMates(doc::AssemblyDocument& assembly, bool reportSuccess = true);
    /// Place @p assembly by its mates as it opens; whether that moved any
    /// component (the assembly is then modified: the placing can be saved).
    bool placeOnOpen(doc::AssemblyDocument& assembly);
    /// Show the part at @p path anew wherever a component places it: its
    /// components' meshes and documents read again, their mates solved again,
    /// and the scene rebuilt if the active assembly is one of them. Called
    /// when a part is saved here, found changed on disk, or its tab closed
    /// with its edits discarded. @p report: say so in the status bar (not
    /// over a message about the part's tab that matters more).
    void refreshComponentsOf(const std::string& path, bool report = true);

    /// An interference check is running on a worker.
    bool busy() const { return m_interferenceTask != nullptr; }
    /// Stop it; it reports "cancelled" when it ends.
    void cancelWork();

    /// The folder @p assembly's relative part paths are in: its file's; none
    /// while it is unsaved.
    static std::string assemblyDir(const doc::AssemblyDocument& assembly);
    /// The file @p comp places, found as DocumentManager::resolveComponent
    /// finds it.
    static std::string partFile(const doc::ComponentInstance& comp, const std::string& dir);

private slots:
    void onInterferenceFinished();

private:
    /// The active tab's assembly, or null.
    doc::AssemblyDocument* assembly() const;

    /// Record the change from @p before as one undo step on the assembly's
    /// backing document.
    void recordAssemblyEdit(doc::AssemblyState before, bool wasDirty, const QString& description);
    /// Change the assembly by @p edit, as one undo step named @p verb: its
    /// mates are solved again after it, and if they cannot be, or @p edit
    /// declines (returns false), the assembly is left as it was.
    bool editAssembly(const QString& verb, const std::function<bool()>& edit);
    /// The component a command acts on: the first clicked, else the one
    /// current in the assembly tree; 0 when neither.
    uint64_t targetComponent() const;
    /// A "component" choice on @p form, @p target chosen; @p ids its rows'.
    QComboBox* componentChoice(FeatureForm& form, uint64_t target,
                               std::vector<uint64_t>& ids) const;
    /// A "mate" choice on @p form, @p target chosen; @p ids its rows'.
    QComboBox* mateChoice(FeatureForm& form, uint64_t target, std::vector<uint64_t>& ids) const;
    void removeComponent(uint64_t id);
    void setComponentSuppressed(uint64_t id, bool suppressed);
    void renameComponent(uint64_t id);
    void editMate(uint64_t id);
    void removeMate(uint64_t id);
    /// Open the part component @p id places, in its tab.
    void openComponentPart(uint64_t id);
    void showInterference(const doc::AssemblyDocument& assembly,
                          const doc::InterferenceReport& report);

    WorkbenchHost& m_host;
    AssemblyTreePanel& m_tree;
    std::unique_ptr<BackgroundTask<doc::InterferenceReport>> m_interferenceTask;
    std::shared_ptr<doc::AssemblyDocument> m_interferenceAssembly;  ///< kept alive for it
};

}  // namespace hz::ui
