#pragma once

#include <QString>
#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <optional>
#include <utility>
#include <vector>

namespace hz::doc {
class Command;
class Document;
class Feature;
}  // namespace hz::doc

namespace hz::ui {

/// A feature just added, whose build is yet to be seen (MainWindow::
/// addModelFeature): the step, as long as the history is where the add left
/// it.
struct PendingAdd {
    std::weak_ptr<doc::Document> document;
    const doc::Command* step = nullptr;
    const doc::Feature* feature = nullptr;
    std::uint64_t history = 0;  ///< the undo revision the add left
    QString verb;
};

/// The pending adds, one for each document: an add in one tab while
/// another's build still runs does not take the other's place. (One for the
/// window let it, and the other's feature, failing itself, stayed.)
class PendingAdds {
public:
    /// Keep @p add, in place of its document's last one, which its push has
    /// settled; those of documents since closed go.
    void put(PendingAdd add) {
        const auto document = add.document.lock();
        std::erase_if(m_adds, [&document](const PendingAdd& p) {
            const auto held = p.document.lock();
            return !held || held == document;
        });
        m_adds.push_back(std::move(add));
    }

    /// The one for @p document, taken out; none if nothing added to it waits.
    std::optional<PendingAdd> take(const doc::Document& document) {
        const auto it = std::find_if(
            m_adds.begin(), m_adds.end(),
            [&document](const PendingAdd& p) { return p.document.lock().get() == &document; });
        if (it == m_adds.end()) return std::nullopt;
        PendingAdd add = std::move(*it);
        m_adds.erase(it);
        return add;
    }

    std::size_t size() const { return m_adds.size(); }

private:
    std::vector<PendingAdd> m_adds;
};

}  // namespace hz::ui
