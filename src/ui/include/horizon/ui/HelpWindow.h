#pragma once

#include <QDialog>
#include <QString>
#include <QStringList>
#include <QUrl>

class QTextBrowser;

namespace hz::ui {

/// Help ▸ User Guide (Phase 168): the guide's pages, Markdown compiled into
/// the application from docs/guide and shown by Qt's own Markdown reader.
/// Pages link to one another; a link to the web opens in the browser. It is
/// not modal: it stays open beside the window while the user works.
class HelpWindow : public QDialog {
    Q_OBJECT

public:
    explicit HelpWindow(QWidget* parent = nullptr);

    /// Show @p page ("parts.md"); the contents when it is empty or not a
    /// page of the guide.
    void showPage(const QString& page);

    /// The page shown ("parts.md").
    QString currentPage() const;

    /// Every page of the guide, by file name ("index.md", ...).
    static QStringList pages();

    /// Where @p page is: "qrc:/guide/<page>".
    static QUrl pageUrl(const QString& page);

    /// @p page's title: its first "# " heading; empty when it has none.
    static QString pageTitle(const QString& page);

private:
    QTextBrowser* m_browser;
};

}  // namespace hz::ui
