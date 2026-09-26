#include "horizon/ui/HelpWindow.h"

#include <QAction>
#include <QDir>
#include <QFile>
#include <QFileInfo>
#include <QKeySequence>
#include <QTextBrowser>
#include <QTextDocument>
#include <QToolBar>
#include <QVBoxLayout>

namespace hz::ui {

namespace {

const QString kContents = QStringLiteral("index.md");

}  // namespace

HelpWindow::HelpWindow(QWidget* parent) : QDialog(parent), m_browser(new QTextBrowser(this)) {
    setObjectName(QStringLiteral("helpWindow"));
    setWindowTitle(tr("Horizon CAD User Guide"));
    resize(780, 680);

    m_browser->setObjectName(QStringLiteral("helpBrowser"));
    m_browser->setAccessibleName(tr("User guide"));
    // A link to the web opens in the browser; a page's links to the guide's
    // other pages open here.
    m_browser->setOpenExternalLinks(true);

    auto* bar = new QToolBar(this);
    bar->setObjectName(QStringLiteral("helpToolBar"));
    QAction* back = bar->addAction(tr("Back"), m_browser, &QTextBrowser::backward);
    back->setShortcut(QKeySequence::Back);
    back->setEnabled(false);
    QAction* forward = bar->addAction(tr("Forward"), m_browser, &QTextBrowser::forward);
    forward->setShortcut(QKeySequence::Forward);
    forward->setEnabled(false);
    bar->addAction(tr("Contents"), this, [this] { showPage(kContents); });
    connect(m_browser, &QTextBrowser::backwardAvailable, back, &QAction::setEnabled);
    connect(m_browser, &QTextBrowser::forwardAvailable, forward, &QAction::setEnabled);
    // From the signal's page: source() is still the page before when it comes.
    // The contents is the guide itself; another page is named in its title.
    connect(m_browser, &QTextBrowser::sourceChanged, this, [this](const QUrl& url) {
        const QString page = QFileInfo(url.path()).fileName();
        const QString title = pageTitle(page);
        setWindowTitle(page == kContents || title.isEmpty()
                           ? tr("Horizon CAD User Guide")
                           : tr("%1 - Horizon CAD User Guide").arg(title));
    });

    auto* layout = new QVBoxLayout(this);
    layout->setContentsMargins(0, 0, 0, 0);
    layout->setSpacing(0);
    layout->addWidget(bar);
    layout->addWidget(m_browser);

    showPage(kContents);
}

void HelpWindow::showPage(const QString& page) {
    const QString shown = pages().contains(page) ? page : kContents;
    m_browser->setSource(pageUrl(shown), QTextDocument::MarkdownResource);
}

QString HelpWindow::currentPage() const {
    return QFileInfo(m_browser->source().path()).fileName();
}

QStringList HelpWindow::pages() {
    return QDir(QStringLiteral(":/guide"))
        .entryList({QStringLiteral("*.md")}, QDir::Files, QDir::Name);
}

QUrl HelpWindow::pageUrl(const QString& page) {
    return QUrl(QStringLiteral("qrc:/guide/") + page);
}

QString HelpWindow::pageTitle(const QString& page) {
    QFile file(QStringLiteral(":/guide/") + page);
    if (!file.open(QIODevice::ReadOnly | QIODevice::Text)) return {};
    while (!file.atEnd()) {
        const QString line = QString::fromUtf8(file.readLine()).trimmed();
        if (line.startsWith(QLatin1String("# "))) return line.mid(2).trimmed();
    }
    return {};
}

}  // namespace hz::ui
