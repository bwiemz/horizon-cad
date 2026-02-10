#include <QApplication>
#include <QSurfaceFormat>

#include <spdlog/spdlog.h>

#include "horizon/ui/MainWindow.h"

int main(int argc, char* argv[]) {
    // Request an OpenGL 3.3 Core Profile context.
    QSurfaceFormat format;
    format.setVersion(3, 3);
    format.setProfile(QSurfaceFormat::CoreProfile);
    format.setDepthBufferSize(24);
    format.setSamples(4);
    QSurfaceFormat::setDefaultFormat(format);

    QApplication app(argc, argv);
    app.setApplicationName("Horizon CAD");
    app.setOrganizationName("Horizon CAD Project");
    app.setApplicationVersion("0.1.0");

    spdlog::info("Horizon CAD starting...");

    hz::ui::MainWindow window;
    window.show();

    return app.exec();
}
