// A viewport that cannot draw says so instead of staying silently blank.

#include <gtest/gtest.h>

#include <QMessageBox>

#include "UiTestSupport.h"
#include "horizon/document/Document.h"
#include "horizon/ui/ViewportWidget.h"

using hz::test::DialogResponder;

TEST(ViewportGraphicsTest, AViewportThatCannotDrawTellsTheUser) {
    hz::doc::Document doc;
    hz::ui::ViewportWidget viewport;
    viewport.setDocument(&doc);
    viewport.resize(320, 240);

    DialogResponder responder(QMessageBox::Ok, QStringLiteral("Graphics Problem"), 5000);
    viewport.show();
    responder.waitForDialog(4000);

    if (viewport.isValid() && viewport.graphicsProblem().isEmpty()) {
        // A platform with working OpenGL (e.g. run under a real display).
        EXPECT_FALSE(responder.seen()) << "a working viewport is not reported";
    } else {
        // Qt's offscreen platform has no OpenGL: the viewport can never paint.
        EXPECT_TRUE(responder.seen()) << "a blank viewport must be explained";
        EXPECT_FALSE(viewport.graphicsProblem().isEmpty());
    }
}
