#include "horizon/ui/TypedUnits.h"

#include <Qt>

namespace hz::ui {

bool typeUnitKey(int key, std::string& text) {
    if (text.empty()) return false;
    if (key >= Qt::Key_A && key <= Qt::Key_Z) {
        text += static_cast<char>('a' + (key - Qt::Key_A));
    } else if (key == Qt::Key_Space) {
        text += ' ';
    } else if (key == Qt::Key_Apostrophe) {
        text += '\'';
    } else if (key == Qt::Key_QuoteDbl) {
        text += '"';
    } else if (key == Qt::Key_Slash) {
        text += '/';
    } else if (key == Qt::Key_degree) {
        text += "\xC2\xB0";
    } else {
        return false;
    }
    return true;
}

void takeBack(std::string& text) {
    // Continuation bytes (10xxxxxx) go with the character they end.
    while (!text.empty() && (static_cast<unsigned char>(text.back()) & 0xC0U) == 0x80U) {
        text.pop_back();
    }
    if (!text.empty()) text.pop_back();
}

}  // namespace hz::ui
