#pragma once

#include <string>

namespace hz::ui {

/// What a unit is written with, typed after a number (Phase 154): once
/// something is typed in @p text, a letter ("in", "mm", "rad"), a space,
/// ' and " (feet and inches), / (3/4) or the degree sign, appended for the
/// Qt::Key @p key. Nothing before a number: a letter first is still the
/// view's, a shortcut. True when @p key was one of these.
bool typeUnitKey(int key, std::string& text);

/// Take back the last character typed in @p text, all of its bytes (the
/// degree sign is two).
void takeBack(std::string& text);

}  // namespace hz::ui
