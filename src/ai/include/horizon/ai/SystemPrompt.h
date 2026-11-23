#pragma once

#include <string>

namespace hz::ai {

/// What the assistant is told about Horizon CAD and how to work in it. Fixed
/// text (no date, no document state), so a provider can cache it.
std::string defaultSystemPrompt();

}  // namespace hz::ai
