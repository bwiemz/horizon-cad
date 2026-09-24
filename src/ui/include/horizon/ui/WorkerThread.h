#pragma once

#include <functional>
#include <thread>

namespace hz::ui {

/// Run @p work on a new thread, held in @p thread. When no thread can be had
/// (std::thread throws std::system_error when the system is out of threads
/// or memory), the work runs on the caller's thread instead. That is slower,
/// but the work gets done: a job whose worker never started would otherwise
/// be waited for forever, and every later one queued behind it.
void startWorker(std::thread& thread, const std::function<void()>& work);

}  // namespace hz::ui
