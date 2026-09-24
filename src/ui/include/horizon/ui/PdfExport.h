#pragma once

#include <QString>

#include "horizon/drafting/PlotScene.h"

namespace hz::ui {

/// Write @p scene to a one-page vector PDF of the layout's paper size: line
/// weights in millimetres, dash patterns, and text at its height (see
/// io::SvgExport, which draws the same). The file is replaced atomically.
/// False, with @p error saying why, when it cannot be written.
bool exportPdf(const QString& path, const draft::PlotScene& scene, const draft::PlotLayout& layout,
               QString* error = nullptr);

}  // namespace hz::ui
