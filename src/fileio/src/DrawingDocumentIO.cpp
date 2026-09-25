#include "horizon/fileio/DrawingDocumentIO.h"

#include <cmath>
#include <filesystem>
#include <fstream>
#include <initializer_list>
#include <nlohmann/json.hpp>
#include <string>

#include "horizon/document/Document.h"
#include "horizon/fileio/AtomicFile.h"
#include "horizon/fileio/NativeFormat.h"
#include "horizon/modeling/SectionView.h"
#include "horizon/topology/Solid.h"

namespace hz::io {

using json = nlohmann::json;
namespace fs = std::filesystem;

namespace {

constexpr int kVersion = 3;
constexpr double kHatchSpacing = 3.0;  ///< sheet millimetres between a section's hatch lines

// Each field read by its type, defaulted when missing or of another type: a
// field of the wrong type in a hostile file threw from json::value().
double number(const json& j, const char* key, double fallback) {
    const auto it = j.find(key);
    return it != j.end() && it->is_number() ? it->get<double>() : fallback;
}

std::string text(const json& j, const char* key, const std::string& fallback = {}) {
    const auto it = j.find(key);
    return it != j.end() && it->is_string() ? it->get<std::string>() : fallback;
}

bool flag(const json& j, const char* key, bool fallback) {
    const auto it = j.find(key);
    return it != j.end() && it->is_boolean() ? it->get<bool>() : fallback;
}

json vec3(const math::Vec3& v) {
    return json::array({v.x, v.y, v.z});
}

math::Vec3 vec3(const json& j, const char* key, const math::Vec3& fallback) {
    const auto it = j.find(key);
    if (it == j.end() || !it->is_array() || it->size() != 3) return fallback;
    for (const auto& c : *it) {
        if (!c.is_number() || !std::isfinite(c.get<double>())) return fallback;
    }
    return {(*it)[0].get<double>(), (*it)[1].get<double>(), (*it)[2].get<double>()};
}

const char* viewName(model::StandardView view) {
    switch (view) {
        case model::StandardView::Front:
            return "front";
        case model::StandardView::Top:
            return "top";
        case model::StandardView::Right:
            return "right";
        case model::StandardView::Isometric:
            return "isometric";
    }
    return "front";
}

const char* roleName(model::ViewRole role) {
    switch (role) {
        case model::ViewRole::Section:
            return "section";
        case model::ViewRole::Detail:
            return "detail";
        case model::ViewRole::Projection:
            break;
    }
    return "projection";
}

model::ViewRole roleNamed(const std::string& name) {
    if (name == "section") return model::ViewRole::Section;
    if (name == "detail") return model::ViewRole::Detail;
    return model::ViewRole::Projection;
}

/// A label as a caption can show it: a few characters, no line breaks.
std::string usableLabel(std::string label) {
    std::erase_if(label, [](char c) { return c == '\n' || c == '\r'; });
    constexpr std::size_t kMaxLabel = 8;
    if (label.size() > kMaxLabel) label.resize(kMaxLabel);
    return label;
}

model::StandardView viewNamed(const std::string& name) {
    for (const auto view : {model::StandardView::Front, model::StandardView::Top,
                            model::StandardView::Right, model::StandardView::Isometric}) {
        if (name == viewName(view)) return view;
    }
    return model::StandardView::Front;
}

model::PaperSize paperNamed(const std::string& name, model::PaperSize fallback) {
    for (const auto size :
         {model::PaperSize::A0, model::PaperSize::A1, model::PaperSize::A2, model::PaperSize::A3,
          model::PaperSize::A4, model::PaperSize::AnsiA, model::PaperSize::AnsiB,
          model::PaperSize::AnsiC, model::PaperSize::AnsiD}) {
        if (name == model::paperSizeName(size)) return size;
    }
    return fallback;
}

/// A scale a sheet can be drawn at: positive and finite.
bool usableScale(double s) {
    return std::isfinite(s) && s > 1e-6 && s < 1e6;
}

}  // namespace

bool DrawingDocumentIO::save(const std::string& path, const DrawingDocumentSpec& spec) {
    json root;
    root["format"] = "hzdwg";
    root["version"] = kVersion;

    // The part beside or below the drawing is written relative to it, so a
    // folder of both can move.
    std::string part = spec.partPath;
    std::error_code ec;
    const fs::path folder = fs::absolute(pathFromUtf8(path), ec).parent_path();
    if (!ec && !part.empty()) {
        const fs::path absolutePart = fs::absolute(pathFromUtf8(part), ec);
        if (!ec) {
            // Compared as a generic (narrow, '/') string: native() is a wide
            // string on Windows, and a narrow ".." does not compare with it.
            const std::string relative = absolutePart.lexically_relative(folder).generic_string();
            if (!relative.empty() && relative.rfind("..", 0) != 0) part = relative;
        }
    }
    root["part"] = part;
    root["gap"] = spec.gap;
    root["scale"] = spec.scale;

    json sheet;
    sheet["paper"] = model::paperSizeName(spec.sheet.size);
    sheet["orientation"] =
        spec.sheet.orientation == model::Orientation::Landscape ? "landscape" : "portrait";
    sheet["margin"] = spec.sheet.margin;
    root["sheet"] = sheet;

    const model::TitleBlock& tb = spec.titleBlock;
    root["titleBlock"] = {{"title", tb.title},
                          {"partNumber", tb.partNumber},
                          {"revision", tb.revision},
                          {"drawnBy", tb.drawnBy},
                          {"date", tb.date},
                          {"scale", tb.scale},
                          {"sheetNumber", tb.sheetNumber},
                          {"material", tb.material},
                          {"company", tb.company},
                          {"width", tb.width},
                          {"height", tb.height}};

    json views = json::array();
    for (const DrawingViewSpec& v : spec.views) {
        views.push_back({{"kind", viewName(v.kind)},
                         {"direction", vec3(v.projection.dir)},
                         {"up", vec3(v.projection.up)},
                         {"origin", vec3(v.projection.origin)},
                         {"scale", v.scale},
                         {"placement", json::array({v.placement.x, v.placement.y})},
                         {"hidden", v.showHidden},
                         {"tangent", v.showTangentEdges},
                         {"role", roleName(v.role)},
                         {"label", v.label},
                         {"source", v.source},
                         {"detailCenter", json::array({v.detailCenter.x, v.detailCenter.y})},
                         {"detailRadius", v.detailRadius}});
    }
    root["views"] = views;

    return writeFileAtomically(pathFromUtf8(path),
                               root.dump(2, ' ', false, json::error_handler_t::replace));
}

bool DrawingDocumentIO::readSpec(const std::string& path, DrawingDocumentSpec& outSpec,
                                 std::string* error) {
    const auto fail = [error](const std::string& why) {
        if (error != nullptr) *error = why;
        return false;
    };
    std::ifstream in(pathFromUtf8(path));
    if (!in) return fail("the file could not be opened");

    json root;
    try {
        in >> root;
    } catch (const json::exception&) {
        return fail("the file is not a drawing: it is not JSON");
    }
    if (!root.is_object()) return fail("the file is not a drawing");

    DrawingDocumentSpec spec;
    spec.partPath = text(root, "part");
    spec.gap = number(root, "gap", 10.0);
    if (!std::isfinite(spec.gap) || spec.gap < 0.0) spec.gap = 10.0;
    if (spec.partPath.empty()) return fail("the drawing names no part");
    // A relative part is the drawing folder's, not the working folder's.
    fs::path part = pathFromUtf8(spec.partPath);
    if (part.is_relative()) {
        std::error_code ec;
        part = (fs::absolute(pathFromUtf8(path), ec).parent_path() / part).lexically_normal();
        spec.partPath = part.string();
    }

    // Compared as a number: a version of 1e300 cast to int was undefined.
    const double versionNumber = number(root, "version", 1.0);
    const int version = !std::isfinite(versionNumber) || versionNumber < 2.0 ? 1
                        : versionNumber < 3.0                                ? 2
                                                                             : 3;
    if (version >= 2) {
        const double scale = number(root, "scale", 0.0);
        spec.scale = usableScale(scale) ? scale : 0.0;
        const auto sheet = root.find("sheet");
        if (sheet != root.end() && sheet->is_object()) {
            spec.sheet.size = paperNamed(text(*sheet, "paper"), spec.sheet.size);
            spec.sheet.orientation = text(*sheet, "orientation") == "portrait"
                                         ? model::Orientation::Portrait
                                         : model::Orientation::Landscape;
            const double margin = number(*sheet, "margin", spec.sheet.margin);
            if (std::isfinite(margin) && margin >= 0.0) spec.sheet.margin = margin;
        }
        const auto tb = root.find("titleBlock");
        if (tb != root.end() && tb->is_object()) {
            model::TitleBlock& t = spec.titleBlock;
            t.title = text(*tb, "title", t.title);
            t.partNumber = text(*tb, "partNumber", t.partNumber);
            t.revision = text(*tb, "revision", t.revision);
            t.drawnBy = text(*tb, "drawnBy", t.drawnBy);
            t.date = text(*tb, "date", t.date);
            t.scale = text(*tb, "scale", t.scale);
            t.sheetNumber = text(*tb, "sheetNumber", t.sheetNumber);
            t.material = text(*tb, "material", t.material);
            t.company = text(*tb, "company", t.company);
            const double w = number(*tb, "width", t.width);
            const double h = number(*tb, "height", t.height);
            if (std::isfinite(w) && w > 0.0) t.width = w;
            if (std::isfinite(h) && h > 0.0) t.height = h;
        }
        const auto views = root.find("views");
        // Each view is a projection of the whole part: a file asking for
        // thousands made its reading as long as it liked.
        constexpr std::size_t kMaxViews = 256;
        if (views != root.end() && views->is_array() && views->size() > kMaxViews) {
            return fail("the drawing has " + std::to_string(views->size()) + " views; at most " +
                        std::to_string(kMaxViews) + " are read");
        }
        if (views != root.end() && views->is_array()) {
            for (const auto& v : *views) {
                if (!v.is_object()) continue;
                DrawingViewSpec view;
                view.kind = viewNamed(text(v, "kind"));
                const model::ViewProjection standard =
                    model::DrawingProjection::standardView(view.kind);
                view.projection.dir = vec3(v, "direction", standard.dir);
                view.projection.up = vec3(v, "up", standard.up);
                view.projection.origin = vec3(v, "origin", standard.origin);
                // A direction must have a length to normalize: none, or one so
                // large it overflows, projects every point to NaN.
                const auto usable = [](const math::Vec3& d) {
                    const double length = d.length();
                    return std::isfinite(length) && length > 1e-12;
                };
                if (!usable(view.projection.dir)) view.projection.dir = standard.dir;
                if (!usable(view.projection.up)) view.projection.up = standard.up;
                view.scale = number(v, "scale", 1.0);
                if (!usableScale(view.scale)) view.scale = 1.0;
                const auto at = v.find("placement");
                if (at != v.end() && at->is_array() && at->size() == 2 && (*at)[0].is_number() &&
                    (*at)[1].is_number() && std::isfinite((*at)[0].get<double>()) &&
                    std::isfinite((*at)[1].get<double>())) {
                    view.placement = {(*at)[0].get<double>(), (*at)[1].get<double>()};
                }
                view.showHidden = flag(v, "hidden", true);
                view.showTangentEdges = flag(v, "tangent", true);
                if (version >= 3) {
                    // A section or detail names a projection view before it:
                    // its mark is drawn there, and a detail is cut from it.
                    const std::string at = "view " + std::to_string(spec.views.size() + 1);
                    view.role = roleNamed(text(v, "role"));
                    view.label = usableLabel(text(v, "label"));
                    const double source = number(v, "source", -1.0);
                    if (source >= 0.0 && source < static_cast<double>(spec.views.size())) {
                        view.source = static_cast<int>(source);
                    }
                    const bool fromProjection =
                        view.source >= 0 &&
                        spec.views[static_cast<std::size_t>(view.source)].role ==
                            model::ViewRole::Projection;
                    if (view.role != model::ViewRole::Projection && view.source >= 0 &&
                        !fromProjection) {
                        return fail(at + " is taken from a view that is not a projection");
                    }
                    if (view.role == model::ViewRole::Detail) {
                        const auto centre = v.find("detailCenter");
                        view.detailRadius = number(v, "detailRadius", 0.0);
                        if (!fromProjection) {
                            return fail(at + " is a detail of no view before it");
                        }
                        if (centre == v.end() || !centre->is_array() || centre->size() != 2 ||
                            !(*centre)[0].is_number() || !(*centre)[1].is_number() ||
                            !std::isfinite((*centre)[0].get<double>()) ||
                            !std::isfinite((*centre)[1].get<double>()) ||
                            !std::isfinite(view.detailRadius) || view.detailRadius <= 0.0) {
                            return fail(at + " is a detail with no circle");
                        }
                        view.detailCenter = {(*centre)[0].get<double>(),
                                             (*centre)[1].get<double>()};
                    }
                }
                spec.views.push_back(view);
            }
        }
    }
    spec.version = version;
    outSpec = spec;
    return true;
}

bool DrawingDocumentIO::load(const std::string& path, DrawingDocumentSpec& outSpec,
                             model::Drawing& outDrawing, std::string* error) {
    const auto fail = [error](const std::string& why) {
        if (error != nullptr) *error = why;
        return false;
    };
    DrawingDocumentSpec spec;
    if (!readSpec(path, spec, error)) return false;
    outSpec = spec;

    // Open and rebuild the referenced part, then project the drawing from it,
    // so the views show the model as it is now.
    doc::Document partDoc;
    std::string partError;
    if (!NativeFormat::load(spec.partPath, partDoc, &partError)) {
        return fail("its part \"" + spec.partPath + "\" could not be read: " + partError);
    }
    if (!partDoc.rebuildModel() || partDoc.solid() == nullptr) {
        return fail("its part \"" + spec.partPath + "\" did not rebuild");
    }
    // A version 1 file: the four standard views, spaced by its gap, at 1:1.
    outDrawing = spec.version >= 2
                     ? build(*partDoc.solid(), spec)
                     : model::DrawingGenerator::standardViews(*partDoc.solid(), spec.gap);
    return true;
}

model::Drawing DrawingDocumentIO::build(const topo::Solid& solid, const DrawingDocumentSpec& spec) {
    if (spec.views.empty()) {
        return model::DrawingGenerator::sheetLayout(solid, spec.sheet, spec.titleBlock, spec.gap,
                                                    nullptr, spec.scale);
    }
    model::Drawing drawing;
    for (const DrawingViewSpec& v : spec.views) {
        const bool fromProjection =
            v.source >= 0 && static_cast<std::size_t>(v.source) < drawing.views.size() &&
            drawing.views[static_cast<std::size_t>(v.source)].role == model::ViewRole::Projection;
        model::DrawingView view;
        if (v.role == model::ViewRole::Section) {
            // Hatched 3 mm apart on the paper, whatever the scale.
            view = model::SectionGenerator::sectionView(
                solid, v.projection.origin, v.projection.dir.normalized() * -1.0,
                kHatchSpacing / (usableScale(v.scale) ? v.scale : 1.0));
        } else if (v.role == model::ViewRole::Detail) {
            // Cut from its source as built, not enlarged: its scale says how
            // large it is drawn. One with no source stays empty, and keeps the
            // views after it where they are.
            if (fromProjection) {
                view = model::DrawingGenerator::detailView(
                    drawing.views[static_cast<std::size_t>(v.source)], v.detailCenter,
                    v.detailRadius, 1.0);
            }
            view.detailCenter = v.detailCenter;
            view.detailRadius = v.detailRadius;
        } else {
            view = model::DrawingGenerator::makeView(solid, v.projection);
        }
        view.role = v.role;
        view.label = v.label;
        view.source = v.role == model::ViewRole::Projection || !fromProjection ? -1 : v.source;
        view.kind = v.kind;
        view.scale = v.scale;
        view.placement = v.placement;
        view.showHidden = v.showHidden;
        view.showTangentEdges = v.showTangentEdges;
        drawing.views.push_back(std::move(view));
    }
    return drawing;
}

std::vector<DrawingViewSpec> DrawingDocumentIO::viewsOf(const model::Drawing& drawing) {
    std::vector<DrawingViewSpec> views;
    views.reserve(drawing.views.size());
    for (const model::DrawingView& v : drawing.views) {
        DrawingViewSpec spec;
        spec.kind = v.kind;
        spec.projection = v.projection;
        spec.scale = v.scale;
        spec.placement = v.placement;
        spec.showHidden = v.showHidden;
        spec.showTangentEdges = v.showTangentEdges;
        spec.role = v.role;
        spec.label = v.label;
        spec.source = v.source;
        spec.detailCenter = v.detailCenter;
        spec.detailRadius = v.detailRadius;
        views.push_back(spec);
    }
    return views;
}

}  // namespace hz::io
