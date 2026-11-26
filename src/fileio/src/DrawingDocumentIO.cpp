#include "horizon/fileio/DrawingDocumentIO.h"

#include <algorithm>
#include <cmath>
#include <exception>
#include <filesystem>
#include <fstream>
#include <initializer_list>
#include <locale>
#include <memory>
#include <nlohmann/json.hpp>
#include <optional>
#include <set>
#include <sstream>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include "horizon/document/Document.h"
#include "horizon/document/FeatureTree.h"
#include "horizon/fileio/AtomicFile.h"
#include "horizon/fileio/NativeFormat.h"
#include "horizon/modeling/FacetResolution.h"
#include "horizon/modeling/Faceting.h"
#include "horizon/modeling/Naming.h"
#include "horizon/modeling/SectionView.h"
#include "horizon/topology/HalfEdge.h"
#include "horizon/topology/Solid.h"

namespace hz::io {

using json = nlohmann::json;
namespace fs = std::filesystem;

namespace {

/// The newest format this version reads. A sheet is written as the oldest
/// format that holds all it has: 5 with geometric tolerances or datums, 4
/// with hole callouts, else 3, so it still opens in a version that reads
/// that far.
constexpr int kVersion = 5;
constexpr int kVersionWithCallouts = 4;
constexpr int kVersionWithoutCallouts = 3;
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

const char* dimensionKindName(DrawingDimensionSpec::Kind kind) {
    switch (kind) {
        case DrawingDimensionSpec::Kind::Radius:
            return "radius";
        case DrawingDimensionSpec::Kind::Diameter:
            return "diameter";
        case DrawingDimensionSpec::Kind::Length:
            break;
    }
    return "length";
}

std::optional<DrawingDimensionSpec::Kind> dimensionKindNamed(const std::string& name) {
    if (name == "length") return DrawingDimensionSpec::Kind::Length;
    if (name == "radius") return DrawingDimensionSpec::Kind::Radius;
    if (name == "diameter") return DrawingDimensionSpec::Kind::Diameter;
    return std::nullopt;
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

/// Name the edges of @p copy, a sheet's copy of the exact part @p exact (DrawingDocumentIO::
/// cutForSheet; Ruling 31): logically (model::nameEdgesLogically), so that a curve's chords
/// share its logical name, as the faceted part's do; a chord of an exact edge inside one face (a
/// cylinder's seam) keeps the name model::facetExact gave it, its exact edge's; and every other
/// line inside a face is `<face>/line:<n>/of:<m>/at:<step>`, in storage order, of the face's
/// @p m such lines (Ruling 36), @p step the copy's tolerance step (Ruling 34): a name the faceted
/// part's lines (`<face>/seam:<n>`), a copy at another step and a face cut into another number of
/// lines (a cylinder of another radius) never have. A dimension or callout on one of those is then
/// lost, and said to be, rather than bound by its number to a line somewhere else.
void nameForSheet(topo::Solid& copy, const topo::Solid& exact, int step) {
    std::unordered_set<std::string> exactNames;
    for (const topo::Edge& e : exact.edges()) {
        if (e.topoId.isValid()) exactNames.insert(e.topoId.tag());
    }
    std::vector<std::string> given;  // facetExact's names, edge by edge
    given.reserve(copy.edges().size());
    for (const topo::Edge& e : copy.edges()) {
        given.push_back(e.topoId.isValid() ? e.topoId.tag() : std::string());
    }
    model::nameEdgesLogically(copy);
    // Each line inside a face, on no exact edge: its face, and how many each face has.
    std::vector<std::string> lineOf(copy.edges().size());
    std::unordered_map<std::string, int> lines;
    std::size_t i = 0;
    for (topo::Edge& e : copy.edges()) {
        const std::string& was = given[i];
        std::string& face = lineOf[i++];
        const topo::HalfEdge* he = e.halfEdge;
        if (he == nullptr || he->twin == nullptr || he->face == nullptr ||
            he->twin->face == nullptr) {
            continue;
        }
        const std::string logical = model::logicalFace(he->face->topoId.tag());
        if (logical != model::logicalFace(he->twin->face->topoId.tag())) continue;  // two faces'
        const bool onExact = !was.empty() && (exactNames.count(was) != 0 ||
                                              (was.find("/chord:") != std::string::npos &&
                                               exactNames.count(model::logicalEdge(was)) != 0));
        if (onExact) {
            e.topoId = topo::TopologyID::fromTag(was);
            continue;
        }
        face = logical;
        ++lines[face];
    }
    std::unordered_map<std::string, int> next;  // by face: its next line's number
    i = 0;
    for (topo::Edge& e : copy.edges()) {
        const std::string& face = lineOf[i++];
        if (face.empty()) continue;
        e.topoId = topo::TopologyID::fromTag(face + "/line:" + std::to_string(next[face]++) +
                                             "/of:" + std::to_string(lines.at(face)) +
                                             "/at:" + std::to_string(step));
    }
}

}  // namespace

namespace {

/// The edge of @p solid a dimension names: the edge of that name, or a chord
/// of the curve of that name (a circle is one edge of many chords). An
/// empty ID when the solid has neither.
topo::TopologyID edgeNamed(const topo::Solid& solid, const std::string& name) {
    for (const topo::Edge& e : solid.edges()) {
        if (e.topoId.tag() == name) return e.topoId;
    }
    for (const topo::Edge& e : solid.edges()) {
        if (model::logicalEdge(e.topoId.tag()) == name) return e.topoId;
    }
    return {};
}

json dimensionsJson(const std::vector<DrawingDimensionSpec>& dimensions) {
    json out = json::array();
    for (const DrawingDimensionSpec& d : dimensions) {
        out.push_back({{"edge", d.edge}, {"kind", dimensionKindName(d.kind)}});
    }
    return out;
}

}  // namespace

bool DrawingDocumentIO::save(const std::string& path, const DrawingDocumentSpec& spec) {
    return writeFileAtomically(pathFromUtf8(path), serialise(path, spec));
}

std::string DrawingDocumentIO::serialise(const std::string& path, const DrawingDocumentSpec& spec) {
    json root;
    root["format"] = "hzdwg";
    const bool gdt = std::any_of(spec.views.begin(), spec.views.end(), [](const auto& v) {
        return !v.tolerances.empty() || !v.datums.empty();
    });
    const bool callouts =
        std::any_of(spec.views.begin(), spec.views.end(),
                    [](const DrawingViewSpec& v) { return !v.holeCallouts.empty(); });
    root["version"] = gdt ? kVersion : callouts ? kVersionWithCallouts : kVersionWithoutCallouts;

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
    root["units"] = {{"length", std::string(math::symbolOf(spec.lengthUnit))}};

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
                         {"detailRadius", v.detailRadius},
                         {"centreLines", v.showCentreLines},
                         {"dimensions", dimensionsJson(v.dimensions)}});
        if (!v.holeCallouts.empty()) views.back()["holeCallouts"] = v.holeCallouts;
        if (!v.tolerances.empty()) {
            json frames = json::array();
            for (const DrawingToleranceSpec& t : v.tolerances) {
                frames.push_back({{"edge", t.edge},
                                  {"characteristic", model::Gdt::key(t.frame.characteristic)},
                                  {"tolerance", t.frame.tolerance},
                                  {"diameter", t.frame.diameterZone},
                                  {"condition", model::Gdt::conditionKey(t.frame.modifier)},
                                  {"datums", t.frame.datumRefs}});
            }
            views.back()["tolerances"] = std::move(frames);
        }
        if (!v.datums.empty()) {
            json datums = json::array();
            for (const DrawingDatumSpec& d : v.datums) {
                datums.push_back({{"edge", d.edge}, {"label", d.label}});
            }
            views.back()["datums"] = std::move(datums);
        }
    }
    root["views"] = views;
    if (spec.annotations) {
        // Written as the native format writes a document, so they read back
        // with everything a drawing can hold.
        root["annotations"] = json::parse(NativeFormat::documentToJson(*spec.annotations, false));
        json frames = json::array();
        for (const DrawingViewFrame& f : spec.frames) {
            frames.push_back({{"role", roleName(f.role)},
                              {"kind", viewName(f.kind)},
                              {"label", f.label},
                              {"low", json::array({f.low.x, f.low.y})},
                              {"high", json::array({f.high.x, f.high.y})}});
        }
        root["frames"] = frames;
    }

    return root.dump(2, ' ', false, json::error_handler_t::replace);
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

    // Its unit (Phase 154): any version may name one; millimetres if not.
    if (const auto units = root.find("units"); units != root.end() && units->is_object()) {
        spec.lengthUnit =
            math::lengthUnitFrom(text(*units, "length")).value_or(math::LengthUnit::Millimetre);
    }

    // Compared as a number: a version of 1e300 cast to int was undefined.
    const double versionNumber = number(root, "version", 1.0);
    // A newer version's file is refused, as a newer part is (NativeFormat):
    // read as this version reads it, what the newer one wrote would be lost
    // without a word, and the next save would destroy it.
    if (versionNumber > kVersion) {
        std::ostringstream newer;
        newer.imbue(std::locale::classic());
        newer << versionNumber;
        return fail("it was written by a newer version of Horizon CAD (drawing format " +
                    newer.str() + "; this version reads up to " + std::to_string(kVersion) + ")");
    }
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
            std::set<std::string> datumLabels;  // one datum a letter, on the whole sheet
            std::size_t viewIndex = 0;
            for (const auto& v : *views) {
                const std::string entry = "view " + std::to_string(++viewIndex);
                if (!v.is_object()) {
                    spec.leftOut.push_back(entry + ": it is not a view");
                    continue;
                }
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
                    // A section with no view to mark its cut on is a caption
                    // that says nothing of where it cuts.
                    if (view.role == model::ViewRole::Section && !fromProjection) {
                        return fail(at + " is a section of no view before it");
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
                    view.showCentreLines = flag(v, "centreLines", true);
                    // Each is measured again from the part when the sheet is
                    // drawn: as many as a view has room for, no more.
                    const auto dims = v.find("dimensions");
                    if (dims != v.end() && dims->is_array()) {
                        if (dims->size() > kMaxPerView) {
                            return fail(at + " has " + std::to_string(dims->size()) +
                                        " dimensions; at most " + std::to_string(kMaxPerView) +
                                        " are read");
                        }
                        std::size_t dimensionIndex = 0;
                        for (const auto& d : *dims) {
                            const std::string dimension =
                                entry + ", dimension " + std::to_string(++dimensionIndex) + ": ";
                            if (!d.is_object()) {
                                spec.leftOut.push_back(dimension + "it is not a dimension");
                                continue;
                            }
                            const auto kind = dimensionKindNamed(text(d, "kind"));
                            const std::string edge = text(d, "edge");
                            if (!kind) {
                                spec.leftOut.push_back(
                                    dimension + "not a kind of dimension this version reads");
                            } else if (edge.empty()) {
                                spec.leftOut.push_back(dimension + "it names no edge");
                            } else {
                                view.dimensions.push_back({edge, *kind});
                            }
                        }
                    }
                    // Version 4: the holes called out, each by an edge of it.
                    const auto callouts = v.find("holeCallouts");
                    if (callouts != v.end() && callouts->is_array()) {
                        if (callouts->size() > kMaxPerView) {
                            return fail(at + " has " + std::to_string(callouts->size()) +
                                        " hole callouts; at most " + std::to_string(kMaxPerView) +
                                        " are read");
                        }
                        std::size_t calloutIndex = 0;
                        for (const auto& c : *callouts) {
                            ++calloutIndex;
                            if (!c.is_string() || c.get<std::string>().empty()) {
                                spec.leftOut.push_back(entry + ", hole callout " +
                                                       std::to_string(calloutIndex) +
                                                       ": it names no edge");
                                continue;
                            }
                            view.holeCallouts.push_back(c.get<std::string>());
                        }
                    }
                    // Version 5: geometric tolerances and datum feature
                    // symbols. One the standard does not allow is left out.
                    const auto frames = v.find("tolerances");
                    if (frames != v.end() && frames->is_array()) {
                        if (frames->size() > kMaxPerView) {
                            return fail(at + " has " + std::to_string(frames->size()) +
                                        " tolerances; at most " + std::to_string(kMaxPerView) +
                                        " are read");
                        }
                        std::size_t frameIndex = 0;
                        for (const auto& f : *frames) {
                            const std::string what =
                                entry + ", tolerance " + std::to_string(++frameIndex) + ": ";
                            if (!f.is_object()) {
                                spec.leftOut.push_back(what + "it is not a tolerance");
                                continue;
                            }
                            DrawingToleranceSpec tolerance;
                            tolerance.edge = text(f, "edge");
                            const auto characteristic =
                                model::Gdt::characteristicNamed(text(f, "characteristic"));
                            const auto condition =
                                model::Gdt::conditionNamed(text(f, "condition", "none"));
                            const auto datums = f.find("datums");
                            const bool letters =
                                datums == f.end() ||
                                (datums->is_array() && datums->size() <= 3 &&
                                 std::all_of(datums->begin(), datums->end(),
                                             [](const json& d) { return d.is_string(); }));
                            if (tolerance.edge.empty()) {
                                spec.leftOut.push_back(what + "it names no edge");
                                continue;
                            }
                            if (!characteristic) {
                                spec.leftOut.push_back(what +
                                                       "not a characteristic this version reads");
                                continue;
                            }
                            if (!condition) {
                                spec.leftOut.push_back(
                                    what + "not a material condition this version reads");
                                continue;
                            }
                            if (!letters) {
                                spec.leftOut.push_back(what + "its datums are not datum letters");
                                continue;
                            }
                            tolerance.frame.characteristic = *characteristic;
                            tolerance.frame.tolerance = number(f, "tolerance", 0.0);
                            tolerance.frame.modifier = *condition;
                            tolerance.frame.diameterZone = flag(f, "diameter", false);
                            if (datums != f.end()) {
                                for (const auto& d : *datums) {
                                    tolerance.frame.datumRefs.push_back(d.get<std::string>());
                                }
                            }
                            const std::string why = model::Gdt::check(tolerance.frame);
                            if (!why.empty()) {
                                spec.leftOut.push_back(what + why);
                                continue;
                            }
                            view.tolerances.push_back(std::move(tolerance));
                        }
                    }
                    const auto datums = v.find("datums");
                    if (datums != v.end() && datums->is_array()) {
                        if (datums->size() > kMaxPerView) {
                            return fail(at + " has " + std::to_string(datums->size()) +
                                        " datums; at most " + std::to_string(kMaxPerView) +
                                        " are read");
                        }
                        std::size_t datumIndex = 0;
                        for (const auto& d : *datums) {
                            const std::string what =
                                entry + ", datum " + std::to_string(++datumIndex) + ": ";
                            DrawingDatumSpec datum;
                            if (d.is_object()) {
                                datum.edge = text(d, "edge");
                                datum.label = text(d, "label");
                            }
                            if (!d.is_object() || datum.edge.empty()) {
                                spec.leftOut.push_back(what + "it names no edge");
                            } else if (!model::Gdt::usableDatumLabel(datum.label)) {
                                spec.leftOut.push_back(
                                    what + "its letter is not one or two capitals (not I, O, Q)");
                            } else if (!datumLabels.insert(datum.label).second) {
                                spec.leftOut.push_back(what + "datum " + datum.label +
                                                       " is on the sheet already");
                            } else {
                                view.datums.push_back(std::move(datum));
                            }
                        }
                    }
                }
                spec.views.push_back(view);
            }
        }
    }
    if (version >= 3) {
        const auto annotations = root.find("annotations");
        if (annotations != root.end() && !annotations->is_null()) {
            if (!annotations->is_object()) return fail("its annotations are not a drawing");
            auto notes = std::make_shared<doc::Document>();
            std::string why;
            if (!NativeFormat::documentFromJson(annotations->dump(), *notes, &why)) {
                return fail("its annotations could not be read: " + why);
            }
            for (const std::string& item : notes->leftOut()) {
                spec.leftOut.push_back("annotations, " + item);
            }
            notes->setLeftOut({});
            spec.annotations = std::move(notes);
        }
        // Where the views were then; one that cannot be read is left out,
        // and what was drawn in it stays where it was.
        constexpr std::size_t kMaxFrames = 256;  // as many as views are read
        const auto frames = root.find("frames");
        if (frames != root.end() && frames->is_array() && frames->size() <= kMaxFrames) {
            const auto point = [](const json& f, const char* key, math::Vec2& out) {
                const auto it = f.find(key);
                if (it == f.end() || !it->is_array() || it->size() != 2 || !(*it)[0].is_number() ||
                    !(*it)[1].is_number()) {
                    return false;
                }
                out = {(*it)[0].get<double>(), (*it)[1].get<double>()};
                return std::isfinite(out.x) && std::isfinite(out.y);
            };
            for (const auto& f : *frames) {
                if (!f.is_object()) continue;
                DrawingViewFrame frame;
                frame.role = roleNamed(text(f, "role"));
                frame.kind = viewNamed(text(f, "kind"));
                frame.label = usableLabel(text(f, "label"));
                if (!point(f, "low", frame.low) || !point(f, "high", frame.high)) continue;
                if (!(frame.high.x > frame.low.x) || !(frame.high.y > frame.low.y)) continue;
                spec.frames.push_back(std::move(frame));
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
    const std::vector<HoleNote> holes = holesOf(partDoc);
    // An exact part is drawn from its exact faces, cut for the sheet's largest scale (stage 3); a
    // sheet of it laid out anew at the scale its faceted part chooses, as before. Any other part
    // is drawn from its faceted solid.
    const topo::Solid* solid = partDoc.solid();
    std::unique_ptr<topo::Solid> cut;
    DrawingDocumentSpec drawn = spec;
    if (const topo::Solid* exact = partDoc.exactSolid()) {
        const double scale = sheetScale(spec, *solid, spec.titleBlock);
        std::string why;
        cut = cutForSheet(*exact, scale, &why);
        if (!cut) return fail("its part \"" + spec.partPath + "\" could not be drawn: " + why);
        solid = cut.get();
        if (drawn.views.empty()) drawn.scale = scale;
    }
    // A version 1 file: the four standard views, spaced by its gap, at 1:1.
    std::vector<std::string> lost;
    outDrawing = spec.version >= 2 ? build(*solid, drawn, &lost, &holes)
                                   : model::DrawingGenerator::standardViews(*solid, spec.gap);
    // What the part no longer has an edge for is said, not drawn (stage 3); the spec keeps it.
    outSpec.notDrawn = notDrawnOf(lost);
    return true;
}

double DrawingDocumentIO::sheetScale(const DrawingDocumentSpec& spec, const topo::Solid& faceted,
                                     const model::TitleBlock& room) {
    if (spec.version < 2) return 1.0;
    if (!spec.views.empty()) {
        // A detail is cut from its view as drawn, the copy cut for the views it is cut from
        // (Ruling 35): at 10:1 it would cut the whole part ten times finer.
        double largest = 0.0;
        for (const DrawingViewSpec& v : spec.views) {
            if (v.role == model::ViewRole::Detail) continue;
            if (usableScale(v.scale)) largest = std::max(largest, v.scale);
        }
        return largest > 0.0 ? largest : 1.0;
    }
    if (usableScale(spec.scale)) return spec.scale;
    double chosen = 1.0;
    model::DrawingGenerator::sheetLayout(faceted, spec.sheet, room, spec.gap, &chosen);
    return chosen;
}

int DrawingDocumentIO::toleranceStep(double scale) {
    if (!usableScale(scale)) return 0;
    // A scale on a step (1:1, 2:1, 1:2) is on it, whatever log2's last bit says.
    return static_cast<int>(std::ceil(4.0 * std::log2(scale) - 1e-9));
}

std::unique_ptr<topo::Solid> DrawingDocumentIO::cutForSheet(const topo::Solid& exact, double scale,
                                                            std::string* error) {
    const int step = toleranceStep(scale);
    model::FacetResolution resolution;
    resolution.segmentsPerTurn = 32;
    resolution.chordTolerance = kChordOnPaper * std::exp2(-static_cast<double>(step) / 4.0);
    model::FacetedSolid cut;
    try {
        cut = model::facetExact(exact, resolution);
    } catch (const std::exception& e) {
        cut.solid.reset();
        cut.error = e.what();
    }
    if (!cut.solid) {
        if (error != nullptr) *error = "its exact part could not be cut into facets: " + cut.error;
        return nullptr;
    }
    nameForSheet(*cut.solid, exact, step);
    return std::move(cut.solid);
}

namespace {

/// How many holes @p hole made in @p solid: one, and each copy of it a
/// pattern or a mirror made, each a wall of its own.
int instancesOf(const topo::Solid& solid, const HoleNote& hole) {
    const std::string wall = hole.feature + "/wall";
    std::set<std::string> walls;
    for (const topo::Face& face : solid.faces()) {
        const std::string name = model::logicalFace(face.topoId.tag());
        if (name.compare(0, wall.size(), wall) != 0) continue;
        if (name.size() > wall.size() && name[wall.size()] != '/') continue;
        // A wall cut into pieces is one wall.
        walls.insert(model::withoutPiece(name));
    }
    return std::max(1, static_cast<int>(walls.size()));
}

}  // namespace

std::vector<HoleNote> DrawingDocumentIO::holesOf(const doc::Document& part) {
    std::vector<HoleNote> holes;
    const auto& tree = part.featureTree();
    for (std::size_t i = 0; i < tree.featureCount(); ++i) {
        const auto* hole = dynamic_cast<const doc::HoleFeature*>(tree.feature(i));
        if (hole == nullptr || hole->isSuppressed()) continue;
        const auto sizes = hole->parameters();
        const auto size = [&sizes](const char* name) {
            const auto it = sizes.find(name);
            return it != sizes.end() ? it->second : 0.0;
        };
        HoleNote note;
        note.feature = hole->featureID();
        switch (hole->type()) {
            case doc::HoleFeature::Type::Simple:
                note.kind = model::HoleCallout::Kind::Simple;
                break;
            case doc::HoleFeature::Type::Counterbore:
                note.kind = model::HoleCallout::Kind::Counterbore;
                break;
            case doc::HoleFeature::Type::Countersink:
                note.kind = model::HoleCallout::Kind::Countersink;
                break;
        }
        switch (hole->extent()) {
            case doc::HoleFeature::Extent::Blind:
                note.depth = model::HoleCallout::Depth::Blind;
                break;
            case doc::HoleFeature::Extent::ThroughAll:
                note.depth = model::HoleCallout::Depth::Through;
                break;
            case doc::HoleFeature::Extent::UpToFace:
                note.depth = model::HoleCallout::Depth::UpToFace;
                break;
        }
        note.diameter = size("diameter");
        note.holeDepth = size("depth");
        note.boreDiameter = size("boreDiameter");
        note.boreDepth = size("boreDepth");
        note.sinkDiameter = size("sinkDiameter");
        note.sinkAngleDegrees = size("sinkAngle") * 180.0 / 3.14159265358979323846;
        holes.push_back(std::move(note));
    }
    return holes;
}

const HoleNote* DrawingDocumentIO::holeOfEdge(const std::vector<HoleNote>& holes,
                                              const std::string& edge) {
    for (const HoleNote& hole : holes) {
        // Its id, whole, at the start of a face's name in the edge's:
        // "edge:hole_1/wall|…", "hole_1/edge:…", "…|hole_1/…" (not hole_11).
        const std::string token = hole.feature + "/";
        for (std::size_t at = edge.find(token); at != std::string::npos;
             at = edge.find(token, at + 1)) {
            if (at == 0 || edge[at - 1] == ':' || edge[at - 1] == '|' || edge[at - 1] == '/') {
                return &hole;
            }
        }
    }
    return nullptr;
}

std::vector<std::string> DrawingDocumentIO::notDrawnOf(const std::vector<std::string>& lost) {
    std::vector<std::string> said;
    said.reserve(lost.size());
    for (const std::string& what : lost) {
        said.push_back(what + " on an edge the part no longer has");
    }
    return said;
}

model::Drawing DrawingDocumentIO::build(const topo::Solid& solid, const DrawingDocumentSpec& spec,
                                        std::vector<std::string>* lost,
                                        const std::vector<HoleNote>* holes) {
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
        view.showCentreLines = v.showCentreLines;
        // Measured from the part as it is: an edge it no longer has is said.
        for (const DrawingDimensionSpec& d : v.dimensions) {
            const topo::TopologyID edge = edgeNamed(solid, d.edge);
            bool found = false;
            if (d.kind == DrawingDimensionSpec::Kind::Length) {
                // A length only for a straight edge: a curve's chords would be
                // measured along whichever came first.
                model::LinearDimension dim;
                found = model::DrawingDimensioner::isStraight(solid, d.edge) &&
                        model::DrawingDimensioner::dimensionEdge(solid, edge, dim);
                if (found) view.dimensions.push_back(dim);
            } else {
                model::RadialDimension dim;
                found = model::DrawingDimensioner::dimensionRadius(
                    solid, edge, d.kind == DrawingDimensionSpec::Kind::Diameter, dim);
                if (found) view.radialDimensions.push_back(dim);
            }
            if (!found && lost != nullptr) {
                lost->push_back("view " + std::to_string(drawing.views.size() + 1) + ": a " +
                                dimensionKindName(d.kind));
            }
        }
        // Each hole called out, as its feature says now, on the edge it was
        // called out on: one the view no longer shows, or no hole made, is said.
        for (const std::string& callout : v.holeCallouts) {
            const std::string named = model::withoutPiece(model::logicalEdge(callout));
            const HoleNote* hole = holes != nullptr ? holeOfEdge(*holes, named) : nullptr;
            const bool shown = std::any_of(
                view.edges.begin(), view.edges.end(), [&named](const model::ProjectedEdge& e) {
                    return model::withoutPiece(model::logicalEdge(e.sourceEdge.tag())) == named;
                });
            if (hole == nullptr || !shown) {
                if (lost != nullptr) {
                    lost->push_back("view " + std::to_string(drawing.views.size() + 1) +
                                    ": a hole callout");
                }
                continue;
            }
            model::HoleCallout note;
            note.edge = named;
            note.count = instancesOf(solid, *hole);
            note.kind = hole->kind;
            note.depth = hole->depth;
            note.diameter = hole->diameter;
            note.holeDepth = hole->holeDepth;
            note.boreDiameter = hole->boreDiameter;
            note.boreDepth = hole->boreDepth;
            note.sinkDiameter = hole->sinkDiameter;
            note.sinkAngleDegrees = hole->sinkAngleDegrees;
            view.holeCallouts.push_back(std::move(note));
        }
        // Each frame and datum on the edge it names, wherever the view shows
        // it now: one the view no longer shows is said. One on a hole the
        // view calls out goes on the callout's edge, to hang under it.
        const auto shown = [&view, holes](const std::string& edge) -> const model::ProjectedEdge* {
            std::string named = model::withoutPiece(model::logicalEdge(edge));
            const HoleNote* hole = holes != nullptr ? holeOfEdge(*holes, named) : nullptr;
            if (hole != nullptr) {
                for (const model::HoleCallout& c : view.holeCallouts) {
                    if (holeOfEdge(*holes, c.edge) == hole) named = c.edge;
                }
            }
            for (const model::ProjectedEdge& e : view.edges) {
                if (model::withoutPiece(model::logicalEdge(e.sourceEdge.tag())) == named) {
                    return &e;
                }
            }
            return nullptr;
        };
        for (const DrawingToleranceSpec& t : v.tolerances) {
            const model::ProjectedEdge* edge = shown(t.edge);
            if (edge == nullptr) {
                if (lost != nullptr) {
                    lost->push_back("view " + std::to_string(drawing.views.size() + 1) +
                                    ": a tolerance frame");
                }
                continue;
            }
            model::FeatureControlFrame frame = t.frame;
            frame.feature = edge->sourceEdge;
            view.tolerances.push_back(std::move(frame));
        }
        for (const DrawingDatumSpec& d : v.datums) {
            const model::ProjectedEdge* edge = shown(d.edge);
            if (edge == nullptr) {
                if (lost != nullptr) {
                    lost->push_back("view " + std::to_string(drawing.views.size() + 1) +
                                    ": datum " + d.label);
                }
                continue;
            }
            view.datums.push_back({d.label, edge->sourceEdge});
        }
        drawing.views.push_back(std::move(view));
    }
    return drawing;
}

std::string DrawingDocumentIO::usableLabel(std::string label) {
    std::erase_if(label, [](char c) { return c == '\n' || c == '\r'; });
    // Characters, as the label is typed, not bytes: "Détail-A" lost its last
    // letter, and a cut inside a character left bytes that are not UTF-8
    // ("断面\xE5\x9B"). A character starts at any byte but a continuation
    // byte (10xxxxxx).
    std::size_t characters = 0;
    for (std::size_t i = 0; i < label.size(); ++i) {
        if ((static_cast<unsigned char>(label[i]) & 0xC0) != 0x80 && ++characters > kMaxLabel) {
            label.resize(i);
            break;
        }
    }
    return label;
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
        spec.showCentreLines = v.showCentreLines;
        views.push_back(spec);
    }
    return views;
}

}  // namespace hz::io
