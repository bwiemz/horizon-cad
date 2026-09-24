#include "horizon/scripting/ScriptEngine.h"

#include <pybind11/embed.h>
#include <pybind11/operators.h>
#include <pybind11/stl.h>

#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>

#include "horizon/cam/FeedsAndSpeeds.h"
#include "horizon/cam/GcodeWriter.h"
#include "horizon/cam/Toolpath.h"
#include "horizon/math/Vec2.h"
#include "horizon/math/Vec3.h"
#include "horizon/modeling/MassProperties.h"
#include "horizon/modeling/ReferenceGeometry.h"
#include "horizon/modeling/SheetMetal.h"
#include "horizon/scripting/ScriptContext.h"
#include "horizon/simulation/Fatigue.h"

namespace py = pybind11;

namespace {

// A single embedded interpreter is shared per process. Initialize on first use
// and NEVER finalize: finalizing an embedded CPython at process exit is
// crash-prone once embedded modules are registered (it segfaults during
// teardown on some Python/pybind11 versions). Leaking the interpreter for the
// process lifetime is the supported embedding pattern; the OS reclaims it.
void ensureInterpreter() {
    if (!Py_IsInitialized()) {
        py::initialize_interpreter();
    }
}

using hz::script::ScriptContext;

/// What a script holds as `doc`: the context of the run it was given to, until
/// that run ends.
///
/// The context belongs to the caller and may be destroyed as soon as run()
/// returns, but a script can keep `doc` in a global (`saved = doc`) and use it
/// in a later run. Every copy a script keeps is this one handle, so releasing
/// it at the end of the run cuts them all off: a later use raises an error
/// instead of reaching freed memory.
class DocHandle {
public:
    explicit DocHandle(ScriptContext* context) : m_context(context) {}

    ScriptContext& context() const {
        if (m_context == nullptr) {
            throw std::runtime_error(
                "this document is no longer available: `doc` is only valid during the run it "
                "was given to");
        }
        return *m_context;
    }

    void release() { m_context = nullptr; }

private:
    ScriptContext* m_context;
};

/// A binding for ScriptContext::method that goes through the handle.
template <typename R, typename... Args>
auto viaHandle(R (ScriptContext::*method)(Args...)) {
    return [method](const DocHandle& handle, Args... args) -> R {
        return (handle.context().*method)(std::forward<Args>(args)...);
    };
}

template <typename R, typename... Args>
auto viaHandle(R (ScriptContext::*method)(Args...) const) {
    return [method](const DocHandle& handle, Args... args) -> R {
        return (handle.context().*method)(std::forward<Args>(args)...);
    };
}

}  // namespace

// The `horizon` API module, importable from embedded scripts.
PYBIND11_EMBEDDED_MODULE(horizon, m) {
    namespace refgeo = hz::model::refgeo;
    using hz::math::Vec3;
    using hz::model::DatumAxis;
    using hz::model::DatumPlane;
    using hz::model::DatumPoint;
    using hz::script::ScriptContext;

    m.doc() = "Horizon CAD scripting API";

    py::class_<Vec3>(m, "Vec3")
        .def(py::init<>())
        .def(py::init<double, double, double>(), py::arg("x"), py::arg("y"), py::arg("z"))
        .def_readwrite("x", &Vec3::x)
        .def_readwrite("y", &Vec3::y)
        .def_readwrite("z", &Vec3::z)
        .def("length", &Vec3::length)
        .def("normalized", &Vec3::normalized)
        .def("dot", &Vec3::dot)
        .def("cross", &Vec3::cross)
        .def(py::self + py::self)
        .def(py::self - py::self)
        .def(py::self * double())
        .def("__repr__", [](const Vec3& v) {
            return "Vec3(" + std::to_string(v.x) + ", " + std::to_string(v.y) + ", " +
                   std::to_string(v.z) + ")";
        });

    py::class_<hz::math::Vec2>(m, "Vec2")
        .def(py::init<>())
        .def(py::init<double, double>(), py::arg("x"), py::arg("y"))
        .def_readwrite("x", &hz::math::Vec2::x)
        .def_readwrite("y", &hz::math::Vec2::y)
        .def("__repr__", [](const hz::math::Vec2& v) {
            return "Vec2(" + std::to_string(v.x) + ", " + std::to_string(v.y) + ")";
        });

    py::class_<DatumPlane>(m, "DatumPlane")
        .def(py::init(
                 [](const Vec3& o, const Vec3& n, const Vec3& x) { return DatumPlane{o, n, x}; }),
             py::arg("origin"), py::arg("normal"), py::arg("x_axis"))
        .def_readwrite("origin", &DatumPlane::origin)
        .def_readwrite("normal", &DatumPlane::normal)
        .def_readwrite("x_axis", &DatumPlane::xAxis)
        .def("y_axis", &DatumPlane::yAxis);

    py::class_<DatumAxis>(m, "DatumAxis")
        .def(py::init([](const Vec3& o, const Vec3& d) { return DatumAxis{o, d}; }),
             py::arg("origin"), py::arg("direction"))
        .def_readwrite("origin", &DatumAxis::origin)
        .def_readwrite("direction", &DatumAxis::direction);

    py::class_<DatumPoint>(m, "DatumPoint")
        .def(py::init([](const Vec3& p) { return DatumPoint{p}; }), py::arg("position"))
        .def_readwrite("position", &DatumPoint::position);

    py::class_<hz::model::MassProperties>(m, "MassProperties")
        .def_readonly("volume", &hz::model::MassProperties::volume)
        .def_readonly("surface_area", &hz::model::MassProperties::surfaceArea)
        .def_readonly("center_of_mass", &hz::model::MassProperties::centerOfMass)
        .def_readonly("mass", &hz::model::MassProperties::mass)
        .def_readonly("density", &hz::model::MassProperties::density)
        .def_readonly("valid", &hz::model::MassProperties::valid);

    // Sheet-metal bend allowance / flat-pattern development.
    using hz::model::SheetMetalParams;
    using hz::model::SheetMetalStrip;

    py::class_<SheetMetalParams>(m, "SheetMetalParams")
        .def(py::init<>())
        .def(py::init([](double t, double r, double k) {
                 SheetMetalParams p;
                 p.thickness = t;
                 p.bendRadius = r;
                 p.kFactor = k;
                 return p;
             }),
             py::arg("thickness"), py::arg("bend_radius"), py::arg("k_factor") = 0.44)
        .def_readwrite("thickness", &SheetMetalParams::thickness)
        .def_readwrite("bend_radius", &SheetMetalParams::bendRadius)
        .def_readwrite("k_factor", &SheetMetalParams::kFactor)
        .def("is_valid", &SheetMetalParams::isValid);

    py::class_<SheetMetalStrip>(m, "SheetMetalStrip")
        .def(py::init<>())
        .def(py::init([](std::vector<double> segments, std::vector<double> bendAngles) {
                 SheetMetalStrip s;
                 s.segments = std::move(segments);
                 s.bendAngles = std::move(bendAngles);
                 return s;
             }),
             py::arg("segments"), py::arg("bend_angles"))
        .def_readwrite("segments", &SheetMetalStrip::segments)
        .def_readwrite("bend_angles", &SheetMetalStrip::bendAngles);

    m.def("bend_allowance", &hz::model::bendAllowance, py::arg("angle"), py::arg("params"));
    m.def("bend_deduction", &hz::model::bendDeduction, py::arg("angle"), py::arg("params"));
    m.def("developed_length", &hz::model::developedLength, py::arg("strip"), py::arg("params"));

    // Stress-life fatigue (Phase 73).
    py::class_<hz::sim::SNCurve>(m, "SNCurve")
        .def(py::init<>())
        .def_readwrite("basquin_coefficient", &hz::sim::SNCurve::basquinCoefficient)
        .def_readwrite("basquin_exponent", &hz::sim::SNCurve::basquinExponent)
        .def_readwrite("endurance_limit", &hz::sim::SNCurve::enduranceLimit)
        .def_readwrite("ultimate_strength", &hz::sim::SNCurve::ultimateStrength)
        .def("is_valid", &hz::sim::SNCurve::isValid)
        .def_static("from_two_points", &hz::sim::SNCurve::fromTwoPoints, py::arg("s1"),
                    py::arg("n1"), py::arg("s2"), py::arg("n2"), py::arg("endurance_limit"),
                    py::arg("ultimate_strength"))
        .def_static("steel", &hz::sim::SNCurve::steel, py::arg("ultimate_strength"));

    m.def("cycles_to_failure", &hz::sim::cyclesToFailure, py::arg("sn"),
          py::arg("stress_amplitude"));
    m.def("goodman_equivalent", &hz::sim::goodmanEquivalent, py::arg("amplitude"), py::arg("mean"),
          py::arg("ultimate_strength"));
    m.def("soderberg_equivalent", &hz::sim::soderbergEquivalent, py::arg("amplitude"),
          py::arg("mean"), py::arg("yield_strength"));
    m.def("fatigue_safety_factor", &hz::sim::fatigueSafetyFactor, py::arg("sn"),
          py::arg("amplitude"), py::arg("mean"));

    // CAM: 2.5-axis toolpaths, G-code, feeds & speeds (Phase 71).
    py::class_<hz::cam::Toolpath>(m, "Toolpath")
        .def("cutting_length", &hz::cam::Toolpath::cuttingLength)
        .def("rapid_length", &hz::cam::Toolpath::rapidLength)
        .def("move_count", [](const hz::cam::Toolpath& t) { return t.moves.size(); });

    py::class_<hz::cam::Tool>(m, "Tool")
        .def(py::init<>())
        .def(py::init([](double d, int f) {
                 hz::cam::Tool t;
                 t.diameter = d;
                 t.flutes = f;
                 return t;
             }),
             py::arg("diameter"), py::arg("flutes"))
        .def_readwrite("diameter", &hz::cam::Tool::diameter)
        .def_readwrite("flutes", &hz::cam::Tool::flutes);

    m.def("cam_contour", &hz::cam::CamGenerator::contour, py::arg("profile"), py::arg("cut_depth"),
          py::arg("safe_z"), py::arg("feed"), py::arg("closed") = true);
    m.def("cam_drill", &hz::cam::CamGenerator::drill, py::arg("holes"), py::arg("cut_depth"),
          py::arg("safe_z"), py::arg("feed"));
    m.def("cam_pocket_rect", &hz::cam::CamGenerator::pocketRect, py::arg("min"), py::arg("max"),
          py::arg("tool_radius"), py::arg("stepover"), py::arg("cut_depth"), py::arg("safe_z"),
          py::arg("feed"));
    // A program is written only when it is safe to start; otherwise the
    // script gets the reason as a ValueError.
    const auto camGcode = [](const hz::cam::Toolpath& path, double spindleRpm, int tool,
                             bool coolant, int decimals) {
        hz::cam::GcodeOptions options;
        options.spindleRpm = spindleRpm;
        options.toolNumber = tool;
        options.coolant = coolant;
        options.decimals = decimals;
        std::string error;
        std::optional<std::string> gcode = hz::cam::GcodeWriter::toGcode(path, options, &error);
        if (!gcode) throw py::value_error(error);
        return *gcode;
    };
    m.def("cam_gcode", camGcode, py::arg("path"), py::arg("spindle_rpm"), py::arg("tool") = 1,
          py::arg("coolant") = false, py::arg("decimals") = 3);
    m.def("spindle_rpm", &hz::cam::spindleRpm, py::arg("surface_speed"), py::arg("diameter"));
    m.def("feed_rate", &hz::cam::feedRate, py::arg("rpm"), py::arg("flutes"), py::arg("chip_load"));

    // Reference-geometry constructions. Fallible ones (std::optional) return
    // None on degenerate input.
    m.def("plane_offset", &refgeo::planeOffset, py::arg("base"), py::arg("offset"));
    m.def("plane_through_points", &refgeo::planeThroughPoints, py::arg("p0"), py::arg("p1"),
          py::arg("p2"));
    m.def("plane_at_angle", &refgeo::planeAtAngle, py::arg("base"), py::arg("hinge_origin"),
          py::arg("hinge_dir"), py::arg("angle"));
    m.def("plane_midplane", &refgeo::planeMidplane, py::arg("a"), py::arg("b"));
    m.def("axis_through_points", &refgeo::axisThroughPoints, py::arg("p0"), py::arg("p1"));
    m.def("axis_plane_intersection", &refgeo::axisPlaneIntersection, py::arg("a"), py::arg("b"));
    m.def("axis_from_direction", &refgeo::axisFromDirection, py::arg("base"), py::arg("direction"));
    m.def("point_at", &refgeo::pointAt, py::arg("position"));
    m.def("point_centroid", &refgeo::pointCentroid, py::arg("points"));
    m.def("point_line_intersection", &refgeo::pointLineIntersection, py::arg("a"), py::arg("b"));

    py::class_<ScriptContext::StaticAnalysisResult>(m, "StaticAnalysisResult")
        .def_readonly("converged", &ScriptContext::StaticAnalysisResult::converged)
        .def_readonly("max_displacement", &ScriptContext::StaticAnalysisResult::maxDisplacement)
        .def_readonly("max_von_mises", &ScriptContext::StaticAnalysisResult::maxVonMises)
        .def_readonly("error", &ScriptContext::StaticAnalysisResult::error);

    py::class_<ScriptContext::ModalAnalysisResult>(m, "ModalAnalysisResult")
        .def_readonly("converged", &ScriptContext::ModalAnalysisResult::converged)
        .def_readonly("natural_frequencies",
                      &ScriptContext::ModalAnalysisResult::naturalFrequencies)
        .def_readonly("error", &ScriptContext::ModalAnalysisResult::error);

    py::class_<ScriptContext::ThermalAnalysisResult>(m, "ThermalAnalysisResult")
        .def_readonly("converged", &ScriptContext::ThermalAnalysisResult::converged)
        .def_readonly("min_temperature", &ScriptContext::ThermalAnalysisResult::minTemperature)
        .def_readonly("max_temperature", &ScriptContext::ThermalAnalysisResult::maxTemperature)
        .def_readonly("max_flux", &ScriptContext::ThermalAnalysisResult::maxFlux)
        .def_readonly("error", &ScriptContext::ThermalAnalysisResult::error);

    // The document is bound through DocHandle, never as a raw ScriptContext*,
    // so a script cannot keep a pointer past its run.
    py::class_<DocHandle, std::shared_ptr<DocHandle>>(m, "Document")
        .def("feature_count", viaHandle(&ScriptContext::featureCount))
        .def("sketch_count", viaHandle(&ScriptContext::sketchCount))
        .def("has_solid", viaHandle(&ScriptContext::hasSolid))
        .def("solid_face_count", viaHandle(&ScriptContext::solidFaceCount))
        .def("solid_shell_count", viaHandle(&ScriptContext::solidShellCount))
        .def("last_error", viaHandle(&ScriptContext::lastError))
        .def("add_box", viaHandle(&ScriptContext::addBox), py::arg("width"), py::arg("height"),
             py::arg("depth"))
        .def("add_cylinder", viaHandle(&ScriptContext::addCylinder), py::arg("radius"),
             py::arg("height"))
        .def("add_sphere", viaHandle(&ScriptContext::addSphere), py::arg("radius"))
        .def("add_cone", viaHandle(&ScriptContext::addCone), py::arg("bottom_radius"),
             py::arg("top_radius"), py::arg("height"))
        .def("add_torus", viaHandle(&ScriptContext::addTorus), py::arg("major_radius"),
             py::arg("minor_radius"))
        .def("add_rectangle_sketch", viaHandle(&ScriptContext::addRectangleSketch), py::arg("w"),
             py::arg("h"))
        .def("add_extrude", viaHandle(&ScriptContext::addExtrude), py::arg("sketch_index"),
             py::arg("direction"), py::arg("distance"))
        .def("add_linear_pattern", viaHandle(&ScriptContext::addLinearPattern),
             py::arg("direction"), py::arg("spacing"), py::arg("count"))
        .def("add_datum_plane", viaHandle(&ScriptContext::addDatumPlane), py::arg("origin"),
             py::arg("normal"), py::arg("x_axis"))
        .def("add_datum_axis", viaHandle(&ScriptContext::addDatumAxis), py::arg("origin"),
             py::arg("direction"))
        .def("add_datum_point", viaHandle(&ScriptContext::addDatumPoint), py::arg("position"))
        .def("mass_properties", viaHandle(&ScriptContext::massProperties), py::arg("density") = 1.0)
        .def("static_analysis", viaHandle(&ScriptContext::staticAnalysis), py::arg("force"),
             py::arg("youngs_modulus"), py::arg("poisson_ratio") = 0.3, py::arg("axis") = 0,
             py::arg("resolution") = 6)
        .def("modal_analysis", viaHandle(&ScriptContext::modalAnalysis), py::arg("youngs_modulus"),
             py::arg("poisson_ratio") = 0.3, py::arg("density") = 7850.0, py::arg("axis") = 0,
             py::arg("num_modes") = 6, py::arg("resolution") = 5)
        .def("thermal_analysis", viaHandle(&ScriptContext::thermalAnalysis),
             py::arg("conductivity"), py::arg("hot_temperature"), py::arg("cold_temperature") = 0.0,
             py::arg("axis") = 0, py::arg("resolution") = 6)
        .def("export_drawing_dxf", viaHandle(&ScriptContext::exportDrawingDxf), py::arg("path"))
        .def("rebuild", viaHandle(&ScriptContext::rebuild));
}

namespace hz::script {

struct ScriptEngine::Impl {
    // A py::object default-constructs to a null handle WITHOUT touching the
    // interpreter; the dict is created in the body once it is guaranteed live.
    // (A py::dict member would call PyDict_New during member init — before
    // ensureInterpreter() runs — and crash.)
    py::object globals;

    Impl() {
        ensureInterpreter();
        globals = py::dict();
        globals["__builtins__"] = py::module_::import("builtins");
        // Pre-import the API so scripts can use `horizon` without an import.
        globals["horizon"] = py::module_::import("horizon");
    }
};

ScriptEngine::ScriptEngine() : m_impl(std::make_unique<Impl>()) {}
ScriptEngine::~ScriptEngine() = default;

namespace {

/// For the length of one run: `doc` bound to the run's context, and stdout
/// captured. Undone however the run ends, including by a C++ exception: stdout
/// is restored, and `doc` is removed and its handle released, so nothing the
/// script kept can reach the context after the caller destroys it.
class RunScope {
public:
    RunScope(py::object& globals, ScriptContext* ctx) : m_globals(globals) {
        py::object io = py::module_::import("io");
        m_sys = py::module_::import("sys");
        m_buffer = io.attr("StringIO")();
        m_oldStdout = m_sys.attr("stdout");
        m_sys.attr("stdout") = m_buffer;
        if (ctx != nullptr) {
            m_doc = std::make_shared<DocHandle>(ctx);
            m_globals["doc"] = py::cast(m_doc);
        }
    }

    // Through the C API, which reports failure instead of throwing: this may
    // run while an exception unwinds.
    ~RunScope() {
        if (m_doc) {
            m_doc->release();
            // Failing to remove the name leaves a released handle, which is safe.
            if (PyDict_DelItemString(m_globals.ptr(), "doc") != 0) PyErr_Clear();
        }
        if (PyObject_SetAttrString(m_sys.ptr(), "stdout", m_oldStdout.ptr()) != 0) PyErr_Clear();
    }

    RunScope(const RunScope&) = delete;
    RunScope& operator=(const RunScope&) = delete;

    std::string output() const { return m_buffer.attr("getvalue")().cast<std::string>(); }

private:
    py::object& m_globals;
    py::object m_sys;
    py::object m_buffer;
    py::object m_oldStdout;
    std::shared_ptr<DocHandle> m_doc;
};

// Run @p body in a RunScope and collect its outcome and captured output into
// @p res.
template <typename Body>
void withCapture(py::object& globals, ScriptContext* ctx, ScriptEngine::Result& res, Body&& body) {
    RunScope scope(globals, ctx);
    try {
        body();
        res.ok = true;
    } catch (py::error_already_set& e) {
        res.ok = false;
        res.error = e.what();
    } catch (const std::exception& e) {  // e.g. a result that does not convert
        res.ok = false;
        res.error = e.what();
    }
    res.output = scope.output();
}

}  // namespace

ScriptEngine::Result ScriptEngine::run(const std::string& code, ScriptContext* ctx) {
    Result res;
    py::gil_scoped_acquire gil;
    withCapture(m_impl->globals, ctx, res,
                [&] { py::exec(code, m_impl->globals, m_impl->globals); });
    return res;
}

ScriptEngine::Result ScriptEngine::eval(const std::string& expr, ScriptContext* ctx) {
    Result res;
    py::gil_scoped_acquire gil;
    withCapture(m_impl->globals, ctx, res, [&] {
        py::object result = py::eval(expr, m_impl->globals, m_impl->globals);
        res.value = py::module_::import("builtins").attr("repr")(result).cast<std::string>();
    });
    return res;
}

}  // namespace hz::script
