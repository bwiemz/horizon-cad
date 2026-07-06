#include "horizon/scripting/ScriptEngine.h"

#include <pybind11/embed.h>
#include <pybind11/operators.h>
#include <pybind11/stl.h>

#include <string>

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

    py::class_<DatumPlane>(m, "DatumPlane")
        .def(py::init([](const Vec3& o, const Vec3& n, const Vec3& x) {
                 return DatumPlane{o, n, x};
             }),
             py::arg("origin"), py::arg("normal"), py::arg("x_axis"))
        .def_readwrite("origin", &DatumPlane::origin)
        .def_readwrite("normal", &DatumPlane::normal)
        .def_readwrite("x_axis", &DatumPlane::xAxis)
        .def("y_axis", &DatumPlane::yAxis);

    py::class_<DatumAxis>(m, "DatumAxis")
        .def(py::init([](const Vec3& o, const Vec3& d) {
                 return DatumAxis{o, d};
             }),
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
        .def_readonly("max_von_mises", &ScriptContext::StaticAnalysisResult::maxVonMises);

    py::class_<ScriptContext::ModalAnalysisResult>(m, "ModalAnalysisResult")
        .def_readonly("converged", &ScriptContext::ModalAnalysisResult::converged)
        .def_readonly("natural_frequencies",
                      &ScriptContext::ModalAnalysisResult::naturalFrequencies);

    py::class_<ScriptContext>(m, "Document")
        .def("feature_count", &ScriptContext::featureCount)
        .def("sketch_count", &ScriptContext::sketchCount)
        .def("has_solid", &ScriptContext::hasSolid)
        .def("solid_face_count", &ScriptContext::solidFaceCount)
        .def("solid_shell_count", &ScriptContext::solidShellCount)
        .def("last_error", &ScriptContext::lastError)
        .def("add_box", &ScriptContext::addBox, py::arg("width"), py::arg("height"),
             py::arg("depth"))
        .def("add_cylinder", &ScriptContext::addCylinder, py::arg("radius"), py::arg("height"))
        .def("add_sphere", &ScriptContext::addSphere, py::arg("radius"))
        .def("add_cone", &ScriptContext::addCone, py::arg("bottom_radius"), py::arg("top_radius"),
             py::arg("height"))
        .def("add_torus", &ScriptContext::addTorus, py::arg("major_radius"),
             py::arg("minor_radius"))
        .def("add_rectangle_sketch", &ScriptContext::addRectangleSketch, py::arg("w"), py::arg("h"))
        .def("add_extrude", &ScriptContext::addExtrude, py::arg("sketch_index"),
             py::arg("direction"), py::arg("distance"))
        .def("add_linear_pattern", &ScriptContext::addLinearPattern, py::arg("direction"),
             py::arg("spacing"), py::arg("count"))
        .def("add_datum_plane", &ScriptContext::addDatumPlane, py::arg("origin"), py::arg("normal"),
             py::arg("x_axis"))
        .def("add_datum_axis", &ScriptContext::addDatumAxis, py::arg("origin"),
             py::arg("direction"))
        .def("add_datum_point", &ScriptContext::addDatumPoint, py::arg("position"))
        .def("mass_properties", &ScriptContext::massProperties, py::arg("density") = 1.0)
        .def("static_analysis", &ScriptContext::staticAnalysis, py::arg("force"),
             py::arg("youngs_modulus"), py::arg("poisson_ratio") = 0.3, py::arg("axis") = 0,
             py::arg("resolution") = 6)
        .def("modal_analysis", &ScriptContext::modalAnalysis, py::arg("youngs_modulus"),
             py::arg("poisson_ratio") = 0.3, py::arg("density") = 7850.0, py::arg("axis") = 0,
             py::arg("num_modes") = 6, py::arg("resolution") = 5)
        .def("export_drawing_dxf", &ScriptContext::exportDrawingDxf, py::arg("path"))
        .def("rebuild", &ScriptContext::rebuild);
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

// Bind `doc`, redirect stdout to a buffer, run @p body, restore stdout and
// collect captured output into @p res.
template <typename Body>
void withCapture(py::object& globals, ScriptContext* ctx, ScriptEngine::Result& res, Body&& body) {
    py::object sys = py::module_::import("sys");
    py::object io = py::module_::import("io");
    py::object buffer = io.attr("StringIO")();
    py::object oldStdout = sys.attr("stdout");
    sys.attr("stdout") = buffer;

    if (ctx) globals["doc"] = py::cast(ctx, py::return_value_policy::reference);

    try {
        body();
        res.ok = true;
    } catch (py::error_already_set& e) {
        res.ok = false;
        res.error = e.what();
    }

    sys.attr("stdout") = oldStdout;
    res.output = buffer.attr("getvalue")().cast<std::string>();
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
