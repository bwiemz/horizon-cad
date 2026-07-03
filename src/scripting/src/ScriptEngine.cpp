#include "horizon/scripting/ScriptEngine.h"

#include <pybind11/embed.h>
#include <pybind11/operators.h>
#include <pybind11/stl.h>

#include <string>

#include "horizon/math/Vec3.h"
#include "horizon/modeling/ReferenceGeometry.h"
#include "horizon/scripting/ScriptContext.h"

namespace py = pybind11;

namespace {

// A single embedded interpreter is shared per process. A function-local static
// is constructed exactly once (on first use) and torn down at process exit,
// after every ScriptEngine has already been destroyed — so no bound object
// outlives the interpreter. The constructing thread holds the GIL thereafter.
void ensureInterpreter() {
    static py::scoped_interpreter interpreter{};
    (void)interpreter;
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

    py::class_<ScriptContext>(m, "Document")
        .def("feature_count", &ScriptContext::featureCount)
        .def("sketch_count", &ScriptContext::sketchCount)
        .def("has_solid", &ScriptContext::hasSolid)
        .def("solid_face_count", &ScriptContext::solidFaceCount)
        .def("solid_shell_count", &ScriptContext::solidShellCount)
        .def("last_error", &ScriptContext::lastError)
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
