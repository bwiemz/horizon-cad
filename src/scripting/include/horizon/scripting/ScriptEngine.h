#pragma once

#include <memory>
#include <string>

namespace hz::script {

class ScriptContext;

/// Embedded Python interpreter exposing the `horizon` API module.
///
/// One interpreter is shared per process (CPython allows a single embedded
/// interpreter). Globals persist across `run`/`eval` calls on the same engine,
/// so it can back a REPL. Scripts run on the calling thread under the GIL.
class ScriptEngine {
public:
    struct Result {
        bool ok = false;
        std::string output;  ///< captured stdout
        std::string value;   ///< repr() of an eval'd expression (eval only)
        std::string error;   ///< exception text, empty when ok
    };

    ScriptEngine();
    ~ScriptEngine();

    ScriptEngine(const ScriptEngine&) = delete;
    ScriptEngine& operator=(const ScriptEngine&) = delete;

    /// Execute Python source. When @p ctx is non-null it is exposed to the
    /// script as the global `doc`.
    Result run(const std::string& code, ScriptContext* ctx = nullptr);

    /// Evaluate a single expression; `value` holds its repr() (for a REPL).
    Result eval(const std::string& expr, ScriptContext* ctx = nullptr);

private:
    struct Impl;
    std::unique_ptr<Impl> m_impl;
};

}  // namespace hz::script
