# horizon-mcp end to end: the executable, fed MCP messages on standard input,
# answers each request with one line on standard output, and builds a part.
#
#   cmake -DEXE=<horizon-mcp> -DWORK_DIR=<scratch> -P RunHorizonMcp.cmake

file(MAKE_DIRECTORY "${WORK_DIR}")
set(input "${WORK_DIR}/requests.jsonl")
set(part "${WORK_DIR}/made-by-mcp.hzpart")
file(WRITE "${input}"
"{\"jsonrpc\":\"2.0\",\"id\":1,\"method\":\"initialize\",\"params\":{\"protocolVersion\":\"2025-06-18\",\"capabilities\":{},\"clientInfo\":{\"name\":\"ctest\",\"version\":\"1\"}}}
{\"jsonrpc\":\"2.0\",\"method\":\"notifications/initialized\"}
{\"jsonrpc\":\"2.0\",\"id\":2,\"method\":\"tools/list\"}
{\"jsonrpc\":\"2.0\",\"id\":3,\"method\":\"tools/call\",\"params\":{\"name\":\"create_sketch\",\"arguments\":{\"entities\":[{\"type\":\"rectangle\",\"corner\":[0,0],\"width\":60,\"height\":40}]}}}
{\"jsonrpc\":\"2.0\",\"id\":4,\"method\":\"tools/call\",\"params\":{\"name\":\"list_sketches\",\"arguments\":{}}}
{\"jsonrpc\":\"2.0\",\"id\":5,\"method\":\"tools/call\",\"params\":{\"name\":\"save_document\",\"arguments\":{\"path\":\"${part}\"}}}
{\"jsonrpc\":\"2.0\",\"id\":6,\"method\":\"nonsense\"}
")
file(REMOVE "${part}")
execute_process(COMMAND "${EXE}" --root "${WORK_DIR}"
    INPUT_FILE "${input}"
    OUTPUT_VARIABLE out
    ERROR_VARIABLE err
    RESULT_VARIABLE status
    TIMEOUT 120)
if(NOT status EQUAL 0)
    message(FATAL_ERROR "horizon-mcp exited with ${status}\n${err}")
endif()

# One reply a line. (Counted by their line ends: a CMake list would split the
# replies at every ";" in their text too.)
string(REGEX MATCHALL "\n" ends "${out}")
list(LENGTH ends count)
if(NOT count EQUAL 6)
    message(FATAL_ERROR "expected 6 replies (none to the notification), got ${count}:\n${out}")
endif()
foreach(expected
        "\"protocolVersion\":\"2025-06-18\""
        "\"name\":\"extrude\""
        "\"name\":\"save_document\""
        "sketch_id"
        "\"code\":-32601")
    string(FIND "${out}" "${expected}" at)
    if(at EQUAL -1)
        message(FATAL_ERROR "the replies lack ${expected}:\n${out}")
    endif()
endforeach()
if(NOT EXISTS "${part}")
    message(FATAL_ERROR "save_document wrote nothing at ${part}:\n${out}")
endif()
message(STATUS "horizon-mcp answered ${count} requests and saved ${part}")
