#!/usr/bin/env bash
# Starts the bundle's horizon-mcp as an AI client would and checks that it
# answers: it prints its version, and over standard input and output it
# answers initialize and lists its tools. It links Qt, so this also shows it
# finds the bundle's Qt and nothing from where it was built.
#
#   packaging/macos/check-mcp.sh <HorizonCAD.app>
set -euo pipefail

app="${1:?usage: check-mcp.sh <HorizonCAD.app>}"
mcp="${app}/Contents/MacOS/horizon-mcp"
test -x "${mcp}"

# No timeout(1) on macOS: perl's alarm stops a server that does not answer.
limit() { perl -e 'alarm shift; exec @ARGV' "$@"; }

version="$(limit 30 "${mcp}" --version)"
echo "${version}"
[[ "${version}" == "horizon-mcp "* ]]

root="$(mktemp -d)"
trap 'rm -rf "${root}"' EXIT
requests='{"jsonrpc":"2.0","id":1,"method":"initialize","params":{"protocolVersion":"2025-06-18","capabilities":{},"clientInfo":{"name":"install-check","version":"1"}}}
{"jsonrpc":"2.0","method":"notifications/initialized"}
{"jsonrpc":"2.0","id":2,"method":"tools/list"}'
# Standard input ends after the requests, and the server with it.
replies="$(limit 60 "${mcp}" --root "${root}" <<<"${requests}")"
if ! grep -q '"id":1' <<<"${replies}" || ! grep -q '"serverInfo"' <<<"${replies}"; then
    echo "::error::horizon-mcp did not answer initialize" >&2
    echo "${replies}" >&2
    exit 1
fi
if ! grep -q '"id":2' <<<"${replies}" || ! grep -q '"tools":\[{' <<<"${replies}"; then
    echo "::error::horizon-mcp did not list its tools" >&2
    echo "${replies}" >&2
    exit 1
fi
echo "horizon-mcp answers and lists its tools"
