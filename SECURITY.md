# Security policy

## Reporting a vulnerability

Please do **not** open a public issue for a security problem.

Report it privately through GitHub: on the repository's **Security** tab,
choose **Report a vulnerability**. The report reaches the maintainers only.
Include:

- what the problem is, and what an attacker could do with it;
- the version or commit you found it in;
- how to reproduce it. A file that triggers it is the most useful thing to
  send.

The maintainers will acknowledge the report, work on a fix with you, and
credit you in the release notes if you wish.

## What counts

Horizon CAD opens files from anywhere, so the file readers are the main
attack surface: `.hcad`, `.hzpart`, `.hzasm`, DXF and STEP. They are fuzzed
(`tests/fuzz`), and a file that makes one crash, hang or use memory it
should not is a security bug.

Two modules are experimental and are not sandboxed:
- **Python scripting:** a script can do anything the user can. It is off in
  release builds.
- **Plugins:** the plugin registry reads and checks plugin manifests, but
  the application does not run plugins yet. The permissions a manifest
  declares record what the plugin asks for; nothing enforces them.

Do not run scripts or plugins you do not trust.

## Supported versions

Fixes go into the next release. There are no maintained release branches
yet.
