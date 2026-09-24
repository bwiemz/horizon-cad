# Milestone 6: Experimental Modules Made Safe (Phases 119–121): Implementation Plan

Roadmap: [2026-09-23-production-readiness-roadmap.md](../specs/2026-09-23-production-readiness-roadmap.md)

## Goal

The experimental modules stay experimental, but none of them can lose data,
run code it should not, or give an answer that looks right and is not:
- the vault cannot hand one document to two users, or overwrite a history it
  could not read;
- a script cannot reach a document that is gone;
- CAM output and FEA results are either sound or refused.

## Phase 119: PDM integrity

### What the audit found

- **Two users could check out the same document.** The lock manifest was one
  JSON file. A check-out read it, added an entry and wrote it back, so two
  users checking out at the same moment both read "free", both wrote, and
  both were told they held the lock. A test racing eight users for one
  document, run against the old code, had more than one winner in all 200
  rounds.
- **A damaged manifest made history disappear, then overwrote it.**
  - A lock manifest that was not valid JSON read as empty, so every document
    read as free.
  - A revision archive whose manifest was damaged or missing read as empty.
    The next commit then wrote revision 0 over the first revision and a
    one-entry manifest over the history.
- **Content was never checked when read.** The archive recorded a 64-bit
  FNV-1a hash of each revision. Sync compared those hashes but never checked
  them against the bytes, except for fetched revisions. FNV-1a also does not
  resist collisions, and the hash identifies content across machines.
- **A stale handle overwrote another's commit.** `commit()` appended to the
  history the handle had loaded, not the one on disk.

### As built

- **Atomic locks** (`VaultManifest`):
  - Each checked-out document is one file, `<docId>.lock`, in a lock
    directory, recording the owner and when the lock was taken.
  - A check-out creates that file exclusively (`O_EXCL` / `CREATE_NEW`), so
    the file system decides the race: of eight users racing for a document,
    exactly one wins, in every round.
  - The exclusive create is `io::createFileExclusively`, next to
    `writeFileAtomically` in a new small library, `Horizon::AtomicFile`,
    split out of fileio so pdm can use it without linking the file formats.
  - Check-in and break remove the file.
- **Locks fail closed:**
  - a lock file that is empty, malformed or unreadable counts as held by
    someone unknown (`LockState::unreadable`), for every user, until an
    administrator breaks it;
  - a lock directory that cannot be read makes every document read as
    locked;
  - a pre-119 vault kept its locks in one JSON file. A lock directory path
    that is a file is such a vault, and every document in it reads as
    locked, so none can be taken twice across the upgrade;
  - document ids name files, so an id that could leave the directory is
    refused (`isValidDocId`, shared with sync). A lock needs a named owner.
- **Archives fail closed** (`RevisionArchive`). `isCorrupt()` reports an
  archive that cannot be trusted:
  - its manifest exists but cannot be read, or is not valid JSON;
  - its manifest does not list revisions numbered 0, 1, 2… with well-formed
    hashes;
  - it has revision blobs but no manifest.

  A corrupt archive reads as empty and refuses every commit, so its files are
  left exactly as they were found.
- **SHA-256 content hashes:**
  - a new in-house SHA-256 (`Sha256.h`), tested against the FIPS 180-4
    examples and against Python's hashlib on every length around the padding
    boundaries;
  - new revisions record SHA-256. Pre-119 revisions keep their FNV-1a hash,
    which is still verified, so old archives load and grow;
  - `read()` checks every revision's content against its recorded hash and
    says whether it is Ok, Missing or Corrupt; `contentAt()` succeeds only
    for Ok;
  - a commit refuses to follow a head that fails verification.
- **A commit re-reads the manifest,** so a stale handle appends after
  another handle's commit instead of overwriting it.
- **Blobs and manifests are written atomically** (temporary file, flush,
  rename). A crash leaves the previous manifest, and at worst an unlisted
  blob that the next commit replaces.
- **Sync verifies at both ends:**
  - a local revision is verified before it is pushed; a damaged one stops
    the document with `corrupt:`;
  - every sync also checks the history the two sides already share,
    reading both sides' bytes against their own hashes. A revision that
    rotted after it was synced is reported as `corrupt:`, even though the
    two manifests still agree about it (found in review);
  - the file-system endpoint refuses content that does not match the hash
    it is pushed with, requires the push to land as the next revision, and
    reads it back;
  - an archive that cannot be trusted, on either side, is reported as
    `corrupt:` and left alone. The endpoint reports it as a count of −1;
  - the same content is recognised across the old and new hashes: when the
    two sides differ in hash kind, the remote bytes must match both.
- **Tests:** 22 new. Two are for the exclusive create; 20 are in
  `hz_pdm_tests`, which now has 48:
  - SHA-256;
  - the lock race;
  - five kinds of unreadable lock;
  - the legacy lock file;
  - unusable ids;
  - seven kinds of inconsistent manifest;
  - blobs without a manifest;
  - tampered content;
  - the stale handle;
  - legacy archives, alone and syncing with new ones;
  - corrupt archives on each side of a sync;
  - revisions that rot on either side after they were synced;
  - a tampered local revision;
  - a push whose content does not match its hash.
- **Not done:** check-in reads the owner and then removes the file. If an
  administrator breaks the lock and someone else takes it between those
  two steps, the holder's check-in removes the new lock. That needs a
  breakLock racing the holder's own release; it is documented at the call.

## Phase 120: Scripting & plugin safety

### What the audit found

- **A script could reach a document after it was gone.**
  - `ScriptEngine::run(code, ctx)` bound `ctx` into the interpreter's
    globals as `doc`, by raw pointer, and never removed it. The caller's
    context could be destroyed as soon as the run returned, but `doc` stayed
    for the next run.
  - Removing the global alone would not have been enough: a script can keep
    `doc` under another name (`kept = doc`), and every such copy held the
    same raw pointer.

  Under AddressSanitizer, the old code fails the new test with a
  stack-use-after-scope.
- **Scripting was built by default** wherever Python and pybind11 were
  found. A script runs with the user's full rights, and nothing sandboxes
  it.
- **Plugins were checked once, at discovery.** Nothing checked them again
  before they would run: an entry script swapped afterwards for a link out
  of the plugin, or a manifest that added a permission after the user
  enabled the plugin, would have gone through. There was also no load path
  at all, so no single place where such a check could live.

### As built

- **`doc` lives exactly as long as its run:**
  - the document is bound as a `DocHandle`, not a raw `ScriptContext*`.
    Every copy a script keeps is that one handle;
  - when the run ends, the handle is released and `doc` removed. A kept copy
    then raises "this document is no longer available" instead of reaching
    freed memory;
  - the end of the run is a scope object, so it happens however the run
    ends, a C++ exception included. Its teardown uses the C API, which
    cannot throw. `eval()` now also reports a result that fails to convert,
    instead of throwing.
- **Scripting is off by default** (`HZ_ENABLE_SCRIPTING=OFF`). CI turns it
  on, so it is still built and tested:
  - in the Windows and Linux build jobs;
  - in the AddressSanitizer job, which it had never been part of. That is
    the job that sees a script reach a destroyed document.

  The release workflow of Phase 117 (#84) builds without it. The scripting
  tests disable only LeakSanitizer, because the embedded interpreter is
  never finalized; every other ASan check stays on.
- **`PluginRegistry::prepareLoad(name, appVersion)`** is the one way to load
  a plugin. It checks, at load time:
  - that the plugin is registered, enabled and compatible;
  - that its `plugin.json` still passes every discovery check, containment
    of the entry included;
  - that its manifest is unchanged since discovery (permissions compared as
    a set). A changed plugin must be rediscovered and enabled again.

  It then reads the entry script through its resolved path and returns the
  source, so a loader runs exactly the bytes that were checked.
- **Tests:** 9 new:
  - four in `hz_scripting_tests`, now 27, passing under ASan:
    - `doc` is gone after a run;
    - a kept copy is cut off, and a new run still gets a working `doc`;
    - a failed run still ends its scope;
    - a script cannot construct a document;
  - five in `hz_plugin_tests`, now 18:
    - enabled only;
    - compatibility;
    - a changed manifest;
    - files broken after discovery;
    - an entry replaced by a link out of the plugin.
- **Not done:**
  - Scripts and plugins are still not sandboxed, and plugin permissions are
    still not enforced. Phase 118's SECURITY.md (#84) says so.
  - Nothing in the application runs scripts or plugins yet.
  - `prepareLoad` narrows the gap between checking the entry and reading
    it to one step; it does not close it. That would need opening each path
    component without following links.

## Phase 121: CAM & FEA honesty

### What the audit found

- **A G-code program could not be run safely.**
  - The preamble was `G21 G90` alone: no plane, no feed mode, and whatever
    cutter compensation, tool length offset or canned cycle the machine was
    left in still applied.
  - No tool was loaded and the spindle was never started, so the first
    cutting move fed a stopped tool into the stock. The program ended with
    `M2`, spindle still running on controls that do not stop it.
  - The first move was a rapid in X, Y and Z at once, from wherever the tool
    was. From below the safe plane that is a diagonal move through the part.
  - Nothing checked the numbers:
    - a cut depth at or above the safe plane made every "plunge" climb and
      every "retract" descend;
    - a zero feed wrote `F0`;
    - a NaN wrote `Xnan`.
- **FEA reported a box's results as the solid's.** The scripting analyses
  meshed the solid's bounding box whatever the solid was. A cylinder was
  analysed as the square bar around it, and nothing said so.
- **The maturity table was out of date:**
  - it still called STEP and glTF/STL library-only and "not yet reachable
    (Phase 107)", though Phase 107 put them in the File menu;
  - it did not say that sheet metal, drawings and the Vulkan path are not
    reachable from the application;
  - the ui row still described Phases 104 and 112 as future work.

### As built

- **G-code** (`GcodeWriter`, with a new `GcodeOptions`: tool, spindle
  speed, coolant, decimals). A program:
  - starts with `G21 G90 G94 G17 G40 G49 G80`;
  - loads the tool (`T<n> M6`) and starts the spindle (`S<rpm> M3`, and
    `M8` if coolant is asked for) before any motion;
  - makes its first rapid climb alone, applying the tool length offset
    (`G0 G43 H<n> Z<safe>`), and only then crosses;
  - ends with `M9`, `M5`, `M30`.
- **Retract-first rapids everywhere:** a rapid that climbs moves Z first and
  then X and Y; one that descends crosses first and then descends.
- **Refusals:**
  - `validate()` refuses a program:
    - with no moves, or that does not start with a rapid;
    - with a non-finite number, or a cutting move whose feed is not
      positive;
    - with a rapid that is not above every cutting move;
    - with no spindle speed, a tool number outside 1–9999, or decimals
      outside 0–6;
  - `toGcode()` writes nothing for such a program and gives the reason;
  - the generators return an empty path for a cut not below the safe plane,
    a feed that is not positive, or any non-finite input.
- **The dialect is stated:** Fanuc-style controls and LinuxCNC. GRBL, for
  one, does not accept `G43 H`; it stops with an error there, before any
  motion.
- **Python:** `cam_gcode(path, spindle_rpm, tool=1, coolant=False,
  decimals=3)` raises `ValueError` with the reason.
- **FEA refuses what it cannot mesh.** The static, modal and thermal
  analyses run only on a solid whose volume is its bounding box's, which is
  an axis-aligned box however it was built. For any other solid they return
  `converged` false and an `error`, for example "…this solid fills 78% of
  its box" for a cylinder. `meshSolidBoundingBox` says the same.
- **The maturity table:**
  - STEP and glTF/STL are marked reachable, with their menu commands;
  - drawings, sheet metal, Coons patches and the Vulkan path are marked
    library-only;
  - the CAM, FEA and ui rows describe what is there now.
- **Tests:** 7 new:
  - five in `hz_cam_tests`, now 16:
    - a whole contour program, line by line;
    - coolant;
    - rapid ordering;
    - eight kinds of refused program;
    - the generators' refusals;
  - two in `hz_scripting_tests`, now 29: a cylinder refused by all three
    analyses, and a box still analysed with no error.
- **Not done:**
  - cutter radius offsetting;
  - gouge and collision checking;
  - a stock model;
  - post-processors for other controls;
  - a mesher for arbitrary solids.

  The maturity table says so.
