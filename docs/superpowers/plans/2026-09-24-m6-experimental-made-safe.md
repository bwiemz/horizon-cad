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

Clear the scripting `doc` global after each run (a use-after-free);
`HZ_ENABLE_SCRIPTING` off by default until scripts are sandboxed; plugin
entry scripts re-validated when loaded.

## Phase 121: CAM & FEA honesty

A G-code preamble with modal resets, spindle and tool, retract-first rapids
and parameter validation; FEA refuses non-box solids until real meshing
exists; the README maturity table marks library-only modules as such.
