# Milestone 19 — Ready for the world (Phases 166–169)

Roadmap: [2026-09-25-professional-workflows-roadmap.md](../specs/2026-09-25-professional-workflows-roadmap.md),
findings W1–W5.

## Baseline (checked in the code)

- **Translations (W1).**
  - Six catalogs (`translations/horizon_{de,es,fr,ja,ko,zh}.ts`) hold 18
    messages each. `lupdate` over `src` finds 1,109 source texts.
  - Catalogs compile with lrelease: `Qt6::lrelease`, or any lrelease on the
    system. A package requires them (`HZ_REQUIRE_TRANSLATIONS`, set by the
    release workflow).
  - `LocaleManager` loads a catalog by locale, with a language fallback, and
    keeps the last good one on failure. Edit ▸ Preferences sets the language.
  - **36 strings could never be translated.** The edit form's labels are
    marked `QT_TRANSLATE_NOOP("MainWindow", …)` but looked up through
    `MainWindow::tr`, whose context is `hz::ui::MainWindow`.
  - No check anywhere says how much of a catalog is translated.
- **Accessibility (W2).** No `setAccessibleName` anywhere in `src`. A button
  with text is named by it; icon-only buttons, the viewport, the docks'
  trees and the ribbon's groups are not named.
- **Crashes (W3).**
  - `Application::notify` contains exceptions from event handlers.
  - `installTerminateHandler` logs `std::terminate`.
  - A signal (a segfault, an abort) leaves nothing: no report, and the log
    ends where the crash was.
  - `RecoveryManager` keeps autosaved copies, and `offerRecovery` offers them
    at the next start.
- **Help (W4).** The Help menu has About and About Qt only. There is no user
  guide, and no sample files.
- **Platforms (W5).**
  - `release.yml` builds Linux (ubuntu-22.04) and Windows (windows-latest),
    unsigned.
  - There is no macOS build.
  - `installer/linux` builds an AppImage.

## Phase 166: Every string translated

### 166a: full catalogs, and a floor (as built)

- **Fix the 36 labels' context** (`hz::ui::MainWindow`).
- **Extract every string.** `lupdate -no-obsolete -locations relative src`
  into the six catalogs, keeping the 18 translations there.
- **Translate them all.** Each string is translated for its context: the
  class it is in, `%1` placeholders kept, one `&` mnemonic kept (on a
  letter of the translation), and the English's trailing `…`, `...`, `:`
  and units kept.
  - Machine translations are marked `type="unfinished"` in the `.ts`. They
    are shipped (lrelease includes them), and Qt Linguist shows every one
    as not yet reviewed by a native speaker.
  - A translation checked by a person is `finished`.
- **The floor.** `tools/check_translations.py` counts, per catalog, the
  messages with a translation, and whether the placeholders and mnemonics
  match the source. It fails below a floor (95%), or on any mismatch.
  - It runs as a ctest test, so CI's builds run it, and so does a local
    build. It needs Python only; it reads the `.ts` files, no network.
  - It also fails when a source string in `src` is missing from a catalog:
    it compares against a fresh `lupdate` into a temporary file when
    lupdate is found; otherwise that half is skipped, and it says so.
- **Tests:** the check itself, on a small catalog with a missing
  translation, a dropped `%1` and a doubled `&`. And one real catalog
  string per language loads through `LocaleManager`.

As built:

- **1,107 strings per language, all translated.** Six translators (one per
  language) worked from the extracted messages with their context, the
  class and the file they are in, and a shared glossary of CAD terms.
  Each checked its own output against the checker's rules before handing
  it back; none of the 6,642 translations failed them on merge.
- **The checker** (`tools/check_translations.py`, with ten tests of its
  own) runs as `TranslationCatalogs`: 100% in each language, 18 reviewed
  (the translations there before).
  - It runs lupdate once over `src` and checks every catalog against what
    it finds: a string added without running lupdate fails it, as checked
    by adding one. On Windows it finds PySide6's lupdate, as the build
    finds its lrelease.
  - An HTML entity (`&copy;`) is not counted as a mnemonic, so a
    translation may write the character itself.
- **The locale test** checks one of the 36 relabelled strings too ("Point
  angle (0: flat)"), which no test covered.
- **`translations/README.md`** is the translators' guide: how to update
  and review the catalogs, the conventions, what each translator was least
  sure of, and the strings the code has to fix (a label lowercased into a
  sentence, one message for two meanings, sentences split across
  messages). Those source fixes are left for a later phase: each changes
  a source string, and so its six translations.
- **Python's bytecode** (`__pycache__`, which running the checker makes)
  is ignored.

### 166b: accessible names on every control (as built)

- **What had no name.** Walking the window through `QAccessible` found
  the controls Qt cannot name from a text or a label: the feature tree,
  the configuration choice and the sketch list; the layer tree; the
  assembly tree; the property panel's colour button and constraint list;
  the viewport; and each document tab's close button. Ribbon and dock
  buttons are named by their actions already.
  - They are named in code, through `tr()`. The viewport is "Viewport",
    with a description of what it is and how to work in it (it does not
    change name between a drawing and a part).
  - A tab's close button is "Close <document>", renamed when the tab is.
- **Forms:** `FeatureForm`'s fields are named by their `QFormLayout`
  labels, as Qt does; the test confirms it.
- **Tests** (`test_Accessibility.cpp`): every focusable or clickable
  control of the window, and of a form with each kind of field, has a
  name. Qt's own inner parts (a spin box's line edit, a combo box's list)
  are reached through their control, and left out. With the German
  catalog, the viewport is "Ansichtsfenster" and a close button
  "... schließen".
- The five new strings are translated in all six catalogs (unreviewed,
  as 166a's).

## Phase 167: Crash reports

- **A handler for signals** (SIGSEGV, SIGBUS, SIGFPE, SIGILL, SIGABRT) and,
  on Windows, an unhandled-exception filter.
  - It writes a report beside the recovery files: the signal, a backtrace
    (`backtrace` / `CaptureStackBackTrace`), the version and the platform.
  - It uses only async-signal-safe calls, into a buffer and a file opened
    at start.
  - On Windows, a minidump too (`MiniDumpWriteDump`).
- **The log is flushed** to its file as the handler runs.
- **At the next start** the report is offered: shown, with the log's last
  lines, to copy or save for attaching to an issue. Nothing is uploaded.
  It is deleted once shown, if the user says so.
- **Tests:** a child process that crashes on purpose (a test-only flag)
  leaves a report with its signal and a frame, and the next start finds
  it. The recovery files are untouched.

## Phase 168: Help and samples

- **A user guide:** Markdown in `docs/guide/`, built into HTML at build time
  (a small converter in `tools/`, no dependency), installed with the
  application, and opened from Help ▸ User Guide in the browser.
  - It covers sketching, part features, assemblies, drawings, and import
    and export.
- **Getting started:** Help ▸ Getting Started, a short tour pointing at the
  ribbon, the tree, the viewport and the status bar, one step at a time.
  It is offered once at first start.
- **Samples:** a bracket part, a drilled plate, a two-part assembly with
  mates, and a drawing of the bracket.
  - They are made by a build step from code (the same API the tests use),
    so they always open in the version that ships them.
  - They are installed, and File ▸ Open Sample… lists them.
- **Tests:** every sample opens and builds; the guide builds, and its links
  resolve.

## Phase 169: macOS, and signed installers

- **A macOS job in CI** (macos-14, Apple silicon) that builds and runs the
  tests, with Homebrew's Qt.
- **`release.yml` gains a DMG**, built with `macdeployqt`.
- **Signing:** each platform's signing step runs when its secrets are there
  (Windows: a certificate for signtool; macOS: a Developer ID, and
  notarization), and is skipped, with a note in the release, when they are
  not. The owner provides the certificates (roadmap decision 2).

## Tracking

Each phase is its own PR (166 as two), with README rows and CHANGELOG
entries. A phase's section here is replaced by "as built" when it lands.
