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

### 166c: strings a translation could not put right (as built)

Found by the translators of 166a, fixed in the code, and translated:

- "%1: the %2 is not one it can take" lowercased a parameter's label into
  the sentence, wrong for a German noun: now `%1: "%2" cannot take that
  value`, the label as it is.
- "Center" (a line type, and a text alignment) and "Linear" (a dimension,
  and a pattern) are two messages each, told apart by Qt's disambiguation
  comment. A test checks that the two Centers differ in every catalog.
- The STEP assembly import's sentence split in two messages is two whole
  sentences, each with its own plural.
- Left: file errors are sentence starts set into `%1 "%2".`; each language
  words them as such. `translations/README.md` says so.

## Phase 167: Crash reports (as built)

- **The handler** (`hz::ui::crash`, `CrashReport.cpp`), installed by the
  application once its log is open.
  - On Linux and macOS: SIGSEGV, SIGBUS, SIGFPE, SIGILL and SIGABRT, on a
    64 KiB stack of its own (`sigaltstack`), so a stack overflow is
    reported too. It uses only `open`, `write`, `backtrace` and
    `backtrace_symbols_fd`, into a path and a header made at start. The
    first `backtrace()` is called at install, so the handler does not load
    anything. `SA_RESETHAND`, then `raise()`: the process dies of the
    signal as it would have (exit 139 for SIGSEGV).
  - On Windows: an unhandled-exception filter writes the report and a
    minidump beside it (`MiniDumpWriteDump`), then lets Windows end the
    process.
  - The report: the signal (or exception code), the faulting address for a
    real fault (none for a signal sent by `raise` or `kill`), the version,
    revision, platform, Qt and start time, the log's path, the backtrace,
    and the log's last lines (its last 4 KiB, from a whole line), read as
    the crash happens with `open`, `lseek` and `read`. A report is later
    read as it is: no path in it is ever opened, so a report planted in the
    folder cannot put another file into what the user attaches to an issue.
    And at the next start the log already holds that session's own lines.
  - One thread writes the report (an `atomic_flag`); another that crashes
    meanwhile dies of its own signal. Frames are module offsets: `addr2line -e horizon -f -C
    <offset>` names them against the same build.
  - Uncaught C++ exceptions reach it too: the terminate handler logs the
    exception and aborts.
- **The log is flushed as it is written.** Flushing from a signal handler
  is not safe, so the logger flushes every line (it logs a handful).
- **At the next start**, before recovery, each report is offered in "Horizon
  CAD Stopped": what happened, with the report in its details. *Save Report...* writes them to a file to attach to an
  issue; *Delete Report* deletes it (and its minidump); *Close* keeps it,
  marked shown, and it is not offered again (deleted if it cannot be
  renamed, so it is not offered at every start). The ten newest shown
  reports are kept. Nothing is sent anywhere.
  - Reports are in `crashes` beside `logs` in the application's data
    folder, or `HZ_CRASH_DIR`.
- **Tests** (`test_CrashReport.cpp`): a helper process
  (`hz_crash_child`, the handler on QtCore alone) crashes by `raise`, by a
  write to address 0x10, and by recursing off its stack. Each leaves one
  report with its signal, the address where there is one, the header and
  a backtrace, and the process still dies of the crash. On Windows it
  checks the exception code and the minidump (no stack overflow there:
  the filter runs on what is left of the stack). The log's tail from a
  long log (whole lines, the end only), a planted report's `Log:` path not
  followed, pending and shown reports (one that cannot be renamed is
  deleted), pruning, and the log's flushing are tested too.
  - The window's dialog (`test_Recovery.cpp`): a report is offered once
    and kept; deleting one deletes its minidump and leaves the recovery
    files, which are still offered after it.
  - CI's "Start the application" step runs `horizon --crash-for-test` (a
    hidden option) on Xvfb, and fails unless it dies of SIGSEGV and
    leaves a report. That is the application's own handler, which the
    tests reach only through the helper.
- The bug report form asks for the crash report.

## Phase 168: Help and samples

In three parts: 168a the guide, 168b the samples, 168c Getting Started.

### 168a: the user guide (as built)

- **In the application, not a browser.** Qt reads Markdown itself
  (`QTextBrowser`, `QTextDocument::MarkdownResource`), so the guide is
  `docs/guide/*.md` compiled into the resources (`qt_add_resources` on
  `hz_ui`, at `:/guide`) and shown in a Help window. No converter, no build
  step, no HTML to install, and it works offline on every platform. The
  plan's converter in `tools/` was not needed.
- **`HelpWindow`**: not modal, with Back, Forward and Contents; links
  between pages open there, a web link in the browser. Its title is the
  page's first heading.
- **Help ▸ User Guide (F1)** opens it at the page for what is being worked
  on: drafting in a drawing, sketches while one is edited, parts in a part,
  assemblies in an assembly.
- **Ten pages**, written from the code (an inventory of every menu, tool,
  form and default, with its source line): Getting Started (the window,
  the view, a first part in five steps), 2D drafting, sketches and
  constraints, parts, assemblies, drawing sheets, files (import, export,
  recovery, crash reports), settings, and keyboard shortcuts.
- **Found writing it:** Polyline Edit's A, D and C were taken by the Arc,
  Linear and Circle shortcuts, so they switched tools. Fixed in its own
  commit: a tool can claim its keys (`Tool::claimsKey`), offered as Qt
  offers them (ShortcutOverride), which the old test had not done.
- **Tests** (`test_Help.cpp`): every page is compiled in and has a title;
  every link goes to a page, and every page is reached from the contents; a
  page is shown formatted (a heading, a table, no Markdown marks); a link
  opens its page and Back returns; F1 picks the page by context; and the
  first part in Getting Started is made in a test exactly as the page says
  (8000 mm³).
- The guide is in English; the window's own strings are translated. A
  translated guide would be a page set per language, looked up by the
  locale, when there are reviewed translations to put in it.

### 168b: samples (as built)

- **Made by a program, at build time.** `hz_make_samples`
  (`src/app/samples/MakeSamples.cpp`) builds them with the document API and
  saves them with the application's own writers:
  - `bracket.hzpart`: an L extruded, its outer corner filleted, a hole
    through each leg.
  - `plate.hzpart`: a plate with a counterbored hole, patterned in a row of
    three.
  - `pin.hzpart`: a pin that fits the holes.
  - `plate-and-pin.hzasm`: the plate, fixed, and the pin, concentric with
    its first hole and flush with its underside.
  - `bracket-drawing.hzdwg`: a sheet of the bracket with its title block.
  - `gasket.hcad`: a 2D gasket on layers of its own, with dimensions and a
    note.
- A part is rebuilt before it is saved, and the build fails when a feature
  does, so a sample that does not build is never shipped. The samples are
  placed next to the executable (`samples/`, as `translations/` is) and
  installed to `bin/samples`; the install-tree test checks them.
- **File ▸ Open Sample** lists them by name and kind ("Plate and pin
  (assembly)"), sorted by that. A sample is opened as a copy in "Horizon
  CAD Samples" in the user's documents: the whole set is copied (an
  assembly and a sheet refer to their parts by relative path), a copy
  already there is kept (it may have been changed), and the shipped
  samples are never written. So Save works, even where the installed
  folder is read-only. A copy that fails stops it, and says which (an
  assembly opened without its parts would look broken, not blocked).
  - `HZ_SAMPLES_DIR` and `HZ_SAMPLE_COPIES_DIR` point these elsewhere, for
    the tests.
- **Tests** (`test_Samples.cpp`, which depend on the samples being made):
  the menu lists all six; every part builds with every feature; the
  bracket's volume is as designed (the L, less the fillet's corner and two
  holes); the assembly opens with the pin where its mates put it; the sheet
  draws its views and title block; the gasket has its geometry on its
  layers; and a sample opens as a copy, a changed copy is kept, and the
  shipped sample is untouched.
- The sample names are English (they are file names); their kind is
  translated.

### 168c: Getting Started (as built)

- **Help ▸ Getting Started** (`GettingStartedTour`): six steps, each
  outlining a part of the window (the ribbon, the feature tree, the view,
  Properties, the status bar, the menus) with a panel beside it saying what
  it is for, and Back, Next (Done on the last) and Close.
  - The outline lets clicks through and the panel is a child of the window,
    so the window stays usable, and it keeps its place as the window is
    resized. A step whose target is not in the window (a panel closed, or
    floated as a window of its own; the menus on macOS, at the top of the
    screen) shows its panel alone, in the middle.
  - Its buttons take the focus: Enter goes on, Escape closes it.
- **Offered once**: the first start shows it after crash reports, recovery
  and files named on the command line (`help/tourOffered` in the settings);
  after that it is in the Help menu.
- **Tests:** each step outlines its target inside the window (the view's
  step contains the viewport's middle), the steps go forward and back, Done
  ends the tour and deletes it; a floated panel's step outlines nothing,
  and Escape closes the tour; it is offered at the first start only, and
  the Help menu shows it again.

## Phase 169: macOS, and signed installers (as built)

### 169a: a macOS job in CI (as built)

- **A `macos-14` (Apple silicon) entry in CI's build matrix**, through a
  `macos-debug` preset: Qt and its Linguist tools from Homebrew, the rest
  from vcpkg. Apple clang's warnings are reported, not failed on. ctest
  has a 15-minute limit per test, so a hang fails its test, not the job.
- **What the first macOS builds found:**
  - Apple's libc++ has no floating-point `std::from_chars`. The units,
    DXF and STEP readers call `hz::math::fromChars`: the standard one
    where it exists, else a reader of the same grammar over `strtod_l` in
    the C locale, compiled and tested everywhere.
  - `GL_ALL_BARRIER_BITS` is not in Apple's GL 4.1 headers; it is defined
    where missing.
  - A message box on macOS has no title, so the tests' dialog helper takes
    an untitled box as a match there, and the mass-properties box is found
    by an object name.
  - Qt on macOS is built without exceptions: the two containment tests
    skip there.
  - The memory test reads the Mach task, as macOS has no `/proc`.
  - Two validator bugs: `orient2d` compared a length squared with a
    length (small parts), and the Newell area vector lost its precision
    far from the origin once a fused multiply-add was used (Apple clang on
    arm64). Tests draw their randoms portably (`hz::test::Uniform`), and a
    rate test bounds unsound Boolean placements over 100 of each kind.

### 169b: an application bundle, a disk image, and signing (as built)

- **The application is a bundle on macOS**, `HorizonCAD.app`. Its
  Info.plist (`packaging/macos/Info.plist.in`) names it, gives its icon,
  its minimum macOS (the one it was built on, as Homebrew's Qt is), and
  the documents it opens: its own four types, declared by it, and DXF.
- **Shipped files in `Contents/Resources`.** A signature takes everything
  in `Contents/MacOS` for code, so the catalogs, samples and licences go
  in `Contents/Resources`. One CMake variable, `HZ_SHIPPED_DIR`, places
  them in the build tree, the tests and the install, and
  `Application::shippedFilesDirectory()` finds them the same way.
- **Documents from the Finder.** macOS hands a document opened from the
  Finder or dropped on the Dock icon to the running application as an
  event, not on the command line. `Application` turns the event into
  `fileOpenRequested`, and the window opens the file in a tab.
- **The icon** is packed into `horizon-cad.icns` by
  `packaging/icons/make-icns.py`, which does what Apple's `iconutil` does
  and runs anywhere; `render.sh` calls it.
- **The disk image** (`packaging/macos/make-dmg.sh`, a `macos` job in
  `release.yml`): `cmake --install` runs Qt's deployment script, which on
  macOS is macdeployqt. The script then removes every search path outside
  the bundle (a Mac with Homebrew's Qt would load that Qt instead of the
  bundle's), fails if anything loads a library from outside the bundle
  and the system, signs the bundle inside out, and makes the disk image
  with a link to Applications. The job starts the application from the
  mounted disk image (`--self-test`) before keeping it.
- **Signing**, when the repository has the secrets (listed in
  `docs/RELEASING.md`); only the signing steps are given them:
  - Windows: `signtool` signs `horizon.exe` before it is packed, then the
    installer.
  - macOS: a Developer ID in a keychain made for the run, the hardened
    runtime and a timestamp; with the Apple ID secrets too, the disk image
    is notarized and stapled. Without a Developer ID the bundle is signed
    ad hoc, which Apple silicon needs to run it at all.
  - The release notes list a package that is not signed, with what a user
    does to open it.
- **Found on the way:** `build-appimage.sh` was stored without its
  executable bit, so the release workflow, which had never run, would have
  failed at the AppImage. It, `make-dmg.sh`, `render.sh` and
  `make-icns.py` are executable now.
- **Why a script, not CPack.** CPack's DragNDrop generator packs the
  bundle straight after the install. The bundle has to be checked and
  signed between macdeployqt and packing, so a script does it.

## Tracking

Each phase is its own PR (166 as two), with README rows and CHANGELOG
entries. A phase's section here is replaced by "as built" when it lands.
