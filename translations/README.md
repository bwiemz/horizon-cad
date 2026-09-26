# Translations

Horizon CAD's interface is translated into German (`de`), Spanish (`es`),
French (`fr`), Japanese (`ja`), Korean (`ko`) and Simplified Chinese (`zh`).
Each language is a Qt Linguist catalog here, `horizon_<lang>.ts`. The build
compiles it to `horizon_<lang>.qm` next to the executable, and the
application loads the one for the language chosen in Preferences, or the
system's.

## Where they stand

Every string is translated. They are **machine translations**, made in
September 2026 (Phase 166a), and no native speaker has reviewed them yet.

- A machine translation is marked `type="unfinished"` in the catalog. It is
  shipped all the same (lrelease includes unfinished translations), and
  Qt Linguist shows it as not yet done.
- A translation a person has checked is finished: in Qt Linguist, *Mark as
  done* (Ctrl+Return).

Reviewing a language means working through its unfinished messages in Qt
Linguist, correcting what reads wrongly and marking each one done. The
notes below list what each translator was least sure of: a good place to
start.

## Updating the catalogs

When the sources gain or change a string, run lupdate from the repository's
root:

```sh
lupdate -no-obsolete -locations relative src -ts translations/horizon_*.ts
```

A new string arrives untranslated. The `TranslationCatalogs` test, which
runs in every build (`ctest -R Translation`), checks that
(`tools/check_translations.py`):

- each catalog has a translation for at least 95% of its messages;
- a translation keeps its source's placeholders (`%1` to `%9`, and `%n` in
  a plural message, whose singular form may leave it out);
- a translation keeps its source's number of mnemonics: one `&` before the
  letter that Alt+ reaches (`&&` is a literal ampersand);
- no string in `src` is missing from the catalogs (when lupdate is found).

## Conventions

- **Mnemonics.** In German, Spanish and French the `&` goes on a letter of
  the translation (`&Datei`). In Japanese, Korean and Chinese the English
  letter follows in brackets: `ファイル(&F)`, `파일(&F)`, `文件(&F)`, with a
  trailing `...` after the brackets.
- **Kept as in the English:** format names and extensions, "Horizon CAD",
  Qt, OpenGL, `%1` placeholders, HTML tags and entities such as `&copy;`.
  The recent-files entry `&%1 %2` keeps its `&` before the number.
- **Terms** follow the usual CAD terms of each language (SolidWorks' and
  AutoCAD's localized interfaces), with one term per concept throughout.

## What to review first

### German

- File errors ("Could not open", "Could not save" and others) are set into
  `%1 "%2".`, so they are translated as the start of a sentence
  ("Fehler beim Öffnen von").
- The plane and face names that go into "Sketching on %1" ("der XZ-Ebene",
  "einer Fläche") are in the dative, for that sentence.
- Terms outside the glossary: Deckungsgleich (Coincident), FANG and RASTER
  (SNAP, GRID), VonLayer (ByLayer), Stutzen, Dehnen, Versetzen, Brechen and
  Strecken (Trim, Extend, Offset, Break, Stretch), Schriftfeld (Title
  Block), Ausformrichtung (Pull direction). The layer list's V and L
  columns are S and G (sichtbar, gesperrt).

### Spanish

- Spain-neutral, with *usted*. Remove and Delete are both "Eliminar".
- Status-bar toggles SNAP, GRID, ORTHO and POLAR are REFENT, REJILLA, ORTO
  and POLAR, as in AutoCAD. The layer columns V, L and LT are V, B and TL.
- "Operation" (how a body combines) is "Operación booleana", so it is not
  confused with *operación* (a feature). "Goes" (an extrusion's extent) is
  "Extensión", "Over:" (a circular pattern's span) "Abarca:", a pattern's
  seed "semilla".

### French

- Suppress is "Désactiver" and unsuppress "Réactiver", since Delete and
  Remove are "Supprimer".
- "Drawing" has two meanings in the application: the 2D document
  (`.hcad`, "Drawing 1") is "Dessin", and drawing sheets (`.hzdwg`, the
  Drawing menu) are "mise en plan".
- 2D arrays are "Réseau rectangulaire / polaire"; feature and component
  patterns are "répétition". SNAP is ACCROBJ.
- Least sure: "Retour arrière" (a feature's "Rolled back" state), and the
  layer list's L column, "Vr".

### Japanese

- "Drawing" in Preferences is 作図 (drafting settings), not 図面.
- Union, Subtract and Intersect are 結合, 除去 and 共通; Vertical is 鉛直
  (Perpendicular 垂直); an extrusion's end conditions are 指定距離, 両側対称,
  全貫通 and 指定面まで.
- Some sentences gained 「」 around a command's name.

### Korean

- Configuration is 컨피규레이션 (not 구성 요소, Component); construction
  geometry 구성선; linetype and lineweight 선종류 and 선가중치; Perpendicular
  직각 and Tangent 접선 (Vertical is 수직). ByLayer stays in English.
- Where the substituted text is unknown, particles are written both ways:
  `%1을(를)`.

### Chinese

- Both meanings of "Drawing" are 工程图. AutoCAD's 图形 may suit the 2D
  document better.
- The 2D Stretch tool is 伸展 (拉伸 is Extrude); Rotate and Revolve are both
  旋转, as in SolidWorks.
- Sentence colons are full-width (：); a label's trailing colon stays ASCII.

## Known problems in the sources

The translators found strings that no translation could put right. These
are fixed in the code (Phase 166c):

- A parameter's label was lowercased into a sentence ("the %2 is not one it
  can take"), wrong for German nouns; it is now quoted as it is.
- "Center" (a line type, and a text alignment) and "Linear" (a dimension,
  and a pattern) were one message each; each is now two, told apart by a
  comment.
- "Imported %n part(s) into "%1", " and "placed %n time(s)." were one
  sentence in two messages; they are now two sentences.

One remains: a file error is a sentence start ("Could not open") set into
another message (`%1 "%2".`), so each language words it as a sentence
start. Rewriting every file error as one message is left for later.
