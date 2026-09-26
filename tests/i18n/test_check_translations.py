"""The catalog checker itself (Phase 166): what it counts and what it rejects."""

from __future__ import annotations

import sys
import tempfile
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "tools"))

import check_translations as ct  # noqa: E402

HEADER = (
    '<?xml version="1.0" encoding="utf-8"?>\n<!DOCTYPE TS>\n<TS version="2.1" language="de_DE">\n'
)


def catalog(messages: str) -> Path:
    path = Path(tempfile.mkdtemp()) / "horizon_de.ts"
    path.write_text(HEADER + "<context><name>C</name>" + messages + "</context></TS>\n", "utf-8")
    return path


def message(source: str, translation: str | None, kind: str = "") -> str:
    attr = f' type="{kind}"' if kind else ""
    body = "" if translation is None else translation
    return f"<message><source>{source}</source><translation{attr}>{body}</translation></message>"


class CheckTranslationsTest(unittest.TestCase):
    def test_counts_translated_and_reviewed(self) -> None:
        path = catalog(
            message("&amp;File", "&amp;Datei")
            + message("Open", "Öffnen", "unfinished")
            + message("Save", "", "unfinished")
        )
        report = ct.check_catalog(path, floor=0.5)
        self.assertEqual((report.total, report.translated, report.reviewed), (3, 2, 1))
        self.assertEqual(report.errors, [])

    def test_below_the_floor_fails(self) -> None:
        path = catalog(message("Open", "Öffnen") + message("Save", "", "unfinished"))
        report = ct.check_catalog(path, floor=0.95)
        self.assertEqual(len(report.errors), 1)
        self.assertIn("below the floor", report.errors[0])

    def test_a_dropped_placeholder_fails(self) -> None:
        path = catalog(message("%1 of %2", "%1 von", "unfinished"))
        report = ct.check_catalog(path, floor=0.0)
        self.assertEqual(len(report.errors), 1)
        self.assertIn("placeholders", report.errors[0])

    def test_placeholders_may_move(self) -> None:
        path = catalog(message("%1 of %2", "%2 の %1", "unfinished"))
        self.assertEqual(ct.check_catalog(path, floor=0.0).errors, [])

    def test_a_doubled_or_lost_mnemonic_fails(self) -> None:
        doubled = catalog(message("&amp;File", "&amp;Da&amp;tei"))
        lost = catalog(message("&amp;File", "Datei"))
        literal = catalog(message("A &amp;&amp; B", "A &amp;&amp; B"))
        self.assertEqual(len(ct.check_catalog(doubled, 0.0).errors), 1)
        self.assertEqual(len(ct.check_catalog(lost, 0.0).errors), 1)
        self.assertEqual(ct.check_catalog(literal, 0.0).errors, [], "&& is a literal ampersand")

    def test_an_html_entity_is_not_a_mnemonic(self) -> None:
        # "&copy;" in rich text; a translation may write the character.
        same = catalog(message("&amp;copy; 2026", "&amp;copy; 2026"))
        written = catalog(message("&amp;copy; 2026", "© 2026"))
        numeric = catalog(message("A&amp;#160;B &amp;Open", "A&amp;#160;B &amp;Öffnen"))
        self.assertEqual(ct.check_catalog(same, 0.0).errors, [])
        self.assertEqual(ct.check_catalog(written, 0.0).errors, [])
        self.assertEqual(ct.check_catalog(numeric, 0.0).errors, [])
        self.assertEqual(ct.mnemonics("&copy; &File"), 1)

    def test_a_string_missing_from_a_catalog_is_found(self) -> None:
        path = catalog(message("Open", "Öffnen") + message("Save", "Speichern"))
        self.assertEqual(ct.missing_from(path, {("C", "Open"), ("C", "Save")}), [])
        self.assertEqual(
            ct.missing_from(path, {("C", "Open"), ("C", "Close"), ("D", "Save")}),
            ["C: 'Close'", "D: 'Save'"],
        )

    def test_every_catalog_is_checked_for_missing_strings(self) -> None:
        # Each catalog on its own: a later one lacking what the first has
        # is found (only the first was looked at).
        full = catalog(message("Open", "Öffnen"))
        short = catalog(message("Save", "Speichern"))
        found = ct.missing_by_catalog([full, short], {("C", "Open")})
        self.assertEqual(found, {short: ["C: 'Open'"]})

    def test_a_plural_form_may_say_one_in_words(self) -> None:
        path = catalog(
            '<message numerus="yes"><source>%n part(s)</source><translation type="unfinished">'
            "<numerusform>ein Teil</numerusform><numerusform>%n Teile</numerusform>"
            "</translation></message>"
        )
        report = ct.check_catalog(path, floor=1.0)
        self.assertEqual((report.translated, report.errors), (1, []))

    def test_an_empty_plural_form_is_untranslated(self) -> None:
        path = catalog(
            '<message numerus="yes"><source>%n part(s)</source><translation type="unfinished">'
            "<numerusform>ein Teil</numerusform><numerusform></numerusform>"
            "</translation></message>"
        )
        self.assertEqual(ct.check_catalog(path, floor=0.0).translated, 0)


if __name__ == "__main__":
    unittest.main()
