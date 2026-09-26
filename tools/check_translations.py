#!/usr/bin/env python3
"""Check Horizon CAD's translation catalogs (Phase 166).

For each catalog (translations/horizon_<lang>.ts) it counts the messages
that have a translation, and fails when fewer than the floor do. Every
translation must keep its source's placeholders (%1..%9, %n) and its one
keyboard mnemonic (&), or it is an error whatever the floor.

With --lupdate and --src it also runs lupdate over the sources into a
temporary catalog, and fails when a string in the sources is missing from
any catalog: it was not brought up to date. Without lupdate that half is
skipped, and said.

Reads files only; no network. Exit 0 when every catalog passes, 1 when not.
"""

from __future__ import annotations

import argparse
import re
import subprocess
import sys
import tempfile
import xml.etree.ElementTree as ET
from collections import Counter
from dataclasses import dataclass, field
from pathlib import Path

PLACEHOLDER = re.compile(r"%(?:[1-9]|n)")


@dataclass
class Message:
    context: str
    source: str
    plural: bool
    forms: list[str]  # the translation, or each plural form
    finished: bool

    @property
    def translated(self) -> bool:
        return bool(self.forms) and all(form.strip() for form in self.forms)


@dataclass
class Report:
    catalog: Path
    total: int = 0
    translated: int = 0
    reviewed: int = 0
    errors: list[str] = field(default_factory=list)

    @property
    def share(self) -> float:
        return self.translated / self.total if self.total else 1.0


# A single '&' ("&&" is a literal ampersand) that does not begin an HTML
# entity ("&copy;" in rich text, which a translation may write as "©").
MNEMONIC = re.compile(r"(?<!&)&(?!&)(?![A-Za-z][A-Za-z0-9]*;|#[0-9]+;|#[xX][0-9A-Fa-f]+;)")


def mnemonics(text: str) -> int:
    """The number of keyboard mnemonics in @p text."""
    return len(MNEMONIC.findall(text))


def read_catalog(path: Path) -> list[Message]:
    # The repository's own catalogs, not input from anyone else; expat does
    # not fetch external entities either.
    root = ET.parse(path).getroot()
    out: list[Message] = []
    for context in root.iter("context"):
        name = context.findtext("name") or ""
        for message in context.iter("message"):
            translation = message.find("translation")
            kind = translation.get("type", "") if translation is not None else "unfinished"
            if kind in ("vanished", "obsolete"):
                continue
            plural = message.get("numerus") == "yes"
            if translation is None:
                forms: list[str] = []
            elif plural:
                forms = [form.text or "" for form in translation.iter("numerusform")]
            else:
                forms = [translation.text or ""]
            out.append(
                Message(
                    context=name,
                    source=message.findtext("source") or "",
                    plural=plural,
                    forms=forms,
                    finished=kind == "",
                )
            )
    return out


def check_message(message: Message) -> list[str]:
    """What is wrong with @p message's translation (none if untranslated)."""
    if not message.translated:
        return []
    problems = []
    want = Counter(PLACEHOLDER.findall(message.source))
    for form in message.forms:
        have = Counter(PLACEHOLDER.findall(form))
        mismatch = f"placeholders {sorted(have.elements())} for {sorted(want.elements())}"
        if message.plural:
            # A plural form may say "one" in words: %n may be left out of it.
            want_plain = Counter({k: v for k, v in want.items() if k != "%n"})
            have_plain = Counter({k: v for k, v in have.items() if k != "%n"})
            if have_plain != want_plain:
                problems.append(mismatch)
        elif have != want:
            problems.append(mismatch)
        if mnemonics(form) != mnemonics(message.source):
            problems.append(
                f"{mnemonics(form)} mnemonic(s) for the source's {mnemonics(message.source)}"
            )
    return [f"{message.context}: {message.source!r}: {p}" for p in problems]


def check_catalog(path: Path, floor: float) -> Report:
    report = Report(catalog=path)
    for message in read_catalog(path):
        report.total += 1
        if message.translated:
            report.translated += 1
            if message.finished:
                report.reviewed += 1
        report.errors.extend(check_message(message))
    if report.share < floor:
        report.errors.append(
            f"{report.translated} of {report.total} translated ({report.share:.1%}), "
            f"below the floor of {floor:.0%}"
        )
    return report


def source_messages(lupdate: str, src: Path) -> set[tuple[str, str]]:
    """Every (context, source) string in @p src, as lupdate finds them."""
    with tempfile.TemporaryDirectory() as tmp:
        fresh = Path(tmp) / "fresh.ts"
        subprocess.run(
            [lupdate, "-silent", "-no-obsolete", "-locations", "none", str(src), "-ts", str(fresh)],
            check=True,
            capture_output=True,
        )
        return {(m.context, m.source) for m in read_catalog(fresh)}


def missing_from(catalog: Path, sources: set[tuple[str, str]]) -> list[str]:
    """The strings of @p sources that @p catalog lacks."""
    have = {(m.context, m.source) for m in read_catalog(catalog)}
    return sorted(f"{context}: {source!r}" for context, source in sources - have)


def missing_by_catalog(
    catalogs: list[Path], sources: set[tuple[str, str]]
) -> dict[Path, list[str]]:
    """Each of @p catalogs that lacks strings of @p sources, with those it lacks."""
    return {catalog: missing for catalog in catalogs if (missing := missing_from(catalog, sources))}


def main(argv: list[str]) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("catalogs", nargs="+", type=Path)
    parser.add_argument("--floor", type=float, default=0.95)
    parser.add_argument("--lupdate", help="lupdate, to find strings missing from the catalogs")
    parser.add_argument("--src", type=Path, help="the sources lupdate reads")
    args = parser.parse_args(argv)

    failed = False
    for catalog in args.catalogs:
        report = check_catalog(catalog, args.floor)
        print(
            f"{catalog.name}: {report.translated}/{report.total} translated "
            f"({report.share:.1%}), {report.reviewed} reviewed"
        )
        for error in report.errors:
            print(f"  error: {error}")
        failed = failed or bool(report.errors)

    if args.lupdate and args.src:
        sources = source_messages(args.lupdate, args.src)
        for catalog, missing in missing_by_catalog(args.catalogs, sources).items():
            failed = True
            print(f"{catalog.name}: {len(missing)} string(s) of the sources missing: run lupdate")
            for line in missing[:20]:
                print(f"  missing: {line}")
    else:
        print("note: no lupdate given; strings missing from the catalogs were not looked for")
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
