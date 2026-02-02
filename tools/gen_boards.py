#!/usr/bin/env python3
from __future__ import annotations

import argparse
import re
from pathlib import Path


BOARD_H_GLOB = "src/boards/*/board.h"


def _first_match(pattern: str, text: str) -> str | None:
    match = re.search(pattern, text, flags=re.MULTILINE)
    return match.group(1).strip() if match else None


def _parse_int(value: str | None) -> int | None:
    if value is None:
        return None
    value = value.strip()
    if value.lower().startswith("0x"):
        try:
            return int(value, 16)
        except ValueError:
            return None
    try:
        return int(value, 10)
    except ValueError:
        return None


def _fmt_vid_pid(vid: int | None, pid: int | None) -> str:
    if vid is None or pid is None:
        return "N/A"
    return f"0x{vid:04X}:0x{pid:04X}"


def _escape_md(value: str) -> str:
    return value.replace("|", "\\|")


def _load_board(board_h: Path, repo_root: Path) -> dict[str, str]:
    text = board_h.read_text(errors="ignore")
    board = board_h.parent.name
    name = _first_match(r'^\s*#define\s+UF2_PRODUCT_NAME\s+"([^"]+)"', text)
    url = _first_match(r'^\s*#define\s+UF2_INDEX_URL\s+"([^"]+)"', text)
    vid = _parse_int(_first_match(r"^\s*#define\s+USB_DESC_VID\s+([0-9A-Fa-fxX]+)", text))
    pid = _parse_int(
        _first_match(r"^\s*#define\s+USB_DESC_UF2_PID\s+([0-9A-Fa-fxX]+)", text)
    )

    return {
        "board": board,
        "name": name or "N/A",
        "vid_pid": _fmt_vid_pid(vid, pid),
        "url": url or "N/A",
    }


def generate(repo_root: Path, output: Path) -> None:
    boards: list[dict[str, str]] = []
    for board_h in sorted(repo_root.glob(BOARD_H_GLOB)):
        boards.append(_load_board(board_h, repo_root))

    lines: list[str] = []
    lines.append("# Supported Boards")
    lines.append("")
    def _is_adafruit(entry: dict[str, str]) -> bool:
        name = entry["name"].lower()
        board = entry["board"].lower()
        return name.startswith("adafruit") or board.startswith("adafruit")

    def _render_table(entries: list[dict[str, str]]) -> None:
        lines.append("| Board | Name | VID PID | URL |")
        lines.append("| --- | --- | --- | --- |")
        for entry in sorted(entries, key=lambda e: e["board"].lower()):
            lines.append(
                f"| {_escape_md(entry['board'])} | {_escape_md(entry['name'])} | "
                f"{_escape_md(entry['vid_pid'])} | {_escape_md(entry['url'])} |"
            )
        lines.append("")

    adafruit = [entry for entry in boards if _is_adafruit(entry)]
    third_party = [entry for entry in boards if not _is_adafruit(entry)]

    lines.append("## Adafruit Boards")
    lines.append("")
    _render_table(adafruit)

    lines.append("## 3rd Party Boards")
    lines.append("")
    _render_table(third_party)

    output.write_text("\n".join(lines))


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Generate supported_boards.md from src/boards/*/board.h"
    )
    parser.add_argument(
        "--output",
        default="supported_boards.md",
        help="Output markdown file (default: supported_boards.md)",
    )
    args = parser.parse_args()

    repo_root = Path(__file__).resolve().parents[1]
    output = Path(args.output)
    if not output.is_absolute():
        output = repo_root / output

    generate(repo_root, output)


if __name__ == "__main__":
    main()
