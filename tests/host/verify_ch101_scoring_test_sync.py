#!/usr/bin/env python3
"""Verify that the host harness embeds the production CH101 control block."""

from __future__ import annotations

from pathlib import Path
import sys


def extract_between(text: str, start: str, end: str) -> str:
    try:
        start_index = text.index(start)
        end_index = text.index(end, start_index)
    except ValueError as exc:
        raise RuntimeError(f"missing marker: {exc}") from exc
    return text[start_index:end_index].rstrip()


def main() -> int:
    root = Path(__file__).resolve().parents[2]
    production_path = root / "src/hand_modules/hand_task/hand_task.c"
    harness_path = root / "tests/host/CH101_scoring_switching_host_test.c"

    production = production_path.read_text(encoding="utf-8")
    harness = harness_path.read_text(encoding="utf-8")

    helper_block = extract_between(
        production,
        "/* CH101 quality/switching state",
        "\n\nvoid hand_task_vl53l1x_collect_data",
    )
    handler_block = extract_between(
        production,
        "static void _hand_ch101_handle_data_ready",
        "\n\nvoid hand_task_ch101_collect_data",
    )

    missing: list[str] = []
    if helper_block not in harness:
        missing.append("quality/switching helper block")
    if handler_block not in harness:
        missing.append("_hand_ch101_handle_data_ready()")

    if missing:
        print("Host harness is stale: " + ", ".join(missing), file=sys.stderr)
        return 1

    print("CH101 host harness matches the production scoring/switching block.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
