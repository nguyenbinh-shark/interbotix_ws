#!/usr/bin/env python3
"""Liệt kê và chạy một bài kiểm tra module trong thư mục này."""

from __future__ import annotations

import argparse
import subprocess
import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parent
GROUPS = ("fuzzy_controller", "perception", "moveit", "hardware")


def available_tests() -> list[Path]:
    tests: list[Path] = []
    for group in GROUPS:
        tests.extend(path for path in (ROOT / group).glob("*.py") if path.name != "__init__.py")
    return sorted(tests)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("test", nargs="?", help="Đường dẫn tương đối của file test")
    parser.add_argument("--list", action="store_true", help="Liệt kê các bài test")
    args, test_args = parser.parse_known_args()

    if args.list:
        for path in available_tests():
            print(path.relative_to(ROOT))
        return 0

    if not args.test:
        parser.error("hãy dùng --list hoặc chọn một file test")

    requested = (ROOT / args.test).resolve()
    try:
        requested.relative_to(ROOT)
    except ValueError:
        parser.error("file test phải nằm trong module_tests")

    if requested not in available_tests():
        parser.error(f"không tìm thấy bài test hợp lệ: {args.test}")

    if test_args[:1] == ["--"]:
        test_args = test_args[1:]
    return subprocess.call([sys.executable, str(requested), *test_args], cwd=ROOT)


if __name__ == "__main__":
    raise SystemExit(main())

