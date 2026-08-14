#!/usr/bin/env python3
"""Smoke test: kiểm tra các thư viện nền của module perception."""

from importlib.util import find_spec


REQUIRED_MODULES = ("cv2", "numpy")
OPTIONAL_MODULES = ("pyrealsense2", "ultralytics")


def main() -> int:
    missing = [name for name in REQUIRED_MODULES if find_spec(name) is None]
    for name in REQUIRED_MODULES + OPTIONAL_MODULES:
        status = "OK" if find_spec(name) is not None else "MISSING"
        print(f"{name}: {status}")
    return 1 if missing else 0


if __name__ == "__main__":
    raise SystemExit(main())

