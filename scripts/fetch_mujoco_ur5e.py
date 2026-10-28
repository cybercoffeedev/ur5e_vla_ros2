#!/usr/bin/env python3
"""Fetch the pinned MuJoCo Menagerie UR5e model into sim/mujoco/ur5e/.

The model files are third-party assets and are not tracked by Git. Only SHA256SUMS, LICENSE, and
README.md are committed, so a fresh clone reproduces the exact bytes by running this script.
"""

from __future__ import annotations

import argparse
import hashlib
import sys
import urllib.error
import urllib.request
from pathlib import Path, PurePosixPath

REPOSITORY = "google-deepmind/mujoco_menagerie"
COMMIT = "71f066ad0be9cd271f7ed58c030243ef157af9f4"
UPSTREAM_DIR = "universal_robots_ur5e"
RAW_URL = f"https://raw.githubusercontent.com/{REPOSITORY}/{COMMIT}/{UPSTREAM_DIR}"

REPO_ROOT = Path(__file__).resolve().parents[1]
TARGET_DIR = REPO_ROOT / "sim" / "mujoco" / "ur5e"
MANIFEST = TARGET_DIR / "SHA256SUMS"
TIMEOUT_SECONDS = 60


def read_manifest(path: Path) -> list[tuple[str, str]]:
    """Return (sha256, relative path) pairs from a sha256sum-format manifest."""
    entries: list[tuple[str, str]] = []
    for number, line in enumerate(path.read_text(encoding="utf-8").splitlines(), start=1):
        stripped = line.strip()
        if not stripped:
            continue
        parts = stripped.split(maxsplit=1)
        if len(parts) != 2:
            raise SystemExit(f"{path}:{number}: expected '<sha256>  <path>', got {line!r}")
        digest, relative = parts[0].lower(), parts[1].lstrip("*")
        candidate = PurePosixPath(relative)
        if candidate.is_absolute() or ".." in candidate.parts:
            raise SystemExit(f"{path}:{number}: refusing unsafe path {relative!r}")
        if len(digest) != 64:
            raise SystemExit(f"{path}:{number}: {digest!r} is not a SHA-256 digest")
        entries.append((digest, str(candidate)))
    if not entries:
        raise SystemExit(f"{path}: manifest is empty")
    return entries


def sha256_of(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1 << 20), b""):
            digest.update(chunk)
    return digest.hexdigest()


def download(relative: str, destination: Path, expected: str) -> None:
    request = urllib.request.Request(f"{RAW_URL}/{relative}", headers={"User-Agent": "ur5e-vla"})
    with urllib.request.urlopen(request, timeout=TIMEOUT_SECONDS) as response:
        payload = response.read()
    actual = hashlib.sha256(payload).hexdigest()
    if actual != expected:
        raise ValueError(f"sha256 mismatch: expected {expected}, got {actual}")
    destination.parent.mkdir(parents=True, exist_ok=True)
    temporary = destination.with_name(destination.name + ".part")
    temporary.write_bytes(payload)
    temporary.replace(destination)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--force",
        action="store_true",
        help="re-download every file even when the local copy already matches",
    )
    args = parser.parse_args()

    entries = read_manifest(MANIFEST)
    downloaded = 0
    failures: list[str] = []

    for expected, relative in entries:
        destination = TARGET_DIR / relative
        if not args.force and destination.is_file() and sha256_of(destination) == expected:
            continue
        try:
            download(relative, destination, expected)
        except (urllib.error.URLError, OSError, ValueError) as error:
            failures.append(f"{relative}: {error}")
        else:
            downloaded += 1

    if failures:
        for failure in failures:
            print(f"error: {failure}", file=sys.stderr)
        print(f"{len(entries) - len(failures)}/{len(entries)} files verified", file=sys.stderr)
        return 1

    location = TARGET_DIR.relative_to(REPO_ROOT)
    print(f"{len(entries)}/{len(entries)} files verified in {location} ({downloaded} downloaded)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
