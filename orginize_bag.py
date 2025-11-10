#!/usr/bin/env python3
"""Utilities for reorganizing ROS bag collections by sampling day.

This script scans a source directory that contains individual bag capture
folders. Each capture folder is expected to contain a ``metadata.yaml`` file
produced by ``rosbag2`` (or equivalent) that includes the recording start time
and duration in nanoseconds. The script creates a new directory tree where the
captures are grouped by sampling day and renamed to include their duration.

Example usage (dry run):

    python orginize_bag.py --dry-run

Actual reorganization (moves the folders):

    python orginize_bag.py

By default the script reads from ``bag_data_collections`` (in the working
directory) and writes to ``bag_data_collections_by_day``. Use ``--source`` and
``--destination`` to customise these paths. The ``--timezone`` option accepts a
Python ``zoneinfo`` timezone key (e.g. ``Europe/Tel_Aviv``) when data should be
organised by local time instead of the system default timezone.
"""

from __future__ import annotations

import argparse
import dataclasses
import datetime as dt
import math
import shutil
import sys
from pathlib import Path
from typing import Iterable, Optional

import yaml
from zoneinfo import ZoneInfo


@dataclasses.dataclass
class SampleInfo:
    """Metadata required to classify and rename a capture."""

    source_dir: Path
    started_at: dt.datetime
    duration_sec: float

    @property
    def day_folder_name(self) -> str:
        return f"{self.started_at.year:04d}_{self.started_at.month:02d}_{self.started_at.day:02d}"

    @property
    def capture_folder_name(self) -> str:
        duration_display = format_duration(self.duration_sec)
        return (

            f"{self.started_at.day:02d}_"
            f"{self.started_at.month:02d}_"
            f"{self.started_at.hour:02d}_"
            f"{self.started_at.minute:02d}_"
            f"{duration_display}"
        )


def parse_args(argv: Optional[Iterable[str]] = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Reorganize bag data collections by day")
    parser.add_argument(
        "--source",
        type=Path,
        default=Path("bag_data_collections"),
        help="Directory with the original captures (default: ./bag_data_collections)",
    )
    parser.add_argument(
        "--destination",
        type=Path,
        default=Path("bag_data_collections_by_day"),
        help="Directory where the reorganized captures will be written",
    )
    parser.add_argument(
        "--timezone",
        type=str,
        default=None,
        help=(
            "Optional timezone (e.g. Europe/Tel_Aviv). When omitted, the system "
            "local timezone is used."
        ),
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="Print the planned moves without modifying the filesystem",
    )
    parser.add_argument(
        "--copy",
        action="store_true",
        help="Copy captures instead of moving them (implies not deleting originals)",
    )

    return parser.parse_args(argv)


def format_duration(duration_seconds: float) -> str:
    """Format duration in seconds; keep one decimal place only when needed."""

    rounded = round(duration_seconds, 1)
    if math.isclose(rounded, round(rounded)):
        return f"{int(round(rounded))}"
    return f"{rounded:.1f}"


def iter_sample_directories(source_root: Path) -> Iterable[Path]:
    for entry in sorted(source_root.iterdir()):
        if entry.is_dir():
            yield entry


def load_sample_info(
    directory: Path,
    timezone: Optional[ZoneInfo],
) -> SampleInfo:
    metadata_path = directory / "metadata.yaml"
    if not metadata_path.exists():
        raise FileNotFoundError(f"Missing metadata.yaml in {directory}")

    with metadata_path.open("r", encoding="utf-8") as handle:
        metadata = yaml.safe_load(handle)

    info = metadata.get("rosbag2_bagfile_information") or {}
    try:
        ns_since_epoch = info["starting_time"]["nanoseconds_since_epoch"]
        duration_ns = info["duration"]["nanoseconds"]
    except KeyError as exc:
        raise KeyError(f"metadata.yaml in {directory} is missing {exc.args[0]}") from exc

    started_at = dt.datetime.fromtimestamp(ns_since_epoch / 1e9, tz=dt.timezone.utc)
    if timezone is not None:
        started_at = started_at.astimezone(timezone)
    else:
        started_at = started_at.astimezone()

    duration_sec = duration_ns / 1e9

    return SampleInfo(source_dir=directory, started_at=started_at, duration_sec=duration_sec)


def ensure_unique_path(target: Path) -> Path:
    if not target.exists():
        return target

    suffix = 1
    while True:
        candidate = target.with_name(f"{target.name}_{suffix}")
        if not candidate.exists():
            return candidate
        suffix += 1


def is_within(child: Path, parent: Path) -> bool:
    try:
        child.relative_to(parent)
    except ValueError:
        return False
    return True


def reorganize_samples(
    samples: Iterable[SampleInfo],
    destination_root: Path,
    *,
    dry_run: bool,
    copy_mode: bool,
) -> None:
    for sample in samples:
        day_dir = destination_root / sample.day_folder_name
        target_dir = ensure_unique_path(day_dir / sample.capture_folder_name)

        action = "Copy" if copy_mode else "Move"
        print(f"{action}: {sample.source_dir} -> {target_dir}")

        if dry_run:
            continue

        day_dir.mkdir(parents=True, exist_ok=True)

        if copy_mode:
            shutil.copytree(sample.source_dir, target_dir)
        else:
            shutil.move(sample.source_dir, target_dir)


def main(argv: Optional[Iterable[str]] = None) -> int:
    args = parse_args(argv)

    if not args.source.exists():
        print(f"Source directory {args.source} does not exist", file=sys.stderr)
        return 1
    if not args.source.is_dir():
        print(f"Source path {args.source} is not a directory", file=sys.stderr)
        return 1

    source_resolved = args.source.resolve()
    destination_resolved = args.destination.resolve()

    if is_within(destination_resolved, source_resolved):
        print(
            "Destination must not be inside the source directory. Choose a different path.",
            file=sys.stderr,
        )
        return 1

    timezone = None
    if args.timezone:
        try:
            timezone = ZoneInfo(args.timezone)
        except Exception as exc:  # pragma: no cover - ZoneInfo raises subclass of Exception
            print(f"Failed to load timezone '{args.timezone}': {exc}", file=sys.stderr)
            return 1

    directories = list(iter_sample_directories(args.source))
    if not directories:
        print(f"No capture directories found in {args.source}")
        return 0

    samples: list[SampleInfo] = []
    skipped: list[Path] = []

    for directory in directories:
        try:
            samples.append(load_sample_info(directory, timezone))
        except FileNotFoundError:
            skipped.append(directory)
        except Exception as exc:
            print(f"Skipping {directory}: {exc}", file=sys.stderr)
            skipped.append(directory)

    if not samples:
        print("No captures with valid metadata were found", file=sys.stderr)
        return 1

    reorganize_samples(samples, args.destination, dry_run=args.dry_run, copy_mode=args.copy)

    if skipped:
        print("\nThe following directories were skipped due to missing or invalid metadata:")
        for directory in skipped:
            print(f"  - {directory}")

    if args.dry_run:
        print("\nDry run complete. Re-run without --dry-run to apply the changes.")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
