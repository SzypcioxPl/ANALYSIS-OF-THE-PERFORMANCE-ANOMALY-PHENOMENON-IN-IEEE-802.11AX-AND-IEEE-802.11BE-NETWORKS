#!/usr/bin/env python3

"""Aggregate ns-3 simulation log metrics into CSV reports."""

from __future__ import annotations

import argparse
import csv
import re
import statistics
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable, Optional, Tuple

LINE_RE = re.compile(
    r"^802\.11(?P<standard>[a-z]+)"
    r"(?: (?P<context>network|STA #\d+))?"
    r" - Throughput: (?P<throughput>[0-9.+-eE]+) Mbit/s,"
    r" Average delay: (?P<delay>[0-9.+-eE]+) ms,"
    r" Average jitter: (?P<jitter>[0-9.+-eE]+) ms"
)

AIRTIME_LINE_RE = re.compile(
    r"^\s*(?P<device>[^:]+):\s+(?P<seconds>[0-9.+-eE]+)\s*s\s+\((?P<percent>[0-9.+-eE]+)%\)"
)

LEGACY_STANDARDS = {"a", "n", "ac"}
LEGACY_AX_FIELDS = [
    "scenario",
    "legacy_standard",
    "legacy_throughput_mbps",
    "legacy_delay_ms",
    "legacy_jitter_ms",
    "legacy_airtime_pct",
    "ax_throughput_mbps",
    "ax_delay_ms",
    "ax_jitter_ms",
    "ax_max_throughput_mbps",
    "ax_max_delay_ms",
    "ax_max_jitter_ms",
    "ax_airtime_avg_pct",
    "ax_airtime_max_pct",
]
LEGACY_BE_FIELDS = [
    "scenario",
    "legacy_standard",
    "legacy_throughput_mbps",
    "legacy_delay_ms",
    "legacy_jitter_ms",
    "legacy_airtime_pct",
    "be_throughput_mbps",
    "be_delay_ms",
    "be_jitter_ms",
    "be_max_throughput_mbps",
    "be_max_delay_ms",
    "be_max_jitter_ms",
    "be_airtime_avg_pct",
    "be_airtime_max_pct",
]


@dataclass
class Metrics:
    throughput_mbps: float
    delay_ms: float
    jitter_ms: float

    @classmethod
    def from_match(cls, match: re.Match[str]) -> "Metrics":
        return cls(
            throughput_mbps=float(match.group("throughput")),
            delay_ms=float(match.group("delay")),
            jitter_ms=float(match.group("jitter")),
        )

    @staticmethod
    def mean(values: Iterable["Metrics"]) -> "Metrics":
        items = list(values)
        if not items:
            raise ValueError("mean requires at least one Metrics value")
        return Metrics(
            throughput_mbps=statistics.fmean(v.throughput_mbps for v in items),
            delay_ms=statistics.fmean(v.delay_ms for v in items),
            jitter_ms=statistics.fmean(v.jitter_ms for v in items),
        )


@dataclass
class ParsedEntry:
    scenario: str
    legacy_standard: Optional[str]
    legacy_metrics: Optional[Metrics]
    ax_mean: Optional[Metrics]
    ax_max: Optional[Metrics]
    be_mean: Optional[Metrics]
    be_max: Optional[Metrics]
    legacy_airtime_pct: Optional[float]
    ax_airtime_avg_pct: Optional[float]
    ax_airtime_max_pct: Optional[float]
    be_airtime_avg_pct: Optional[float]
    be_airtime_max_pct: Optional[float]


def aggregate_metrics(
    network_metrics: Optional[Metrics],
    sta_metrics: Iterable[Metrics],
) -> Tuple[Optional[Metrics], Optional[Metrics]]:
    sta_metrics = list(sta_metrics)
    if sta_metrics:
        mean_metrics = Metrics.mean(sta_metrics)
        max_metrics = max(sta_metrics, key=lambda m: m.throughput_mbps)
        return mean_metrics, max_metrics
    if network_metrics is not None:
        return network_metrics, network_metrics
    return None, None


def classify_airtime_label(label: str, has_ax: bool, has_be: bool) -> str:
    """Classify airtime entry as legacy/ax/be/unknown."""
    compact = label.replace(" ", "")
    normalized = compact.lower()
    base = normalized.split("#", 1)[0]

    if "legacy" in normalized:
        return "legacy"

    if base.endswith("stadevicea") or base.endswith("stadevicesa") or base.endswith("stadevicelegacy"):
        return "legacy"

    if "ax" in normalized:
        return "ax"

    if "be" in normalized:
        return "be"

    if base.endswith("stadeviceb") or "stacomparisonb" in normalized:
        if has_ax and not has_be:
            return "ax"
        if has_be and not has_ax:
            return "be"

    if base.startswith("stadeviceb") or base.startswith("stadevicesb"):
        if has_ax and not has_be:
            return "ax"
        if has_be and not has_ax:
            return "be"

    return "unknown"


def summarize_airtime(
    entries: list[tuple[str, float]],
    has_ax: bool,
    has_be: bool,
) -> tuple[Optional[float], Optional[float], Optional[float], Optional[float], Optional[float]]:
    if not entries:
        return None, None, None, None, None

    legacy: list[float] = []
    ax: list[float] = []
    be: list[float] = []
    unknown: list[float] = []

    for label, percent in entries:
        category = classify_airtime_label(label, has_ax, has_be)
        if category == "legacy":
            legacy.append(percent)
        elif category == "ax":
            ax.append(percent)
        elif category == "be":
            be.append(percent)
        else:
            unknown.append(percent)

    if has_ax and not ax and unknown:
        ax = unknown.copy()
        unknown.clear()
    if has_be and not be and unknown:
        be = unknown.copy()
        unknown.clear()

    legacy_mean = statistics.fmean(legacy) if legacy else None
    ax_mean = statistics.fmean(ax) if ax else None
    ax_max = max(ax) if ax else None
    be_mean = statistics.fmean(be) if be else None
    be_max = max(be) if be else None

    return legacy_mean, ax_mean, ax_max, be_mean, be_max


def parse_log(log_path: Path) -> Optional[ParsedEntry]:
    legacy_standard: Optional[str] = None
    legacy_metrics: Optional[Metrics] = None

    ax_network_metrics: Optional[Metrics] = None
    ax_station_metrics: list[Metrics] = []

    be_network_metrics: Optional[Metrics] = None
    be_station_metrics: list[Metrics] = []

    airtime_entries: list[tuple[str, float]] = []
    in_airtime_block = False

    with log_path.open("r", encoding="utf-8") as log_file:
        for raw_line in log_file:
            stripped = raw_line.strip()
            if stripped.startswith("Airtime usage"):
                in_airtime_block = True
                continue

            if in_airtime_block:
                airtime_match = AIRTIME_LINE_RE.match(raw_line)
                if airtime_match:
                    label = airtime_match.group("device").strip()
                    percent = float(airtime_match.group("percent"))
                    airtime_entries.append((label, percent))
                    continue
                if not stripped:
                    continue
                in_airtime_block = False

            match = LINE_RE.search(raw_line)
            if not match:
                continue

            standard = match.group("standard")
            context = (match.group("context") or "network").lower()
            metrics = Metrics.from_match(match)

            if standard == "ax":
                if context.startswith("sta"):
                    ax_station_metrics.append(metrics)
                else:
                    ax_network_metrics = metrics
            elif standard == "be":
                if context.startswith("sta"):
                    be_station_metrics.append(metrics)
                else:
                    be_network_metrics = metrics
            elif standard in LEGACY_STANDARDS:
                legacy_standard = standard
                legacy_metrics = metrics

    if not (legacy_metrics or ax_network_metrics or ax_station_metrics or be_network_metrics or be_station_metrics):
        return None

    ax_mean, ax_max = aggregate_metrics(ax_network_metrics, ax_station_metrics)
    be_mean, be_max = aggregate_metrics(be_network_metrics, be_station_metrics)

    has_ax = bool(ax_network_metrics or ax_station_metrics)
    has_be = bool(be_network_metrics or be_station_metrics)
    (
        legacy_airtime_pct,
        ax_airtime_avg_pct,
        ax_airtime_max_pct,
        be_airtime_avg_pct,
        be_airtime_max_pct,
    ) = summarize_airtime(airtime_entries, has_ax, has_be)

    return ParsedEntry(
        scenario=log_path.stem,
        legacy_standard=f"802.11{legacy_standard}" if legacy_standard else None,
        legacy_metrics=legacy_metrics,
        ax_mean=ax_mean,
        ax_max=ax_max,
        be_mean=be_mean,
        be_max=be_max,
        legacy_airtime_pct=legacy_airtime_pct,
        ax_airtime_avg_pct=ax_airtime_avg_pct,
        ax_airtime_max_pct=ax_airtime_max_pct,
        be_airtime_avg_pct=be_airtime_avg_pct,
        be_airtime_max_pct=be_airtime_max_pct,
    )


def entry_for_legacy_ax(parsed: ParsedEntry) -> Optional[dict]:
    if not (parsed.legacy_metrics and parsed.ax_mean and parsed.ax_max):
        return None
    return {
        "scenario": parsed.scenario,
        "legacy_standard": parsed.legacy_standard or "",
        "legacy_throughput_mbps": parsed.legacy_metrics.throughput_mbps,
        "legacy_delay_ms": parsed.legacy_metrics.delay_ms,
        "legacy_jitter_ms": parsed.legacy_metrics.jitter_ms,
        "legacy_airtime_pct": parsed.legacy_airtime_pct,
        "ax_throughput_mbps": parsed.ax_mean.throughput_mbps,
        "ax_delay_ms": parsed.ax_mean.delay_ms,
        "ax_jitter_ms": parsed.ax_mean.jitter_ms,
        "ax_max_throughput_mbps": parsed.ax_max.throughput_mbps,
        "ax_max_delay_ms": parsed.ax_max.delay_ms,
        "ax_max_jitter_ms": parsed.ax_max.jitter_ms,
        "ax_airtime_avg_pct": parsed.ax_airtime_avg_pct,
        "ax_airtime_max_pct": parsed.ax_airtime_max_pct,
    }


def entry_for_legacy_be(parsed: ParsedEntry) -> Optional[dict]:
    if not (parsed.legacy_metrics and parsed.be_mean and parsed.be_max):
        return None
    return {
        "scenario": parsed.scenario,
        "legacy_standard": parsed.legacy_standard or "",
        "legacy_throughput_mbps": parsed.legacy_metrics.throughput_mbps,
        "legacy_delay_ms": parsed.legacy_metrics.delay_ms,
        "legacy_jitter_ms": parsed.legacy_metrics.jitter_ms,
        "legacy_airtime_pct": parsed.legacy_airtime_pct,
        "be_throughput_mbps": parsed.be_mean.throughput_mbps,
        "be_delay_ms": parsed.be_mean.delay_ms,
        "be_jitter_ms": parsed.be_mean.jitter_ms,
        "be_max_throughput_mbps": parsed.be_max.throughput_mbps,
        "be_max_delay_ms": parsed.be_max.delay_ms,
        "be_max_jitter_ms": parsed.be_max.jitter_ms,
        "be_airtime_avg_pct": parsed.be_airtime_avg_pct,
        "be_airtime_max_pct": parsed.be_airtime_max_pct,
    }


def aggregate_logs(log_dir: Path) -> tuple[list[dict], list[dict]]:
    legacy_ax_entries: list[dict] = []
    legacy_be_entries: list[dict] = []

    for log_path in sorted(log_dir.glob("*.log")):
        if not log_path.is_file():
            continue
        parsed = parse_log(log_path)
        if not parsed:
            continue

        ax_entry = entry_for_legacy_ax(parsed)
        if ax_entry:
            legacy_ax_entries.append(ax_entry)

        be_entry = entry_for_legacy_be(parsed)
        if be_entry:
            legacy_be_entries.append(be_entry)

    return legacy_ax_entries, legacy_be_entries


def write_csv(entries: list[dict], output_path: Path, fieldnames: list[str]) -> None:
    if not entries:
        return

    with output_path.open("w", newline="", encoding="utf-8") as csvfile:
        writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
        writer.writeheader()
        for entry in entries:
            writer.writerow(entry)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Aggregate ns-3 log metrics into CSV files.")
    parser.add_argument(
        "--logs",
        type=Path,
        default=Path(__file__).resolve().parent / "logs",
        help="Directory containing ns-3 log files (default: %(default)s)",
    )
    parser.add_argument(
        "--out",
        dest="out_legacy_ax",
        type=Path,
        default=Path("aggregated_legacy_ax.csv"),
        help="Path to the legacy-vs-ax CSV file (default: %(default)s)",
    )
    parser.add_argument(
        "--out-legacy-ax",
        dest="out_legacy_ax",
        type=Path,
        help="Override path for the legacy-vs-ax CSV file.",
    )
    parser.add_argument(
        "--out-legacy-be",
        dest="out_legacy_be",
        type=Path,
        default=Path("aggregated_legacy_be.csv"),
        help="Path to the legacy-vs-be CSV file (default: %(default)s)",
    )
    return parser


def main() -> None:
    parser = build_parser()
    args = parser.parse_args()

    log_dir: Path = args.logs
    if not log_dir.exists():
        raise SystemExit(f"Log directory not found: {log_dir}")

    legacy_ax_entries, legacy_be_entries = aggregate_logs(log_dir)

    if legacy_ax_entries:
        write_csv(legacy_ax_entries, args.out_legacy_ax, LEGACY_AX_FIELDS)
        print(f"Wrote {len(legacy_ax_entries)} rows to {args.out_legacy_ax}")
    else:
        print("No legacy vs 802.11ax results found; skipping CSV generation.")

    if legacy_be_entries:
        write_csv(legacy_be_entries, args.out_legacy_be, LEGACY_BE_FIELDS)
        print(f"Wrote {len(legacy_be_entries)} rows to {args.out_legacy_be}")
    else:
        print("No legacy vs 802.11be results found; skipping CSV generation.")


if __name__ == "__main__":
    main()
