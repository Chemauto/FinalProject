#!/usr/bin/env python3
"""Summarize navigation evaluation reports by scene.

The script reads one or more ``navigation_eval_*.json`` files, aggregates
trial-level results by scene, and writes a table plus a figure similar to the
paper-style scene comparison chart.
"""

from __future__ import annotations

import argparse
import csv
import json
import statistics
from collections import defaultdict
from datetime import datetime
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_EVAL_DIR = PROJECT_ROOT / "eval_results"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Compute scene-wise navigation success rates.")
    parser.add_argument(
        "reports",
        nargs="*",
        type=Path,
        help="Evaluation JSON or Markdown files. Defaults to eval_results/navigation_eval_*.json.",
    )
    parser.add_argument("--eval-dir", type=Path, default=DEFAULT_EVAL_DIR)
    parser.add_argument("--output-dir", type=Path, default=DEFAULT_EVAL_DIR)
    parser.add_argument("--prefix", default=None, help="Output filename prefix.")
    parser.add_argument(
        "--planning-times",
        default=None,
        help="Manual planning times by scene, e.g. '30,40,45,43,44' or '0:30,1:40,2:45'.",
    )
    parser.add_argument(
        "--label-offset",
        type=int,
        default=1,
        help="Scene label offset. Use 1 to display scene_id=0 as 场景1.",
    )
    parser.add_argument("--title", default="导航任务各场景评测结果")
    parser.add_argument("--no-plot", action="store_true", help="Only write JSON/CSV summaries.")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    reports = args.reports or sorted(args.eval_dir.glob("navigation_eval_*.json"))
    if not reports:
        raise SystemExit(f"No evaluation JSON files found in {args.eval_dir}")

    rows = collect_trials(reports)
    if not rows:
        raise SystemExit("No trial results found in the selected reports.")

    summary = summarize_by_scene(rows, label_offset=args.label_offset)
    apply_manual_planning_times(summary, args.planning_times)
    args.output_dir.mkdir(parents=True, exist_ok=True)
    prefix = args.prefix or f"navigation_scene_summary_{datetime.now().strftime('%Y%m%d_%H%M%S')}"
    json_path = args.output_dir / f"{prefix}.json"
    csv_path = args.output_dir / f"{prefix}.csv"
    png_path = args.output_dir / f"{prefix}.png"

    json_path.write_text(json.dumps(summary, ensure_ascii=False, indent=2), encoding="utf-8")
    write_csv(summary, csv_path)
    if not args.no_plot:
        write_plot(summary, png_path, args.title)

    print(f"[summary] reports: {len(reports)}")
    print(f"[summary] trials: {sum(item['trial_count'] for item in summary)}")
    print_scene_table(summary)
    print(f"[summary] json: {json_path}")
    print(f"[summary] csv:  {csv_path}")
    if not args.no_plot:
        print(f"[summary] png:  {png_path}")
    return 0


def collect_trials(paths: list[Path]) -> list[dict]:
    rows: list[dict] = []
    for path in paths:
        if path.suffix.lower() == ".json":
            rows.extend(collect_json_trials(path))
        elif path.suffix.lower() == ".md":
            rows.extend(collect_markdown_trials(path))
        else:
            raise ValueError(f"Unsupported report format: {path}")
    return rows


def collect_json_trials(path: Path) -> list[dict]:
    data = json.loads(path.read_text(encoding="utf-8"))
    rows = []
    for item in data.get("results", []):
        row = dict(item)
        row["_source"] = str(path)
        rows.append(row)
    return rows


def collect_markdown_trials(path: Path) -> list[dict]:
    rows = []
    headers: list[str] | None = None
    for line in path.read_text(encoding="utf-8").splitlines():
        line = line.strip()
        if not line.startswith("|"):
            continue
        cells = [cell.strip() for cell in line.strip("|").split("|")]
        if cells and cells[0] == "scene":
            headers = cells
            continue
        if not headers:
            continue
        if len(cells) != len(headers) or cells[0].startswith("---"):
            continue
        values = dict(zip(headers, cells))
        try:
            scene_id = int(values["scene"])
            trial_no = int(values["trial"])
            error_xy = parse_optional_float(values.get("error_xy_m"))
            duration = parse_optional_float(values.get("duration_s"))
            planning = parse_optional_float(values.get("planning_s"))
        except ValueError:
            continue
        rows.append(
            {
                "scene_id": scene_id,
                "trial": trial_no,
                "passed": values.get("pass", "").upper() == "PASS",
                "error_xy_m": error_xy,
                "duration_sec": duration,
                "planning_duration_sec": planning,
                "failure": values.get("failure", "").strip(),
                "_source": str(path),
            }
        )
    return rows


def parse_optional_float(value: str | None) -> float | None:
    if value is None or value == "" or value == "None":
        return None
    return float(value)


def summarize_by_scene(rows: list[dict], label_offset: int) -> list[dict]:
    grouped: dict[int, list[dict]] = defaultdict(list)
    for row in rows:
        grouped[int(row["scene_id"])].append(row)

    summary = []
    for scene_id in sorted(grouped):
        items = grouped[scene_id]
        passed = sum(1 for item in items if bool(item.get("passed")))
        total = len(items)
        durations = [float(item["duration_sec"]) for item in items if item.get("duration_sec") is not None]
        errors = [float(item["error_xy_m"]) for item in items if item.get("error_xy_m") is not None]
        planning_times = [extract_planning_time(item) for item in items]
        known_planning_times = [value for value in planning_times if value is not None]
        summary.append(
            {
                "scene_id": scene_id,
                "scene_label": f"场景{scene_id + label_offset}",
                "trial_count": total,
                "success_count": passed,
                "failure_count": total - passed,
                "success_rate": round(passed / total, 4) if total else 0.0,
                "success_rate_percent": round(passed * 100.0 / total, 2) if total else 0.0,
                "avg_total_time_sec": round(mean(durations), 3),
                "sum_total_time_sec": round(sum(durations), 3),
                "avg_planning_time_sec": round(mean(known_planning_times), 3) if known_planning_times else None,
                "avg_error_xy_m": round(mean(errors), 4) if errors else None,
                "min_error_xy_m": round(min(errors), 4) if errors else None,
                "max_error_xy_m": round(max(errors), 4) if errors else None,
            }
        )
    return summary


def apply_manual_planning_times(summary: list[dict], value: str | None) -> None:
    if not value:
        return
    mapping = parse_planning_times(value)
    for index, item in enumerate(summary):
        scene_id = item["scene_id"]
        if scene_id in mapping:
            item["avg_planning_time_sec"] = mapping[scene_id]
        elif index in mapping:
            item["avg_planning_time_sec"] = mapping[index]


def parse_planning_times(value: str) -> dict[int, float]:
    value = value.strip()
    if not value:
        return {}
    mapping: dict[int, float] = {}
    parts = [part.strip() for part in value.split(",") if part.strip()]
    if all(":" not in part for part in parts):
        return {index: float(part) for index, part in enumerate(parts)}
    for part in parts:
        if ":" not in part:
            raise ValueError(f"Invalid planning time entry: {part}")
        key, time_value = part.split(":", 1)
        mapping[int(key.strip())] = float(time_value.strip())
    return mapping


def extract_planning_time(item: dict) -> float | None:
    for key in ("planning_duration_sec", "planning_time_sec", "planner_duration_sec"):
        if item.get(key) is not None:
            return float(item[key])
    return None


def mean(values: list[float]) -> float:
    if not values:
        return 0.0
    return statistics.fmean(values)


def write_csv(summary: list[dict], path: Path) -> None:
    fields = [
        "scene_id",
        "scene_label",
        "trial_count",
        "success_count",
        "failure_count",
        "success_rate",
        "success_rate_percent",
        "avg_total_time_sec",
        "sum_total_time_sec",
        "avg_planning_time_sec",
        "avg_error_xy_m",
        "min_error_xy_m",
        "max_error_xy_m",
    ]
    with path.open("w", encoding="utf-8", newline="") as file:
        writer = csv.DictWriter(file, fieldnames=fields)
        writer.writeheader()
        writer.writerows(summary)


def write_plot(summary: list[dict], path: Path, title: str) -> None:
    import matplotlib.pyplot as plt
    from matplotlib import font_manager

    font_name = configure_chinese_font(font_manager)
    if font_name:
        plt.rcParams["font.sans-serif"] = [font_name, "DejaVu Sans"]
        plt.rcParams["font.family"] = "sans-serif"
    else:
        plt.rcParams["font.sans-serif"] = ["DejaVu Sans"]
    plt.rcParams["axes.unicode_minus"] = False

    labels = [item["scene_label"] for item in summary]
    avg_planning = [item["avg_planning_time_sec"] or 0.0 for item in summary]
    has_planning_time = any(item["avg_planning_time_sec"] is not None for item in summary)
    avg_total = [item["avg_total_time_sec"] for item in summary]
    success = [item["success_rate_percent"] for item in summary]
    x_positions = list(range(len(labels)))
    width = 0.32

    fig, ax_time = plt.subplots(figsize=(9.6, 5.6), dpi=150)
    if has_planning_time:
        planning_bars = ax_time.bar(
            [x - width / 2 for x in x_positions],
            avg_planning,
            width,
            label="规划平均时间",
            color="#1f77b4",
        )
        total_positions = [x + width / 2 for x in x_positions]
        total_width = width
    else:
        planning_bars = []
        total_positions = x_positions
        total_width = 0.38
    total_bars = ax_time.bar(total_positions, avg_total, total_width, label="平均总时间", color="#ff7f0e")
    ax_time.set_ylabel("时间 / s")
    ax_time.set_xlabel("测试场景")
    ax_time.set_xticks(x_positions)
    ax_time.set_xticklabels(labels)
    ax_time.set_title(title)
    ax_time.grid(axis="y", linestyle="--", alpha=0.3)

    ax_success = ax_time.twinx()
    line = ax_success.plot(
        x_positions,
        success,
        marker="o",
        linewidth=2,
        color="#1f77b4",
        label="成功率",
    )
    ax_success.set_ylabel("成功率 / %")
    ax_success.set_ylim(0, 105)

    if has_planning_time:
        annotate_bars(ax_time, planning_bars)
    annotate_bars(ax_time, total_bars)
    for x, y in zip(x_positions, success):
        ax_success.annotate(
            f"{y:.0f}%",
            xy=(x, y),
            xytext=(0, -14 if y >= 96 else 8),
            textcoords="offset points",
            ha="center",
            fontsize=8,
        )

    handles, legend_labels = ax_time.get_legend_handles_labels()
    line_handles, line_labels = ax_success.get_legend_handles_labels()
    ax_time.legend(
        handles + line_handles,
        legend_labels + line_labels,
        loc="upper left",
        bbox_to_anchor=(0.01, 0.88),
        borderaxespad=0.0,
    )
    fig.tight_layout()
    fig.savefig(path)
    plt.close(fig)


def configure_chinese_font(font_manager) -> str | None:
    candidates = [
        Path("/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc"),
        Path("/usr/share/fonts/truetype/wqy/wqy-microhei.ttc"),
        Path("/usr/share/fonts/truetype/droid/DroidSansFallbackFull.ttf"),
        Path("/usr/share/fonts/truetype/arphic/uming.ttc"),
        Path("/usr/share/fonts/truetype/arphic/ukai.ttc"),
    ]
    for path in candidates:
        if path.exists():
            font_manager.fontManager.addfont(str(path))
            return font_manager.FontProperties(fname=str(path)).get_name()
    return None


def annotate_bars(axis, bars) -> None:
    for bar in bars:
        height = bar.get_height()
        if height <= 0:
            continue
        axis.annotate(
            f"{height:.0f}",
            xy=(bar.get_x() + bar.get_width() / 2, height),
            xytext=(0, 3),
            textcoords="offset points",
            ha="center",
            va="bottom",
            fontsize=8,
        )


def print_scene_table(summary: list[dict]) -> None:
    print("scene,trials,success,success_rate,avg_total_s,avg_planning_s,avg_error_xy_m")
    for item in summary:
        print(
            f"{item['scene_label']},{item['trial_count']},{item['success_count']},"
            f"{item['success_rate_percent']:.2f}%,{item['avg_total_time_sec']},"
            f"{item['avg_planning_time_sec']},{item['avg_error_xy_m']}"
        )


if __name__ == "__main__":
    raise SystemExit(main())
