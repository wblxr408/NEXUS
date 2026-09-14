"""Generate the paginated full-sandbox truth table used by the report appendix.

The source is the validated E030 registry.  The generator deliberately keeps
the source coordinate field and the height-field name in the table: not every
registered object has a three-dimensional target reference point.
"""

from __future__ import annotations

import json
from pathlib import Path


ROOT = Path(__file__).resolve().parents[3]
SOURCE = ROOT / "experiments/runs/2026-09-07_E030_physical_sandbox_metric_reference/physical_sandbox_ground_truth_v01.json"
OUTPUT = ROOT / "defense/report/appendix_full_sandbox_truth_v01.tex"


def tex(value: str) -> str:
    return value.replace("_", r"\_").replace("&", r"\&")


def coordinate(item: dict) -> tuple[list[float], str]:
    for field in ("center_xy_m", "position_xy_m", "corner_anchor_xy_m"):
        if field in item:
            return item[field], field
    raise ValueError(f"No registered XY coordinate for {item['id']}")


def height(item: dict) -> tuple[str, str]:
    if "height_m" in item:
        return f"{item['height_m']:.4f}", "height_m"
    if "legacy_height_m" in item:
        return f"{item['legacy_height_m']:.4f}", "legacy_height_m"
    return "--", "not_registered"


def main() -> None:
    data = json.loads(SOURCE.read_text(encoding="utf-8"))
    items = sorted(data["items"], key=lambda item: (item["category"], item["id"]))
    lines = [
        "% Auto-generated from E030 physical_sandbox_ground_truth_v01.json.",
        "% Do not hand-edit; run generate_full_sandbox_truth_appendix.py after a registry update.",
        r"\begingroup\scriptsize",
        r"\begin{longtable}{p{3.35cm}p{1.8cm}rrp{1.3cm}p{2.7cm}}",
        rf"\caption{{E030 实体沙盘全部 {len(items)} 个登记物体的真值登记（\texttt{{physical\_sandbox\_map\_v01}}，m）}}\label{{tab:full-sandbox-truth}}\\",
        r"\toprule",
        r"物体 ID & 类别 & $x$ & $y$ & 高度字段值 & 坐标/高度字段 \\",
        r"\midrule",
        r"\endfirsthead",
        r"\toprule",
        r"物体 ID & 类别 & $x$ & $y$ & 高度字段值 & 坐标/高度字段 \\",
        r"\midrule",
        r"\endhead",
    ]
    for item in items:
        xy, coordinate_field = coordinate(item)
        height_value, height_field = height(item)
        lines.append(
            f"\\texttt{{{tex(item['id'])}}} & \\texttt{{{tex(item['category'])}}} & "
            f"{xy[0]:.4f} & {xy[1]:.4f} & {height_value} & "
            f"\\texttt{{{tex(coordinate_field)}}}; \\texttt{{{tex(height_field)}}} \\\\"
        )
    lines.extend([r"\bottomrule", r"\end{longtable}", r"\endgroup", ""])
    OUTPUT.write_text("\n".join(lines), encoding="utf-8")
    print(f"wrote {OUTPUT} with {len(items)} rows")


if __name__ == "__main__":
    main()
