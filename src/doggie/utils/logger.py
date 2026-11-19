from __future__ import annotations

import csv
import json
from pathlib import Path


class TrialLogger:
    """单次试验的数据记录：时间序列 CSV + 元数据 JSON。"""

    COLUMNS = [
        "t",
        "state",
        "mode",
        "roll_deg",
        "pitch_deg",
        "height_m",
        "feet",
        "attempts",
        "fault",
        "detected",
        "lp_roll_deg",
        "lp_pitch_deg",
    ]

    def __init__(self, run_dir: Path) -> None:
        self.run_dir = run_dir
        run_dir.mkdir(parents=True, exist_ok=True)
        self._rows: list[dict] = []

    def row(self, **kw) -> None:
        self._rows.append({c: kw.get(c, "") for c in self.COLUMNS})

    def finish(self, meta: dict) -> None:
        with open(self.run_dir / "timeseries.csv", "w", newline="") as f:
            w = csv.DictWriter(f, fieldnames=self.COLUMNS)
            w.writeheader()
            w.writerows(self._rows)
        with open(self.run_dir / "meta.json", "w") as f:
            json.dump(meta, f, indent=2, ensure_ascii=False)


def load_config(path: Path) -> dict:
    import yaml

    with open(path) as f:
        return yaml.safe_load(f)
