#!/usr/bin/env python3
"""
End-to-end evaluation runner for set-valued-viz.

Outputs:
- hardware.json
- command_timings.csv
- backend_performance.json / backend_performance.csv
- frontend_performance.json / frontend_performance.csv
- frontend_dist_sizes.csv
- summary.md
"""

from __future__ import annotations

import argparse
import csv
import json
import platform
import subprocess
import sys
import time
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Any


@dataclass
class CommandResult:
    name: str
    cmd: list[str]
    cwd: str
    returncode: int
    duration_sec: float
    stdout_path: str
    stderr_path: str
    ok: bool
    error: str | None = None


def utc_timestamp() -> str:
    return datetime.now(timezone.utc).strftime("%Y%m%d_%H%M%S")


def run_command(
    name: str,
    cmd: list[str],
    cwd: Path,
    output_dir: Path,
    env: dict[str, str] | None = None,
) -> CommandResult:
    stdout_path = output_dir / f"{name}.stdout.log"
    stderr_path = output_dir / f"{name}.stderr.log"
    started = time.perf_counter()

    try:
        proc = subprocess.run(
            cmd,
            cwd=str(cwd),
            env=env,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
            check=False,
        )
        elapsed = time.perf_counter() - started
        stdout_path.write_text(proc.stdout, encoding="utf-8", errors="replace")
        stderr_path.write_text(proc.stderr, encoding="utf-8", errors="replace")
        return CommandResult(
            name=name,
            cmd=cmd,
            cwd=str(cwd),
            returncode=proc.returncode,
            duration_sec=elapsed,
            stdout_path=str(stdout_path),
            stderr_path=str(stderr_path),
            ok=(proc.returncode == 0),
        )
    except FileNotFoundError as exc:
        elapsed = time.perf_counter() - started
        stdout_path.write_text("", encoding="utf-8")
        stderr_path.write_text(str(exc), encoding="utf-8")
        return CommandResult(
            name=name,
            cmd=cmd,
            cwd=str(cwd),
            returncode=127,
            duration_sec=elapsed,
            stdout_path=str(stdout_path),
            stderr_path=str(stderr_path),
            ok=False,
            error=str(exc),
        )


def quick_output(cmd: list[str], cwd: Path) -> str | None:
    try:
        proc = subprocess.run(
            cmd,
            cwd=str(cwd),
            stdout=subprocess.PIPE,
            stderr=subprocess.DEVNULL,
            text=True,
            check=False,
        )
        if proc.returncode != 0:
            return None
        return proc.stdout.strip()
    except FileNotFoundError:
        return None


def collect_hardware_info(root: Path) -> dict[str, Any]:
    info: dict[str, Any] = {
        "collected_at_utc": datetime.now(timezone.utc).isoformat(),
        "platform": {
            "system": platform.system(),
            "release": platform.release(),
            "version": platform.version(),
            "machine": platform.machine(),
            "processor": platform.processor(),
        },
        "python": sys.version,
    }

    info["tool_versions"] = {
        "cargo": quick_output(["cargo", "--version"], root),
        "rustc": quick_output(["rustc", "--version"], root),
        "node": quick_output(["node", "--version"], root),
        "npm": quick_output(["npm", "--version"], root),
    }

    sysctl_cpu = quick_output(["sysctl", "-n", "machdep.cpu.brand_string"], root)
    sysctl_cores = quick_output(["sysctl", "-n", "hw.ncpu"], root)
    sysctl_mem = quick_output(["sysctl", "-n", "hw.memsize"], root)
    sw_vers = quick_output(["sw_vers"], root)
    uname = quick_output(["uname", "-a"], root)

    if sysctl_mem and sysctl_mem.isdigit():
        mem_bytes = int(sysctl_mem)
        mem_gib = mem_bytes / (1024 ** 3)
    else:
        mem_bytes = None
        mem_gib = None

    info["host"] = {
        "cpu_model": sysctl_cpu,
        "logical_cores": int(sysctl_cores) if sysctl_cores and sysctl_cores.isdigit() else None,
        "memory_bytes": mem_bytes,
        "memory_gib": round(mem_gib, 2) if mem_gib is not None else None,
        "os_version_raw": sw_vers,
        "uname": uname,
    }
    return info


def write_command_csv(results: list[CommandResult], path: Path) -> None:
    with path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(
            f,
            fieldnames=[
                "name",
                "ok",
                "returncode",
                "duration_sec",
                "cmd",
                "cwd",
                "stdout_path",
                "stderr_path",
                "error",
            ],
        )
        writer.writeheader()
        for r in results:
            writer.writerow(
                {
                    "name": r.name,
                    "ok": int(r.ok),
                    "returncode": r.returncode,
                    "duration_sec": f"{r.duration_sec:.4f}",
                    "cmd": " ".join(r.cmd),
                    "cwd": r.cwd,
                    "stdout_path": r.stdout_path,
                    "stderr_path": r.stderr_path,
                    "error": r.error or "",
                }
            )


def write_perf_csv(json_path: Path, csv_path: Path) -> None:
    payload = json.loads(json_path.read_text(encoding="utf-8"))
    records = payload.get("records", [])
    with csv_path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(
            f,
            fieldnames=[
                "case_id",
                "category",
                "scenario",
                "duration_ms",
                "params_json",
                "outputs_json",
            ],
        )
        writer.writeheader()
        for rec in records:
            writer.writerow(
                {
                    "case_id": rec.get("case_id", ""),
                    "category": rec.get("category", ""),
                    "scenario": rec.get("scenario", ""),
                    "duration_ms": rec.get("duration_ms", rec.get("avg_ms", "")),
                    "params_json": json.dumps(rec.get("params", {}), separators=(",", ":")),
                    "outputs_json": json.dumps(rec.get("outputs", {}), separators=(",", ":")),
                }
            )


def collect_dist_sizes(frontend_dir: Path, out_csv: Path) -> None:
    dist_dir = frontend_dir / "dist"
    rows: list[dict[str, Any]] = []
    if dist_dir.exists():
        for file in sorted(dist_dir.rglob("*")):
            if file.is_file():
                size = file.stat().st_size
                rows.append(
                    {
                        "path": str(file.relative_to(frontend_dir)),
                        "size_bytes": size,
                        "size_kb": round(size / 1024.0, 3),
                    }
                )
    with out_csv.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=["path", "size_bytes", "size_kb"])
        writer.writeheader()
        writer.writerows(rows)


def write_summary(
    out_path: Path,
    run_id: str,
    hardware_path: Path,
    command_results: list[CommandResult],
    backend_perf_json: Path,
    frontend_perf_json: Path,
) -> None:
    failures = [r for r in command_results if not r.ok]
    lines: list[str] = []
    lines.append(f"# Evaluation Summary ({run_id})")
    lines.append("")
    lines.append(f"- Hardware: `{hardware_path}`")
    lines.append(f"- Commands run: {len(command_results)}")
    lines.append(f"- Failed commands: {len(failures)}")
    lines.append("")

    lines.append("## Command Results")
    lines.append("")
    for r in command_results:
        status = "PASS" if r.ok else "FAIL"
        lines.append(
            f"- `{r.name}`: {status} ({r.duration_sec:.2f}s) | log: `{r.stdout_path}`"
        )

    if backend_perf_json.exists():
        payload = json.loads(backend_perf_json.read_text(encoding="utf-8"))
        records = payload.get("records", [])
        lines.append("")
        lines.append("## Backend Performance Cases")
        lines.append("")
        for rec in records:
            duration = rec.get("duration_ms", 0.0)
            lines.append(
                f"- `{rec.get('case_id')}` ({rec.get('scenario')}): {duration:.2f} ms"
            )

    if frontend_perf_json.exists():
        payload = json.loads(frontend_perf_json.read_text(encoding="utf-8"))
        records = payload.get("records", [])
        lines.append("")
        lines.append("## Frontend Runtime-Prep Cases")
        lines.append("")
        for rec in records:
            avg_ms = rec.get("avg_ms", 0.0)
            lines.append(
                f"- `{rec.get('case_id')}` ({rec.get('scenario')}): avg {avg_ms:.2f} ms"
            )

    out_path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def main() -> int:
    parser = argparse.ArgumentParser(description="Run frontend/backend evaluation suite.")
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=None,
        help="Output directory. Default: evaluation/results_<timestamp>",
    )
    parser.add_argument(
        "--skip-frontend",
        action="store_true",
        help="Skip frontend correctness/performance steps.",
    )
    parser.add_argument(
        "--skip-backend",
        action="store_true",
        help="Skip backend correctness/performance steps.",
    )
    args = parser.parse_args()

    root = Path(__file__).resolve().parents[2]
    frontend_dir = root / "frontend"

    run_id = utc_timestamp()
    out_dir = args.output_dir or (root / "evaluation" / f"results_{run_id}")
    out_dir.mkdir(parents=True, exist_ok=True)

    hardware = collect_hardware_info(root)
    hardware_path = out_dir / "hardware.json"
    hardware_path.write_text(json.dumps(hardware, indent=2), encoding="utf-8")

    results: list[CommandResult] = []

    backend_perf_json = out_dir / "backend_performance.json"
    frontend_perf_json = out_dir / "frontend_performance.json"

    if not args.skip_backend:
        results.append(
            run_command(
                "backend_correctness_all_tests",
                ["cargo", "test", "--release"],
                root,
                out_dir,
            )
        )
        results.append(
            run_command(
                "backend_perf_benchmarks",
                [
                    "cargo",
                    "run",
                    "--release",
                    "--bin",
                    "eval_perf",
                    "--",
                    "--out",
                    str(backend_perf_json),
                ],
                root,
                out_dir,
            )
        )

    if not args.skip_frontend:
        if not (frontend_dir / "node_modules").exists():
            results.append(
                run_command("frontend_npm_install", ["npm", "install"], frontend_dir, out_dir)
            )

        results.append(
            run_command("frontend_correctness_tests", ["npm", "run", "test"], frontend_dir, out_dir)
        )
        results.append(
            run_command("frontend_build", ["npm", "run", "build"], frontend_dir, out_dir)
        )
        results.append(
            run_command(
                "frontend_runtime_prep_perf",
                ["node", "scripts/eval_frontend_perf.mjs", "--out", str(frontend_perf_json)],
                frontend_dir,
                out_dir,
            )
        )

    write_command_csv(results, out_dir / "command_timings.csv")

    if backend_perf_json.exists():
        write_perf_csv(backend_perf_json, out_dir / "backend_performance.csv")
    if frontend_perf_json.exists():
        write_perf_csv(frontend_perf_json, out_dir / "frontend_performance.csv")

    if not args.skip_frontend:
        collect_dist_sizes(frontend_dir, out_dir / "frontend_dist_sizes.csv")

    write_summary(
        out_dir / "summary.md",
        run_id,
        hardware_path,
        results,
        backend_perf_json,
        frontend_perf_json,
    )

    failed_required = [r for r in results if not r.ok]
    return 1 if failed_required else 0


if __name__ == "__main__":
    raise SystemExit(main())
