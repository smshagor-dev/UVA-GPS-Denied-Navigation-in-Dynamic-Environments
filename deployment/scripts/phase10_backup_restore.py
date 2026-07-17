#!/usr/bin/env python3
from __future__ import annotations

import hashlib
import shutil
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from deployment.scripts.common import DOC_ROOT, RESULTS_ROOT, ensure_clean_dir, report_date, temp_dir, utc_now, write_json, write_markdown


OUTPUT_JSON = RESULTS_ROOT / "disaster_recovery_results.json"
OUTPUT_MD = DOC_ROOT / "DISASTER_RECOVERY_REPORT.md"


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    digest.update(path.read_bytes())
    return digest.hexdigest()


def main() -> int:
    backup_root = RESULTS_ROOT / "backup_payload"
    ensure_clean_dir(backup_root)
    tracked_paths = [
        Path("config"),
        Path("datasets/autonomy"),
        Path("docs/phase9"),
        Path("docs/phase10"),
        Path("research/ai"),
    ]
    manifest: list[dict[str, str]] = []
    for rel_path in tracked_paths:
        source = rel_path
        target = backup_root / rel_path
        if source.is_dir():
            shutil.copytree(source, target, dirs_exist_ok=True)
            for file_path in target.rglob("*"):
                if file_path.is_file():
                    manifest.append(
                        {
                            "path": str(file_path.relative_to(backup_root)),
                            "sha256": sha256_file(file_path),
                        }
                    )
        elif source.is_file():
            target.parent.mkdir(parents=True, exist_ok=True)
            shutil.copy2(source, target)
            manifest.append({"path": str(rel_path), "sha256": sha256_file(target)})
    archive_base = RESULTS_ROOT / "phase10_backup"
    shutil.make_archive(str(archive_base), "zip", root_dir=backup_root)
    with temp_dir("phase10-restore-") as restore_dir_text:
        restore_dir = Path(restore_dir_text)
        shutil.unpack_archive(str(archive_base) + ".zip", restore_dir)
        restored_manifest = []
        for file_path in restore_dir.rglob("*"):
            if file_path.is_file():
                restored_manifest.append(
                    {
                        "path": str(file_path.relative_to(restore_dir)),
                        "sha256": sha256_file(file_path),
                    }
                )
    manifest_sorted = sorted(manifest, key=lambda item: item["path"])
    restored_sorted = sorted(restored_manifest, key=lambda item: item["path"])
    payload = {
        "generated_at": utc_now(),
            "archive_path": str((archive_base.with_suffix(".zip")).relative_to(REPO_ROOT)),
        "file_count": len(manifest_sorted),
        "manifest_matches": manifest_sorted == restored_sorted,
        "tracked_roots": [str(path) for path in tracked_paths],
        "status": "PASS" if manifest_sorted == restored_sorted and len(manifest_sorted) > 0 else "FAIL",
    }
    write_json(OUTPUT_JSON, payload)
    write_markdown(
        OUTPUT_MD,
        [
            "# Phase 10 Disaster Recovery Report",
            "",
            f"Date: {report_date()}",
            "",
            "## Summary",
            "",
            f"- backup archive created: `{payload['archive_path']}`",
            f"- tracked files exported: `{payload['file_count']}`",
            f"- restore manifest matches: `{payload['manifest_matches']}`",
            "",
            "## Covered Material",
            "",
            "- configuration",
            "- autonomy datasets",
            "- AI research scaffolding",
            "- Phase 9 evidence",
            "- Phase 10 evidence",
            "",
            "## Verdict",
            "",
            f"Status: {payload['status']}",
        ],
    )
    return 0 if payload["status"] == "PASS" else 1


if __name__ == "__main__":
    raise SystemExit(main())
