#!/usr/bin/env python3

from __future__ import annotations

import importlib.util
import json
import os
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path
from unittest import mock


SCRIPT = (
    Path(__file__).resolve().parents[1]
    / "scripts"
    / "inspect_context_compactions.py"
)
SPEC = importlib.util.spec_from_file_location("inspect_context_compactions", SCRIPT)
assert SPEC and SPEC.loader
MODULE = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = MODULE
SPEC.loader.exec_module(MODULE)


THREAD_ID = "019fb726-2ddf-7763-acb3-59988faeb614"


def session_meta(thread_id: str = THREAD_ID) -> dict:
    return {
        "timestamp": "2026-08-01T00:00:00Z",
        "type": "session_meta",
        "payload": {"id": thread_id, "session_id": thread_id},
    }


def compacted(window_number: int) -> dict:
    return {
        "timestamp": f"2026-08-01T00:00:0{window_number}Z",
        "type": "compacted",
        "payload": {
            "window_number": window_number,
            "replacement_history": [
                {"type": "message", "text": 'nested "type":"compacted" text'}
            ],
        },
    }


def context_compacted() -> dict:
    return {
        "timestamp": "2026-08-01T00:00:09Z",
        "type": "event_msg",
        "payload": {"type": "context_compacted"},
    }


class ContextCompactionInspectionTests(unittest.TestCase):
    def setUp(self) -> None:
        self.temporary_directory = tempfile.TemporaryDirectory()
        self.addCleanup(self.temporary_directory.cleanup)
        self.sessions_root = Path(self.temporary_directory.name)

    def write_rollout(
        self,
        records: list[dict],
        *,
        parent: str = "2026/08/01",
        sessions_root: Path | None = None,
    ) -> Path:
        directory = (sessions_root or self.sessions_root) / parent
        directory.mkdir(parents=True, exist_ok=True)
        path = directory / f"rollout-test-{THREAD_ID}.jsonl"
        path.write_text(
            "".join(json.dumps(record) + "\n" for record in records),
            encoding="utf-8",
        )
        return path

    def inspect(self, **kwargs: object) -> dict:
        return MODULE.inspect_context_compactions(
            sessions_root=self.sessions_root,
            threshold=5,
            thread_id=THREAD_ID,
            retry_delay_seconds=0,
            **kwargs,
        )

    def run_cli(
        self,
        *arguments: str,
        environment: dict[str, str] | None = None,
    ) -> dict:
        cli_environment = os.environ.copy()
        if environment:
            cli_environment.update(environment)
        completed = subprocess.run(
            [sys.executable, str(SCRIPT), *arguments],
            check=True,
            capture_output=True,
            text=True,
            env=cli_environment,
        )
        return json.loads(completed.stdout)

    def test_zero_compactions_is_available(self) -> None:
        self.write_rollout([session_meta()])
        report = self.inspect()
        self.assertEqual(report["status"], "available")
        self.assertEqual(report["compaction_count"], 0)
        self.assertFalse(report["threshold_reached"])

    def test_two_compactions_are_counted_once_each(self) -> None:
        self.write_rollout(
            [
                session_meta(),
                compacted(1),
                context_compacted(),
                compacted(2),
                context_compacted(),
            ]
        )
        report = self.inspect()
        self.assertEqual(report["compaction_count"], 2)
        self.assertEqual(report["window_numbers"], [1, 2])
        self.assertEqual(report["event_cross_check"]["context_compacted_count"], 2)
        self.assertNotIn("nested", json.dumps(report))

    def test_five_compactions_reach_threshold(self) -> None:
        records = [session_meta()]
        for window_number in range(1, 6):
            records.extend([compacted(window_number), context_compacted()])
        self.write_rollout(records)
        report = self.inspect()
        self.assertEqual(report["compaction_count"], 5)
        self.assertTrue(report["threshold_reached"])

    def test_gap_in_window_numbers_is_inconsistent(self) -> None:
        self.write_rollout(
            [
                session_meta(),
                compacted(1),
                context_compacted(),
                compacted(3),
                context_compacted(),
            ]
        )
        report = self.inspect()
        self.assertEqual(report["status"], "inconsistent")
        self.assertIsNone(report["compaction_count"])
        self.assertIn("compaction_window_sequence_invalid", report["errors"])

    def test_event_count_mismatch_is_inconsistent(self) -> None:
        self.write_rollout([session_meta(), compacted(1)])
        report = self.inspect()
        self.assertEqual(report["status"], "inconsistent")
        self.assertIn("context_compacted_event_count_mismatch", report["errors"])

    def test_missing_thread_id_is_unavailable(self) -> None:
        report = MODULE.inspect_context_compactions(
            sessions_root=self.sessions_root,
            threshold=5,
            thread_id=None,
        )
        self.assertEqual(report["status"], "unavailable")
        self.assertIn("codex_thread_id_unavailable", report["errors"])

    def test_invalid_thread_id_is_unavailable(self) -> None:
        report = MODULE.inspect_context_compactions(
            sessions_root=self.sessions_root,
            threshold=5,
            thread_id="../../*",
        )
        self.assertEqual(report["status"], "unavailable")
        self.assertIn("invalid_thread_id", report["errors"])

    def test_rollout_thread_id_mismatch_is_inconsistent(self) -> None:
        path = self.write_rollout([session_meta("different-thread")])
        report = self.inspect(rollout_path=path)
        self.assertEqual(report["status"], "inconsistent")
        self.assertIn("rollout_thread_id_mismatch", report["errors"])

    def test_explicit_rollout_infers_thread_id(self) -> None:
        path = self.write_rollout([session_meta()])
        report = MODULE.inspect_context_compactions(
            sessions_root=self.sessions_root,
            threshold=5,
            thread_id=None,
            rollout_path=path,
        )
        self.assertEqual(report["status"], "available")
        self.assertEqual(report["thread_id"], THREAD_ID)

    def test_cli_explicit_rollout_ignores_environment_thread_id(self) -> None:
        path = self.write_rollout([session_meta()])
        report = self.run_cli(
            "--rollout",
            str(path),
            environment={"CODEX_THREAD_ID": "different-current-thread"},
        )
        self.assertEqual(report["status"], "available")
        self.assertEqual(report["thread_id"], THREAD_ID)

    def test_cli_uses_codex_home_for_automatic_lookup(self) -> None:
        codex_home = self.sessions_root / "codex-home"
        sessions_root = codex_home / "sessions"
        self.write_rollout([session_meta()], sessions_root=sessions_root)
        report = self.run_cli(
            environment={
                "CODEX_HOME": str(codex_home),
                "CODEX_THREAD_ID": THREAD_ID,
            }
        )
        self.assertEqual(report["status"], "available")
        self.assertEqual(report["thread_id"], THREAD_ID)

    def test_multiple_matching_rollouts_are_unavailable(self) -> None:
        records = [session_meta()]
        self.write_rollout(records, parent="2026/08/01")
        self.write_rollout(records, parent="2026/08/02")
        report = self.inspect()
        self.assertEqual(report["status"], "unavailable")
        self.assertIn("multiple_rollouts_match_thread_id", report["errors"])

    def test_malformed_json_is_inconsistent(self) -> None:
        path = self.write_rollout([session_meta()])
        with path.open("a", encoding="utf-8") as stream:
            stream.write("{not-json}\n")
        report = self.inspect()
        self.assertEqual(report["status"], "inconsistent")
        self.assertTrue(
            any(error.startswith("invalid_json_line:") for error in report["errors"])
        )

    def test_transient_event_mismatch_is_retried(self) -> None:
        inconsistent = MODULE._base_result(THREAD_ID, 5)
        inconsistent["status"] = "inconsistent"
        inconsistent["errors"] = ["context_compacted_event_count_mismatch"]
        available = MODULE._base_result(THREAD_ID, 5)
        available["status"] = "available"
        available["compaction_count"] = 1
        available["threshold_reached"] = False
        with (
            mock.patch.object(
                MODULE,
                "_inspect_rollout_once",
                side_effect=[inconsistent, available],
            ) as inspect_once,
            mock.patch.object(MODULE, "_rollout_signature", return_value=(1, 1)),
        ):
            report = MODULE._inspect_rollout(
                Path("unused.jsonl"),
                THREAD_ID,
                5,
                stability_retries=1,
                retry_delay_seconds=0,
            )
        self.assertEqual(report["status"], "available")
        self.assertEqual(inspect_once.call_count, 2)

    def test_continuously_changing_rollout_is_unavailable(self) -> None:
        available = MODULE._base_result(THREAD_ID, 5)
        available["status"] = "available"
        available["compaction_count"] = 0
        available["threshold_reached"] = False
        with (
            mock.patch.object(MODULE, "_inspect_rollout_once", return_value=available),
            mock.patch.object(
                MODULE,
                "_rollout_signature",
                side_effect=[(1, 1), (2, 2), (2, 2), (3, 3)],
            ),
        ):
            report = MODULE._inspect_rollout(
                Path("unused.jsonl"),
                THREAD_ID,
                5,
                stability_retries=1,
                retry_delay_seconds=0,
            )
        self.assertEqual(report["status"], "unavailable")
        self.assertIn("rollout_changed_during_read", report["errors"])


if __name__ == "__main__":
    unittest.main()
