"""Portable tests: python3 -m unittest discover -s <skill>/tests -p test_sim2sim_report_bundle.py."""

import importlib.util
import io
import json
from pathlib import Path
import subprocess
import sys
import tarfile
import tempfile
import unittest


SCRIPT = Path(__file__).resolve().parents[1] / "scripts/sim2sim_report_bundle.py"
spec = importlib.util.spec_from_file_location("sim2sim_report_bundle", SCRIPT)
bundle = importlib.util.module_from_spec(spec)
spec.loader.exec_module(bundle)


class BundleTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.root = Path(self.tmp.name)
        self.source = self.root / "sealed"
        self.source.mkdir()
        self.original = {
            "config_snapshot/assets/body.stl": b"mesh data\x00\xff" * 100,
            "config_snapshot/assets/skin.png": b"texture bytes\x00" * 80,
            "config_snapshot/scene.xml": b'<mujoco><mesh file="assets/body.stl"/></mujoco>',
            "config_snapshot/runtime.yaml": b"wheel_position: zero\n",
            "cases/stand/telemetry.jsonl.gz": b"already compressed telemetry\x00\xff",
            "cases/stand/side_view.mp4": b"video binary\x00\xfe",
            "tools/collect": b"#!/bin/sh\nexit 0\n",
            "manifest.json": b'{"sealed":true}\n',
            "report.md": b"Original report with trailing whitespace.  \n",
        }
        for name, data in self.original.items():
            p = self.source / name
            p.parent.mkdir(parents=True, exist_ok=True)
            p.write_bytes(data)
        (self.source / "tools/collect").chmod(0o755)
        self.resources = self.root / "resources.json"
        self.resources.write_text(json.dumps([n for n in self.original if "/assets/" in n]))
        self.cache = self.root / "cache"
        self.inventory = self.root / "inventory.json"
        self.archive = self.root / "transfer.tar.gz"
        self.output = self.root / "received"

    def confirm_resources(self):
        return bundle.build_inventory(self.source, self.resources, self.cache, "receiver", self.inventory)

    def pack(self, reuse=False):
        kwargs = {}
        if reuse:
            receipt = self.confirm_resources()
            kwargs = dict(inventory=self.inventory, inventory_sha256=receipt["sha256"], receiver_id="receiver")
        return bundle.pack(self.source, self.resources, self.archive, **kwargs)

    def receive(self, result, receiver_id=None):
        return bundle.receive(self.archive, result["sha256"], self.output, self.cache, receiver_id)

    def assert_original_unchanged(self):
        self.assertEqual({p.relative_to(self.source).as_posix(): p.read_bytes()
                          for p in self.source.rglob("*") if p.is_file()}, self.original)

    def assert_restored(self):
        for name, data in self.original.items():
            self.assertEqual((self.output / "payload" / name).read_bytes(), data, name)
        self.assertEqual((self.output / "payload/tools/collect").stat().st_mode & 0o777, 0o755)

    def test_first_delivery_is_full_and_byte_identical(self):
        result = self.pack()
        self.assertEqual(result["cached_resources"], 0)
        self.assertEqual(result["bundled_files"], len(self.original))
        self.assertFalse(self.cache.exists())
        self.receive(result)
        self.assert_restored()
        self.assert_original_unchanged()
        self.assertTrue(Path(str(self.archive) + ".sha256").read_text().startswith(result["sha256"]))

    def test_confirmed_reuse_omits_only_resources_and_restores_them(self):
        result = self.pack(reuse=True)
        self.assertEqual(result["cached_resources"], 2)
        with tarfile.open(self.archive) as archive:
            names = archive.getnames()
            self.assertNotIn("payload/config_snapshot/assets/body.stl", names)
            self.assertIn("payload/config_snapshot/scene.xml", names)
            self.assertIn("payload/cases/stand/telemetry.jsonl.gz", names)
            self.assertIn("payload/cases/stand/side_view.mp4", names)
        self.receive(result, "receiver")
        self.assert_restored()
        self.assert_original_unchanged()

    def test_changed_resource_is_bundled(self):
        receipt = self.confirm_resources()
        changed = self.source / "config_snapshot/assets/body.stl"
        changed.write_bytes(b"changed mesh")
        result = bundle.pack(self.source, self.resources, self.archive, self.inventory, receipt["sha256"], "receiver")
        self.assertEqual(result["cached_resources"], 1)
        self.receive(result, "receiver")
        self.assertEqual((self.output / "payload/config_snapshot/assets/body.stl").read_bytes(), b"changed mesh")

    def test_missing_cache_is_reported_without_publishing(self):
        result = self.pack(reuse=True)
        cache_file = next(self.cache.iterdir())
        cache_file.unlink()
        with self.assertRaises(bundle.MissingResources) as ctx:
            self.receive(result, "receiver")
        self.assertEqual(ctx.exception.items[0]["sha256"], cache_file.name)
        self.assertFalse(self.output.exists())
        self.assertFalse(list(self.root.glob(".sim2sim-receive-*")))

    def test_corrupt_cache_is_rejected_even_with_matching_size(self):
        result = self.pack(reuse=True)
        cache_file = next(self.cache.iterdir())
        cache_file.write_bytes(b"x" * cache_file.stat().st_size)
        with self.assertRaises(bundle.MissingResources):
            self.receive(result, "receiver")
        self.assertFalse(self.output.exists())

    def test_receiver_binding_and_inventory_hash_are_required(self):
        receipt = self.confirm_resources()
        for sha, receiver in [("0" * 64, "receiver"), (receipt["sha256"], "different-host")]:
            with self.subTest(receiver=receiver), self.assertRaises(ValueError):
                bundle.pack(self.source, self.resources, self.archive, self.inventory, sha, receiver)
        self.assertFalse(self.archive.exists())
        result = bundle.pack(self.source, self.resources, self.archive, self.inventory, receipt["sha256"], "receiver")
        with self.assertRaisesRegex(ValueError, "receiver ID"):
            self.receive(result, "different-host")

    def test_archive_digest_and_no_overwrite(self):
        result = self.pack()
        with self.assertRaisesRegex(ValueError, "already exists"):
            self.pack()
        with self.assertRaisesRegex(ValueError, "SHA-256"):
            bundle.receive(self.archive, "0" * 64, self.output, self.cache)
        self.receive(result)
        with self.assertRaisesRegex(ValueError, "already exists"):
            self.receive(result)

    def test_inventory_does_not_modify_sealed_root_or_attest_bad_cache(self):
        with self.assertRaisesRegex(ValueError, "outside the sealed"):
            bundle.build_inventory(self.source, self.resources, self.source / "cache", "receiver", self.inventory)
        self.assert_original_unchanged()
        self.confirm_resources()
        cache_file = next(self.cache.iterdir())
        cache_file.write_bytes(b"bad cache")
        next_inventory = self.root / "next-inventory.json"
        with self.assertRaisesRegex(ValueError, "Corrupt cached"):
            bundle.build_inventory(self.source, self.resources, self.cache, "receiver", next_inventory)
        self.assertFalse(next_inventory.exists())

    def test_config_paths_and_symlink_resources_are_rejected(self):
        for name in ["config_snapshot/scene.xml", "../outside.stl"]:
            self.resources.write_text(json.dumps([name]))
            with self.assertRaises(ValueError):
                self.pack()
        target = self.source / "config_snapshot/assets/alias.stl"
        target.symlink_to(self.source / "config_snapshot/assets/body.stl")
        self.resources.write_text(json.dumps(["config_snapshot/assets/alias.stl"]))
        with self.assertRaisesRegex(ValueError, "Symlink"):
            self.pack()

    def test_tampered_payload_rejected_after_archive_digest_matches(self):
        self.pack()
        modified = self.root / "tampered.tar.gz"
        with tarfile.open(self.archive) as original, tarfile.open(modified, "w:gz") as altered:
            for member in original.getmembers():
                payload = original.extractfile(member).read()
                if member.name == "payload/report.md":
                    payload = b"x" * len(payload)
                altered.addfile(member, io.BytesIO(payload))
        with self.assertRaisesRegex(ValueError, "Payload SHA-256"):
            bundle.receive(modified, bundle.digest(modified), self.output, self.cache)
        self.assertFalse(self.output.exists())
        self.assertFalse(list(self.root.glob(".sim2sim-receive-*")))

    def test_archive_traversal_rejected(self):
        self.pack()
        modified = self.root / "unsafe.tar.gz"
        with tarfile.open(self.archive) as original, tarfile.open(modified, "w:gz") as altered:
            for member in original.getmembers():
                payload = original.extractfile(member).read()
                if member.name == bundle.MANIFEST:
                    manifest = json.loads(payload)
                    manifest["files"][0]["path"] = "../escaped"
                    payload = json.dumps(manifest).encode()
                    member.size = len(payload)
                altered.addfile(member, io.BytesIO(payload))
        with self.assertRaisesRegex(ValueError, "Unsafe relative path"):
            bundle.receive(modified, bundle.digest(modified), self.output, self.cache)
        self.assertFalse((self.root / "escaped").exists())

    def test_cli_runs_without_site_packages(self):
        # -I -S excludes workspace imports, user site and installed dependencies.
        result = subprocess.run([sys.executable, "-I", "-S", str(SCRIPT), "pack", "--source", str(self.source),
                                 "--resources", str(self.resources), "--output", str(self.archive)],
                                capture_output=True, text=True)
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        receipt = json.loads(result.stdout)
        result = subprocess.run([sys.executable, "-I", "-S", str(SCRIPT), "receive", "--archive", str(self.archive),
                                 "--sha256", receipt["sha256"], "--output", str(self.output), "--cache", str(self.cache)],
                                capture_output=True, text=True)
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        self.assert_restored()


if __name__ == "__main__":
    unittest.main()
