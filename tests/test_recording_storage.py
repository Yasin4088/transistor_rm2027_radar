import tempfile
import unittest
from pathlib import Path
from unittest import mock

from recording_storage import (
    allocate_match_directory,
    allocate_named_match_directory,
    discover_external_mounts,
    prepare_match_recording_directory,
    select_recording_storage,
    suggest_recording_storage,
)


class RecordingStorageTest(unittest.TestCase):
    def setUp(self):
        self.temporary_directory = tempfile.TemporaryDirectory()
        self.root = Path(self.temporary_directory.name)
        self.mount_root = self.root / "media"
        self.external_mount = self.mount_root / "MATCH DISK"
        self.external_mount.mkdir(parents=True)
        self.sys_dev_block = self.root / "sys" / "dev" / "block"
        self.sys_dev_block.mkdir(parents=True)

    def tearDown(self):
        self.temporary_directory.cleanup()

    def _add_sysfs_device(self, device_number, path_parts, removable="0"):
        device_path = self.root.joinpath("sys", "devices", *path_parts)
        device_path.mkdir(parents=True)
        block_root = device_path.parent if device_path.name.endswith("1") else device_path
        (block_root / "removable").write_text(removable, encoding="ascii")
        (self.sys_dev_block / device_number).symlink_to(device_path)

    def _write_mountinfo(self, rows):
        mountinfo = self.root / "mountinfo"
        mountinfo.write_text("\n".join(rows) + "\n", encoding="utf-8")
        return mountinfo

    def test_discovers_only_mounted_external_block_devices(self):
        self._add_sysfs_device(
            "8:17",
            ("pci0000:00", "usb1", "1-2", "block", "sdb", "sdb1"),
        )
        internal_mount = self.mount_root / "internal"
        internal_mount.mkdir()
        self._add_sysfs_device(
            "259:2",
            ("pci0000:00", "nvme", "block", "nvme0n1", "nvme0n1p2"),
        )
        mountinfo = self._write_mountinfo(
            [
                "40 25 8:17 / "
                + str(self.external_mount).replace(" ", r"\040")
                + " rw,nosuid - ext4 /dev/sdb1 rw",
                f"41 25 259:2 / {internal_mount} rw - ext4 /dev/nvme0n1p2 rw",
                "42 25 0:99 / /dev rw - devtmpfs devtmpfs rw",
            ]
        )

        mounts = discover_external_mounts(
            mountinfo_path=mountinfo,
            sys_dev_block=self.sys_dev_block,
            mount_roots=(self.mount_root,),
        )

        self.assertEqual([mount.mount_point for mount in mounts], [self.external_mount])
        self.assertEqual(mounts[0].source, "/dev/sdb1")

    def test_read_only_external_mount_is_ignored(self):
        self._add_sysfs_device(
            "8:17",
            ("pci0000:00", "usb1", "1-2", "block", "sdb", "sdb1"),
        )
        mountinfo = self._write_mountinfo(
            [f"40 25 8:17 / {self.external_mount} ro,nosuid - ext4 /dev/sdb1 ro"]
        )

        mounts = discover_external_mounts(
            mountinfo_path=mountinfo,
            sys_dev_block=self.sys_dev_block,
            mount_roots=(self.mount_root,),
        )

        self.assertEqual(mounts, [])

    def test_prefers_external_storage_and_allocates_incrementing_directory(self):
        self._add_sysfs_device(
            "8:17",
            ("pci0000:00", "usb1", "1-2", "block", "sdb", "sdb1"),
        )
        encoded_mount = str(self.external_mount).replace(" ", r"\040")
        mountinfo = self._write_mountinfo(
            [f"40 25 8:17 / {encoded_mount} rw - ext4 /dev/sdb1 rw"]
        )
        default_root = self.root / "local-recordings"

        first, first_storage = prepare_match_recording_directory(
            default_root,
            external_directory="Transistor",
            mountinfo_path=mountinfo,
            sys_dev_block=self.sys_dev_block,
            mount_roots=(self.mount_root,),
        )
        second = allocate_match_directory(first_storage.root)

        self.assertTrue(first_storage.is_external)
        self.assertEqual(first, self.external_mount / "Transistor" / "game1")
        self.assertEqual(second, self.external_mount / "Transistor" / "game2")
        self.assertFalse(default_root.exists())

    def test_falls_back_to_default_without_external_disk(self):
        mountinfo = self._write_mountinfo([])
        default_root = self.root / "local-recordings"

        directory, storage = prepare_match_recording_directory(
            default_root,
            mountinfo_path=mountinfo,
            sys_dev_block=self.sys_dev_block,
            mount_roots=(self.mount_root,),
        )

        self.assertFalse(storage.is_external)
        self.assertEqual(directory, default_root / "game1")

    def test_named_session_does_not_depend_on_missing_component_counters(self):
        root = self.root / "recordings"

        first = allocate_named_match_directory(root, "match_20260731_120000_vision")
        second = allocate_named_match_directory(root, "match_20260731_120000_vision")

        self.assertEqual(first.name, "match_20260731_120000_vision")
        self.assertEqual(second.name, "match_20260731_120000_vision_01")

    def test_suggests_external_path_without_creating_it(self):
        self._add_sysfs_device(
            "8:17",
            ("pci0000:00", "usb1", "1-2", "block", "sdb", "sdb1"),
        )
        encoded_mount = str(self.external_mount).replace(" ", r"\040")
        mountinfo = self._write_mountinfo(
            [f"40 25 8:17 / {encoded_mount} rw - ext4 /dev/sdb1 rw"]
        )

        storage = suggest_recording_storage(
            self.root / "local-recordings",
            external_directory="Transistor",
            mountinfo_path=mountinfo,
            sys_dev_block=self.sys_dev_block,
            mount_roots=(self.mount_root,),
        )

        self.assertTrue(storage.is_external)
        self.assertEqual(storage.root, self.external_mount / "Transistor")
        self.assertFalse(storage.root.exists())

    def test_selected_path_is_used_exactly(self):
        mountinfo = self._write_mountinfo([])
        selected_root = self.root / "operator-choice"

        directory, storage = prepare_match_recording_directory(
            self.root / "local-recordings",
            selected_root=selected_root,
            mountinfo_path=mountinfo,
            sys_dev_block=self.sys_dev_block,
            mount_roots=(self.mount_root,),
        )

        self.assertFalse(storage.is_external)
        self.assertEqual(directory, selected_root / "game1")

    def test_selected_external_subdirectory_keeps_external_metadata(self):
        self._add_sysfs_device(
            "8:17",
            ("pci0000:00", "usb1", "1-2", "block", "sdb", "sdb1"),
        )
        encoded_mount = str(self.external_mount).replace(" ", r"\040")
        mountinfo = self._write_mountinfo(
            [f"40 25 8:17 / {encoded_mount} rw - ext4 /dev/sdb1 rw"]
        )
        selected_root = self.external_mount / "custom-recordings"

        directory, storage = prepare_match_recording_directory(
            self.root / "local-recordings",
            selected_root=selected_root,
            mountinfo_path=mountinfo,
            sys_dev_block=self.sys_dev_block,
            mount_roots=(self.mount_root,),
        )

        self.assertTrue(storage.is_external)
        self.assertEqual(storage.mount_point, self.external_mount)
        self.assertEqual(directory, selected_root / "game1")

    def test_unplugged_selected_external_path_falls_back_without_recreating_mount(self):
        mountinfo = self._write_mountinfo([])
        selected_root = self.mount_root / "UNPLUGGED" / "Transistor"
        default_root = self.root / "local-recordings"

        directory, storage = prepare_match_recording_directory(
            default_root,
            selected_root=selected_root,
            mountinfo_path=mountinfo,
            sys_dev_block=self.sys_dev_block,
            mount_roots=(self.mount_root,),
            session_name="match_unplugged_vision",
        )

        self.assertFalse(storage.is_external)
        self.assertEqual(directory, default_root / "match_unplugged_vision")
        self.assertFalse(selected_root.exists())

    def test_discovery_does_not_enumerate_dev_or_usb_bus(self):
        mountinfo = self._write_mountinfo([])
        with mock.patch("recording_storage.Path.iterdir") as iterdir:
            discover_external_mounts(
                mountinfo_path=mountinfo,
                sys_dev_block=self.sys_dev_block,
                mount_roots=(self.mount_root,),
            )
        iterdir.assert_not_called()

    def test_external_directory_cannot_escape_mount(self):
        with self.assertRaises(ValueError):
            select_recording_storage(
                self.root / "local-recordings",
                external_directory="../outside",
                mountinfo_path=self._write_mountinfo([]),
                sys_dev_block=self.sys_dev_block,
                mount_roots=(self.mount_root,),
            )


if __name__ == "__main__":
    unittest.main()
