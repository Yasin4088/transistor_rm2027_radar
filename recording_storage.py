import os
import re
import shutil
import stat
from dataclasses import dataclass
from pathlib import Path


DEFAULT_MOUNT_ROOTS = (Path("/media"), Path("/run/media"), Path("/mnt"))
_MOUNT_ESCAPE = re.compile(r"\\([0-7]{3})")
_USB_BUS_NAME = re.compile(r"usb\d+")
_SESSION_NAME = re.compile(r"^[A-Za-z0-9][A-Za-z0-9_.-]{0,95}$")


@dataclass(frozen=True)
class ExternalMount:
    mount_point: Path
    device_number: str
    source: str
    free_bytes: int


@dataclass(frozen=True)
class RecordingStorage:
    root: Path
    is_external: bool
    mount_point: Path | None = None
    source: str | None = None


def _decode_mount_field(value):
    return _MOUNT_ESCAPE.sub(lambda match: chr(int(match.group(1), 8)), value)


def _path_is_within(path, roots):
    for root in roots:
        try:
            path.relative_to(root)
            return True
        except ValueError:
            continue
    return False


def _mounted_block_devices(mountinfo_path):
    """Read mounted filesystems without enumerating USB or video devices."""
    try:
        lines = Path(mountinfo_path).read_text(encoding="utf-8").splitlines()
    except OSError:
        return

    for line in lines:
        left, separator, right = line.partition(" - ")
        if not separator:
            continue
        fields = left.split()
        filesystem_fields = right.split()
        if len(fields) < 6 or len(filesystem_fields) < 2:
            continue
        yield {
            "device_number": fields[2],
            "mount_point": Path(_decode_mount_field(fields[4])),
            "mount_options": frozenset(fields[5].split(",")),
            "filesystem": filesystem_fields[0],
            "source": _decode_mount_field(filesystem_fields[1]),
        }


def _is_external_block_device(device_number, sys_dev_block):
    """
    Inspect only the sysfs node belonging to an already-mounted block device.

    This intentionally never walks /sys/bus/usb, /dev, /dev/video*, or camera
    SDK device lists, so storage discovery cannot probe the vision interface.
    """
    device_link = Path(sys_dev_block) / device_number
    try:
        resolved = device_link.resolve(strict=True)
    except OSError:
        return False

    if any(_USB_BUS_NAME.fullmatch(part) for part in resolved.parts):
        return True

    for directory in (resolved, *resolved.parents):
        removable_path = directory / "removable"
        try:
            if removable_path.read_text(encoding="ascii").strip() == "1":
                return True
        except (OSError, UnicodeError):
            continue
    return False


def _backing_device_number(entry, sys_dev_block):
    """Resolve FUSE-backed mounts through their exact mounted /dev source."""
    device_number = entry["device_number"]
    if (Path(sys_dev_block) / device_number).exists():
        return device_number
    source = entry["source"]
    if not source.startswith("/dev/"):
        return device_number
    try:
        source_stat = os.stat(source)
    except OSError:
        return device_number
    if not stat.S_ISBLK(source_stat.st_mode):
        return device_number
    return f"{os.major(source_stat.st_rdev)}:{os.minor(source_stat.st_rdev)}"


def discover_external_mounts(
    mountinfo_path="/proc/self/mountinfo",
    sys_dev_block="/sys/dev/block",
    mount_roots=DEFAULT_MOUNT_ROOTS,
):
    """
    Return writable, mounted external block devices, largest free space first.

    Discovery is intentionally mount-driven. Unmounted disks and every
    non-block USB device (including industrial cameras) are ignored.
    """
    roots = tuple(Path(root).resolve() for root in mount_roots)
    mounts = []
    seen = set()
    for entry in _mounted_block_devices(mountinfo_path) or ():
        mount_point = entry["mount_point"]
        try:
            resolved_mount = mount_point.resolve(strict=True)
        except OSError:
            continue
        if resolved_mount in seen or not _path_is_within(resolved_mount, roots):
            continue
        if "ro" in entry["mount_options"]:
            continue
        device_number = _backing_device_number(entry, sys_dev_block)
        if not _is_external_block_device(device_number, sys_dev_block):
            continue
        if not os.access(resolved_mount, os.W_OK | os.X_OK):
            continue
        try:
            free_bytes = int(shutil.disk_usage(resolved_mount).free)
        except OSError:
            continue
        seen.add(resolved_mount)
        mounts.append(
            ExternalMount(
                mount_point=resolved_mount,
                device_number=device_number,
                source=entry["source"],
                free_bytes=free_bytes,
            )
        )
    return sorted(mounts, key=lambda mount: mount.free_bytes, reverse=True)


def suggest_recording_storage(
    default_root,
    prefer_external=True,
    external_directory="SHARK-radar-recordings",
    **discovery_options,
):
    """Return the preferred recording root without creating any directories."""
    external_relative = Path(str(external_directory))
    if external_relative.is_absolute() or ".." in external_relative.parts:
        raise ValueError("recording.external_directory 必须是外接硬盘内的相对目录")
    if prefer_external:
        mounts = discover_external_mounts(**discovery_options)
        if mounts:
            mount = mounts[0]
            return RecordingStorage(
                root=mount.mount_point / external_relative,
                is_external=True,
                mount_point=mount.mount_point,
                source=mount.source,
            )
    return RecordingStorage(root=Path(default_root).resolve(), is_external=False)


def select_recording_storage(
    default_root,
    prefer_external=True,
    external_directory="SHARK-radar-recordings",
    selected_root=None,
    **discovery_options,
):
    """Select and create the recording root, falling back to the local root."""
    external_relative = Path(str(external_directory))
    if external_relative.is_absolute() or ".." in external_relative.parts:
        raise ValueError("recording.external_directory 必须是外接硬盘内的相对目录")
    if selected_root is not None and str(selected_root).strip():
        root = Path(selected_root).resolve()
        for mount in discover_external_mounts(**discovery_options):
            if not _path_is_within(root, (mount.mount_point,)):
                continue
            root.mkdir(parents=True, exist_ok=True)
            return RecordingStorage(
                root=root,
                is_external=True,
                mount_point=mount.mount_point,
                source=mount.source,
            )
        # A launcher-selected path may point below a drive that has since been
        # unplugged. Never recreate that /media or /run/media path on the
        # internal filesystem and falsely report it as the selected disk.
        mount_roots = discovery_options.get("mount_roots", DEFAULT_MOUNT_ROOTS)
        resolved_mount_roots = tuple(Path(item).resolve() for item in mount_roots)
        if not _path_is_within(root, resolved_mount_roots):
            root.mkdir(parents=True, exist_ok=True)
            return RecordingStorage(root=root, is_external=False)

    if prefer_external:
        for mount in discover_external_mounts(**discovery_options):
            root = mount.mount_point / external_relative
            try:
                root.mkdir(parents=True, exist_ok=True)
            except OSError:
                continue
            return RecordingStorage(
                root=root,
                is_external=True,
                mount_point=mount.mount_point,
                source=mount.source,
            )

    root = Path(default_root).resolve()
    root.mkdir(parents=True, exist_ok=True)
    return RecordingStorage(root=root, is_external=False)


def allocate_match_directory(storage_root, prefix="game"):
    """Atomically allocate the next gameN directory without listing its files."""
    root = Path(storage_root)
    root.mkdir(parents=True, exist_ok=True)
    for index in range(1, 1_000_000):
        candidate = root / f"{prefix}{index}"
        try:
            candidate.mkdir()
            return candidate
        except FileExistsError:
            continue
    raise RuntimeError(f"录像目录数量过多，无法在 {root} 下分配新目录")


def allocate_named_match_directory(storage_root, session_name):
    """Atomically allocate a stable cross-component session directory."""
    name = str(session_name).strip()
    if not _SESSION_NAME.fullmatch(name):
        raise ValueError(f"无效的比赛录像 session_name: {session_name!r}")
    root = Path(storage_root)
    root.mkdir(parents=True, exist_ok=True)
    for index in range(1_000_000):
        suffix = "" if index == 0 else f"_{index:02d}"
        candidate = root / f"{name}{suffix}"
        try:
            candidate.mkdir()
            return candidate
        except FileExistsError:
            continue
    raise RuntimeError(f"同名录像目录数量过多，无法在 {root} 下分配 {name}")


def prepare_match_recording_directory(
    default_root,
    prefer_external=True,
    external_directory="SHARK-radar-recordings",
    selected_root=None,
    session_name=None,
    **discovery_options,
):
    """
    Select storage and allocate one match directory.

    If an external disk disappears between selection and allocation, retry once
    against the configured local default.
    """
    storage = select_recording_storage(
        default_root,
        prefer_external=prefer_external,
        external_directory=external_directory,
        selected_root=selected_root,
        **discovery_options,
    )
    try:
        directory = (
            allocate_named_match_directory(storage.root, session_name)
            if session_name
            else allocate_match_directory(storage.root)
        )
        return directory, storage
    except OSError:
        if not storage.is_external:
            raise
    fallback = select_recording_storage(default_root, prefer_external=False)
    directory = (
        allocate_named_match_directory(fallback.root, session_name)
        if session_name
        else allocate_match_directory(fallback.root)
    )
    return directory, fallback
