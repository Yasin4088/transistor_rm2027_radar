import os
from pathlib import Path


MVS_LIBRARY_RELATIVE_PATH = Path("64") / "libMvCameraControl.so"
DEFAULT_MVS_RUNTIME_ROOTS = (
    Path("/opt/MVS/lib"),
    Path("/opt/MVS"),
    Path("/usr/local/MVS/lib"),
)


def find_mvs_runtime_root(environ=None, candidates=None):
    """Return the runtime root whose 64-bit MVS control library exists."""
    environ = os.environ if environ is None else environ
    roots = []
    configured = str(environ.get("MVCAM_COMMON_RUNENV", "") or "").strip()
    if configured:
        roots.append(Path(configured).expanduser())
    roots.extend(DEFAULT_MVS_RUNTIME_ROOTS if candidates is None else candidates)

    seen = set()
    for root in roots:
        root = Path(root).expanduser()
        normalized = str(root)
        if normalized in seen:
            continue
        seen.add(normalized)
        if (root / MVS_LIBRARY_RELATIVE_PATH).is_file():
            return root
    return None


def configure_mvs_runtime(environ=None, candidates=None):
    environ = os.environ if environ is None else environ
    root = find_mvs_runtime_root(environ=environ, candidates=candidates)
    if root is None:
        configured = str(environ.get("MVCAM_COMMON_RUNENV", "") or "").strip()
        suffix = f"（当前值: {configured}）" if configured else ""
        raise RuntimeError(
            "未找到海康 MVS SDK 的 64/libMvCameraControl.so。"
            "请安装 Linux x86_64 MVS SDK，或设置 MVCAM_COMMON_RUNENV 指向 SDK lib 目录"
            f"{suffix}。"
        )
    environ["MVCAM_COMMON_RUNENV"] = str(root)
    return root / MVS_LIBRARY_RELATIVE_PATH
