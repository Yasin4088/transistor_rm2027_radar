import tempfile
import unittest
from pathlib import Path

from capture.mvs_runtime import configure_mvs_runtime, find_mvs_runtime_root


class MvsRuntimeTest(unittest.TestCase):
    def test_configured_runtime_is_preferred(self):
        with tempfile.TemporaryDirectory() as temporary_directory:
            root = Path(temporary_directory) / "configured"
            library = root / "64" / "libMvCameraControl.so"
            library.parent.mkdir(parents=True)
            library.touch()
            environ = {"MVCAM_COMMON_RUNENV": str(root)}

            self.assertEqual(find_mvs_runtime_root(environ, []), root)
            self.assertEqual(configure_mvs_runtime(environ, []), library)

    def test_default_candidate_is_discovered_and_exported(self):
        with tempfile.TemporaryDirectory() as temporary_directory:
            root = Path(temporary_directory) / "MVS" / "lib"
            library = root / "64" / "libMvCameraControl.so"
            library.parent.mkdir(parents=True)
            library.touch()
            environ = {}

            self.assertEqual(configure_mvs_runtime(environ, [root]), library)
            self.assertEqual(environ["MVCAM_COMMON_RUNENV"], str(root))

    def test_missing_runtime_has_actionable_error(self):
        with self.assertRaisesRegex(RuntimeError, "MVCAM_COMMON_RUNENV"):
            configure_mvs_runtime({}, [])


if __name__ == "__main__":
    unittest.main()
