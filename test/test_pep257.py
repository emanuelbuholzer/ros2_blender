from ament_pep257.main import main
import unittest


class TestPep257Compatibility(unittest.TestCase):
    def test_no_code_style_errors_and_warnings(self):
        rc = main(argv=[".", "test"])
        self.assertEqual(rc, 0, "Found code style errors / warnings")


if __name__ == "__main__":
    unittest.main()
