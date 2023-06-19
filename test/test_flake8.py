from ament_flake8.main import main_with_errors
import unittest


class TestFlake8Compatibility(unittest.TestCase):

    def test_no_code_style_errors(self):
        rc, errors = main_with_errors(argv=[])
        self.assertEqual(rc, 0, "Found %d code style errors / warnings:\n" % len(
            errors
        ) + "\n".join(errors))


if __name__ == '__main__':
    unittest.main()
