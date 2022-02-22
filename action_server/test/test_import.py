import unittest


class TestImport(unittest.TestCase):
    def test_import(self):
        """
        If no exception is raised, this test will succeed
        """
        import action_server


if __name__ == '__main__':
    unittest.main()
