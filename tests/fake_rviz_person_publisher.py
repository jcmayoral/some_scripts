import unittest
import rospy
import numpy as np

class Test(unittest.TestCase):
    """
    Our basic test class
    """
    def test(self):
        """
        The actual test.
        Any method which starts with ``test_`` will considered as a test case.
        """
        res = np.power(5,2)
        self.assertEqual(res, 25)


if __name__ == '__main__':
    unittest.main()
