"""Test module of the urdf2webots script."""
import os
import unittest


testsDirectory = os.path.abspath(os.path.dirname(os.path.realpath(__file__)))
urdf2webotsPath = os.path.abspath(os.path.join(testsDirectory, '..', 'urdf2webots.py'))

modelPaths = [
    {
        'input': os.path.join(testsDirectory, 'sources/motoman/motoman_sia20d_support/urdf/sia20d.urdf'),
        'output': os.path.join(testsDirectory, 'results/sia20d.proto'),
        'expected': os.path.join(testsDirectory, 'results/sia20d.expected.proto')
    },
    {
        'input': os.path.join(testsDirectory, 'sources/universal_robot/ur_description/urdf/ur10.urdf.xacro'),
        'output': os.path.join(testsDirectory, 'results/ur10.proto'),
        'expected': os.path.join(testsDirectory, 'results/ur10.expected.proto')
    }
]


class TestScript(unittest.TestCase):
    """Unit test of the the urdf2webots script."""

    def test_script_produces_the_correct_result(self):
        """Test that urdf2webots produces an expected result."""
        for paths in modelPaths:
            retcode = os.system('python %s %s -o %s' % (urdf2webotsPath, paths['input'], paths['output']))
            self.assertEqual(retcode, 0, msg='Error when exporting "%s"' % (paths['input']))
