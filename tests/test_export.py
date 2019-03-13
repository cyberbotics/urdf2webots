"""Test module of the urdf2webots script."""
import filecmp
import os
import unittest
import shutil

testDirectory = os.path.abspath(os.path.dirname(os.path.realpath(__file__)))
sourceDirectory = os.path.join(testDirectory, 'sources')
resultDirectory = os.path.join(testDirectory, 'results')
expectedDirectory = os.path.join(testDirectory, 'expected')
urdf2webotsPath = os.path.abspath(os.path.join(testDirectory, '..', 'urdf2webots.py'))

modelPaths = [
    {
        'input': os.path.join(sourceDirectory, 'motoman/motoman_sia20d_support/urdf/sia20d.urdf'),
        'output': os.path.join(resultDirectory, 'MotomanSia20d.proto'),
        'expected': os.path.join(expectedDirectory, 'MotomanSia20d.proto')
    }
]


class TestScript(unittest.TestCase):
    """Unit test of the the urdf2webots script."""

    def setUp(self):
        """Cleanup results directory."""
        shutil.rmtree(resultDirectory, ignore_errors=True)

    def test_script_produces_the_correct_result(self):
        """Test that urdf2webots produces an expected result."""
        for paths in modelPaths:
            command = 'python %s --input=%s --output=%s' % (urdf2webotsPath, paths['input'], paths['output'])
            retcode = os.system(command)
            self.assertEqual(retcode, 0, msg='Error when exporting "%s"' % (paths['input']))

            print('### OUTPUT ###')
            print(open(paths['output'], 'r').read())
            print('### EXPECTED ###')
            print(open(paths['expected'], 'r').read())
            print('### DONE ###')
            self.assertTrue(filecmp.cmp(paths['output'], paths['expected']), msg='Expected result mismatch when exporting "%s"' % (paths['input']))
