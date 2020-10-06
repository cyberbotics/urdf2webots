"""Test module of the urdf2webots script."""
import os
import sys
import unittest
import shutil

testDirectory = os.path.abspath(os.path.dirname(os.path.realpath(__file__)))
sourceDirectory = os.path.join(testDirectory, 'sources')
resultDirectory = os.path.join(testDirectory, 'results')
expectedDirectory = os.path.join(testDirectory, 'expected')
urdf2webotsPath = os.path.abspath(os.path.join(testDirectory, '..', 'demo.py'))

modelPaths = [
    {
        'input': os.path.join(sourceDirectory, 'motoman/motoman_sia20d_support/urdf/sia20d.urdf'),
        'output': os.path.join(resultDirectory, 'MotomanSia20d.proto'),
        'expected': [
            os.path.join(expectedDirectory, 'MotomanSia20d.proto'),
            os.path.join(expectedDirectory, 'MotomanSia20d_meshes', 'MotomanSia20d_MOTOMAN_AXIS_BMesh.proto'),
            os.path.join(expectedDirectory, 'MotomanSia20d_meshes', 'MotomanSia20d_MOTOMAN_AXIS_EMesh.proto'),
            os.path.join(expectedDirectory, 'MotomanSia20d_meshes', 'MotomanSia20d_MOTOMAN_AXIS_LMesh.proto'),
            os.path.join(expectedDirectory, 'MotomanSia20d_meshes', 'MotomanSia20d_MOTOMAN_AXIS_RMesh.proto'),
            os.path.join(expectedDirectory, 'MotomanSia20d_meshes', 'MotomanSia20d_MOTOMAN_AXIS_SMesh.proto'),
            os.path.join(expectedDirectory, 'MotomanSia20d_meshes', 'MotomanSia20d_MOTOMAN_AXIS_TMesh.proto'),
            os.path.join(expectedDirectory, 'MotomanSia20d_meshes', 'MotomanSia20d_MOTOMAN_AXIS_UMesh.proto'),
            os.path.join(expectedDirectory, 'MotomanSia20d_meshes', 'MotomanSia20d_MOTOMAN_BASEMesh.proto'),
        ],
        'arguments': '--multi-file --static-base --tool-slot=tool0 --rotation="1 0 0 0" --init-pos="[0.1, -0.1, 0.2]"'
    },
    {
        'input': os.path.join(sourceDirectory, 'gait2392_simbody/urdf/human.urdf'),
        'output': os.path.join(resultDirectory, 'Human.proto'),
        'expected': [os.path.join(expectedDirectory, 'Human.proto')],
        'arguments': ''
    }
]


def fileCompare(file1, file2):
    """Compare content of two files."""
    with open(file1) as f1, open(file2) as f2:
        for line1, line2 in zip(f1, f2):
            if line1.startswith('# Extracted from:') and line2.startswith('# Extracted from:'):
                # This line may differ.
                continue
            elif line1 != line2:
                return False
    return True


class TestScript(unittest.TestCase):
    """Unit test of the the urdf2webots script."""

    def setUp(self):
        """Cleanup results directory."""
        shutil.rmtree(resultDirectory, ignore_errors=True)

    def test_script_produces_the_correct_result(self):
        """Test that urdf2webots produces an expected result."""
        for paths in modelPaths:
            command = ('%s %s --input=%s --output=%s %s' %
                       (sys.executable, urdf2webotsPath, paths['input'], paths['output'], paths['arguments']))
            retcode = os.system(command)
            self.assertEqual(retcode, 0, msg='Error when exporting "%s"' % (paths['input']))
            for expected in paths['expected']:
                self.assertTrue(fileCompare(expected.replace('expected', 'results'), expected),
                                msg='Expected result mismatch when exporting "%s"' % paths['input'])


if __name__ == '__main__':
    unittest.main()
