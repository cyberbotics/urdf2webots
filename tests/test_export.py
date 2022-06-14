"""Test module of the urdf2webots script."""
import os
import sys
import unittest
import shutil
from urdf2webots.importer import convert2urdf

testDirectory = os.path.abspath(os.path.dirname(os.path.realpath(__file__)))
sourceDirectory = os.path.join(testDirectory, 'sources')
resultDirectory = os.path.join(testDirectory, 'results')
expectedDirectory = os.path.join(testDirectory, 'expected')
urdf2webotsPath = os.path.abspath(os.path.join(testDirectory, '..', 'urdf2webots/importer.py'))

modelPathsProto = [
    {
        'input': os.path.join(sourceDirectory, 'motoman/motoman_sia20d_support/urdf/sia20d.urdf'),
        'output': os.path.join(resultDirectory, 'MotomanSia20d.proto'),
        'expected': [os.path.join(expectedDirectory, 'MotomanSia20d.proto')],
        'arguments': '--tool-slot=tool0 --rotation="1 0 0 0" --init-pos="[0.1, -0.1, 0.2]"'
    },
    {
        'input': os.path.join(sourceDirectory, 'gait2392_simbody/urdf/human.urdf'),
        'output': os.path.join(resultDirectory, 'Human.proto'),
        'expected': [os.path.join(expectedDirectory, 'Human.proto')],
        'arguments': ''
    },
    {
        'input': os.path.join(sourceDirectory, 'kuka_lbr_iiwa_support/urdf/model.urdf'),
        'output': os.path.join(resultDirectory, 'KukaLbrIiwa14R820.proto'),
        'expected': [os.path.join(expectedDirectory, 'KukaLbrIiwa14R820.proto')],
        'arguments': '--box-collision --tool-slot=tool0 --rotation="1 0 0 -1.5708"'
    }
]

modelPathsRobotString = [
    {
        'input': os.path.join(sourceDirectory, 'kuka_lbr_iiwa_support/urdf/model.urdf'),
        'output': os.path.join(resultDirectory, 'KukaLbrIiwa14R820.txt'),
        'expected': [os.path.join(expectedDirectory, 'KukaLbrIiwa14R820.txt')],
        'robotName': 'kuka',
        'translation': '0 0 2',
        'rotation': '1 0 0 -1.5708',
    }
]


def fileCompare(file1, file2):
    """Compare content of two files."""
    with open(file1) as f1, open(file2) as f2:
        for i, (line1, line2) in enumerate(zip(f1, f2)):
            if line1.startswith('# Extracted from:') and line2.startswith('# Extracted from:'):
                # This line may differ.
                continue
            elif line1.startswith('#VRML_SIM') and line2.startswith('#VRML_SIM'):
                # This line may differ according to Webots version used
                continue
            elif 'CI' not in os.environ and '/home/runner/work/' in line2:
                # When testing locally, the paths may differ.
                continue
            elif line1 != line2:
                # Prints the difference between result and expected line
                print('Diff (line ' + str(i) + ') in ' + file1)
                print('result: ' + line1 + 'expect: ' + line2)
                return False
    return True


class TestScript(unittest.TestCase):
    """Unit test of the the urdf2webots script."""

    def setUp(self):
        """Cleanup results directory."""
        shutil.rmtree(resultDirectory, ignore_errors=True)

        # prepare urls
        rootPath = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

        for root, dirs, files in os.walk(os.path.join(rootPath, 'tests', 'expected')):
            for file in files:
                with open(os.path.join(root, file), 'r') as f:
                    contents = f.read()
                    contents = contents.replace('root://', rootPath + '/')

                with open(os.path.join(root, file), 'w') as f:
                    f.write(contents)

    def test_script_produces_the_correct_result(self):
        """Test that urdf2webots produces an expected result."""
        print("Proto files tests...")
        for paths in modelPathsProto:
            command = ('%s %s --input=%s --output=%s %s' %
                       (sys.executable, urdf2webotsPath, paths['input'], paths['output'], paths['arguments']))
            retcode = os.system(command)
            self.assertEqual(retcode, 0, msg='Error when exporting "%s"' % (paths['input']))
            for expected in paths['expected']:
                self.assertTrue(fileCompare(expected.replace('expected', 'results'), expected),
                                msg='Expected result mismatch when exporting "%s"' % paths['input'])

        print("Robot node strings tests...")
        for paths in modelPathsRobotString:
            robot_string = convert2urdf(inFile=paths['input'], robotName=paths['robotName'],
                                        initTranslation=paths['translation'], initRotation=paths['rotation'])
            f = open(paths['output'], "a")
            f.write(robot_string)
            f.close()
            for expected in paths['expected']:
                self.assertTrue(fileCompare(expected.replace('expected', 'results'), expected),
                                msg='Expected result mismatch when exporting "%s"' % paths['input'])

    def tearDown(self):
        # undo url changes
        rootPath = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

        for root, dirs, files in os.walk(os.path.join(rootPath, 'tests', 'expected')):
            for file in files:
                with open(os.path.join(root, file), 'r') as f:
                    contents = f.read()
                    contents = contents.replace(rootPath + '/', 'root://')

                with open(os.path.join(root, file), 'w') as f:
                    f.write(contents)


if __name__ == '__main__':
    unittest.main()
