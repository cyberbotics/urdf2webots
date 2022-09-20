"""Test module of the urdf2webots script."""
import io
import os
import pathlib
import shutil
import sys
import unittest

from urdf2webots.importer import convertUrdfContent, convertUrdfFile

rootDirectory = os.path.dirname(os.path.dirname(__file__))
testDirectory = os.path.join(rootDirectory, 'tests')
sourceDirectory = os.path.join(testDirectory, 'sources')
resultDirectory = os.path.join(testDirectory, 'results')
expectedDirectory = os.path.join(testDirectory, 'expected')
urdf2webotsPath = os.path.abspath(os.path.join(testDirectory, '..', 'urdf2webots/importer.py'))

humanFilePath = os.path.join(sourceDirectory, 'gait2392_simbody/urdf/human.urdf')

modelPathsProto = [
    {
        'input': os.path.join(sourceDirectory, 'motoman/motoman_sia20d_support/urdf/sia20d.urdf'),
        'output': os.path.join(resultDirectory, 'MotomanSia20d.proto'),
        'expected': [os.path.join(expectedDirectory, 'MotomanSia20d.proto')],
        'arguments': '--tool-slot=tool0 --rotation="1 0 0 0" --init-pos="[0.1, -0.1, 0.2]"'
    },
    {
        'input': humanFilePath,
        'output': os.path.join(resultDirectory, 'Human.proto'),
        'expected': [os.path.join(expectedDirectory, 'Human.proto')],
        'arguments': ''
    },
    {
        'input': humanFilePath,
        'output': os.path.join(resultDirectory, 'HumanR2022a.proto'),
        'expected': [os.path.join(expectedDirectory, 'HumanR2022a.proto')],
        'arguments': '--target=R2022a'
    },
    {
        'input': os.path.join(sourceDirectory, 'kuka_lbr_iiwa_support/urdf/model.urdf'),
        'output': os.path.join(resultDirectory, 'KukaLbrIiwa14R820.proto'),
        'expected': [os.path.join(expectedDirectory, 'KukaLbrIiwa14R820.proto')],
        'arguments': '--box-collision --tool-slot=tool0 --rotation="1 0 0 -1.5708"'
    },
    {
        'input': os.path.join(sourceDirectory, 'RobotWithDummyLink.urdf'),
        'output': os.path.join(resultDirectory, 'RobotWithDummyLink.proto'),
        'expected': [os.path.join(expectedDirectory, 'RobotWithDummyLink.proto')],
        'arguments': ''
    }
]

modelContentProto = [
    {
        'input': pathlib.Path(humanFilePath).read_text(),
        'output': os.path.join(resultDirectory, 'Human.proto'),
        'expected': [os.path.join(expectedDirectory, 'Human.proto')],
        'relativePathPrefix': os.path.join(rootDirectory, 'tests/sources/gait2392_simbody/urdf')
    }
]

modelPathsRobotString = [
    {
        'input': os.path.join(sourceDirectory, 'kuka_lbr_iiwa_support/urdf/model.urdf'),
        'output': os.path.join(resultDirectory, 'KukaLbrIiwa14R820.txt'),
        'expected': [os.path.join(expectedDirectory, 'KukaLbrIiwa14R820.txt')],
        'robotName': 'kuka',
        'translation': '0 0 2',
        'rotation': '1 0 0 -1.5708'
    }
]


def fileCompare(file1, file2):
    """Compare content of two files."""
    with open(file1) as f1, open(file2) as f2:
        for i, (line1, line2) in enumerate(zip(f1, f2)):
            if line1.startswith('# Extracted from') and line2.startswith('# Extracted from'):
                # This line may differ.
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
        os.makedirs(resultDirectory)
        # prepare urls
        for root, _, files in os.walk(expectedDirectory):
            for file in files:
                with open(os.path.join(root, file), 'r') as f:
                    contents = f.read()
                    contents = contents.replace('root://', rootDirectory + os.path.sep)

                with open(os.path.join(root, file), 'w') as f:
                    f.write(contents)

    def testInputFileOutputProto(self):
        """Test that urdf2webots produces an expected PROTO file using URDF file as input."""
        print('Start tests with input "URDF file" and output "PROTO file"...')
        for paths in modelPathsProto:
            command = ('%s %s --input=%s --output=%s %s' %
                       (sys.executable, urdf2webotsPath, paths['input'], paths['output'], paths['arguments']))
            retcode = os.system(command)
            self.assertEqual(retcode, 0, msg='Error when exporting "%s"' % (paths['input']))
            for expected in paths['expected']:
                self.assertTrue(fileCompare(expected.replace('expected', 'results'), expected),
                                msg='Expected result mismatch when exporting "%s"' % paths['input'])

    def testInputContentOutputProto(self):
        """Test that urdf2webots produces an expected PROTO file using URDF content as input."""
        print('Start tests with input "URDF content" and output "PROTO file"...')
        for paths in modelContentProto:
            convertUrdfContent(input=paths['input'], output=paths['output'], relativePathPrefix=paths['relativePathPrefix'])
            for expected in paths['expected']:
                self.assertTrue(fileCompare(expected.replace('expected', 'results'), expected),
                                msg='Expected result mismatch when exporting input to "%s"' % paths['output'])

    def testInputContentStdinOutputProto(self):
        """Test that urdf2webots produces an expected PROTO file using URDF content in the stdin buffer as input."""
        print('Start tests with input "URDF content" and output "PROTO file"...')
        for paths in modelContentProto:
            sys.stdin = io.StringIO(paths['input'])
            convertUrdfFile(output=paths['output'], relativePathPrefix=paths['relativePathPrefix'])
            for expected in paths['expected']:
                self.assertTrue(fileCompare(expected.replace('expected', 'results'), expected),
                                msg='Expected result mismatch when exporting input to "%s"' % paths['output'])

    def testInputFileOutputRobotString(self):
        """Test that urdf2webots produces an expected Robot node string using URDF file as input."""
        print('Start tests with input "URDF file" and output "Robot node strings"...')
        for paths in modelPathsRobotString:
            robot_string = convertUrdfFile(input=paths['input'], robotName=paths['robotName'],
                                           initTranslation=paths['translation'], initRotation=paths['rotation'])
            f = open(paths['output'], 'w')
            f.write(robot_string)
            f.close()
            for expected in paths['expected']:
                self.assertTrue(fileCompare(expected.replace('expected', 'results'), expected),
                                msg='Expected result mismatch when exporting "%s"' % paths['input'])

    def tearDown(self):
        # undo url changes
        for root, dirs, files in os.walk(expectedDirectory):
            for file in files:
                with open(os.path.join(root, file), 'r') as f:
                    contents = f.read()
                    contents = contents.replace(rootDirectory + os.path.sep, 'root://')

                with open(os.path.join(root, file), 'w') as f:
                    f.write(contents)


if __name__ == '__main__':
    unittest.main()
