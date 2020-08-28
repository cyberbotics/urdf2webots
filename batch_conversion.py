#!/usr/bin/env python

"""URDF files to Webots PROTO converter."""

import optparse
import datetime
import os
import shutil
import json
import math
# from urdf2webots.importer import convert2urdf
import urdf2webots.importer as importer
from copy import deepcopy

# Set Override = True  if you want to override ALL config settings with the
# value in the OverrideCfg. This can be usefull for testing functionality
# quickly, and then doing a slow optimized conversion
Override = False
OverrideCfg = {
    'disableMeshOptimization': False
}

# This defines the default config. If a urdf model doesnt have a config, one
# will created after this. If the importer.py is updated, so should this to
# add the additional functionality-
default_config = {
    # outFile will get constructed with protosPath + robotName + '.proto'
    'robotName': None,
    'normal': False,
    'boxCollision': True,
    'disableMeshOptimization': True,
    'enableMultiFile': True,
    'staticBase': True,
    'toolSlot': None,
    'initRotation': "1 0 0 -1.5708"
}


class BatchConversion():
    def __init__(self, sourcePath, outPath):
        self.urdf2webots_path = os.path.dirname(os.path.abspath(__file__))
        # create a unique path for a new batch-conversion
        today = datetime.date.today()
        todaystr = today.isoformat()
        self.protoTargetDir = outPath + '/protos_' + todaystr
        n = 2
        while os.path.exists(self.protoTargetDir):
            while os.path.exists(self.protoTargetDir + '_Nr-' + str(n)):
                n += 1
            self.protoTargetDir = self.protoTargetDir + '_Nr-' + str(n)

        # Find all the urdf files, and create the corresponding proto filePaths
        urdf_root_dir = sourcePath
        os.chdir(urdf_root_dir)
        # Walk the tree.
        self.urdf_files = []  # List of the full filepaths.
        self.protosPaths = []
        for root, directories, files in os.walk('./'):
            for filename in files:
                # Join the two strings in order to form the full filepath.
                if filename.endswith(".urdf"):
                    filepath = os.path.join(root, filename)
                    filepath = filepath[1:]
                    self.urdf_files.append(urdf_root_dir + filepath)
                    self.protosPaths.append(self.protoTargetDir + os.path.dirname(filepath) + '/')
        os.chdir(self.urdf2webots_path)

        # Report dict we can print at the end
        self.EndReportMessage = {
            'updateConfigs': [],
            'newConfigs': [],
            'converted': [],
            'failed': []
        }

    def update_and_convert(self):
        """Make sure all configs exist and are up to date, then convert all URDF files."""
        self.check_all_configs()
        self.convert_all_urdfs()
        self.print_end_report()

    def create_proto_dir(self):
        """Create a new unique directory for our conversions."""
        print(self.protoTargetDir)
        os.makedirs(self.protoTargetDir)

    def replace_ExtraProjectPath(self):
        # this copies the converted files and puts them in the "<protoTargetDir>/ExtraProjectTest" directory
        # This path can be added to webots, so we can test each new conversion, without having to adjust the Path
        # every time
        destination = os.path.dirname(self.protoTargetDir) + '/ExtraProjectTest'
        if os.path.isfile(destination) or os.path.islink(destination):
            os.remove(destination)  # remove the file
        elif os.path.isdir(destination):
            shutil.rmtree(destination)  # remove dir and all contains
        shutil.copytree(self.protoTargetDir,  destination)

    def update_config(self, configFile, config=None):
        """Makes sure the existing configs are in the same format as the default, existing options are conserved."""
        new_config = deepcopy(default_config)
        for key in new_config.keys():
            try:
                new_config[key] = config[key]
            except:
                pass
        with open(configFile, 'w') as outfile:
            json.dump(new_config, outfile, indent=4, sort_keys=True)

    def check_all_configs(self):
        """Makes sure ever URDF file has a config and it is up to date."""
        print('Checking config files...')
        print('---------------------------------------')
        for i in range(len(self.urdf_files)):
            configFile = os.path.splitext(self.urdf_files[i])[0] + '.json'
            try:
                with open(configFile) as json_file:
                    config = json.load(json_file)
                if config.keys() != default_config.keys():
                    print('Updating config (old settings will be carried over) - ', configFile)
                    self.EndReportMessage['updateConfigs'].append(configFile)
                    self.update_config(configFile, config)
                else:
                    print('Config up to date - ', configFile)
            except:
                print('Generating new config for - ' + os.path.splitext(self.urdf_files[i])[0])
                self.EndReportMessage['newConfigs'].append(configFile)
                self.update_config(configFile)

    def convert_all_urdfs(self):
        """Converts all URDF files according to their '.json' config files."""
        self.create_proto_dir()
        print('---------------------------------------')
        print('Converting URDF to PROTO...')
        print('---------------------------------------')
        print('---------------------------------------')
        for i in range(len(self.urdf_files)):
            # clears any previous Geometry and Material references, so there is no conflict
            importer.urdf2webots.parserURDF.Geometry.reference = {}
            importer.urdf2webots.parserURDF.Material.namedMaterial = {}
            configFile = os.path.splitext(self.urdf_files[i])[0] + '.json'
            print('---------------------------------------')
            print('Converting: ', self.urdf_files[i])
            print('---------------------------------------')
            try:
                with open(configFile) as json_file:
                    config = json.load(json_file)
                importer.convert2urdf(
                    inFile=self.urdf_files[i],
                    outFile=self.protosPaths[i] if not config['robotName'] else self.protosPaths[i] + config['robotName'] + '.proto',
                    normal=config['normal'],
                    boxCollision=config['boxCollision'],
                    disableMeshOptimization=OverrideCfg['disableMeshOptimization'] if Override else config['disableMeshOptimization'],
                    enableMultiFile=config['enableMultiFile'],
                    staticBase=config['staticBase'],
                    toolSlot=config['toolSlot'],
                    initRotation=config['initRotation'])
                self.EndReportMessage['converted'].append(self.urdf_files[i])
            except Exception as e:
                print(e)
                self.EndReportMessage['failed'].append(self.urdf_files[i])
            print('---------------------------------------')

    def print_end_report(self):
        """Print the report."""
        print('---------------------------------------')
        print('------------End Report-----------------')
        print('---------------------------------------')
        print('Configs updated:')
        print('---------------------------------------')
        for config in self.EndReportMessage['updateConfigs']:
            print(config)
        print('---------------------------------------')
        print('Configs created:')
        print('---------------------------------------')
        for config in self.EndReportMessage['newConfigs']:
            print(config)
        print('---------------------------------------')
        print('Successfully converted URDF files:')
        print('---------------------------------------')
        for urdf in self.EndReportMessage['converted']:
            print(urdf)
        if len(self.EndReportMessage['failed']) > 0:
            print('---------------------------------------')
            print('Failedd URDF conversions:')
            print('---------------------------------------')
            for urdf in self.EndReportMessage['failed']:
                print(urdf)

    def create_test_world(self, spacing=3):
        n_models = len(self.urdf_files)
        n_row = math.ceil(n_models ** 0.5)  # round up the sqrt of the number of models
        grid_size = spacing * (n_row + 1)
        projectDir = os.path.dirname(self.protoTargetDir) + '/ExtraProjectTest/TestAllRobots.wbt'
        worldFile = open(projectDir, 'w')
        worldFile.write('#VRML_SIM R2020b utf8\n')
        worldFile.write('\n')
        worldFile.write('WorldInfo {\n')
        worldFile.write('  basicTimeStep 16\n')
        worldFile.write('  coordinateSystem "NUE"\n')
        worldFile.write('}\n')
        worldFile.write('Viewpoint {\n')
        worldFile.write('  orientation 0.7180951961816571 -0.6372947429425837 -0.27963315225232777 5.235063704863283\n')
        worldFile.write('  position 1.9410928989638234 2.447392518518642 1.7311802992777219\n')
        worldFile.write('}\n')
        worldFile.write('TexturedBackground {\n')
        worldFile.write('}\n')
        worldFile.write('TexturedBackgroundLight {\n')
        worldFile.write('}\n')
        worldFile.write('RectangleArena {\n')
        worldFile.write('  floorSize ' + str(grid_size) + ' ' + str(grid_size) + '\n')
        worldFile.write('  floorTileSize 0.25 0.25\n')
        worldFile.write('  wallHeight 0.05\n')
        worldFile.write('}\n')
        row = 0
        column = 1
        for root, directories, files in os.walk(os.path.dirname(projectDir)):
            for filename in files:
                # Join the two strings in order to form the full filepath.
                if filename.endswith(".proto"):
                    filepath = os.path.join(root, filename)
                    if os.path.dirname(filepath).split('_')[-1] != 'meshes':
                        name = os.path.splitext(os.path.basename(filename))[0]
                        if row == n_row:
                            column += 1
                            row = 0
                        row += 1
                        # add the model to the world file with translation to be spaced in a grid
                        worldFile.write(name + '  {\n')
                        worldFile.write('  translation ' + str(column * spacing - grid_size / 2) + ' 0 ' + str(row * spacing - grid_size / 2) + '\n')
                        worldFile.write('}\n')
        worldFile.close()
        os.chdir(self.urdf2webots_path)


if __name__ == '__main__':
    optParser = optparse.OptionParser(usage='usage: %prog  [options]')
    optParser.add_option('--input', dest='sourcePath', default='', help='Specifies the urdf file to convert.')
    optParser.add_option('--output', dest='outPath', default='', help='Specifies the path and, if ending in ".proto", name of the resulting PROTO file.'
                         ' The filename minus the .proto extension will be the robot name.')
    optParser.add_option('--force-mesh-optimization', dest='forceMesh', action='store_true', default=False, help='Set if mesh-optimization should be turned on for all conversions. This will take much longer!')
    optParser.add_option('--no-project-override', dest='extraProj', action='store_true', default=False, help='Set if new conversions should NOT replace existing ones in "<input>/ExtraProjectTest".')
    optParser.add_option('--update-cfg', dest='cfgUpdate', action='store_true', default=False, help='No conversion. Only updates or creates .json config for every URDF file in "automatic_conversion/urdf".')

    options, args = optParser.parse_args()
    bc = BatchConversion(options.sourcePath, options.outPath)
    Override = options.forceMesh
    if options.cfgUpdate:
        bc.check_all_configs()
    else:
        bc.update_and_convert()
        if not options.extraProj:
            bc.replace_ExtraProjectPath()
    bc.create_test_world()
