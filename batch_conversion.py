#!/usr/bin/env python

"""URDF files to Webots PROTO converter."""

import optparse
import time
import datetime
import os
import shutil
import json
import math
#from urdf2webots.importer import convert2urdf
import urdf2webots.importer as importer
import urdf2webots.parserURDF
from copy import deepcopy

# Set Override = True  if you want to override ALL config settings with the value in the OverrideCfg.
# This can be usefull for testing functionality quickly, and then doing a slow optimized conversion
Override = False
OverrideCfg = {            
            'disableMeshOptimization': False
}

# This defines the default config. If a urdf model doesnt have a config, one will created after this. 
# If the importer.py is updated, so should this to add the additional functionality
default_config = {
            'robotName': None, # outFile will get constructed with protosPath + robotName + '.proto'
            'normal': False, 
            'boxCollision': True, 
            'disableMeshOptimization': True,
            'enableMultiFile': True, 
            'staticBase': True, 
            'toolSlot': None, 
            'initRotation': "1 0 0 -1.5708"
}





urdf2webots_path = os.path.dirname(os.path.abspath(__file__))   
# create a unique path for a new batch-conversion
today = datetime.date.today() 
todaystr = today.isoformat()  
protoTargetDir = 'automatic_conversion/protos/protos_' + todaystr
n = 2
while os.path.exists(protoTargetDir):
    while os.path.exists(protoTargetDir + '_Nr-' + str(n)):
        n += 1
    protoTargetDir = protoTargetDir + '_Nr-' + str(n)

# Find all the urdf files, and create the corresponding proto filePaths
urdf_root_dir = 'automatic_conversion/urdf'#/robots/kuka/lbr_iiwa/protos'
os.chdir(urdf_root_dir)
# Walk the tree.
urdf_files = []  # List which will store all of the full filepaths.
protosPaths = []
for root, directories, files in os.walk('./'):
    for filename in files:
        # Join the two strings in order to form the full filepath.
        if filename.endswith(".urdf"):
            filepath = os.path.join(root, filename)
            filepath = filepath[1:]
            urdf_files.append(urdf_root_dir + filepath)  
            protosPaths.append(protoTargetDir + os.path.dirname(filepath) + '/')
os.chdir(urdf2webots_path)

# Report dict we can print at the end
EndReportMessage =  {
    'updateConfigs': [],
    'newConfigs': [],
    'converted': [],
    'failed': []
    }


def update_and_convert():
    # Make sure all configs exist and are up to date, then convert all URDF files
    check_all_configs()
    convert_all_urdfs()
    print_end_report()

def create_proto_dir():  
    # create a new unique directory for our conversions   
    print(protoTargetDir)   
    os.mkdir(protoTargetDir)

def replace_ExtraProjectPath():
    # this copies the converted files and puts them in the "automatic_conversion/ExtraProjectTest" directory
    # This path can be added to webots, so we can test each new conversion, without having to adjust the Path 
    # every time
    destination = 'automatic_conversion/ExtraProjectTest'
    if os.path.isfile(destination) or os.path.islink(destination):
        os.remove(destination)  # remove the file
    elif os.path.isdir(destination):
        shutil.rmtree(destination)  # remove dir and all contains
    shutil.copytree(protoTargetDir,  destination)  

def update_config(configFile, config=None):
    # makes sure the existing configs are in the same format as the default. Existing options are conserved
    new_config = deepcopy(default_config)
    for key in new_config.keys():
        try:
            new_config[key] = config[key]
        except:
            pass
    with open(configFile, 'w') as outfile:
        json.dump(new_config, outfile, indent=4, sort_keys=True) 


def check_all_configs():
    # makes sure ever URDF file has a config and it is up do date
    print('Checking config files...')
    print('---------------------------------------')
    for i in range(len(urdf_files)):
        configFile = os.path.splitext(urdf_files[i])[0] + '.json'          
        try:
            with open(configFile) as json_file:
                config  = json.load(json_file)
            if config.keys() != default_config.keys():                
                print('Updating config (old settings will be carried over) - ', configFile)
                EndReportMessage['updateConfigs'].append(configFile)
                update_config(configFile, config)  
            else:
                print('Config up to date - ', configFile)              
        except:
            print('Generating new config for - ' + os.path.splitext(urdf_files[i])[0])
            EndReportMessage['newConfigs'].append(configFile)
            update_config(configFile)

def convert_all_urdfs():
    # convertes all URDF files according to their .json config files
    create_proto_dir()
    print('---------------------------------------')
    print('Converting URDF to PROTO...')
    print('---------------------------------------')
    print('---------------------------------------')
    
    for i in range(len(urdf_files)):
        # clears any previous Geometry and Material references, so there is no conflict
        importer.urdf2webots.parserURDF.Geometry.reference = {}
        importer.urdf2webots.parserURDF.Material.namedMaterial = {}
        configFile = os.path.splitext(urdf_files[i])[0] + '.json'   
        print('---------------------------------------') 
        print('Converting: ', urdf_files[i]) 
        print('---------------------------------------') 
        try:
            with open(configFile) as json_file:
                config  = json.load(json_file)
            importer.convert2urdf(
                inFile = urdf_files[i], 
                outFile = protosPaths[i]  if not config['robotName'] else protosPaths[i] + config['robotName'] + '.proto',
                normal = config['normal'], 
                boxCollision = config['boxCollision'], 
                disableMeshOptimization = OverrideCfg['disableMeshOptimization']  if Override else config['disableMeshOptimization'],
                enableMultiFile = config['enableMultiFile'], 
                staticBase = config['staticBase'], 
                toolSlot = config['toolSlot'], 
                initRotation = config['initRotation'])
            EndReportMessage['converted'].append(urdf_files[i])
        except Exception as e:         
            print(e)
            EndReportMessage['failed'].append(urdf_files[i])
        print('---------------------------------------') 

def print_end_report():
    # print the Report
    print('---------------------------------------') 
    print('------------End Report-----------------') 
    print('---------------------------------------') 
    print('Configs updated:')
    print('---------------------------------------') 
    for config in EndReportMessage['updateConfigs']:
        print(config)
    print('---------------------------------------') 
    print('Configs created:')
    print('---------------------------------------')     
    for config in EndReportMessage['newConfigs']:
        print(config)
    print('---------------------------------------') 
    print('Successfully converted URDF files:')
    print('---------------------------------------') 
    for urdf in EndReportMessage['converted']:
        print(urdf)
    if len(EndReportMessage['failed']) > 0:
        print('---------------------------------------') 
        print('Failedd URDF conversions:')
        print('---------------------------------------')     
        for urdf in EndReportMessage['failed']:
            print(urdf)


def create_test_world(spacing = 3):
    #os.chdir(protoTargetDir)
    n_models = len(urdf_files)
    n_row = math.ceil(n_models ** 0.5) # round up the sqrt of the number of models
    grid_size = spacing * (n_row +1)
    worldFile = open('automatic_conversion/ExtraProjectTest/TestAllRobots.wbt', 'w')
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
    filenames = []  # List which will store all of the full filepaths.
    row = 0
    column = 1
    for root, directories, files in os.walk('automatic_conversion/ExtraProjectTest'):
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
                    worldFile.write('  translation ' + str(column * spacing - grid_size/2) + ' 0 ' + str(row * spacing - grid_size/2) + '\n')
                    worldFile.write('}\n')
    worldFile.close()
    os.chdir(urdf2webots_path)
    

if __name__ == '__main__':
    optParser = optparse.OptionParser(usage='usage: %prog  [options]')
    optParser.add_option('--force-mesh-optimization', dest='forceMesh', action='store_true', default=False, help='Set if mesh-optimization should be turned on for all conversions. This will take much longer!')
    optParser.add_option('--no-project-override', dest='extraProj', action='store_true', default=False, help='Set if new conversions should NOT replace existing ones in "automatic_conversion/ExtraProjectTest".')
    optParser.add_option('--update-cfg', dest='cfgUpdate', action='store_true', default=False, help='No conversion. Only updates or creates .json config for every URDF file in "automatic_conversion/urdf".')
    
    options, args = optParser.parse_args()
    
    Override = options.forceMesh
    if options.cfgUpdate:
        check_all_configs()
    else:
        update_and_convert()
        if not options.extraProj:
            replace_ExtraProjectPath()

    create_test_world()
            

