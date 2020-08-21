#!/usr/bin/env python

"""URDF files to Webots PROTO converter."""

import optparse
import time, datetime, os
from urdf2webots.importer import convert2urdf
import os
import json
from copy import deepcopy
import shutil  

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
os.chdir('automatic_conversion/urdf')
# Walk the tree.
urdf_files = []  # List which will store all of the full filepaths.
protosPaths = []
for root, directories, files in os.walk('./'):
    for filename in files:
        # Join the two strings in order to form the full filepath.
        if filename.endswith(".urdf"):
            filepath = os.path.join(root, filename)
            filepath = filepath[1:]
            urdf_files.append('automatic_conversion/urdf' + filepath)  
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
        print('---------------------------------------') 
        print(configFile)
        try:
            with open(configFile) as json_file:
                config  = json.load(json_file)
            if config.keys() != default_config.keys():                
                print('Config out of date. Updating... (old settings will be carried over)')
                EndReportMessage['updateConfigs'].append(configFile)
                update_config(configFile, config)  
            else:
                print('Config file is up do date')              
        except:
            print('No config file found for "' + os.path.splitext(urdf_files[i])[0] + '". Generating new default config in same directory as the URDF file.')
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
        configFile = os.path.splitext(urdf_files[i])[0] + '.json'   
        print('---------------------------------------') 
        print('Converting: ', urdf_files[i]) 
        print('---------------------------------------') 
        try:
            with open(configFile) as json_file:
                config  = json.load(json_file)
            convert2urdf(
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
            print(dict())
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

if __name__ == '__main__':
    #cvrt.check_all_configs()
    update_and_convert()
    #cvrt.create_proto_dir()
    replace_ExtraProjectPath()
