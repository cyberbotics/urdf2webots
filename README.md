# urdf2webots

This tool converts URDF files to a Webots-readable format.

## Usage

`python urdf2webots.py someRobot.urdf [-o outputFile] [--box-collision]`

Outputs: someRobot_textures (folder), someRobot.proto.

Test in webots: put the outputs in protos folder within webots project folder.

## notes
urdf file should be in same folder as python code. 
This has been tested with ur10, pr2 and motoman using webots7.4.3 on Ubuntu16.04.
