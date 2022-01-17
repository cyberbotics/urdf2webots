# urdf2webots

![Test Status](https://github.com/cyberbotics/urdf2webots/actions/workflows/test.yml/badge.svg)
[![PyPI version](https://badge.fury.io/py/urdf2webots.svg)](https://badge.fury.io/py/urdf2webots)

This tool converts URDF files into Webots PROTO files or into Webots VRML robot strings.

## Install

### From pip

```
pip install urdf2webots
```

On macOS, export the pip binary path to the PATH: `export PATH="/Users/$USER/Library/Python/3.7/bin:$PATH"`

### From Sources

```
git clone --recurse-submodules https://github.com/cyberbotics/urdf2webots.git
pip install --upgrade --editable urdf2webots
```

## Usage

### From pip

```
python -m urdf2webots.importer --input=someRobot.urdf [--output=outputFile] [--box-collision] [--normal] [--static-base] [--tool-slot=linkName] [--name-to-def] [--help]
```

### Arguments

The script accepts the following arguments:
  - **-h, --help**: Show the help message and exit.
  - **--input=INFILE**: Specifies the urdf file to convert.
  - **--output=OUTFILE**: If set, specifies the path and, if ending in ".proto", name of the resulting PROTO file. The filename minus the .proto extension will be the robot name (for PROTO conversion only).
  - **--is-proto=True**: If set to false, the conversion will return a Webots VRML robot string.
  - **--robotName**: Specify the name of your robot (has to be specified if you set --is-proto to `False`)..
  - **--normal**: If set, the normals are exported if present in the URDF definition.
  - **--box-collision**: If set, the bounding objects are approximated using boxes.
  - **--tool-slot=LinkName**: Specify the link that you want to add a tool slot to (exact link name from urdf, for PROTO conversion only).
  - **--translation="0 0 0"**: Set the translation field of your PROTO file or Webots VRML robot string.
  - **--rotation="0 0 1 0"**: Set the rotation field of your PROTO file or Webots VRML robot string.
  - **--init-pos=JointPositions**: Set the initial positions of your robot joints. Example: `--init-pos="[1.2, 0.5, -1.5]"` would set the first 3 joints of your robot to the specified values, and leave the rest with their default value.
  - **--link-to-def**: Creates a DEF with the link name for each solid to be able to access it using getFromProtoDef(defName)
  - **--joint-to-def**: Creates a DEF with the joint name for each joint to be able to access it using getFromProtoDef(defName)

> Previously the **--static-base** argument was supported in order to set the base link static (disable physics). It has been removed as a common way to do it in an URDF file is to add the following to your URFD file (assuming **base_link** is the root link of your robot):
>
>```
> <link name="world" />
><joint name="world_joint" type="fixed">
>    <parent link="world" />
>    <child link = "base_link" />
>    <origin xyz="0 0 0" rpy="0 0 0" />
></joint>
>
>```

### In your Python Code

```
from urdf2webots.importer import convert2urdf
convert2urdf('MY_PATH/MY_URDF.urdf')
```

### In-Depth Tutorial
Check out [this tutorial](./docs/tutorial.md) for a more in-depth, step by step instruction, on how to:
- Generate a URDF file from a ROS repository.
- Convert your URDF file to a Webots PROTO file.
- Load your converted model into Webots and make final adjustments.


## Notes
This tool have been tested using Webots R2022a on Ubuntu20.04 and Windows.
You can find the sources of these URDF files here:
  - universal robot: https://github.com/ros-industrial/universal_robot/tree/kinetic-devel/ur_description
  - pr2 robot: https://github.com/PR2/pr2_common/tree/kinetic-devel/pr2_description
  - motoman robot: https://github.com/ros-industrial/motoman/tree/kinetic-devel/motoman_sia20d_support
  - kinova robot: https://github.com/Kinovarobotics/kinova-ros/tree/kinetic/kinova_description
  - gait2392 human skeleton: https://github.com/cyberbotics/urdf2webots/tree/master/tests/sources/gait2392_simbody

## Acknowledgements

<a href="http://rosin-project.eu">
  <img src="http://rosin-project.eu/wp-content/uploads/rosin_ack_logo_wide.png"
       alt="rosin_logo" height="60" >
</a></br>

Supported by ROSIN - ROS-Industrial Quality-Assured Robot Software Components.
More information: <a href="http://rosin-project.eu">rosin-project.eu</a>

<img src="http://rosin-project.eu/wp-content/uploads/rosin_eu_flag.jpg"
     alt="eu_flag" height="45" align="left" >

This project has received funding from the European Union’s Horizon 2020
research and innovation programme under grant agreement no. 732287.

<br>

<a href="https://opendr.eu/">
  <img src="https://opendr.eu/wp-content/uploads/2020/01/logo-300x125.png"
       alt="opendr_logo" height="60" >
</a></br>

Supported by OpenDR - Open Deep Learning Toolkit for Robotics.
More information: <a href="https://opendr.eu/">opendr.eu</a>

<img src="https://opendr.csd.auth.gr/wp-content/uploads/2019/12/Flag_of_Europe-300x200.png"
     alt="eu_flag" height="45" align="left" >

This project has received funding from the European Union’s Horizon 2020
research and innovation programme under grant agreement no. 871449.
