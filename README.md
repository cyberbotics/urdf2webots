# urdf2webots

![Test Status](https://github.com/cyberbotics/urdf2webots/actions/workflows/test.yml/badge.svg)
[![PyPI version](https://badge.fury.io/py/urdf2webots.svg)](https://badge.fury.io/py/urdf2webots)

This tool converts URDF files into Webots PROTO files or into Webots Robot node strings.
Python 3.5 or higher is required.

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
python -m urdf2webots.importer --input=someRobot.urdf [--output=outputFile] [--normal] [--box-collision] [--tool-slot=linkName] [--help]
```

### Arguments

The script accepts the following arguments:
  - **-h, --help**: Show the help message and exit.
  - **--input=INPUT**: Specifies the URDF file to convert.
  - **--output=OUTPUT**: If set, specifies the path and, if ending in ".proto", name of the resulting PROTO file. The filename minus the .proto extension will be the robot name (for PROTO conversion only).
  - **--robot-name**: Specify the name of the robot and generate a Robot node string instead of a PROTO file (has to be unique).
  - **--normal**: If set, the normals are exported if present in the URDF definition.
  - **--box-collision**: If set, the bounding objects are approximated using boxes.
  - **--tool-slot=LinkName**: Specify the link that you want to add a tool slot to (exact link name from URDF, for PROTO conversion only).
  - **--translation="0 0 0"**: Set the translation field of the PROTO file or Webots Robot node string.
  - **--rotation="0 0 1 0"**: Set the rotation field of the PROTO file or Webots Robot node string.
  - **--init-pos=JointPositions**: Set the initial positions of your robot joints. Example: `--init-pos="[1.2, 0.5, -1.5]"` would set the first 3 joints of your robot to the specified values, and leave the rest with their default value.
  - **--link-to-def**: Creates a DEF with the link name for each solid to be able to access it using getFromProtoDef(defName) (for PROTO conversion only).
  - **--joint-to-def**: Creates a DEF with the joint name for each joint to be able to access it using getFromProtoDef(defName) (for PROTO conversion only).
  - **--relative-path-prefix**: If **--input** is not set, the relative paths in your URDF file sent through stdin will use this prefix. For example: `filename="head.obj"` with `--relative-path-prefix="/home/user/myRobot/"` will become `filename="/home/user/myRobot/head.obj"`.

In case the **--input** option is missing, the script will read the URDF content from `stdin`.
In that case, you can pipe the content of your URDF file into the script: `cat my_robot.urdf | urdf2proto.py`.
Relative paths present in your URDF file will be treated relatively to the current directory from which the script is called unless **--relative-path-prefix** is set.

> Previously the **--static-base** argument was supported in order to set the base link to be static (disabled physics). It has been removed as there is a better way to do it by adding the following to your URDF file (assuming **base_link** is the root link of your robot):
>
>```
> <link name="world" />
><joint name="world_joint" type="fixed">
>    <parent link="world" />
>    <child link="base_link" />
></joint>
>```

### In your Python Code

#### Arguments

The command line arguments available from the terminal are also available from the Python interface, but some have different names:

| Terminal   |      Python      |
|----------|-------------|
| --input |  input |
| --output |  output |
| --robot-name |  robotName |
| --normal |  normal |
| --box-collision |  boxCollision |
| --tool-slot |  toolSlot |
| --translation |  initTranslation |
| --rotation |  initRotation |
| --init-pos |  initPos |
| --link-to-def |  linkToDef |
| --joint-to-def |  jointToDef |
| --relative-path-prefix |  relativePathPrefix |

In Python, you can convert a URDF file by passing its path as an argument to the `convertUrdfFile()` function or directly by passing its content as an argument to the `convertUrdfContent()` function.

#### Convert into Webots PROTO files

```
from urdf2webots.importer import convertUrdfFile
convertUrdfFile(input = 'MY_PATH/MY_URDF.urdf')
```

or

```
import pathlib
from urdf2webots.importer import convertUrdfContent
robot_description = pathlib.Path('MY_PATH/MY_URDF.urdf').read_text()
convertUrdfContent(input = robot_description)
```

#### Convert into Webots Robot node strings

```
from urdf2webots.importer import convertUrdfFile
convertUrdfFile(input = 'MY_PATH/MY_URDF.urdf', robotName="myRobot")
```

or

```
import pathlib
from urdf2webots.importer import convertUrdfContent
robot_description = pathlib.Path('MY_PATH/MY_URDF.urdf').read_text()
convertUrdfContent(input = robot_description, robotName="myRobot")
```

### In-Depth Tutorial
Check out [this tutorial](./docs/tutorial.md) for a more in-depth, step by step instruction, on how to:
- Generate a URDF file from a ROS repository.
- Convert your URDF file to a Webots PROTO file.
- Load your converted model into Webots and make final adjustments.
- Convert your URDF file to a Webots Robot string and import it.


## Notes
This tool was tested using Webots R2022b on Ubuntu22.04.
You can find the sources of these URDF files here:
  - Universal Robot: https://github.com/ros-industrial/universal_robot/tree/kinetic-devel/ur_description
  - PR2 robot: https://github.com/PR2/pr2_common/tree/kinetic-devel/pr2_description
  - Motoman robot: https://github.com/ros-industrial/motoman/tree/kinetic-devel/motoman_sia20d_support
  - Kinova robot: https://github.com/Kinovarobotics/kinova-ros/tree/kinetic/kinova_description
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
