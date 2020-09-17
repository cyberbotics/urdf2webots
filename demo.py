#!/usr/bin/env python

"""URDF files to Webots PROTO converter."""

import optparse

from urdf2webots.importer import convert2urdf


if __name__ == '__main__':
    optParser = optparse.OptionParser(usage='usage: %prog --input=my_robot.urdf [options]')
    optParser.add_option('--input', dest='inFile', default='', help='Specifies the urdf file to convert.')
    optParser.add_option('--output', dest='outFile', default='', help='Specifies the path and, if ending in ".proto", name of the resulting PROTO file.'
                         ' The filename minus the .proto extension will be the robot name.')
    optParser.add_option('--normal', dest='normal', action='store_true', default=False,
                         help='If set, the normals are exported if present in the URDF definition.')
    optParser.add_option('--box-collision', dest='boxCollision', action='store_true', default=False,
                         help='If set, the bounding objects are approximated using boxes.')
    optParser.add_option('--disable-mesh-optimization', dest='disableMeshOptimization', action='store_true', default=False,
                         help='If set, the duplicated vertices are not removed from the meshes (this can speed up a lot the '
                         'conversion).')
    optParser.add_option('--multi-file', dest='enableMultiFile', action='store_true', default=False,
                         help='If set, the mesh files are exported as separated PROTO files')
    optParser.add_option('--static-base', dest='staticBase', action='store_true', default=False,
                         help='If set, the base link will have the option to be static (disable physics)')
    optParser.add_option('--tool-slot', dest='toolSlot', default=None,
                         help='Specify the link that you want to add a tool slot too (exact link name from urdf)')
    optParser.add_option('--rotation', dest='initRotation', default='0 1 0 0',
                         help='Set the rotation field of your PROTO file.')
    optParser.add_option('--init-pos', dest='initPos', default=None,
                         help='Set the initial positions of your robot joints. Example: --init-pos="[1.2, 0.5, -1.5]" would set '
                         'the first 3 joints of your robot to the specified values, and leave the rest with their default value.')
    options, args = optParser.parse_args()

    convert2urdf(options.inFile, options.outFile, options.normal, options.boxCollision, options.disableMeshOptimization,
                 options.enableMultiFile, options.staticBase, options.toolSlot, options.initRotation, options.initPos)
