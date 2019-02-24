import os
import sys
import numpy as np
import mathutils
from math import pi
import bpy
from mathutils import Vector
from mathutils.bvhtree import BVHTree
from bpy_extras.object_utils import world_to_camera_view


obj = bpy.data.objects['Compound Ceiling:Plain:263090:Plain:263090 : Compound Ceiling:P']

def BVHTreeAndVerticesInWorldFromObj( obj ):
    mWorld = obj.matrix_world
    vertsInWorld = [mWorld * v.co for v in obj.data.vertices]

    bvh = BVHTree.FromPolygons( vertsInWorld, [p.vertices for p in obj.data.polygons] )
    
    return bvh, vertsInWorld

bvh, vertices = BVHTreeAndVerticesInWorldFromObj( obj )

for i, v in enumerate( vertices ):
    bpy.ops.mesh.primitive_cube_add(location=(v))
    bpy.ops.transform.resize(value=(0.01, 0.01, 0.01)) # thrushold of the cube size. decresing will result in missed detections and incresing will result in wrong detections.
    print(i)
