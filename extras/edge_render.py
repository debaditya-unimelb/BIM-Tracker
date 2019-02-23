#!/usr/bin/env python

import os
import sys
sys.path.append('/usr/lib/python3/dist-packages') # Add python3 packages to the environment
#import yaml
import numpy as np
import mathutils
from math import pi
import bpy



from mathutils import Vector
from mathutils.bvhtree import BVHTree
from bpy_extras.object_utils import world_to_camera_view


def goto_frame(scene, k):
    scene.frame_current = k
    bpy.context.scene.frame_set(bpy.context.scene.frame_current)
    
# Create a BVH tree and return bvh and vertices in world coordinates 
def BVHTreeAndVerticesInWorldFromObj( obj ):
    mWorld = obj.matrix_world
    vertsInWorld = [mWorld * v.co for v in obj.data.vertices]

    bvh = BVHTree.FromPolygons( vertsInWorld, [p.vertices for p in obj.data.polygons] )
    
    return bvh, vertsInWorld

# Deselect mesh polygons and vertices
def DeselectEdgesAndPolygons( obj ):
    for p in obj.data.polygons:
        p.select = False
    for e in obj.data.edges:
        e.select = False

cam = bpy.data.objects['Camera']
cam_new = bpy.data.objects['Camera.001']
scene = bpy.data.scenes['Scene']
obj = bpy.data.objects['Compound Ceiling:Plain:263090:Plain:263090 : Compound Ceiling:P']

# Threshold to test if ray cast corresponds to the original vertex
limit = 0.1

# Deselect mesh elements
DeselectEdgesAndPolygons( obj )

# In world coordinates, get a bvh tree and vertices
bvh, vertices = BVHTreeAndVerticesInWorldFromObj( obj )

print( '-------------------' )

use_omnicam_coords = False

duration_trajectory = 2200/30

num_frame_origin = scene.frame_start
num_frame_end = scene.frame_end
num_frames = num_frame_end - num_frame_origin + 1

framerate = 30 # fps

print('Frame rate: {} fps'.format(framerate))

scaleFactor = 1000.0
        
T_blender_svo = mathutils.Matrix(((1, 0, 0, 0),
    (0, -1, 0, 0),
    (0, 0, -1, 0),
    (0.0, 0.0, 0.0, 1.0)))
    
T_svo_blender = T_blender_svo

if use_omnicam_coords:
    T_camsvo_camsvo_omni = mathutils.Matrix(((0, 1, 0, 0),
        (1, 0, 0, 0),
        (0, 0, -1, 0),
        (0.0, 0.0, 0.0, 1.0)))
else:
    T_camsvo_camsvo_omni = mathutils.Matrix(((1, 0, 0, 0),
        (0, 1, 0, 0),
        (0, 0, 1, 0),
        (0.0, 0.0, 0.0, 1.0)))

traj_out = open("trajectory_highres.txt", "w")
file = open("visible_vertices_all_frames.txt", "w")
file1 = open("visible_edges_whole_mesh.txt", "w")


timestamp = 0.0
for k in range(num_frame_origin, num_frame_end+1):
    
    print('{}/{}'.format(k,num_frame_end+1))
    
    goto_frame(scene, k)
    timestamp += 1.0 / framerate
    
    T_world_camblender = scaleFactor * cam.matrix_world.copy()
    T_world_camsvo = T_world_camblender * T_blender_svo
    
    T_world_camsvo_omni = T_world_camsvo * T_camsvo_camsvo_omni
    
    T = (T_world_camsvo_omni).translation
    q = (T_world_camsvo_omni).to_euler()
    #gt_out.write("%d %f %f %f %f %f %f \n" % (k, T[0], T[1], T[2], q[0], q[1], q[2]))
    traj_out.write("%f %f %f %f %f %f %f\n" % (timestamp, T[0], T[1], T[2], q[0], q[1], q[2]))
    #img_out.write("%d %06f img/img%04d_0.png\n" % (k, timestamp, k))
    #depth_out.write("%d depth/img%04d_0.depth\n" % (k, k))

    cam_new.location=T/1000
    cam_new.rotation_euler=[pi+q[0], q[1], -2*pi+q[2] ]
    
    for i, v in enumerate( vertices ):
    # Get the 2D projection of the vertex
        co2D = world_to_camera_view( scene, cam_new, v )

    # By default, deselect it
        obj.data.vertices[i].select = False

    # If inside the camera view
        if 0.0 <= co2D.x <= 1.0 and 0.0 <= co2D.y <= 1.0 and co2D.z >0: 
        # Try a ray cast, in order to test the vertex visibility from the camera
            location, normal, index, distance = bvh.ray_cast( cam_new.location, (v - cam_new.location).normalized() )
        # If the ray hits something and if this hit is close to the vertex, we assume this is the vertex
            if location and (v - location).length < limit:
                obj.data.vertices[i].select = True
                v = obj.data.vertices[i].co
                mat = obj.matrix_world
                loc = mat * v
                print(loc)
                file.write("%d %f %f %f %f %d\n" % (i, loc[0]*1000, loc[1]*1000, loc[2]*1000, timestamp, k))

del bvh
for k in range (0, len(obj.data.edges)):
    A = (bpy.data.meshes['mesh1379'].edges[k].vertices[0])
    B = (bpy.data.meshes['mesh1379'].edges[k].vertices[1])
    file1.write("%d %d %d\n" % (k, A, B))

traj_out.close()
file.close()
file1.close()
