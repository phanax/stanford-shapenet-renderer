# A simple script that uses blender to render views of a single object by rotation the camera around it.
# Also produces depth map at the same time.
#
# Example:
# blender --background --python  /Users/canyuce/Downloads/stanford-shapenet-renderer-master/render_blender.py --
# --output_folder /Users/canyuce/Renderer/output/ /Users/canyuce/Downloads/mitsuba-shapenet-master/shapenet/02958343/1a0bc9ab92c915167ae33d942430658c/model.obj 1> /dev/null
#

import argparse, sys, os, math
import numpy as np
import bpy
import random
from mathutils import Vector
from math import radians

parser = argparse.ArgumentParser(description='Renders given obj file by rotation a camera around it.')
parser.add_argument('--views', type=int, default=30,
                    help='number of views to be rendered')
parser.add_argument('obj', type=str,
                    help='Path to the obj file to be rendered.')
parser.add_argument('--output_folder', type=str, default='/tmp',
                    help='The path the output will be dumped to.')
parser.add_argument('--remove_doubles', type=bool, default=True,
                    help='Remove double vertices to improve mesh quality.')
parser.add_argument('--edge_split', type=bool, default=True,
                    help='Adds edge split filter.')
parser.add_argument('--depth_scale', type=float, default=0,
                    help='Scaling that is applied to depth. Depends on side of mesh. Try out various values until you get a good result.')

argv = sys.argv[sys.argv.index("--") + 1:]
args = parser.parse_args(argv)
f = args.obj

# Set up rendering of depth map:
bpy.context.scene.use_nodes = True
tree = bpy.context.scene.node_tree
links = tree.links

# Add passes for additionally dumping albed and normals.
# bpy.context.scene.render.layers["RenderLayer"].use_pass_normal = True
# bpy.context.scene.render.layers["RenderLayer"].use_pass_color = True

# clear default nodes
for n in tree.nodes:
    tree.nodes.remove(n)

# create input render layer node
rl = tree.nodes.new('CompositorNodeRLayers')

map = tree.nodes.new(type="CompositorNodeMapValue")
# Size is chosen kind of arbitrarily, try out until you're satisfied with resulting depth map.
map.offset = [0]
#map.size = [1]
map.use_min = True
map.min = [0]
map.use_max = True
map.max = [1000]

#invert = tree.nodes.new(type="CompositorNodeInvert")

# create a file output node and set the path
depthFileOutput = tree.nodes.new(type="CompositorNodeOutputFile")
depthFileOutput.label = 'Depth Output'
depthFileOutput.format.file_format = 'HDR'
#depthFileOutput.format.color_depth = '32'
depthFileOutput.format.use_zbuffer = True

links.new(rl.outputs['Z'], map.inputs[0])
links.new(map.outputs[0], depthFileOutput.inputs[0])

#links.new(rl.outputs['Z'], depthFileOutput.inputs[0])
#scale_normal = tree.nodes.new(type="CompositorNodeMixRGB")
#scale_normal.blend_type = 'MULTIPLY'
# scale_normal.use_alpha = True
#scale_normal.inputs[2].default_value = (0.5, 0.5, 0.5, 1)
#links.new(rl.outputs['Normal'], scale_normal.inputs[1])

#bias_normal = tree.nodes.new(type="CompositorNodeMixRGB")
#bias_normal.blend_type = 'ADD'
# bias_normal.use_alpha = True
#bias_normal.inputs[2].default_value = (0.5, 0.5, 0.5, 0)
#links.new(scale_normal.outputs[0], bias_normal.inputs[1])

#normalFileOutput = tree.nodes.new(type="CompositorNodeOutputFile")
#normalFileOutput.label = 'Normal Output'
#links.new(bias_normal.outputs[0], normalFileOutput.inputs[0])

#albedoFileOutput = tree.nodes.new(type="CompositorNodeOutputFile")
#albedoFileOutput.label = 'Albedo Output'
## For some reason,
#links.new(rl.outputs['Color'], albedoFileOutput.inputs[0])

# Delete default cube
bpy.data.objects['Cube'].select = True
bpy.ops.object.delete()

bpy.ops.import_scene.obj(filepath=f)
#print(list())
for object in bpy.context.scene.objects:
    if object.name in ['Camera', 'Lamp']:
        continue
    bpy.context.scene.objects.active = object
    # Some examples have duplicate vertices, these are removed here.
    if args.remove_doubles:
        bpy.ops.object.mode_set(mode='EDIT')
        bpy.ops.mesh.remove_doubles()
        bpy.ops.object.mode_set(mode='OBJECT')
    if args.edge_split:
        bpy.ops.object.modifier_add(type='EDGE_SPLIT')
        bpy.context.object.modifiers["EdgeSplit"].split_angle = 1.32645
        bpy.ops.object.modifier_apply(apply_as='DATA', modifier="EdgeSplit")

def parent_obj_to_camera(b_camera):
    origin = (0, 0, 0)
    b_empty = bpy.data.objects.new("Empty", None)
    b_empty.location = origin
    b_camera.parent = b_empty  # setup parenting

    scn = bpy.context.scene
    scn.objects.link(b_empty)
    scn.objects.active = b_empty
    return b_empty

bpy.data.objects['Lamp'].data.energy = 0

light_dist = 12
light_num_highbound = 8

def obj_centened_camera_pos(dist, azimuth_deg, elevation_deg):
    phi = float(elevation_deg) / 180 * math.pi
    theta = float(azimuth_deg) / 180 * math.pi
    x = (dist * math.cos(theta) * math.cos(phi))
    y = (dist * math.sin(theta) * math.cos(phi))
    z = (dist * math.sin(phi))
    return (x, y, z)

scene = bpy.context.scene
scene.render.resolution_x = 600
scene.render.resolution_y = 600
scene.render.resolution_percentage = 100
scene.render.alpha_mode = 'TRANSPARENT'
#scene.world.light_settings.use_environment_light = True
scene.world.light_settings.gather_method = 'APPROXIMATE'
#scene.world.light_settings.use_ambient_occlusion = True
scene.world.light_settings.use_indirect_light = True
scene.camera.data.angle = 50*(math.pi/180.0)

cam = scene.objects['Camera']
cam.location = Vector((0, 1.20, 0))
cam_constraint = cam.constraints.new(type='TRACK_TO')
cam_constraint.track_axis = 'TRACK_NEGATIVE_Z'
cam_constraint.up_axis = 'UP_Y'
b_empty = parent_obj_to_camera(cam)
cam_constraint.target = b_empty

model_identifier = os.path.split(os.path.split(os.path.split(args.obj)[0])[0])[1]
fp = os.path.join(args.output_folder, model_identifier, model_identifier)
scene.render.image_settings.file_format = 'PNG' 
rotation_mode = 'XYZ'

for output_node in [depthFileOutput]:
    output_node.base_path = ''

# set lights
bpy.ops.object.select_all(action='TOGGLE')
if 'Lamp' in list(bpy.data.objects.keys()):
   bpy.data.objects['Lamp'].select = True # remove default light
bpy.ops.object.delete()

# clear default lights
bpy.ops.object.select_by_type(type='LAMP')
bpy.ops.object.delete(use_global=False)

# set point lights
for j in range(0, light_num_highbound):
    if j < (light_num_highbound / 2):
        light_azimuth_deg = j * 360 / light_num_highbound * 2 + 45
        light_elevation_deg  = -45
    else:
        light_azimuth_deg = (j-light_num_highbound / 2) * 360 / light_num_highbound * 2 + 45
        light_elevation_deg  = 45

    lx, ly, lz = obj_centened_camera_pos(light_dist, light_azimuth_deg, light_elevation_deg)
    bpy.ops.object.lamp_add(type='POINT', view_align = True, location=(lx, ly, lz))
    bpy.data.objects['Point'].data.energy = 2.7

for i in range(0, args.views):
    scene.render.filepath = fp + '_r_{0:03d}'.format(int(i))
    depthFileOutput.file_slots[0].path = scene.render.filepath
    # normalFileOutput.file_slots[0].path = scene.render.filepath + "_normal.png"
    # albedoFileOutput.file_slots[0].path = scene.render.filepath + "_albedo.png"

    bpy.ops.render.render(write_still=True)  # render still

    phi = random.uniform(-0.5, 0.5) * math.pi / 2 # theta should be in  []
    theta = random.uniform(0, 1) * math.pi / 4 # theta should be in  []
    b_empty.rotation_euler[2] = phi;
    b_empty.rotation_euler[0] = theta;


# -*- coding: utf-8 -*-
