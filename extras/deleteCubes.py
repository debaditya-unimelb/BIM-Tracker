import bpy

scene = bpy.context.scene

for ob in scene.objects:
    if ob.type == 'MESH' and ob.name.startswith("Cube"):
        ob.select = True
    else: 
        ob.select = False

bpy.ops.object.delete() 
