"""Adapt the authored Expedition study to the validated body-only game contract.

Blender -b /tmp/expedition_paint.blend --python tools/cars/blender/export_expedition.py
The engine owns all road/spare tires and suspension. No reference mesh is exported.
"""
import os
import sys
from types import SimpleNamespace
import bpy
import bmesh
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import make_body_blend as base
import tf_vehicle
import export_body

repo=tf_vehicle.REPO
vehicle=tf_vehicle.Vehicle('trailforge_expedition')
body=bpy.data.collections['SURFACE_STUDY'];body.name='BODY'
roles={'clay':'paint_primary','roof':'paint_secondary','accent':'paint_accent','trim':'cladding',
       'powder_coated_steel':'metal','alloy':'metal','edge':'rubber','hooks':'paint_accent',
       'glass':'glass','mirror_glass':'chrome','lens':'lens','led':'light_head',
       'amber':'light_indicator','blue':'plastic'}
liner=bpy.data.materials.new('liner.expedition');liner.use_nodes=True;liner['tf_role']='liner'
liner['tf_mud']=1.0
liner.diffuse_color=(.006,.007,.008,1)
shader=liner.node_tree.nodes['Principled BSDF']
shader.inputs['Base Color'].default_value=liner.diffuse_color
shader.inputs['Roughness'].default_value=.93
shader.inputs['Metallic'].default_value=0
material_cache={}
for o in list(body.objects):
    bpy.ops.object.select_all(action='DESELECT')
    o.hide_set(False);o.select_set(True);bpy.context.view_layer.objects.active=o
    if o.type=='CURVE':bpy.ops.object.convert(target='MESH')
    if o.type!='MESH':continue
    # Bake authored geometry before generating the matching convex collision hulls.
    for modifier in list(o.modifiers):
        bpy.ops.object.modifier_apply(modifier=modifier.name)
    # Optical faces look rearward (+Y in Blender); the two-sided studio hid reversed winding.
    if o.name.startswith('mirror_optical_glass_'):
        if sum(face.normal.y*face.area for face in o.data.polygons)<0:
            bm=bmesh.new();bm.from_mesh(o.data)
            bmesh.ops.reverse_faces(bm,faces=list(bm.faces))
            bm.to_mesh(o.data);bm.free();o.data.update()
            if o.data.has_custom_normals:o.data.normals_split_custom_set([(0,0,0)]*len(o.data.loops))
    for slot in o.material_slots:
        m=slot.material
        role=roles.get(m.name.split('.')[0])
        if role is None:raise ValueError('Unmapped material: '+m.name)
        if o.name.startswith(('tail_red','tail_reflector','rear_lamp_red')):role='light_tail'
        if o.name.startswith(('tail_reverse','rear_reverse')):role='light_reverse'
        key=(m.name,role)
        if key in material_cache:
            slot.material=material_cache[key];continue
        copy=m.copy();copy.name=role+'.expedition';copy['tf_role']=role
        material_cache[key]=copy
        # The game contract supports scalar PBR properties, not Blender noise nodes.
        shader=copy.node_tree.nodes.get('Principled BSDF')
        for name in ('Roughness','Normal'):
            for link in list(shader.inputs[name].links):copy.node_tree.links.remove(link)
        slot.material=copy
    # The cut wheelhouse walls inherit paint from the shell. Assign the dark liner
    # only inside the authored openings; keep exterior fenders and doors painted.
    if o.name.startswith(('body_shell','front_door_','rear_door_')):
        slot_index=len(o.data.materials);o.data.materials.append(liner)
        assigned=0
        for face in o.data.polygons:
            p=o.matrix_world@face.center
            dz=min(abs(-p.y-1.4),abs(-p.y+1.4))
            lip=.220 if dz<=.335 else .220-(dz-.335)*(.265/.170)
            if dz<.515 and .32<abs(p.x)<.852 and p.z<lip+.012:
                face.material_index=slot_index;assigned+=1
        print('LINER',o.name,assigned,'faces')
    # "rim" is reserved for procedural wheel parts, including substring matches.
    o.name=o.name.replace('rim','surround')
    if 'mirror' in o.name:o['tf_collides']=False

for name,point,rotation in [('rear',(0,.40,-2.36),(-1.57079632679,0,0)),
                            ('roof',(0,1.075,-.78),(0,0,0))]:
    empty=bpy.data.objects.new('SPARE_'+name,None);body.objects.link(empty)
    empty.location=tf_vehicle.blender(point[0],point[1],point[2]);empty.rotation_euler=rotation
col=base.collection('COL')
k=SimpleNamespace(front_wheel=vehicle.wheels[0],rear_wheel=vehicle.wheels[2],deck=.53)
base.collision_pieces(vehicle,k,body,col)
bpy.context.scene['tf_vehicle']='trailforge_expedition'
bpy.ops.wm.save_as_mainfile(filepath=os.path.join(repo,'build/expedition-game.blend'))
raise SystemExit(export_body.run(out=os.path.join(vehicle.dir,'body.glb'),vehicle=vehicle.json_path,
                                source=os.path.join(repo,'build/expedition-source.glb')))
