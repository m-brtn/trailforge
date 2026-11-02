"""Expedition design study: primary surfaces and equipment from the four-view reference.

Blender -b --factory-startup --python tools/cars/blender/expedition_clay.py
Outputs /tmp/expedition_clay.blend and neutral studio renders; pass -- --paint for the painted version.
Use -- --paint --detail-only to render the mirror and front-corner inspections.
Use -- --paint --finish-only for the combined body, glazing, materials and rear-detail review.
Use -- --paint --no-render to build only the source for export_expedition.py.
Does not overwrite the game body; the exporter validates wheel sweep and collision clearance.
Run trailforge_model_export for the existing SUV into /tmp/trailforge_suv_reference.obj first;
only its physics-size wheels and undercarriage are used as scale references. No external geometry is imported.
"""
import math
import os
import sys
import bpy
import bmesh
from mathutils import Vector
from mathutils.bvhtree import BVHTree
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import make_body_blend as base
from tf_vehicle import blender as B


def main():
    bpy.ops.wm.read_factory_settings(use_empty=True)
    col=base.collection('SURFACE_STUDY')
    helpers=base.collection('CUT_helpers')
    painted='--paint' in sys.argv
    mats={}
    for name,color,rough in [('clay',(.42,.44,.46),.36),('trim',(.09,.105,.12),.48),
                            ('glass',(.075,.09,.10),.23),
                            ('mirror_glass',(.40,.44,.47),.065),('lens',(.025,.035,.045),.19),('alloy',(.30,.33,.35),.30),
                            ('accent',(.45,.46,.47),.32),('roof',(.64,.65,.64),.32),
                            ('edge',(.06,.065,.060),.42),('hooks',(.28,.29,.30),.34),('led',(.72,.78,.82),.22),
                            ('amber',(.4,.42,.44),.30),('blue',(.18,.2,.22),.4),
                            ('ground',(.19,.205,.22),.85)]:
        m=bpy.data.materials.new(name);m.diffuse_color=(*color,1);m.use_nodes=True
        p=m.node_tree.nodes['Principled BSDF'];p.inputs['Base Color'].default_value=(*color,1)
        p.inputs['Roughness'].default_value=rough;mats[name]=m
    if painted:
        colors={'clay':(.105,.14,.052),'trim':(.022,.027,.028),'glass':(.009,.016,.023),
                'accent':(.95,.245,.008),'roof':(.72,.73,.65),
                'hooks':(.68,.012,.006),'amber':(.85,.15,.003),'blue':(.015,.20,.40)}
        for name,color in colors.items():
            mats[name].node_tree.nodes['Principled BSDF'].inputs['Base Color'].default_value=(*color,1)
        for name in ('clay','accent','roof'):
            mats[name].node_tree.nodes['Principled BSDF'].inputs['Coat Weight'].default_value=.35
        for name in ('led','amber'):
            shader=mats[name].node_tree.nodes['Principled BSDF']
            shader.inputs['Emission Color'].default_value=shader.inputs['Base Color'].default_value
            shader.inputs['Emission Strength'].default_value=1.5
    mats['alloy'].node_tree.nodes['Principled BSDF'].inputs['Metallic'].default_value=.75
    mats['mirror_glass'].node_tree.nodes['Principled BSDF'].inputs['Metallic'].default_value=1
    if painted:
        for name in ('clay','accent','roof'):
            shader=mats[name].node_tree.nodes['Principled BSDF']
            shader.inputs['Metallic'].default_value=0
            shader.inputs['Roughness'].default_value=.32
            shader.inputs['Coat Weight'].default_value=.85
            shader.inputs['Coat Roughness'].default_value=.18
        for name,rough in [('glass',.065),('lens',.12),('trim',.52),('hooks',.30)]:
            mats[name].node_tree.nodes['Principled BSDF'].inputs['Roughness'].default_value=rough
        armour=mats['trim'].copy();armour.name='powder_coated_steel';mats['armour']=armour
        shader=armour.node_tree.nodes['Principled BSDF'];shader.inputs['Metallic'].default_value=0
        shader.inputs['Roughness'].default_value=.43
        for name,scale,strength,distance in [('trim',330,.16,.00020),('armour',440,.12,.00012)]:
            nodes=mats[name].node_tree.nodes;links=mats[name].node_tree.links
            noise=nodes.new('ShaderNodeTexNoise');noise.inputs['Scale'].default_value=scale
            noise.inputs['Detail'].default_value=2
            coords=nodes.new('ShaderNodeTexCoord');links.new(coords.outputs['Object'],noise.inputs['Vector'])
            variation=nodes.new('ShaderNodeMapRange')
            roughness_base=nodes['Principled BSDF'].inputs['Roughness'].default_value
            variation.inputs['To Min'].default_value=roughness_base-.045;variation.inputs['To Max'].default_value=roughness_base+.045
            links.new(noise.outputs['Fac'],variation.inputs['Value'])
            links.new(variation.outputs['Result'],nodes['Principled BSDF'].inputs['Roughness'])
            bump=nodes.new('ShaderNodeBump');bump.inputs['Strength'].default_value=strength
            bump.inputs['Distance'].default_value=distance
            links.new(noise.outputs['Fac'],bump.inputs['Height'])
            links.new(bump.outputs['Normal'],nodes['Principled BSDF'].inputs['Normal'])
    else:mats['armour']=mats['trim']
    if painted:
        glass=mats['glass'].node_tree.nodes['Principled BSDF']
        glass.inputs['IOR'].default_value=1.52
        glass.inputs['Transmission Weight'].default_value=.20
        glass.inputs['Coat Weight'].default_value=.12
        mats['amber'].node_tree.nodes['Principled BSDF'].inputs['Emission Strength'].default_value=.35
        mats['alloy'].node_tree.nodes['Principled BSDF'].inputs['Metallic'].default_value=1



    def finish_normals(o):
        for poly in o.data.polygons: poly.use_smooth=True
        o.data.set_sharp_from_angle(angle=math.radians(38))
        modifier=o.modifiers.new('surface_normals','WEIGHTED_NORMAL')
        modifier.keep_sharp=True
        modifier.weight=40
        return o

    def obj(name,verts,faces,mat='clay',bevel=.012,collection=col):
        bm=bmesh.new(); vv=[bm.verts.new(B(*v)) for v in verts]
        for f in faces: bm.faces.new([vv[i] for i in f])
        bmesh.ops.recalc_face_normals(bm,faces=list(bm.faces))
        o=base.mesh_object(name,bm,collection,mats[mat])
        if bevel: base.bevel(o,bevel,4,25)
        return finish_normals(o)

    def box(name,c,h,mat='clay',bevel=.012):
        o=base.mesh_object(name,base.box(c,h),col,mats[mat])
        if bevel: base.bevel(o,bevel,4)
        return finish_normals(o)

    def loft(name,rings,mat='clay',bevel=.015):
        n=len(rings[0]);v=[p for ring in rings for p in ring];f=[]
        for j in range(len(rings)-1):
            for i in range(n):
                f.append((j*n+i,j*n+(i+1)%n,(j+1)*n+(i+1)%n,(j+1)*n+i))
        f.extend([tuple(reversed(range(n))),tuple(range((len(rings)-1)*n,len(rings)*n))])
        return obj(name,v,f,mat,bevel)

    def tube(name,pts,r,mat='trim'):
        curve=bpy.data.curves.new(name,'CURVE');curve.dimensions='3D';curve.resolution_u=2
        curve.bevel_depth=r;curve.bevel_resolution=3;curve.use_fill_caps=True
        s=curve.splines.new('POLY');s.points.add(len(pts)-1)
        for p,c in zip(s.points,pts): p.co=(*B(*c),1)
        o=bpy.data.objects.new(name,curve);col.objects.link(o);o.data.materials.append(mats[mat]);return o

    def closed_curve(points,steps=5):
        """Closed Catmull-Rom contour through authored shape landmarks, not a rounded rectangle."""
        out=[]
        for i,p1 in enumerate(points):
            p0=Vector(points[(i-1)%len(points)]);p1=Vector(p1)
            p2=Vector(points[(i+1)%len(points)]);p3=Vector(points[(i+2)%len(points)])
            for j in range(steps):
                t=j/steps
                out.append(tuple(.5*((2*p1)+(-p0+p2)*t+(2*p0-5*p1+4*p2-p3)*t*t
                                      +(-p0+3*p1-3*p2+p3)*t*t*t)))
        return out

    def rounded_path(points,radius=.035,steps=5):
        out=[Vector(points[0])]
        for i in range(1,len(points)-1):
            p=Vector(points[i]);a=Vector(points[i-1]);b=Vector(points[i+1])
            reach=min(radius,(p-a).length*.25,(b-p).length*.25)
            left=p+(a-p).normalized()*reach;right=p+(b-p).normalized()*reach
            for j in range(steps+1):
                t=j/steps;out.append((1-t)**2*left+2*t*(1-t)*p+t*t*right)
        out.append(Vector(points[-1]));return out

    def sculpt_surface(name,rings,mat='trim',close=True,cap_end=None,cap_start=None):
        """Smooth all-quad surface with authored changing cross-sections and a separate seam boundary."""
        n=len(rings[0]);verts=[v for ring in rings for v in ring];faces=[]
        for j in range(len(rings)-1):
            for i in range(n if close else n-1):
                faces.append((j*n+i,j*n+(i+1)%n,(j+1)*n+(i+1)%n,(j+1)*n+i))
        if cap_end is not None:
            center=len(verts);verts.append(cap_end);start=(len(rings)-1)*n
            faces.extend((start+i,start+(i+1)%n,center) for i in range(n))
        if cap_start is not None:
            center=len(verts);verts.append(cap_start)
            faces.extend(((i+1)%n,i,center) for i in range(n))
        o=obj(name,verts,faces,mat,0)
        for mod in list(o.modifiers):o.modifiers.remove(mod)
        for face in o.data.polygons:face.use_smooth=True
        return o

    def mirror(sx):
        side='left' if sx>0 else 'right'
        # Full-size asymmetric silhouette: a high inboard shoulder, falling outer forehead,
        # swept lower chin and narrower throat where the folding joint meets the door.
        shape=closed_curve([(-.99,-.40),(-1.00,.36),(-.83,.80),(-.47,.92),
                            (.46,.88),(.84,.70),(.99,.32),(.96,-.36),(.70,-.72),
                            (.20,-.86),(-.52,-.77)],6)
        cx,cy,cz=1.003,.659,.653
        def point(u,v,z):return (sx*(cx+.111*u),cy+.080*v,cz+z)
        # Cross-sections progress from the glass flange to a compound-curved front cap.
        rings=[]
        for scale,depth in [(1,-.040),(1.01,-.024),(1.01,-.008),(.98,.008),
                            (.90,.020),(.75,.031),(.55,.038),(.30,.042),(.08,.043)]:
            rings.append([point(u*scale,v*scale,depth-.030*u*scale+.004*v*scale) for u,v in shape])
        cap=sculpt_surface('mirror_cap_'+side,rings,'trim',cap_end=point(0,0,.0432))
        # A molded parting line and a curved lower shell are separate from the cap.
        tube('mirror_cap_seam_'+side,[point(u,v,-.034-.030*u+.004*v) for u,v in shape]+[point(*shape[0],-.034-.030*shape[0][0]+.004*shape[0][1])],.0016,'edge')
        # Rear flange, stepped gasket, inset optical glass; the glass is slightly convex.
        bezel=[]
        for scale,depth in [(1.00,-.045),(.965,-.051),(.88,-.052),(.855,-.043)]:
            bezel.append([point(u*scale,v*scale,depth-.030*u+.004*v) for u,v in shape])
        sculpt_surface('mirror_glass_bezel_'+side,bezel,'trim')
        glass=[]
        for scale in (.85,.65,.42,.18,.015):
            glass.append([point(u*scale,v*scale,-.046-.007*(1-scale*scale)-.030*u*scale) for u,v in shape])
        optical=sculpt_surface('mirror_optical_glass_'+side,glass,'mirror_glass',cap_end=point(0,0,-.053))
        if sum(face.normal.y*face.area for face in optical.data.polygons)<0:
            bm=bmesh.new();bm.from_mesh(optical.data)
            bmesh.ops.reverse_faces(bm,faces=list(bm.faces))
            bm.to_mesh(optical.data);bm.free();optical.data.update()
        # The indicator is a conforming optical ribbon on the actual shell, not a tube attached to it.
        tree=BVHTree.FromPolygons([v.co for v in cap.data.vertices],[tuple(f.vertices) for f in cap.data.polygons])
        for name,width,mat,offset in [('recess',.052,'lens',.0006),('signal',.024,'amber',.0010)]:
            rings=[]
            for i in range(25):
                t=i/24;u=.02+.74*t;v=-.54+.08*t*t
                pair=[]
                for dv in (-width,width):
                    ray=Vector(B(*point(u,v+dv,.20)))
                    hit,normal,_,_=tree.ray_cast(ray,Vector(B(0,0,-1)))
                    if hit is None:raise RuntimeError('Indicator left mirror shell')
                    q=hit+normal*offset;pair.append((q.x,q.z,-q.y))
                rings.append(pair)
            sculpt_surface('mirror_indicator_'+name+'_'+side,rings,mat,close=False)
        # Tapered aero neck and a broad sail foot, instead of a cylindrical stalk.
        neckshape=closed_curve([(-1,-.3),(-.7,.8),(.0,1),(.8,.65),(1,-.3),(.35,-.8),(-.5,-.8)],4)
        neck=[]
        for x,y,z,ry,rz in [(.800,.565,.683,.024,.038),(.842,.565,.676,.026,.034),
                            (.880,.568,.669,.025,.030),(.925,.578,.638,.022,.024),(.950,.595,.625,.021,.023)]:
            neck.append([(sx*x,y+u*ry,z+v*rz) for u,v in neckshape])
        sculpt_surface('mirror_sculpted_neck_'+side,neck,'trim',
                       cap_start=(sx*.800,.565,.683),cap_end=(sx*.950,.595,.625))
        # The broad neck root enters the cabin skin directly; an overlapping sail
        # plate would emerge through the curved neck as disconnected slivers.

    # The hood, belt and sills are separate shaping stations, not a scaled rectangular primitive.
    # Wheelbase 2.80 m, track 1.80 m and 0.436 m wheel radius remain the game's physical values.
    rings=[]
    for z,inset,deck,floor in [(-2.15,.065,.53,-.10),(-2.06,.018,.55,-.20),
                              (-1.86,0,.56,-.23),(-.65,0,.555,-.23),
                              (.78,0,.55,-.23),(1.72,.012,.535,-.20),(2.12,.055,.50,-.10)]:
        section=[(0,floor),(.63,floor),(.80-inset,floor+.045),(.865-inset,-.075),
                 (.865-inset,.03),(.860-inset,deck-.105),(.840-inset,deck-.035),
                 (.805-inset,deck),(.63,deck+.012),(0,deck+.018)]
        ring=[(x,y,z) for x,y in section]+[(-x,y,z) for x,y in reversed(section[1:-1])]
        rings.append(ring)
    shell=loft('body_shell',rings,bevel=.022)

    # Angular wheelhouses: a flat crown with diagonal shoulders, softened edges, front to rear symmetry.
    # Reference silhouette study only: these closer arches require a shorter compression travel
    # than the original SUV. Do not export to the game before configuring and validating that travel.
    arch=[(.505,-.55),(.505,.005),(.335,.270),(-.335,.270),(-.505,.005),(-.505,-.55)]
    cutters=[]
    for sx in (-1,1):
        for zc in (-1.4,1.4):
            bm=base.prism([(zc+z,y) for z,y in arch],.55,sx*.95)
            cut=base.mesh_object('CUT_arch',bm,helpers,None);cut.hide_render=True;cut.display_type='WIRE'
            cutters.append(cut)
    base.cut_arches(shell,cutters)

    # Skin panels sit on sculpted sides with shallow door dishes, upper shoulder and lower kick-out.
    def side_panel(name,sx,z0,z1):
        # Dense compound surface with rolled perimeter, shallow lower dish and upper crown.
        section=[(-.16,.844),(-.11,.860),(-.07,.868),(-.025,.868),(.01,.868),
                 (.06,.868),(.15,.867),(.27,.865),(.38,.864),(.44,.863),
                 (.48,.8545),(.515,.8445),(.550,.813)]
        curve=[]
        for i in range(len(section)-1):
            p0=Vector(section[max(0,i-1)]);p1=Vector(section[i])
            p2=Vector(section[i+1]);p3=Vector(section[min(len(section)-1,i+2)])
            for k in range(4):
                t=k/4
                curve.append(.5*((2*p1)+(-p0+p2)*t+(2*p0-5*p1+4*p2-p3)*t*t+(-p0+3*p1-3*p2+p3)*t*t*t))
        curve.append(Vector(section[-1]));rings=[]
        for j in range(25):
            u=j/24;z=z0+(z1-z0)*u
            # A shared longitudinal shoulder replaces per-door sinusoidal bulges and ray noise.
            stations=[(-1.86,.560),(-.65,.555),(.78,.550),(1.72,.535)]
            deck=.550
            for (za,ya),(zb,yb) in zip(stations,stations[1:]):
                if za<=z<=zb:deck=ya+(yb-ya)*(z-za)/(zb-za);break
            row=[]
            for y,x in curve:
                shoulder=max(0,min(1,(y-.38)/.17));shoulder=shoulder*shoulder*(3-2*shoulder)
                yy=y+(deck-.555)*shoulder
                row.append((sx*x,yy,z))
            rings.append(row)
        o=sculpt_surface(name,rings,'clay',close=False)
        solid=o.modifiers.new('panel_return','SOLIDIFY');solid.thickness=.008
        bpy.context.view_layer.objects.active=o
        bpy.ops.object.modifier_apply(modifier=solid.name)
        base.cut_arches(o,cutters)
        for face in o.data.polygons:
            if abs(face.normal.x)<.35:face.use_smooth=False
        return o
    for sx,side in ((1,'left'),(-1,'right')):
        side_panel('front_door_'+side,sx,-.26,.82)
        side_panel('rear_door_'+side,sx,-1.16,-.266)
        # Evaluated skin intersections seat each flare flange against the actual tapered body.
        bpy.context.view_layer.update()
        skin_trees=[]
        for skin in (shell,bpy.data.objects['front_door_'+side],bpy.data.objects['rear_door_'+side]):
            evaluated=skin.evaluated_get(bpy.context.evaluated_depsgraph_get());mesh=evaluated.to_mesh()
            skin_trees.append(BVHTree.FromPolygons([v.co for v in mesh.vertices],
                              [tuple(f.vertices) for f in mesh.polygons]))
            evaluated.to_mesh_clear()
        def seated_flange(y,z):
            hits=[tree.ray_cast(Vector(B(sx*2,y,z)),Vector(B(-sx,0,0)))[0] for tree in skin_trees]
            return max([sx*p.x for p in hits if p is not None]+[.79])-.005
        # Swept fender shell: its section changes with height, rolling over a convex shoulder
        # and returning to the body. The lower ends tuck in rather than forming vertical slabs.
        for zc in (-1.4,1.4):
            path=rounded_path([(.505,-.065 if zc>0 else -.18),(.505,.005),(.335,.270),(-.335,.270),
                               (-.505,.005),(-.505,-.065 if zc<0 else -.18)],.050,10)
            rings=[];seam=[]
            for i,p in enumerate(path):
                tangent=(path[min(i+1,len(path)-1)]-path[max(i-1,0)]).normalized()
                normal=Vector((tangent.y,-tangent.x))
                z,y=p;t=max(0,min(1,(y+.23)/.50));t=t*t*(3-2*t)
                proud=.958+.080*t
                seat=seated_flange(y+normal.y*.098,zc+z+normal.x*.098)
                section=[(0,min(.845,seat)),(.004,proud-.035),(.013,proud-.010),(.030,proud),
                         (.054,proud-.005),(.074,proud-.030),(.091,max(seat+.012,proud-.10)),(.098,seat)]
                rings.append([(sx*x,y+normal.y*d,zc+z+normal.x*d) for d,x in section])
                seam.append((sx*(proud-.027),y+normal.y*.075,zc+z+normal.x*.075))
            flare=sculpt_surface('sculpted_flare_'+side+str(zc),rings,'trim')
            if zc<0:
                # The leading flare section belongs to the rear door. Cut both meshes,
                # leaving a real 5 mm shut gap through the entire molded cross-section.
                original=flare.data.copy()
                for part,plane,clear_outer in [('door',-1.1575,False),('quarter',-1.1625,True)]:
                    target=flare if part=='door' else bpy.data.objects.new('rear_flare_quarter_'+side,original.copy())
                    if part=='quarter':col.objects.link(target)
                    target.name='rear_flare_'+part+'_'+side
                    bm=bmesh.new();bm.from_mesh(original)
                    result=bmesh.ops.bisect_plane(bm,geom=list(bm.verts)+list(bm.edges)+list(bm.faces),
                        plane_co=B(0,0,plane),plane_no=B(0,0,1),dist=.000001,
                        clear_outer=clear_outer,clear_inner=not clear_outer)
                    edges=[e for e in result['geom_cut'] if isinstance(e,bmesh.types.BMEdge) and e.is_boundary]
                    if edges:
                        caps=bmesh.ops.holes_fill(bm,edges=edges,sides=0)
                        for f in caps['faces']:f.smooth=False
                    bm.to_mesh(target.data);bm.free();target.data.update()
                bpy.data.meshes.remove(original)
            for idx in (0,len(rings)-1):
                obj('flare_end_'+side+str(zc)+str(idx),rings[idx],[tuple(range(len(rings[idx])))],'trim',0)
            if zc>0:
                tube('flare_mold_seam_'+side+str(zc),seam,.0025,'edge')
            else:
                # Clip the decorative parting line to the same physical shut gap.
                for label,plane,keep_front in [('door',-1.1575,True),('quarter',-1.1625,False)]:
                    points=[]
                    for a,b in zip(seam,seam[1:]):
                        inside=lambda p:p[2]>=plane if keep_front else p[2]<=plane
                        if inside(a):points.append(a)
                        if inside(a)!=inside(b):
                            t=(plane-a[2])/(b[2]-a[2]);points.append(tuple(a[k]+t*(b[k]-a[k]) for k in range(3)))
                    if inside(seam[-1]):points.append(seam[-1])
                    if len(points)>1:tube('rear_flare_seam_'+side+label,points,.0025,'edge')
        # Shaped pull handle: attached feet, a raised grip and a shallow dark backing cup.
        for z in (-.10,-1.00):
            outline=closed_curve([(-1,-.35),(-.85,.7),(-.45,1),(.6,1),(1,.45),(.9,-.65),(.4,-1),(-.6,-1)],5)
            cup=[]
            for scale,x in ((1,.877),(.87,.880),(.30,.879)):
                cup.append([(sx*x,.405+v*.021*scale,z+u*.077*scale) for u,v in outline])
            sculpt_surface('handle_cup_'+side+str(z),cup,'trim',cap_end=(sx*.879,.405,z))
            grip=[]
            for dz,x,r in [(-.061,.878,.006),(-.052,.889,.009),(-.038,.897,.010),
                           (.035,.897,.010),(.050,.890,.009),(.060,.878,.006)]:
                grip.append([(sx*(x+math.cos(k*math.tau/16)*r*.65),.410+math.sin(k*math.tau/16)*r,z+dz) for k in range(16)])
            sculpt_surface('handle_grip_'+side+str(z),grip,'trim',
                           cap_start=(sx*.878,.410,z-.061),cap_end=(sx*.878,.410,z+.060))
        # Thin molded rocker cladding follows the lower door / body surface, not a stand-off slab.
        rings=[]
        upper=[(-.80,-.10),(-.65,-.025),(.48,-.025),(.62,-.10),(.65,-.155)]
        for i in range(65):
            z=-.80+1.45*i/64;top=upper[-1][1]
            for (za,ya),(zb,yb) in zip(upper,upper[1:]):
                if za<=z<=zb:
                    top=ya+(yb-ya)*(z-za)/(zb-za);break
            row=[]
            for j in range(13):
                t=j/12;y=-.17+(top+.17)*t
                hits=[tree.ray_cast(Vector(B(sx*2,y,z)),Vector(B(-sx,0,0)))[0] for tree in skin_trees]
                hits=[hit for hit in hits if hit is not None]
                if not hits:raise RuntimeError('Rocker cladding missed lower body')
                x=max(sx*hit.x for hit in hits)+.004+.004*math.sin(math.pi*t)
                row.append((sx*x,y,z))
            rings.append(row)
        panel=sculpt_surface('sculpted_rocker_'+side,rings,'trim',close=False)
        solid=panel.modifiers.new('cladding_return','SOLIDIFY');solid.thickness=.003
        bpy.context.view_layer.update()
        original=bpy.data.meshes.new_from_object(panel.evaluated_get(bpy.context.evaluated_depsgraph_get()))
        panel.modifiers.clear()
        for label,plane,clear_outer in [('front',-.2605,False),('rear',-.2655,True)]:
            target=panel if label=='front' else bpy.data.objects.new('rocker_rear_'+side,original.copy())
            if label=='front':target.data=original.copy()
            else:col.objects.link(target)
            target.name='rocker_'+label+'_'+side
            bm=bmesh.new();bm.from_mesh(original)
            result=bmesh.ops.bisect_plane(bm,geom=list(bm.verts)+list(bm.edges)+list(bm.faces),
                plane_co=B(0,0,plane),plane_no=B(0,0,1),dist=.000001,
                clear_outer=clear_outer,clear_inner=not clear_outer)
            edges=[e for e in result['geom_cut'] if isinstance(e,bmesh.types.BMEdge) and e.is_boundary]
            if edges:
                caps=bmesh.ops.holes_fill(bm,edges=edges,sides=0)
                for face in caps['faces']:face.smooth=False
            bm.to_mesh(target.data);bm.free();target.data.update()
        bpy.data.meshes.remove(original)
        tube('rock_slider_'+side,[(sx*.93,-.205,-.80),(sx*.975,-.205,-.67),
                                 (sx*.975,-.205,.49),(sx*.93,-.205,.63)],.023,'alloy')
        for z in (-.65,.47):
            tube('slider_bracket_'+side+str(z),[(sx*.66,-.18,z),(sx*.97,-.205,z)],.018,'trim')
            box('slider_end_'+side+str(z),(sx*.965,-.198,z),(.026,.022,.028),'accent',.005)
    # Hood skin: slight compound crown, raised center with gently falling shoulder, crisp perimeter reveal.
    hoodrings=[]
    for z,half,y,crown in [(.86,.77,.565,.010),(1.02,.775,.567,.022),
                         (1.70,.775,.55,.022),(2.015,.73,.52,.012)]:
        row=[(-half,y),(-half+.08,y+.006),(-.48,y+crown),(-.24,y+crown+.009),
             (0,y+crown+.012),(.24,y+crown+.009),(.48,y+crown),(half-.08,y+.006),(half,y)]
        hoodrings.append([(x,yy,z+(.12*(1-(x/half)**2) if z<1.05 else 0)) for x,yy in row]
                         +[(half,y-.015,z),(-half,y-.015,z)])
    loft('hood',hoodrings,bevel=.009)
    # Greenhouse from independent stations. Roof is broad and almost flat, rear wall nearly upright.
    # Each side window gets its own gasket and inset pane, leaving visible A/B/C pillars.
    def gx(y):return .837-(y-.555)/(.995-.555)*.085
    def side_surface(name,sx,profile,mat,offset,depth=.009,bevel=.007):
        vv=[(sx*(gx(y)+dx),y,z) for dx in (offset,offset-depth) for z,y in profile]
        n=len(profile);ff=[tuple(range(n)),tuple(reversed(range(n,2*n)))]
        ff += [(i,(i+1)%n,(i+1)%n+n,i+n) for i in range(n)]
        return obj(name,vv,ff,mat,bevel)
    cab=[(.84,.555),(.265,.982),(-1.88,.982),(-2.015,.555)]
    cabin=[]
    for y in (.555,.60,.85,.95,.982):
        t=(y-.555)/(.982-.555);w=gx(y);front=.84+(.265-.84)*t;back=-2.015+.135*t;r=.075
        ring=[(w,y,front),(w,y,back+r)]
        for k in range(1,13):
            a=k*math.pi/24;ring.append((w-r+r*math.cos(a),y,back+r-r*math.sin(a)))
        ring += [(0,y,back),(-w+r,y,back)]
        for k in range(1,13):
            a=k*math.pi/24;ring.append((-w+r-r*math.sin(a),y,back+r-r*math.cos(a)))
        ring.append((-w,y,front));cabin.append(ring)
    sculpt_surface('cabin_frame',cabin,'trim')
    for sx,side in ((1,'left'),(-1,'right')):
        windows=[('front',[(.735,.588),(.237,.944),(-.225,.944),(-.225,.588)]),
                 ('rear',[(-.29,.588),(-.29,.944),(-.88,.944),(-.86,.588)]),
                 ('rear_fixed',[(-.884,.588),(-.905,.944),(-1.025,.944),(-1.16,.588)])]
        for name,profile in windows:
            # Rounded aperture perimeter, thin rubber lip and shallow compound glass crown.
            contour=[]
            for i,p in enumerate(profile):
                p=Vector(p);before=Vector(profile[(i-1)%len(profile)]);after=Vector(profile[(i+1)%len(profile)])
                reach=.010 if name=='rear_fixed' else .018
                a=p+(before-p).normalized()*reach;b=p+(after-p).normalized()*reach
                for j in range(7):
                    t=j/6;contour.append(tuple((1-t)**2*a+2*t*(1-t)*p+t*t*b))
            cz=sum(p[0] for p in profile)/len(profile);cy=sum(p[1] for p in profile)/len(profile)
            def pane_ring(scale,offset):
                return [(sx*(gx(cy+(y-cy)*scale)+offset),cy+(y-cy)*scale,cz+(z-cz)*scale) for z,y in contour]
            sculpt_surface('gasket_'+name+'_'+side,
                           [pane_ring(1.02,.003),pane_ring(1,.008),pane_ring(.984,.010),pane_ring(.975,.008)],'trim')
            glass=[pane_ring(scale,.009+.010*(1-scale*scale)) for scale in (.976,.85,.65,.4,.15)]
            pane=sculpt_surface('glass_'+name+'_'+side,glass,'glass',cap_end=(sx*(gx(cy)+.019),cy,cz))
            hy=max(abs(y-cy) for z,y in profile);hz=max(abs(z-cz) for z,y in profile)
            normals=[]
            for vertex in pane.data.vertices:
                yy=vertex.co.z;zz=-vertex.co.y
                dy=-.085/.44-.020*(yy-cy)/(hy*hy)
                dz=-.020*(zz-cz)/(hz*hz)
                normals.append(tuple(Vector(B(sx,-dy,-dz)).normalized()))
            pane.data.normals_split_custom_set_from_vertices(normals)
        # Formed quarter skin follows the cabin's round rear corner and has a shallow crown.
        bpy.context.view_layer.update()
        evaluated=bpy.data.objects['cabin_frame'].evaluated_get(bpy.context.evaluated_depsgraph_get())
        mesh=evaluated.to_mesh()
        tree=BVHTree.FromPolygons([v.co for v in mesh.vertices],[tuple(f.vertices) for f in mesh.polygons])
        def quarter_point(u,t):
            yy=.579+.376*t
            front=-1.215+.110*t;back=-1.965+.118*t
            end=max(0,1-min(t,1-t)/.045)
            inset=.008*end*end
            zz=(front-inset)*(1-u)+(back+inset)*u
            hit,normal,_,_=tree.ray_cast(Vector(B(sx*2,yy,zz)),Vector(B(-sx,0,0)))
            if hit is None:raise RuntimeError('Quarter panel missed curved cabin')
            crown=.018*math.sin(math.pi*u)**2*math.sin(math.pi*t)**2
            q=hit+normal*(.003+crown);return (q.x,q.z,-q.y)
        rows=[[quarter_point(i/48,j/24) for i in range(49)] for j in range(25)]
        panel=sculpt_surface('rear_quarter_'+side,rows,'accent',close=False)
        solid=panel.modifiers.new('quarter_return','SOLIDIFY');solid.thickness=.003
        perimeter=[quarter_point(i/48,0) for i in range(49)]
        perimeter += [quarter_point(1,j/24) for j in range(1,25)]
        perimeter += [quarter_point(1-i/48,1) for i in range(1,49)]
        perimeter += [quarter_point(0,1-j/24) for j in range(1,25)]
        tube('quarter_panel_gasket_'+side,perimeter,.0015,'trim')
        evaluated.to_mesh_clear()
        tube('rear_door_c_pillar_seam_'+side,
             [(sx*(gx(y)+.006),y,z) for z,y in [(-1.061,.974),(-1.19,.588),(-1.19,.552)]],.002,'edge')
    for sx in (-1,1):
        tube('b_pillar_reveal'+str(sx),[(sx*(gx(y)+.013),y,-.258) for y in (.585,.94)],.003,'alloy')

    # Windshield: 32 cross-car segments and 10 vertical segments. Its center is 110 mm
    # forward of the corners; both header and cowl follow the same compound-curved surface.
    def shield(u,t,offset=0):
        end=max(0,1-min(t,1-t)/.075)
        width=.791*(1-t)+.706*t-.020*end*end
        y=.580*(1-t)+.950*t+.016*(1-u*u)
        z=.812*(1-t)+.311*t
        bow=(1-u*u)*(.110+.012*math.sin(math.pi*t))
        return (width*u,y+.008*(1-u*u)*math.sin(math.pi*t)+offset*.81,z+bow+offset*.59)
    nu,nv=32,10
    verts=[shield(-1+2*i/nu,j/nv,.004) for j in range(nv+1) for i in range(nu+1)]
    faces=[(j*(nu+1)+i,j*(nu+1)+i+1,(j+1)*(nu+1)+i+1,(j+1)*(nu+1)+i)
           for j in range(nv) for i in range(nu)]
    obj('curved_windshield',verts,faces,'glass',0)
    # One continuous U-shaped seal avoids overlapping capped tubes at the lower corners.
    perimeter=[shield(-1,j/16) for j in range(16,-1,-1)]
    perimeter += [shield(-1+2*i/32,0) for i in range(1,33)]
    perimeter += [shield(1,j/16) for j in range(1,17)]
    tube('windshield_continuous_seal',perimeter,.010,'trim')
    tube('windshield_header_seal',[shield(-1+2*i/32,1,.002) for i in range(33)],.012,'trim')
    tube('windshield_header_frame',[shield(-1+2*i/64,1,-.010) for i in range(65)],.006,'trim')
    for u in (-1,1):
        tube('a_pillar_accent_'+str(u),[shield(u*1.025,.04+.96*j/16,-.005) for j in range(17)],.015,'accent')
    # Continuous cowl skin rolls down into the shoulder at both A-pillar feet.
    for t,name,zedge,yedge in ((0,'curved_cowl',.895,.565),(1,'curved_header',.235,1.006)):
        rows=[]
        for i in range(65):
            u=-1+2*i/64;x,y,z=shield(u,t,-.012)
            blend=max(0,(abs(u)-.88)/.12);blend=blend*blend*(3-2*blend)
            edge=(x,yedge,zedge) if t else (x*.963+(1 if u>=0 else -1)*.061*blend,
                                            yedge-.023*blend,zedge-.075*blend)
            rows.append([tuple((1-v/8)*a+(v/8)*b for a,b in zip((x,y,z),edge)) for v in range(9)])
        panel=sculpt_surface(name,rows,'clay' if t==0 else 'accent',close=False)
        if t==0:
            solid=panel.modifiers.new('cowl_return','SOLIDIFY');solid.thickness=.012
    # Wiper blades sit against the curved glass, with separate short drive arms at the cowl.
    for ua,ub in ((-.82,-.12),(.06,.73)):
        points=[shield(ua+(ub-ua)*i/16,.065+.035*math.sin(math.pi*i/16),.014) for i in range(17)]
        tube('wiper_blade_'+str(ua),points,.007,'trim')
        tube('wiper_arm_'+str(ua),[shield((ua+ub)*.5,.02,.025),shield(ub,.065,.022)],.005,'trim')
    def backlight(u,t):
        end=max(0,1-min(t,1-t)/.08)
        w=.765*(1-t)+.691*t-.018*end*end
        return (w*u,.595+.344*t+.009*(1-u*u),-2.020+.112*t-.006*(1-u*u))
    rearrows=[[backlight(-1+2*i/48,j/16) for i in range(49)] for j in range(17)]
    sculpt_surface('rear_glass',rearrows,'glass',close=False)
    perimeter=[backlight(-1+2*i/48,0) for i in range(49)]
    perimeter += [backlight(1,j/16) for j in range(1,17)]
    perimeter += [backlight(1-2*i/48,1) for i in range(1,49)]
    perimeter += [backlight(-1,1-j/16) for j in range(1,17)]
    tube('rear_glass_seal',perimeter,.004,'trim')
    roofrings=[]
    cross=[(math.copysign(abs(math.cos(i*math.tau/64))**.5,math.cos(i*math.tau/64)),
            math.copysign(abs(math.sin(i*math.tau/64))**.5,math.sin(i*math.tau/64))) for i in range(64)]
    for z,w,y in [(.305,.680,.985),(.285,.720,.990),(.245,.755,.994),(.18,.766,.995),
                  (-.3,.766,.998),(-1.65,.766,1.000),(-1.79,.751,.999),(-1.86,.715,.993),(-1.89,.670,.986)]:
        roofrings.append([(u*w,y+v*.028,z) for u,v in cross])
    sculpt_surface('roof',roofrings,'roof',cap_start=(0,.985,.305),cap_end=(0,.986,-1.89))

    # Hood shoulder stampings taper into the nose and the curved cowl; side accents sit flush.
    for sx in (-1,1):
        rings=[]
        for z,y,h in ((1.005,.573,.012),(1.16,.571,.028),(1.73,.555,.025),(2.02,.524,.008)):
            rings.append([(sx*.43,y-.006,z),(sx*.48,y+h,z),(sx*.57,y+h+.006,z),
                          (sx*.64,y+.004,z),(sx*.64,y-.01,z)])
        loft('hood_ridge_'+str(sx),rings,'clay',.014)
        hood=bpy.data.objects['hood']
        bpy.context.view_layer.update()
        evaluated=hood.evaluated_get(bpy.context.evaluated_depsgraph_get());mesh=evaluated.to_mesh()
        tree=BVHTree.FromPolygons([v.co for v in mesh.vertices],[tuple(f.vertices) for f in mesh.polygons])
        paint=[]
        for i in range(65):
            t=i/64;z=1.015+.90*t;outside=.747-.028*t
            chamfer=.009*max(0,1-min(t,1-t)/.025)
            row=[]
            for j in range(9):
                x=sx*(.665+chamfer+(outside-.665-2*chamfer)*j/8)
                hit,normal,_,_=tree.ray_cast(Vector(B(x,2,z)),Vector(B(0,-1,0)))
                if hit is None:raise RuntimeError('Hood accent left panel')
                q=hit+normal*.0012;row.append((q.x,q.z,-q.y))
            paint.append(row)
        sculpt_surface('hood_accent_'+str(sx),paint,'accent',close=False)
        evaluated.to_mesh_clear()
        # Flush vents use the same evaluated hood surface as the adjacent paint.
        def vent_point(u,v,offset):
            xx=sx*(.34+.070*u);zz=1.035+.048*v
            hit,normal,_,_=tree.ray_cast(Vector(B(xx,2,zz)),Vector(B(0,-1,0)))
            if hit is None:raise RuntimeError('Hood vent missed skin')
            q=hit+normal*offset;return (q.x,q.z,-q.y)
        rows=[[vent_point(-1+2*i/16,-1+2*j/12,.0008) for i in range(17)] for j in range(13)]
        sculpt_surface('hood_vent_recess_'+str(sx),rows,'trim',close=False)
        for j in range(5):
            v=-.72+j*.36
            tube('hood_vent_louver_'+str(sx)+str(j),[vent_point(-.91+1.82*i/16,v,.0026) for i in range(17)],.0025,'armour')
        # Seat the housing and lens on the evaluated tapered fender, not a fixed X plane.
        evaluated=shell.evaluated_get(bpy.context.evaluated_depsgraph_get());mesh=evaluated.to_mesh()
        marker_tree=BVHTree.FromPolygons([v.co for v in mesh.vertices],[tuple(f.vertices) for f in mesh.polygons])
        contour=closed_curve([(-1,-.5),(-.85,.85),(-.4,1),(.75,1),(1,.5),(.85,-.85),(.4,-1),(-.75,-1)],5)
        def marker_point(u,v,offset):
            hit,normal,_,_=marker_tree.ray_cast(Vector(B(sx*2,.430+.019*v,1.96+.035*u)),Vector(B(-sx,0,0)))
            if hit is None:raise RuntimeError('Side marker missed fender')
            q=hit+normal*offset;return (q.x,q.z,-q.y)
        sculpt_surface('side_marker_seal_'+str(sx),
            [[marker_point(u*scale,v*scale,depth) for u,v in contour] for scale,depth in ((1.08,-.001),(1.08,.001),(1,.002))],
            'trim')
        sculpt_surface('side_marker_lens_'+str(sx),
            [[marker_point(u*scale,v*scale,depth) for u,v in contour] for scale,depth in ((1,.0015),(.94,.004),(.6,.0055),(.2,.006))],
            'amber',cap_end=marker_point(0,0,.006))
        evaluated.to_mesh_clear()

    # Recessed nose: deep lamp buckets, separate black bezel, light guides and projector lenses.
    box('front_grille_surround',(0,.295,2.125),(.458,.130,.025),'trim',.017)
    box('front_grille_well',(0,.295,2.151),(.424,.100,.008),'lens',.009)
    for row in range(5):
        for column in range(20):
            x=-.395+column*.041+(row%2)*.0205
            if x>.405:continue
            y=.215+row*.040
            pts=[(x+.019*math.cos(math.pi*i/3),y+.014*math.sin(math.pi*i/3),2.163) for i in range(7)]
            tube('grille_cell_'+str(row)+'_'+str(column),pts,.0025,'trim')
    box('grille_lower_rail',(0,.185,2.17),(.438,.012,.014),'trim',.008)
    for sx in (-1,1):
        x=sx*.64
        outline=closed_curve([(-1,-.58),(-1,.66),(-.79,.97),(.66,.98),(.98,.65),
                              (.96,-.52),(.69,-.93),(-.70,-.95)],5)
        def lamp_point(u,v,d):return (x+sx*u*.159,.315+v*.113,2.145+d-.012*u)
        surround=[]
        for scale,depth in [(1.01,-.040),(1.035,-.020),(1.00,.002),(.92,.010),(.85,-.004)]:
            surround.append([lamp_point(u*scale,v*scale,depth) for u,v in outline])
        sculpt_surface('headlamp_molded_bezel'+str(sx),surround,'trim')
        lens=[]
        for scale in (.85,.63,.38,.12,.01):
            lens.append([lamp_point(u*scale,v*scale,-.004+.004*(1-scale*scale)) for u,v in outline])
        sculpt_surface('headlamp_curved_lens'+str(sx),lens,'lens',cap_end=lamp_point(0,0,0))
        for u,v in ((-.88,.68),(.81,-.65)):
            a=lamp_point(u,v,.006);tube('lamp_fastener'+str(sx)+str(u),[a,(a[0],a[1],a[2]+.002)],.003,'alloy')
        pts=[(x+sx*.105,.374,2.153),(x-sx*.098,.365,2.157),(x-sx*.112,.345,2.157),
             (x-sx*.112,.274,2.157),(x-sx*.092,.257,2.157),(x+sx*.104,.267,2.153)]
        tube('drl'+str(sx),pts,.008,'led')
        box('projector_recess'+str(sx),(x+sx*.008,.309,2.148),(.054,.027,.008),'trim',.018)
        box('projector_lens'+str(sx),(x+sx*.008,.309,2.160),(.035,.015,.008),'led',.012)
    box('nose_chrome_line',(0,.434,2.147),(.801,.007,.012),'alloy',.005)
    box('hood_badge',(0,.507,2.060),(.022,.010,.008),'trim',.005)

    # Front bumper is a center winch cradle plus swept, raised wings, not one full-width bar.
    box('winch_cradle',(0,.005,2.205),(.35,.110,.105),'trim',.018)
    for sx in (-1,1):
        rings=[]
        for x,z,y,h in ((.34,2.24,.035,.12),(.38,2.242,.032,.121),(.52,2.22,.020,.12),
                        (.68,2.18,.030,.112),(.80,2.13,.040,.10),(.93,2.035,.061,.085),(.99,1.98,.070,.075)):
            rings.append([(sx*x,y-h+.015,z-.095),(sx*x,y-h,z-.02),
                          (sx*x,y-h+.018,z+.040),(sx*x,y-h+.045,z+.063),
                          (sx*x,y+h-.045,z+.069),(sx*x,y+h-.018,z+.053),
                          (sx*x,y+h,z+.018),(sx*x,y+h+.008,z-.060),(sx*x,y+h-.018,z-.095)])
        sculpt_surface('front_bumper_formed_wing'+str(sx),rings,'trim')
        for j in (0,len(rings)-1):obj('bumper_end'+str(sx)+str(j),rings[j],[tuple(range(len(rings[j])))],'trim',0)
        tube('bumper_fold'+str(sx),[(sx*x,y+h-.019,z+.054) for x,z,y,h in
             ((.39,2.24,.03,.12),(.52,2.22,.02,.12),(.68,2.18,.03,.112),(.8,2.13,.04,.1),(.93,2.035,.061,.085))],.0025,'edge')
        box('fog_bucket'+str(sx),(sx*.645,.040,2.247),(.108,.039,.013),'lens',.010)
        box('fog_lamp'+str(sx),(sx*.645,.044,2.262),(.065,.010,.004),'amber',.004)
        box('fog_end'+str(sx),(sx*.745,.040,2.263),(.009,.027,.005),'amber',.004)
        tube('bumper_upright'+str(sx),[(sx*.435,-.04,2.32),(sx*.435,.184,2.29),(sx*.40,.217,2.27)],.024,'trim')
        x=sx*.37
        box('recovery_lug'+str(sx),(x,-.082,2.326),(.022,.046,.026),'alloy',.008)
        # Closed D-shackle, with a separate cross pin.
        pts=[(x+.034*math.cos(math.pi+i*math.pi/24),-.114+.045*math.sin(math.pi+i*math.pi/24),2.366) for i in range(25)]
        tube('recovery_bow'+str(sx),[(x-.034,-.073,2.366)]+pts+[(x+.034,-.073,2.366)],.010,'hooks')
        tube('recovery_pin'+str(sx),[(x-.043,-.073,2.366),(x+.043,-.073,2.366)],.009,'alloy')
    tube('guard_top',[(-.40,.217,2.27),(.40,.217,2.27)],.020,'trim')
    tube('winch_drum',[(-.14,.055,2.30),(.14,.055,2.30)],.046,'alloy')
    box('fairlead',(0,.055,2.357),(.141,.041,.014),'alloy',.012)
    box('fairlead_slot',(0,.055,2.375),(.105,.014,.005),'lens',.009)
    for sx in (-1,1):box('fairlead_cheek'+str(sx),(sx*.131,.055,2.375),(.014,.050,.011),'hooks',.006)
    # Formed skid plate: tapered corners and a depressed center with recessed cooling slots.
    rings=[]
    for y,z,w in ((-.115,2.28,.43),(-.22,2.34,.42),(-.32,2.23,.34),(-.35,2.10,.31)):
        rings.append([(-w,y,z),(-.25,y,z+.02),(0,y-.012,z+.023),(.25,y,z+.02),(w,y,z),
                      (w,y+.018,z-.015),(-w,y+.018,z-.015)])
    loft('front_skid',rings,'alloy',.014)
    for sx in (-1,1):
        box('skid_slot'+str(sx),(sx*.28,-.215,2.363),(.055,.015,.004),'trim',.009)
        box('skid_bolt'+str(sx),(sx*.35,-.16,2.323),(.009,.009,.006),'trim',.004)
    box('skid_center_recess',(0,-.275,2.310),(.14,.020,.006),'trim',.008)
    # Rear bumper: lowered center step, swept armored corners and recessed lamp cartridges.
    rings=[]
    for x,top,z in [(-.49,-.055,-2.24),(-.40,-.075,-2.28),(0,-.075,-2.29),(.40,-.075,-2.28),(.49,-.055,-2.24)]:
        rings.append([(x,-.205,z+.11),(x,-.225,z+.04),(x,-.211,z-.018),
                      (x,top-.025,z-.025),(x,top,z),(x,top+.006,z+.10)])
    loft('rear_center_step',rings,'trim',.008)
    for i in range(17):
        x=-.38+i*.0475
        box('rear_step_tread_'+str(i),(x,-.068,-2.235),(.012,.004,.045),'edge',.002)
    for sx in (-1,1):
        rings=[]
        for x,back,top,bottom in [(.435,-2.265,.025,-.217),(.50,-2.278,.073,-.215),
                                  (.77,-2.245,.075,-.197),(.88,-2.175,.058,-.162),(.99,-2.015,.015,-.11)]:
            rings.append([(sx*x,bottom,back+.10),(sx*x,bottom+.012,back+.02),
                          (sx*x,bottom+.032,back),(sx*x,top-.020,back),
                          (sx*x,top,back+.025),(sx*x,top+.005,back+.12),
                          (sx*x,top-.035,back+.16)])
        wing=loft('rear_bumper_corner_'+str(sx),rings,'trim',.010)
        cut=base.mesh_object('CUT_rear_lamp',base.box((sx*.685,-.031,-2.275),(.133,.052,.085)),helpers,None)
        cut.hide_render=True;cut.display_type='WIRE';base.bevel(cut,.012,4)
        base.cut_arches(wing,[cut])
        box('rear_lamp_backing_'+str(sx),(sx*.685,-.031,-2.195),(.127,.047,.005),'lens',.010)
        contour=closed_curve([(-1,-.6),(-.88,.85),(.88,.85),(1,-.6),(.7,-1),(-.7,-1)],6)
        sculpt_surface('rear_lamp_red_rim_'+str(sx),
            [[(sx*(.685+u*.116*scale),-.031+v*.041*scale,z) for u,v in contour]
             for scale,z in [(1,-2.207),(.93,-2.218),(.70,-2.219)]],'hooks')
        box('rear_reverse_lens_'+str(sx),(sx*.685,-.025,-2.222),(.079,.020,.006),'led',.007)
        for dx in (-.065,-.039,-.013,.013,.039,.065):
            tube('rear_reverse_flute_'+str(sx)+str(dx),[(sx*.685+dx,-.039,-2.229),(sx*.685+dx,-.011,-2.229)],.001,'alloy')
        # Separate bolted recovery tab and rounded D-ring below each corner.
        tab=loft('rear_recovery_tab_'+str(sx),[
            [(sx*(.49+dx),y,z) for dx,y in [(-.027,-.09),(.027,-.09),(.027,-.21),(-.027,-.21)]]
            for z in (-2.22,-2.31)],'alloy',.006)
        ring=closed_curve([(-.032,.018),(-.035,-.025),(-.018,-.045),(.018,-.045),(.035,-.025),(.032,.018)],5)
        tube('rear_shackle_'+str(sx),[(sx*.49+dx,-.205+dy,-2.326) for dx,dy in ring]+[(sx*.49+ring[0][0],-.205+ring[0][1],-2.326)],.009,'trim')
        tube('rear_shackle_pin_'+str(sx),[(sx*.49-.043,-.187,-2.326),(sx*.49+.043,-.187,-2.326)],.009,'alloy')
        bpy.context.view_layer.update()
        evaluated=wing.evaluated_get(bpy.context.evaluated_depsgraph_get());mesh=evaluated.to_mesh()
        tree=BVHTree.FromPolygons([v.co for v in mesh.vertices],[tuple(f.vertices) for f in mesh.polygons])
        for x in (.53,.84):
            hit,normal,_,_=tree.ray_cast(Vector(B(sx*x,.035,-3)),Vector(B(0,0,1)))
            if hit is None:raise RuntimeError('Rear bumper fastener missed shell')
            points=[]
            for offset in (-.001,.003):
                q=hit+normal*offset;points.append((q.x,q.z,-q.y))
            tube('rear_bumper_fastener_'+str(sx)+str(x),points,.007,'alloy')
        evaluated.to_mesh_clear()
    box('hitch_crossmember',(0,-.247,-2.12),(.36,.029,.045),'trim',.006)
    # Hollow receiver, with four walls and a visible dark bore.
    for dx,dy,hx,hy in [(0,.043,.052,.008),(0,-.043,.052,.008),(.044,0,.008,.035),(-.044,0,.008,.035)]:
        box('hitch_wall_'+str(dx)+str(dy),(dx,-.265+dy,-2.235),(hx,hy,.095),'alloy',.003)
    box('hitch_bore',(0,-.265,-2.145),(.035,.034,.002),'trim',.001)
    # Rear tailgate skin and lamp pockets give the rear the same surface hierarchy as the front.
    # Regular quads preserve the shallow crown through the rear-envelope deformation.
    # The old beveled box intersected the shell and stretched its cap triangulation.
    verts=[];faces=[]
    for j in range(33):
        v=j/32; y=.045+.508*v
        corner=max(0,abs(y-.299)-.230)
        halfwidth=.666+math.sqrt(max(0,.024**2-corner**2))
        for i in range(65):
            u=i/64; x=(2*u-1)*halfwidth
            crown=.002*math.sin(math.pi*u)*math.sin(math.pi*v)
            verts.append((x,y,-2.159-crown))
            if i<64 and j<32:
                k=j*65+i;faces.append((k,k+65,k+66,k+1))
    gate=obj('tailgate',verts,faces,'clay',0)
    gate.modifiers.clear()
    skin=gate.modifiers.new('inward_return','SOLIDIFY');skin.thickness=.004;skin.offset=-1

    for sx in (-1,1):
        # Exact rounded rectangle: tangent arcs avoid Catmull-Rom overshoot.
        outline=[]
        for cx,cy,start in ((.76,.88,0),(-.76,.88,90),(-.76,-.88,180),(.76,-.88,270)):
            for i in range(13):
                a=math.radians(start+i*90/12)
                outline.append((cx+.24*math.cos(a),cy+.12*math.sin(a)))
        def tailring(scale,z):return [(sx*(.759+u*.063*scale),.31+v*.136*scale,z+.012*u) for u,v in outline]
        sculpt_surface('tail_housing_'+str(sx),[tailring(scale,z) for scale,z in [(1,-2.130),(1,-2.151),(.90,-2.159),(.80,-2.153)]],'trim')
        sculpt_surface('tail_optical_cup_'+str(sx),[tailring(scale,z) for scale,z in [(.81,-2.153),(.60,-2.157),(.2,-2.160)]],'lens',cap_end=(sx*.759,.31,-2.160))
        guide=rounded_path([(sx*.782,.395,-2.171),(sx*.728,.395,-2.178),(sx*.728,.269,-2.178),(sx*.782,.269,-2.171)],.013,8)
        tube('tail_red_lightguide_'+str(sx),[tuple(v) for v in guide],.009,'hooks')
        for j in range(5):
            box('tail_reflector_cell_'+str(sx)+str(j),(sx*.766,.293+j*.018,-2.169),(.016,.005,.003),'hooks',.003)
        box('tail_reverse_lens_'+str(sx),(sx*.757,.220,-2.166),(.034,.013,.005),'led',.005)
        for dx in (-.024,-.012,0,.012,.024):
            tube('tail_reverse_prism_'+str(sx)+str(dx),[(sx*.757+dx,.210,-2.172),(sx*.757+dx,.230,-2.172)],.001,'alloy')
    for sx in (-1,1):mirror(sx)

    # Low expedition rack follows the roof outline, with slatted deck, feet and two auxiliary pods.
    for sx in (-1,1):
        for z in (.05,-.85,-1.72):
            box('rack_foot'+str(sx)+str(z),(sx*.68,1.049,z),(.038,.020,.050),'trim',.010)
        tube('rack_side'+str(sx),[(sx*.68,1.075,.12),(sx*.68,1.075,-1.82)],.021,'trim')
        tube('rack_upper'+str(sx),[(sx*.68,1.13,.04),(sx*.68,1.13,-1.76)],.013,'trim')
        for z in (.03,-.85,-1.74):tube('rack_post'+str(sx)+str(z),[(sx*.68,1.075,z),(sx*.68,1.13,z)],.010,'trim')
        outline=closed_curve([(-1,-.6),(-.97,.53),(-.62,.94),(.63,.94),(.97,.53),(1,-.6),(.60,-.85),(-.60,-.85)],4)
        pod=[]
        for scale,z,y in [(.65,.015,1.132),(.92,.035,1.135),(1,.085,1.138),(.98,.108,1.138),(.83,.113,1.138)]:
            pod.append([(sx*.62+u*.067*scale,y+v*.045*scale,z) for u,v in outline])
        sculpt_surface('roof_pod_shell'+str(sx),pod,'trim')
        obj('roof_pod_lens'+str(sx),[(sx*.62+u*.055,1.138+v*.036,.111) for u,v in outline],
            [tuple(range(len(outline)))],'lens',0)
        for dx in (-.028,0,.028):
            tube('roof_pod_projector'+str(sx)+str(dx),[(sx*.62+dx,1.138,.112),(sx*.62+dx,1.138,.116)],.010,'led')
        for z in (.025,.040,.055):
            tube('roof_pod_cooling_fin'+str(sx)+str(z),[(sx*.62-.04,1.159,z),(sx*.62+.04,1.159,z)],.003,'trim')
    for i in range(11):
        z=.04-i*.172
        box('rack_slat'+str(i),(0,1.085,z),(.67,.009,.017),'trim',.004)
    for z in (.06,-1.77):tube('rack_cross'+str(z),[(-.68,1.13,z),(.68,1.13,z)],.013,'trim')
    for sx in (-1,1):
        box('rack_rear_case'+str(sx),(sx*.32,1.15,-1.60),(.22,.055,.105),'clay',.015)
        tube('case_strap'+str(sx),[(sx*.32,1.09,-1.72),(sx*.32,1.21,-1.70),
                                 (sx*.32,1.21,-1.49),(sx*.32,1.09,-1.48)],.009,'trim')
        tube('roof_equipment'+str(sx),[(sx*.12,1.12,-1.80),(sx*.55,1.12,-1.80)],.023,'blue')
    # Tailgate canisters have hollow handles and pressed diagonal ribs, not featureless boxes.
    for sx,mat in ((1,'accent'),(-1,'hooks')):
        x=sx*.57;y=.39;z=-2.285
        box('jerry_can'+str(sx),(x,y,z),(.10,.22,.065),mat,.025)
        tube('can_handle'+str(sx),[tuple(v) for v in rounded_path([(x-.06,y+.20,z),(x-.06,y+.275,z),(x+.06,y+.275,z),(x+.06,y+.20,z)],.025,10)],.012,mat)
        for d in (-1,1):
            tube('can_rib'+str(sx)+str(d),[(x-.075,y-d*.15,z-.066),(x,y,z-.075),(x+.075,y+d*.15,z-.066)],.006,mat)
        for dy in (-.14,.14):box('can_retainer'+str(sx)+str(dy),(x,y+dy,z-.073),(.105,.011,.007),'trim',.005)

    for part in col.objects:
        if part.name.startswith(('tail_housing','tail_optical','tail_red_lightguide','tail_reflector','tail_reverse')):
            if part.type=='MESH':
                for vertex in part.data.vertices:vertex.co.y+=.040
            elif part.type=='CURVE':
                for spline in part.data.splines:
                    for point in spline.points:point.co.y+=.040
        if any(token in part.name for token in ('bumper','rear_center_step','rear_recovery','hitch_','rack_','front_skid')):
            for slot in part.material_slots:
                if slot.material==mats['trim']:slot.material=mats['armour']

    # The rear envelope bows in plan: center aft, corners swept forward into the side skins.
    # Apply one continuous mapping to the skin, glazing and bumper so their junctions agree.
    def rear_bow(x,z):
        t=max(0,min(1,(-z-1.60)/.50));t=t*t*(3-2*t)
        return t*(-.10+.17*(x/.85)**2)
    bpy.context.view_layer.update()
    for part in list(col.objects):
        if part.name.startswith(('rack_','roof_equipment','case_strap')):continue
        if part.type=='CURVE':
            for spline in part.data.splines:
                for point in spline.points:
                    x,y,z,w=point.co
                    point.co.y-=rear_bow(math.copysign(.57,x),-2.20) if part.name.startswith('can_') else rear_bow(x,-y)
            continue
        if part.type!='MESH' or not any(v.co.y>1.60 for v in part.data.vertices):continue
        # Bake bevels before shaping. Long rear-face spans receive actual intermediate vertices.
        evaluated=part.evaluated_get(bpy.context.evaluated_depsgraph_get())
        mesh=bpy.data.meshes.new_from_object(evaluated)
        part.modifiers.clear();part.data=mesh
        if part.name.startswith(('body_shell','cabin_frame','roof','rear_glass','rear_center_step','rear_bumper_corner')):
            bm=bmesh.new();bm.from_mesh(mesh)
            faces=[f for f in bm.faces if any(v.co.y>1.60 for v in f.verts)
                   and max(v.co.x for v in f.verts)-min(v.co.x for v in f.verts)>.10]
            bmesh.ops.triangulate(bm,faces=faces)
            edges=[e for e in bm.edges if any(v.co.y>1.60 for v in e.verts) and e.calc_length()>.10]
            bmesh.ops.subdivide_edges(bm,edges=edges,cuts=7,use_grid_fill=True)
            bm.to_mesh(mesh);bm.free()
        if part.name.startswith(('jerry_can','can_')):
            # Equipment remains rigid; only its mounting position follows the curved tailgate.
            center=sum((v.co for v in mesh.vertices),Vector())/len(mesh.vertices)
            delta=rear_bow(math.copysign(.57,center.x),-2.20)
            for vertex in mesh.vertices:vertex.co.y-=delta
        else:
            for vertex in mesh.vertices:vertex.co.y-=rear_bow(vertex.co.x,-vertex.co.y)
        mesh.update()
        # Evaluated meshes retain custom normals from the pre-bend weighted modifier.
        # Discard those directions so shading follows the deformed surface.
        if mesh.has_custom_normals:
            mesh.normals_split_custom_set([(0,0,0)]*len(mesh.loops))
        if part.name in ('body_shell','rear_glass','roof','cabin_frame','tailgate') or part.name.startswith(('rear_quarter_','tail_housing','tail_optical')):
            for face in mesh.polygons:face.use_smooth=True
        else:finish_normals(part)

    # Mounts bridge the curved door to each rigid canister; straps wrap around its sides.
    for sx in (-1,1):
        x=sx*.57
        shift=rear_bow(x,-2.285)
        for dy in (-.14,.14):
            for dx in (-.075,.075):
                xx=x+dx
                door_z=-2.159+rear_bow(xx,-2.159)
                can_z=-2.220+shift
                box('can_mount_'+str((sx,dy,dx)),(xx,.39+dy,(door_z+can_z)/2),
                    (.016,.017,abs(door_z-can_z)/2+.003),'armour',.003)
            for dx in (-.104,.104):
                tube('can_strap_return_'+str((sx,dy,dx)),
                     [(x+dx,.39+dy,-2.358+shift),(x+dx,.39+dy,-2.216+shift)],.008,'trim')

    # Bring in the actual game wheels and undercarriage, preserving physical sizes and settled transforms.
    for o in list(col.objects):
        o.location.z -= .05
    for o in list(helpers.objects):
        o.location.z -= .05
    references=base.collection('ENGINE_REFERENCE_DO_NOT_EXPORT')
    references.hide_render=True
    bpy.ops.wm.obj_import(filepath='/tmp/trailforge_suv_reference.obj',forward_axis='NEGATIVE_Z',up_axis='Y')
    for o in list(bpy.context.selected_objects):
        if not (o.name == 'chassis' or o.name.startswith(('wheel_', 'arm_', 'half_shaft_', 'shock_', 'spring_'))):
            bpy.data.objects.remove(o,do_unlink=True);continue
        # Imported engine geometry is a read-only fit reference, not authored body content.
        for collection in list(o.users_collection):collection.objects.unlink(o)
        references.objects.link(o)
        o.name='REF_'+o.name

    # Actual game spare tires: rotate copies of the rear road wheel; never scale their dimensions.
    wheel=bpy.data.objects['REF_wheel_2']
    worldverts=[wheel.matrix_world @ v.co for v in wheel.data.vertices]
    center=Vector(tuple((min(p[i] for p in worldverts)+max(p[i] for p in worldverts))*.5 for i in range(3)))
    for name,target,normal in [('roof',(0,1.235,-.78),(0,0,1)),('rear',(0,.40,-2.40),(0,1,0))]:
        rotation=Vector((1,0,0)).rotation_difference(Vector(normal))
        mesh=wheel.data.copy()
        for vertex,world in zip(mesh.vertices,worldverts):vertex.co=rotation @ (world-center)
        o=bpy.data.objects.new('REF_spare_'+name,mesh);references.objects.link(o)
        o.location=B(*target)
    # Central hold-down sits on the hub, with its support fixed to the rack deck.
    tube('spare_support_post',[(0,1.075,-.78),(0,1.414,-.78)],.014,'alloy')
    tube('spare_hold_down_plate',[(0,1.396,-.78),(0,1.408,-.78)],.058,'trim')
    tube('spare_hold_down_nut',[(0,1.408,-.78),(0,1.430,-.78)],.021,'alloy')
    tube('spare_hold_down_handle',[(-.052,1.437,-.78),(.052,1.437,-.78)],.009,'trim')

    # Studio lighting keeps silhouette and panel transitions legible in both neutral and painted previews.
    floor=box('studio_floor',(0,-.802,0),(200,.01,200),'ground',0)
    studio=base.collection('STUDIO_PREVIEW')
    col.objects.unlink(floor);studio.objects.link(floor)
    scene=bpy.context.scene
    scene.render.engine='CYCLES';scene.cycles.samples=48
    scene.cycles.use_denoising=True
    scene.world=bpy.data.worlds.new('Studio')
    scene.world.use_nodes=True
    scene.world.node_tree.nodes['Background'].inputs['Color'].default_value=(.22,.22,.22,1)
    scene.world.node_tree.nodes['Background'].inputs['Strength'].default_value=.45
    def light(name,loc,energy,size):
        data=bpy.data.lights.new(name,'AREA');data.energy=energy*.65;data.shape='RECTANGLE';data.size=size;data.size_y=size*.45
        o=bpy.data.objects.new(name,data);scene.collection.objects.link(o);o.location=B(*loc)
        o.rotation_euler=(Vector(B(0,.3,0))-o.location).to_track_quat('-Z','Y').to_euler()
    light('key',(4,7,5),1800,5);light('fill',(-5,3,2),1000,5);light('rim',(1,5,-5),2000,4)
    cam=bpy.data.objects.new('camera',bpy.data.cameras.new('camera'));scene.collection.objects.link(cam)
    scene.camera=cam;cam.data.type='ORTHO';cam.data.lens=50
    scene.render.resolution_percentage=100
    scene.view_settings.view_transform='AgX'
    scene.render.image_settings.file_format='PNG'
    views=[('rear_closeup',(2,1.0,-6),(0,.32,-2.18),1.85,1600,1100),
           ('three_quarter',(6,3.1,7),(0,.18,0),6.3,1500,1100),
           ('side',(8,.12,0),(0,.12,0),5.6,1600,800),
           ('front',(0,.18,8),(0,.18,0),3.1,1000,1000),
           ('top',(0,9,0),(0,0,0),5.5,1000,1500),
           ('windshield',(0,7,.85),(0,.65,.85),2.25,1400,1050),
           ('rear_three_quarter',(-6,2.7,-7),(0,.18,0),6.3,1500,1100),
           ('mirror_closeup',(3,1.45,3),(.99,.61,.66),.52,1400,1050),
           ('mirror_rear',(3,1.1,-1),(.99,.61,.66),.52,1400,1050),
           ('hood_detail',(1.8,3,2.7),(0,.55,1.3),1.85,1400,1000),
           ('quarter_detail',(4,2.3,-4),(.65,.55,-1.72),1.65,1500,1100),
           ('rear_top',(0,8,-1.80),(0,.30,-1.80),2.65,1400,1200),
           ('rear_bumper',(3,1.0,-6),(0,-.05,-2.18),2.5,1600,1000),
           ('rear_straight',(0,.30,-8),(0,.10,-2.1),2.6,1400,1100),
           ('pillar_closeup',(3,2,4),(.78,.55,.83),.48,1200,1100),
           ('door_profile',(8,.40,0),(0,.40,-.35),2.8,1600,850),
           ('doors',(6,1.8,-2),(.85,.36,-.30),2.55,1600,1100),
           ('front_corner',(4,1.4,5),(.80,.12,1.79),1.55,1400,1200)]
    if '--materials-only' in sys.argv:
        views=[v for v in views if v[0] in ('doors','hood_detail','rear_bumper','quarter_detail')]
    if '--finish-only' in sys.argv:
        views=[v for v in views if v[0] in ('three_quarter','front','doors','hood_detail','quarter_detail','rear_bumper','rear_top','rear_straight','rear_three_quarter')]
    if '--rear-shape-only' in sys.argv:
        views=[v for v in views if v[0] in ('rear_top','rear_straight','rear_three_quarter')]
    if '--rear-closeup-only' in sys.argv:
        views=[v for v in views if v[0]=='rear_closeup']
    if '--rear-only' in sys.argv:
        views=[v for v in views if v[0] in ('rear_closeup','rear_bumper','rear_straight','rear_three_quarter')]
    if '--fit-only' in sys.argv:
        views=[v for v in views if v[0] in ('top','front_corner','pillar_closeup')]
    if '--doors-only' in sys.argv:
        views=[v for v in views if v[0] in ('doors','door_profile','rear_three_quarter')]
    if '--detail-only' in sys.argv:
        views=[v for v in views if v[0] in ('mirror_closeup','mirror_rear','front_corner')]
    if '--no-render' in sys.argv:
        views=[]
    for name,loc,target,scale,width,height in views:
        cam.location=B(*loc);cam.rotation_euler=(Vector(B(*target))-cam.location).to_track_quat('-Z','Y').to_euler()
        cam.data.ortho_scale=scale;scene.render.resolution_x=width;scene.render.resolution_y=height
        scene.render.filepath='/tmp/expedition_'+('paint_' if painted else 'clay_')+name+'.png'
        bpy.ops.render.render(write_still=True)
    bpy.context.preferences.filepaths.file_preview_type='NONE'
    bpy.ops.wm.save_as_mainfile(filepath='/tmp/expedition_'+('paint' if painted else 'clay')+'.blend')

if __name__=='__main__':main()
