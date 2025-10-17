from OCC.Core.gp import gp_Pnt, gp_Dir, gp_Circ, gp_Pln, gp_Vec, gp_Ax3, gp_Ax2, gp_Lin
from OCC.Core.BRepBuilderAPI import (BRepBuilderAPI_MakeEdge, BRepBuilderAPI_MakeFace, BRepBuilderAPI_MakeWire)
from OCC.Core.BRepPrimAPI import BRepPrimAPI_MakePrism
from OCC.Core.BRepAlgoAPI import BRepAlgoAPI_Cut, BRepAlgoAPI_Fuse, BRepAlgoAPI_Common
from OCC.Core.GC import GC_MakeArcOfCircle
from OCC.Extend.DataExchange import write_stl_file
from OCC.Core.Bnd import Bnd_Box
from OCC.Core.BRepBndLib import brepbndlib_Add
from OCC.Core.ShapeUpgrade import ShapeUpgrade_UnifySameDomain
# from OCC.Core.BRepTools import BRepTools_ShapeHealing
# from OCC.Core.BRepTools impor

from OCC.Core.BRep import BRep_Builder
from OCC.Core.TopoDS import TopoDS_Compound, TopoDS_CompSolid
from OCC.Core.ShapeFix import ShapeFix_Face, ShapeFix_FaceConnect, ShapeFix_Shape

from copy import copy
from .extrude import *
from .sketch import Loop, Profile
from .curves import *
import os
import trimesh
from trimesh.sample import sample_surface
import random

from OCC.Core.TopTools import TopTools_ListOfShape

def vec2CADsolid(vec, is_numerical=True, n=256):
    cad = CADSequence.from_vector(vec, is_numerical=is_numerical, n=256)
    cad = create_CAD(cad)
    return cad

def debug__create_face(cad_seq:CADSequence, idx=0):
    extr = cad_seq.seq[idx]
    profile = copy(extr.profile) # use copy to prevent changing extrude_op internally
    profile.denormalize(extr.sketch_size)

    sketch_plane = copy(extr.sketch_plane)
    sketch_plane.origin = extr.sketch_pos

    face = create_profile_face(profile, sketch_plane)
    return face

def debug__create_extrude(cad_seq:CADSequence, idx=0):
    extr = cad_seq.seq[idx]
    extr = create_by_extrude(extr)
    return extr


def create_CAD(cad_seq: CADSequence, override_ops=False):
    """create a 3D CAD model from CADSequence. Only support extrude with boolean operation."""
    body = create_by_extrude(cad_seq.seq[0])
    for extrude_op in cad_seq.seq[1:]:
        new_body = create_by_extrude(extrude_op)
        if override_ops:
            body = BRepAlgoAPI_Fuse(body, new_body).Shape()
        else:
            if extrude_op.operation == EXTRUDE_OPERATIONS.index("NewBodyFeatureOperation") or \
                    extrude_op.operation == EXTRUDE_OPERATIONS.index("JoinFeatureOperation"):
                body = BRepAlgoAPI_Fuse(body, new_body).Shape()
                # body.Build()
                # body.SimplifyResult()
                # body = body.Shape()

                # unifier = ShapeUpgrade_UnifySameDomain(body, False, True, False)
                # unifier.Build()
                # body = unifier.Shape()
                
            elif extrude_op.operation == EXTRUDE_OPERATIONS.index("CutFeatureOperation"):
                body = BRepAlgoAPI_Cut(body, new_body).Shape()
            elif extrude_op.operation == EXTRUDE_OPERATIONS.index("IntersectFeatureOperation"):
                body = BRepAlgoAPI_Common(body, new_body).Shape()

    # unifier = ShapeUpgrade_UnifySameDomain(body, False, True, False)
    # unifier.Build()
    # body = unifier.Shape()

    # healer = ShapeFix_Shape()
    # healer.Init(body)
    # healer.Perform()
    # body = healer.Result()

    # builder = BRep_Builder()
    # compsolid = TopoDS_CompSolid()
    # builder.MakeCompSolid(compsolid)
    # builder.Add(compsolid, body)

    return body


def create_by_extrude(extrude_op: Extrude):
    """create a solid body from Extrude instance."""
    profile = copy(extrude_op.profile) # use copy to prevent changing extrude_op internally
    profile.denormalize(extrude_op.sketch_size)

    sketch_plane = copy(extrude_op.sketch_plane)
    sketch_plane.origin = extrude_op.sketch_pos

    face = create_profile_face(profile, sketch_plane)

    # healer = ShapeFix_Face()
    # healer.Init(face)
    # # healer.FixSplitFace()
    # healer.SetPrecision(1e2)
    # healer.Perform()
    # # print("healer.Status()")
    # # print(healer.Status())
    # face = healer.Result()
    
    normal = gp_Dir(*extrude_op.sketch_plane.normal)
    ext_vec = gp_Vec(normal).Multiplied(extrude_op.extent_one)
    body = BRepPrimAPI_MakePrism(face, ext_vec).Shape()

    if extrude_op.extent_type == EXTENT_TYPE.index("SymmetricFeatureExtentType"):
        body_sym = BRepPrimAPI_MakePrism(face, ext_vec.Reversed()).Shape()
        body = BRepAlgoAPI_Fuse(body, body_sym).Shape()
        # body.Build()
        # body.SimplifyResult() 
        # body = body.Shape()

    if extrude_op.extent_type == EXTENT_TYPE.index("TwoSidesFeatureExtentType"):
        ext_vec = gp_Vec(normal.Reversed()).Multiplied(extrude_op.extent_two)
        body_two = BRepPrimAPI_MakePrism(face, ext_vec).Shape()
        body = BRepAlgoAPI_Fuse(body, body_two).Shape()
        # body.Build()
        # body.SimplifyResult()
        # body = body.Shape()

    return body


def create_profile_face(profile: Profile, sketch_plane: CoordSystem):
    """create a face from a sketch profile and the sketch plane"""
    origin = gp_Pnt(*sketch_plane.origin)
    normal = gp_Dir(*sketch_plane.normal)
    x_axis = gp_Dir(*sketch_plane.x_axis)
    gp_face = gp_Pln(gp_Ax3(origin, normal, x_axis))

    all_loops = [create_loop_3d(loop, sketch_plane) for loop in profile.children]
    topo_face = BRepBuilderAPI_MakeFace(gp_face, all_loops[0])
    for loop in all_loops[1:]:
        topo_face.Add(loop.Reversed())

    topo_face.Build()

    # print("topo_face.Error()")
    # print(topo_face.Error())
    # print("topo_face.IsDone()")
    # print(topo_face.IsDone())
    return topo_face.Face()


    # args = TopTools_ListOfShape()
    # args.Append(BRepBuilderAPI_MakeFace(gp_face, all_loops[0]).Face())
    # tools = TopTools_ListOfShape()
    # for loop in all_loops[1:]:
    #     tools.Append(BRepBuilderAPI_MakeFace(gp_face, loop).Face())

    # topo_face = BRepAlgoAPI_Fuse()
    # topo_face.SetArguments(args)
    # topo_face.SetTools(tools)
    # topo_face.Build()
    # topo_face.SimplifyResult()

    # return topo_face.Shape()


def create_loop_3d(loop: Loop, sketch_plane: CoordSystem):
    """create a 3D sketch loop"""
    topo_wire = BRepBuilderAPI_MakeWire()
    for curve in loop.children:
        topo_edge = create_edge_3d(curve, sketch_plane)
        if topo_edge == -1: # omitted
            continue
        topo_wire.Add(topo_edge)

    return topo_wire.Wire()


def create_edge_3d(curve: CurveBase, sketch_plane: CoordSystem):
    """create a 3D edge"""
    if isinstance(curve, Line):
        if np.allclose(curve.start_point, curve.end_point):
            return -1
        start_point = point_local2global(curve.start_point, sketch_plane)
        end_point = point_local2global(curve.end_point, sketch_plane)
        topo_edge = BRepBuilderAPI_MakeEdge(start_point, end_point)
    elif isinstance(curve, Circle):
        center = point_local2global(curve.center, sketch_plane)
        axis = gp_Dir(*sketch_plane.normal)
        gp_circle = gp_Circ(gp_Ax2(center, axis), abs(float(curve.radius)))
        topo_edge = BRepBuilderAPI_MakeEdge(gp_circle)
    elif isinstance(curve, Arc):
        # print(curve.start_point, curve.mid_point, curve.end_point)
        start_point = point_local2global(curve.start_point, sketch_plane)
        mid_point = point_local2global(curve.mid_point, sketch_plane)
        end_point = point_local2global(curve.end_point, sketch_plane)
        arc = GC_MakeArcOfCircle(start_point, mid_point, end_point).Value()
        topo_edge = BRepBuilderAPI_MakeEdge(arc)
    else:
        raise NotImplementedError(type(curve))
    return topo_edge.Edge()


def point_local2global(point, sketch_plane: CoordSystem, to_gp_Pnt=True):
    """convert point in sketch plane local coordinates to global coordinates"""
    g_point = point[0] * sketch_plane.x_axis + point[1] * sketch_plane.y_axis + sketch_plane.origin
    if to_gp_Pnt:
        return gp_Pnt(*g_point)
    return g_point


def CADsolid2pc(shape, n_points, name=None):
    """convert opencascade solid to point clouds"""
    bbox = Bnd_Box()
    brepbndlib_Add(shape, bbox)
    if bbox.IsVoid():
        raise ValueError("box check failed")

    if name is None:
        name = random.randint(100000, 999999)
    write_stl_file(shape, "tmp_out_{}.stl".format(name))
    out_mesh = trimesh.load("tmp_out_{}.stl".format(name))
    os.system("rm tmp_out_{}.stl".format(name))
    out_pc, _ = sample_surface(out_mesh, n_points)
    return out_pc
