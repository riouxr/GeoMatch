import bpy
import bmesh
from bpy.types import Panel, Operator, PropertyGroup
from mathutils import Matrix, Vector
from mathutils.bvhtree import BVHTree
from mathutils.kdtree import KDTree

bl_info = {
    "name": "Match Crawl (Rigid Transport)",
    "author": "Robert Rioux",
    "version": (4, 0),
    "blender": (3, 6, 0),
    "location": "View3D > Sidebar > Tool",
    "category": "Mesh",
}

# ============================================================
# Properties
# ============================================================
class MatchProps(PropertyGroup):
    source_obj: bpy.props.PointerProperty(type=bpy.types.Object)
    target_obj: bpy.props.PointerProperty(type=bpy.types.Object)

    source_seeds: bpy.props.CollectionProperty(type=bpy.types.PropertyGroup)
    target_seeds: bpy.props.CollectionProperty(type=bpy.types.PropertyGroup)

    frontier: bpy.props.CollectionProperty(type=bpy.types.PropertyGroup)
    matched: bpy.props.CollectionProperty(type=bpy.types.PropertyGroup)

    initialized: bpy.props.BoolProperty(default=False)

    R: bpy.props.FloatVectorProperty(size=9)
    T: bpy.props.FloatVectorProperty(size=3)

    def _get(self, col):
        return {int(p.name) for p in col}

    def _set(self, col, values):
        col.clear()
        for v in values:
            col.add().name = str(int(v))

    def set_RT(self, R, T):
        self.R = [R[i][j] for i in range(3) for j in range(3)]
        self.T = [T.x, T.y, T.z]

    def get_R(self):
        r = self.R
        return Matrix(((r[0], r[1], r[2]),
                       (r[3], r[4], r[5]),
                       (r[6], r[7], r[8])))

    def get_T(self):
        return Vector(self.T)

    def reset(self):
        self.source_obj = None
        self.target_obj = None
        self.source_seeds.clear()
        self.target_seeds.clear()
        self.frontier.clear()
        self.matched.clear()
        self.initialized = False

# ============================================================
# Helpers
# ============================================================
def ordered_selected_verts(context):
    obj = context.active_object
    if not obj or obj.mode != 'EDIT':
        return None, []
    bm = bmesh.from_edit_mesh(obj.data)
    out = []
    seen = set()
    for e in bm.select_history:
        if isinstance(e, bmesh.types.BMVert) and e.index not in seen:
            seen.add(e.index)
            out.append(e.index)
    return obj, out

def select_only(bm, indices):
    for v in bm.verts:
        v.select = False
    for i in indices:
        bm.verts[i].select = True

def build_kdtree(context, obj):
    deps = context.evaluated_depsgraph_get()
    eval_obj = obj.evaluated_get(deps)
    mesh = eval_obj.to_mesh()
    size = len(mesh.vertices)
    if size == 0:
        return None
    kd = KDTree(size)
    for i, v in enumerate(mesh.vertices):
        kd.insert(eval_obj.matrix_world @ v.co, i)
    kd.balance()
    eval_obj.to_mesh_clear()
    return kd

def rigid_from_3(src, tgt):
    def frame(a, b, c):
        x = (b - a).normalized()
        z = x.cross(c - a).normalized()
        y = z.cross(x).normalized()
        return Matrix((x, y, z)).transposed(), a

    R1, T1 = frame(*src)
    R2, T2 = frame(*tgt)

    R = R2 @ R1.inverted()
    T = T2 - R @ T1
    return R, T

# ============================================================
# Operators
# ============================================================
class MATCH_OT_SetSourceSeeds(Operator):
    bl_idname = "match.set_source"
    bl_label = "Set Source Seeds (3)"

    def execute(self, context):
        obj, verts = ordered_selected_verts(context)
        if len(verts) != 3:
            self.report({'ERROR'}, "Select exactly 3 vertices (ordered).")
            return {'CANCELLED'}

        p = context.scene.match_props
        p.reset()
        p.source_obj = obj
        for v in verts:
            p.source_seeds.add().name = str(v)

        return {'FINISHED'}

class MATCH_OT_SetTargetSeeds(Operator):
    bl_idname = "match.set_target"
    bl_label = "Set Target Seeds (3)"

    def execute(self, context):
        obj, verts = ordered_selected_verts(context)
        if len(verts) != 3:
            self.report({'ERROR'}, "Select exactly 3 vertices (same order).")
            return {'CANCELLED'}

        p = context.scene.match_props
        p.target_obj = obj
        p.target_seeds.clear()
        for v in verts:
            p.target_seeds.add().name = str(v)

        return {'FINISHED'}

class MATCH_OT_Crawl(Operator):
    bl_idname = "match.crawl"
    bl_label = "Crawl"

    def execute(self, context):
        p = context.scene.match_props
        src, tgt = p.source_obj, p.target_obj
        if not src or not tgt:
            return {'CANCELLED'}

        context.view_layer.objects.active = src
        bpy.ops.object.mode_set(mode='EDIT')

        bm = bmesh.from_edit_mesh(src.data)
        bm.verts.ensure_lookup_table()

        src_mw = src.matrix_world
        src_inv = src_mw.inverted()
        tgt_mw = tgt.matrix_world

        # ---------- INIT ----------
        if not p.initialized:
            src_pts = [src_mw @ bm.verts[int(s.name)].co for s in p.source_seeds]
            tgt_pts = [tgt_mw @ tgt.data.vertices[int(s.name)].co for s in p.target_seeds]

            R, T = rigid_from_3(src_pts, tgt_pts)
            p.set_RT(R, T)

            matched = set()
            frontier = set()

            for s, t in zip(p.source_seeds, p.target_seeds):
                si = int(s.name)
                ti = int(t.name)
                bm.verts[si].co = src_inv @ (tgt_mw @ tgt.data.vertices[ti].co)
                matched.add(si)

            for i in matched:
                for e in bm.verts[i].link_edges:
                    frontier.add(e.other_vert(bm.verts[i]).index)

            p._set(p.matched, matched)
            p._set(p.frontier, frontier)
            p.initialized = True

            bmesh.update_edit_mesh(src.data)
            select_only(bm, frontier)
            return {'FINISHED'}

        # ---------- CRAWL ----------
        R = p.get_R()
        T = p.get_T()
        kdtree = build_kdtree(context, tgt)

        if kdtree is None:
            self.report({'ERROR'}, "Target mesh has no vertices.")
            return {'CANCELLED'}

        frontier = p._get(p.frontier)
        matched = p._get(p.matched)
        next_frontier = set()

        for i in frontier:
            v = bm.verts[i]
            guess = R @ (src_mw @ v.co) + T
            hit = kdtree.find(guess)
            if hit:
                co, index, dist = hit
                v.co = src_inv @ co

            for e in v.link_edges:
                j = e.other_vert(v).index
                if j not in matched and j not in frontier:
                    next_frontier.add(j)

        matched |= frontier
        p._set(p.matched, matched)
        p._set(p.frontier, next_frontier)

        bmesh.update_edit_mesh(src.data)
        select_only(bm, next_frontier)
        return {'FINISHED'}

class MATCH_OT_Reset(Operator):
    bl_idname = "match.reset"
    bl_label = "Reset"

    def execute(self, context):
        context.scene.match_props.reset()
        return {'FINISHED'}

# ============================================================
# UI
# ============================================================
class MATCH_PT_Panel(Panel):
    bl_label = "Match Crawl"
    bl_space_type = 'VIEW_3D'
    bl_region_type = 'UI'
    bl_category = "Tool"

    def draw(self, context):
        col = self.layout.column()
        col.operator("match.set_source")
        col.operator("match.set_target")
        col.separator()
        col.operator("match.crawl")
        col.operator("match.reset")

# ============================================================
# Register
# ============================================================
classes = (
    MatchProps,
    MATCH_OT_SetSourceSeeds,
    MATCH_OT_SetTargetSeeds,
    MATCH_OT_Crawl,
    MATCH_OT_Reset,
    MATCH_PT_Panel,
)

def register():
    for c in classes:
        bpy.utils.register_class(c)
    bpy.types.Scene.match_props = bpy.props.PointerProperty(type=MatchProps)

def unregister():
    del bpy.types.Scene.match_props
    for c in reversed(classes):
        bpy.utils.unregister_class(c)

if __name__ == "__main__":
    register()