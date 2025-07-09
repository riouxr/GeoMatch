bl_info = {
    "name": "Geo Match",
    "author": "Your Name",
    "version": (3, 7),
    "blender": (4, 2, 0),
    "location": "View3D > Sidebar > Edit Tab",
    "description": "Visualize geometry match spreading by comparing topology, not indices, and move vertices to match.",
    "category": "Mesh",
}

import bpy
import bmesh
from bpy.types import Panel, Operator, PropertyGroup
from bpy.props import CollectionProperty, PointerProperty, IntProperty

class IntItem(PropertyGroup):
    value: IntProperty()

class GeoMatchProperties(PropertyGroup):
    source_object: PointerProperty(name="Source Object", type=bpy.types.Object)
    target_object: PointerProperty(name="Target Object", type=bpy.types.Object)
    source_start: CollectionProperty(type=IntItem)
    target_start: CollectionProperty(type=IntItem)
    visited_target: CollectionProperty(type=IntItem)
    visited_source: CollectionProperty(type=IntItem)

def store_indices(prop, indices):
    prop.clear()
    for i in indices:
        item = prop.add()
        item.value = i

def get_indices(prop):
    return [i.value for i in prop]

def get_selected_verts(obj):
    bm = bmesh.from_edit_mesh(obj.data)
    bm.verts.ensure_lookup_table()
    return [v.index for v in bm.verts if v.select]

def get_neighbors(bm, index):
    bm.verts.ensure_lookup_table()
    return set(e.other_vert(bm.verts[index]).index for e in bm.verts[index].link_edges)

def crawl_one_ring(obj, visited_set):
    bm = bmesh.from_edit_mesh(obj.data)
    bm.verts.ensure_lookup_table()
    next_ring = set()
    for i in visited_set:
        next_ring.update(get_neighbors(bm, i))
    return sorted(list(next_ring - visited_set))

def select_vertices(obj, indices):
    bpy.ops.object.mode_set(mode='EDIT')
    bm = bmesh.from_edit_mesh(obj.data)
    bm.verts.ensure_lookup_table()
    for v in bm.verts:
        v.select = v.index in indices
    bmesh.update_edit_mesh(obj.data)

def move_verts_to_match(src_obj, tgt_obj, src_indices, tgt_indices):
    bpy.ops.object.mode_set(mode='OBJECT')

    src_bm = bmesh.new()
    src_bm.from_mesh(src_obj.data)
    src_bm.verts.ensure_lookup_table()

    tgt_bm = bmesh.new()
    tgt_bm.from_mesh(tgt_obj.data)
    tgt_bm.verts.ensure_lookup_table()

    inv_mat = src_obj.matrix_world.inverted()
    for s_idx, t_idx in zip(src_indices, tgt_indices):
        if s_idx < len(src_bm.verts) and t_idx < len(tgt_bm.verts):
            sw = tgt_obj.matrix_world @ tgt_bm.verts[t_idx].co
            src_bm.verts[s_idx].co = inv_mat @ sw

    src_bm.to_mesh(src_obj.data)
    src_bm.free()
    tgt_bm.free()

    bpy.ops.object.mode_set(mode='EDIT')

def ensure_both_selected(src_obj, tgt_obj):
    for obj in bpy.context.selected_objects:
        obj.select_set(False)
    src_obj.select_set(True)
    tgt_obj.select_set(True)
    bpy.context.view_layer.objects.active = tgt_obj

class GEO_OT_AddSource(Operator):
    bl_idname = "geo_match.add_source"
    bl_label = "Add Source Vertices"
    def execute(self, context):
        props = context.scene.geo_match_props
        obj = context.active_object
        props.source_object = obj
        if obj.mode != 'EDIT': bpy.ops.object.mode_set(mode='EDIT')
        sel = get_selected_verts(obj)
        if not sel:
            self.report({'WARNING'}, "No vertices selected.")
            return {'CANCELLED'}
        store_indices(props.source_start, sel)
        store_indices(props.visited_source, sel)
        return {'FINISHED'}

class GEO_OT_AddTarget(Operator):
    bl_idname = "geo_match.add_target"
    bl_label = "Add Target Vertices"
    def execute(self, context):
        props = context.scene.geo_match_props
        obj = context.active_object
        props.target_object = obj
        if obj.mode != 'EDIT': bpy.ops.object.mode_set(mode='EDIT')
        sel = get_selected_verts(obj)
        if not sel:
            self.report({'WARNING'}, "No vertices selected.")
            return {'CANCELLED'}
        store_indices(props.target_start, sel)
        store_indices(props.visited_target, sel)
        return {'FINISHED'}

class GEO_OT_CrawlTopology(Operator):
    bl_idname = "geo_match.visual_crawl"
    bl_label = "Crawl Topology"
    bl_options = {'REGISTER', 'UNDO'}

    def execute(self, context):
        props = context.scene.geo_match_props
        src_obj = props.source_object
        tgt_obj = props.target_object

        if not src_obj or not tgt_obj:
            self.report({'ERROR'}, "Both source and target objects must be set.")
            return {'CANCELLED'}

        ensure_both_selected(src_obj, tgt_obj)

        initial_src = get_indices(props.source_start)
        initial_tgt = get_indices(props.target_start)
        if initial_src and initial_tgt:
            move_verts_to_match(src_obj, tgt_obj, initial_src, initial_tgt)

        src_visited = set(get_indices(props.visited_source))
        tgt_visited = set(get_indices(props.visited_target))

        new_tgt = crawl_one_ring(tgt_obj, tgt_visited)
        new_src = crawl_one_ring(src_obj, src_visited)

        if not new_src or not new_tgt:
            self.report({'INFO'}, "No new vertices crawled.")
            return {'FINISHED'}

        src_visited.update(new_src)
        tgt_visited.update(new_tgt)

        store_indices(props.visited_source, list(src_visited))
        store_indices(props.visited_target, list(tgt_visited))

        select_vertices(src_obj, new_src)
        select_vertices(tgt_obj, new_tgt)

        move_verts_to_match(src_obj, tgt_obj, new_src, new_tgt)

        return {'FINISHED'}

class GEO_OT_MatchAll(Operator):
    bl_idname = "geo_match.match_all"
    bl_label = "Match All"
    bl_options = {'REGISTER', 'UNDO'}

    def execute(self, context):
        props = context.scene.geo_match_props
        src_obj = props.source_object
        tgt_obj = props.target_object

        if not src_obj or not tgt_obj:
            self.report({'ERROR'}, "Both source and target objects must be set.")
            return {'CANCELLED'}

        ensure_both_selected(src_obj, tgt_obj)

        src_visited = set(get_indices(props.source_start))
        tgt_visited = set(get_indices(props.target_start))

        if not src_visited or not tgt_visited:
            self.report({'ERROR'}, "Please define both source and target seeds.")
            return {'CANCELLED'}

        move_verts_to_match(src_obj, tgt_obj, list(src_visited), list(tgt_visited))

        while True:
            new_tgt = crawl_one_ring(tgt_obj, tgt_visited)
            new_src = crawl_one_ring(src_obj, src_visited)

            if not new_tgt or not new_src:
                break

            move_verts_to_match(src_obj, tgt_obj, new_src, new_tgt)
            tgt_visited.update(new_tgt)
            src_visited.update(new_src)

        store_indices(props.visited_source, list(src_visited))
        store_indices(props.visited_target, list(tgt_visited))

        select_vertices(src_obj, list(src_visited))
        select_vertices(tgt_obj, list(tgt_visited))

        self.report({'INFO'}, f"Matched {min(len(src_visited), len(tgt_visited))} vertices.")
        return {'FINISHED'}

class GEO_PT_GeoMatchPanel(Panel):
    bl_label = "Geo Match"
    bl_space_type = 'VIEW_3D'
    bl_region_type = 'UI'
    bl_category = 'Edit'
    bl_context = "mesh_edit"

    def draw(self, context):
        layout = self.layout
        props = context.scene.geo_match_props

        layout.operator("geo_match.add_source")
        layout.operator("geo_match.add_target")
        layout.operator("geo_match.visual_crawl")
        layout.operator("geo_match.match_all")

classes = (
    IntItem,
    GeoMatchProperties,
    GEO_OT_AddSource,
    GEO_OT_AddTarget,
    GEO_OT_CrawlTopology,
    GEO_OT_MatchAll,
    GEO_PT_GeoMatchPanel
)

def register():
    for cls in classes:
        bpy.utils.register_class(cls)
    bpy.types.Scene.geo_match_props = PointerProperty(type=GeoMatchProperties)

def unregister():
    for cls in reversed(classes):
        bpy.utils.unregister_class(cls)
    del bpy.types.Scene.geo_match_props

if __name__ == "__main__":
    register()
