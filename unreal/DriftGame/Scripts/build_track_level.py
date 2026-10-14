"""Builds a playable DriftGame level from a Drift Lab "Export for Unreal" folder.

In the Unreal editor: Output Log -> switch the input box from "Cmd" to "Python" and run
    py "<project>/Scripts/build_track_level.py" "C:/path/to/shinamoyama"
(no argument = every folder in <project>/Tracks/). It:
  - imports <id>.glb into /Game/Tracks/<id> with per-triangle collision
  - downloads CC0 photo-scanned PBR textures from Poly Haven (cached in
    <project>/SourceTextures) and builds world-projected materials per surface
  - makes /Game/Maps/<id>: the track, sun/sky/clouds/fog, a PlayerStart on
    the start line, DriftGameMode - and makes it the startup/default map.
Re-running rebuilds the level (handy after re-exporting the track).
"""
import json
import math
import os
import re
import sys
import urllib.request

import unreal

PROJECT = unreal.Paths.convert_relative_path_to_full(unreal.Paths.project_dir())
TEX_CACHE = os.path.join(PROJECT, "SourceTextures")
MAT_DIR = "/Game/DriftMaterials"

# surface -> (Poly Haven asset, resolution, colour tint)
SURFACES = {
    "asphalt": ("asphalt_track", "4k", None),
    "grass": ("leafy_grass", "2k", (0.72, 1.0, 0.55)),
    "dirt": ("forest_ground_04", "2k", None),
    "gravel": ("gravel_floor_02", "2k", None),
    "concrete": ("brushed_concrete_03", "2k", None),
}
# Mesh-name word -> surface material ("kerb" = red/white from vertex colours).
WORDS = [("ROAD", "asphalt"), ("KERB", "kerb"), ("GRASS", "grass"), ("DIRT", "dirt"),
         ("GRAVEL", "gravel"), ("SAND", "gravel"), ("WALL", "concrete")]

asset_tools = unreal.AssetToolsHelpers.get_asset_tools()
eal = unreal.EditorAssetLibrary
mel = unreal.MaterialEditingLibrary


def log(msg):
    unreal.log("[DriftGame] " + msg)


def surface_of(name):
    """Last surface word in the name wins (track names may contain words too)."""
    up = name.upper()
    best, at = None, -1
    for word, surf in WORDS:
        i = up.rfind(word)
        if i > at:
            best, at = surf, i
    return best


# --- Textures & materials ----------------------------------------------------

def fetch(url):
    req = urllib.request.Request(url, headers={"User-Agent": "DriftGame-level-builder/1.0"})
    with urllib.request.urlopen(req, timeout=180) as r:
        return r.read()


def download_surface(asset, res):
    """Returns {map: local jpg path} and the real-world tile size in metres."""
    folder = os.path.join(TEX_CACHE, asset)
    os.makedirs(folder, exist_ok=True)
    meta_path = os.path.join(folder, "meta.json")
    if os.path.exists(meta_path):
        with open(meta_path) as f:
            meta = json.load(f)
    else:
        files = json.loads(fetch("https://api.polyhaven.com/files/" + asset))
        info = json.loads(fetch("https://api.polyhaven.com/info/" + asset))
        meta = {"tile_m": float((info.get("dimensions") or [2000])[0]) / 1000.0, "maps": {}}
        for key, poly in (("D", "Diffuse"), ("N", "nor_dx"), ("R", "Rough")):
            path = os.path.join(folder, "T_%s_%s.jpg" % (asset, key))
            log("downloading %s %s (%s)..." % (asset, poly, res))
            with open(path, "wb") as f:
                f.write(fetch(files[poly][res]["jpg"]["url"]))
            meta["maps"][key] = path
        with open(meta_path, "w") as f:
            json.dump(meta, f)
    return meta


def import_texture(path, dest, normal=False, linear=False):
    name = os.path.splitext(os.path.basename(path))[0]
    existing = dest + "/" + name
    if eal.does_asset_exist(existing):
        return eal.load_asset(existing)
    task = unreal.AssetImportTask()
    task.set_editor_property("filename", path)
    task.set_editor_property("destination_path", dest)
    task.set_editor_property("automated", True)
    task.set_editor_property("save", True)
    asset_tools.import_asset_tasks([task])
    tex = eal.load_asset(existing)
    if tex and normal:
        tex.set_editor_property("compression_settings", unreal.TextureCompressionSettings.TC_NORMALMAP)
        tex.set_editor_property("srgb", False)
        tex.set_editor_property("flip_green_channel", False)
    elif tex and linear:
        tex.set_editor_property("compression_settings", unreal.TextureCompressionSettings.TC_MASKS)
        tex.set_editor_property("srgb", False)
    if tex:
        eal.save_loaded_asset(tex)
    return tex


def new_material(name):
    path = MAT_DIR + "/" + name
    if eal.does_asset_exist(path):
        eal.delete_asset(path)
    return asset_tools.create_asset(name, MAT_DIR, unreal.Material, unreal.MaterialFactoryNew())


def surface_material(surf):
    asset, res, tint = SURFACES[surf]
    meta = download_surface(asset, res)
    tex_dir = MAT_DIR + "/Textures"
    d = import_texture(meta["maps"]["D"], tex_dir)
    n = import_texture(meta["maps"]["N"], tex_dir, normal=True)
    r = import_texture(meta["maps"]["R"], tex_dir, linear=True)
    mat = new_material("M_" + surf.capitalize())
    # UVs from world position (top-down), scaled to the scan's real size: no
    # stretching however the road mesh is laid out.
    wp = mel.create_material_expression(mat, unreal.MaterialExpressionWorldPosition, -1100, 0)
    mask = mel.create_material_expression(mat, unreal.MaterialExpressionComponentMask, -950, 0)
    mask.set_editor_property("r", True)
    mask.set_editor_property("g", True)
    uv = mel.create_material_expression(mat, unreal.MaterialExpressionMultiply, -800, 0)
    uv.set_editor_property("const_b", 1.0 / (max(meta["tile_m"], 0.5) * 100.0))
    mel.connect_material_expressions(wp, "", mask, "")
    mel.connect_material_expressions(mask, "", uv, "A")

    def sample(tex, y, sampler):
        s = mel.create_material_expression(mat, unreal.MaterialExpressionTextureSample, -550, y)
        s.set_editor_property("texture", tex)
        s.set_editor_property("sampler_type", sampler)
        mel.connect_material_expressions(uv, "", s, "UVs")
        return s

    sd = sample(d, -300, unreal.MaterialSamplerType.SAMPLERTYPE_COLOR)
    if tint:
        tn = mel.create_material_expression(mat, unreal.MaterialExpressionMultiply, -250, -300)
        c = mel.create_material_expression(mat, unreal.MaterialExpressionConstant3Vector, -450, -450)
        c.set_editor_property("constant", unreal.LinearColor(tint[0], tint[1], tint[2], 1.0))
        mel.connect_material_expressions(sd, "RGB", tn, "A")
        mel.connect_material_expressions(c, "", tn, "B")
        mel.connect_material_property(tn, "", unreal.MaterialProperty.MP_BASE_COLOR)
    else:
        mel.connect_material_property(sd, "RGB", unreal.MaterialProperty.MP_BASE_COLOR)
    sn = sample(n, 0, unreal.MaterialSamplerType.SAMPLERTYPE_NORMAL)
    mel.connect_material_property(sn, "RGB", unreal.MaterialProperty.MP_NORMAL)
    # Sampler type must match the texture's compression (TC_MASKS -> Masks), or
    # the material fails to compile and renders as the default checkerboard.
    sr = sample(r, 300, unreal.MaterialSamplerType.SAMPLERTYPE_MASKS)
    mel.connect_material_property(sr, "R", unreal.MaterialProperty.MP_ROUGHNESS)
    mel.recompile_material(mat)
    eal.save_loaded_asset(mat)
    return mat


def kerb_material():
    mat = new_material("M_Kerb")
    vc = mel.create_material_expression(mat, unreal.MaterialExpressionVertexColor, -400, 0)
    mel.connect_material_property(vc, "", unreal.MaterialProperty.MP_BASE_COLOR)
    rough = mel.create_material_expression(mat, unreal.MaterialExpressionConstant, -400, 200)
    rough.set_editor_property("r", 0.55)
    mel.connect_material_property(rough, "", unreal.MaterialProperty.MP_ROUGHNESS)
    mel.recompile_material(mat)
    eal.save_loaded_asset(mat)
    return mat


_materials = {}


def material_for(surf):
    if surf not in _materials:
        try:
            _materials[surf] = kerb_material() if surf == "kerb" else surface_material(surf)
        except Exception as e:  # no internet etc: keep the imported material
            unreal.log_warning("[DriftGame] material %s failed (%s) - keeping the imported one" % (surf, e))
            _materials[surf] = None
    return _materials[surf]


# --- Level -------------------------------------------------------------------

def import_track(folder):
    track_id = os.path.basename(os.path.normpath(folder))
    glb = os.path.join(folder, track_id + ".glb")
    if not os.path.exists(glb):
        globs = [f for f in os.listdir(folder) if f.lower().endswith(".glb")]
        if not globs:
            raise RuntimeError("no .glb in " + folder)
        glb = os.path.join(folder, globs[0])
        track_id = os.path.splitext(globs[0])[0]
    dest = "/Game/Tracks/" + track_id
    if eal.does_directory_exist(dest):
        eal.delete_directory(dest)
    task = unreal.AssetImportTask()
    task.set_editor_property("filename", glb)
    task.set_editor_property("destination_path", dest)
    task.set_editor_property("automated", True)
    task.set_editor_property("replace_existing", True)
    task.set_editor_property("save", True)
    asset_tools.import_asset_tasks([task])
    meshes = []
    for p in eal.list_assets(dest, recursive=True):
        a = eal.load_asset(p)
        if isinstance(a, unreal.StaticMesh):
            meshes.append(a)
    if not meshes:
        raise RuntimeError("import of %s produced no static meshes" % glb)
    log("imported %d meshes from %s" % (len(meshes), glb))
    return track_id, meshes


def set_collision(mesh, enabled):
    try:
        body = mesh.get_editor_property("body_setup")
        if body:
            flag = unreal.CollisionTraceFlag.CTF_USE_COMPLEX_AS_SIMPLE if enabled else unreal.CollisionTraceFlag.CTF_USE_SIMPLE_AS_COMPLEX
            body.set_editor_property("collision_trace_flag", flag)
            eal.save_loaded_asset(mesh)
    except Exception as e:
        unreal.log_warning("[DriftGame] collision setup failed on %s: %s" % (mesh.get_name(), e))


def remove_simple_collision(mesh):
    try:
        unreal.get_editor_subsystem(unreal.StaticMeshEditorSubsystem).remove_collisions(mesh)
        eal.save_loaded_asset(mesh)
    except Exception as e:
        unreal.log_warning("[DriftGame] couldn't strip collision from %s: %s" % (mesh.get_name(), e))


def open_fresh_level(path):
    les = unreal.get_editor_subsystem(unreal.LevelEditorSubsystem)
    eas = unreal.get_editor_subsystem(unreal.EditorActorSubsystem)
    if eal.does_asset_exist(path):
        les.load_level(path)
        actors = eas.get_all_level_actors()
        if actors:
            eas.destroy_actors(actors)
    else:
        les.new_level(path)
    return les, eas


def spawn(eas, cls, loc=None, yaw=0.0, pitch=0.0):
    rot = unreal.Rotator()
    rot.yaw = yaw
    rot.pitch = pitch
    return eas.spawn_actor_from_class(cls, loc or unreal.Vector(0, 0, 0), rot)


def add_sky(eas):
    sun = spawn(eas, unreal.DirectionalLight, unreal.Vector(0, 0, 2000), yaw=35.0, pitch=-38.0)
    sun.root_component.set_mobility(unreal.ComponentMobility.MOVABLE)
    sun.light_component.set_editor_property("atmosphere_sun_light", True)
    sun.light_component.set_editor_property("intensity", 9.0)
    spawn(eas, unreal.SkyAtmosphere)
    sky = spawn(eas, unreal.SkyLight, unreal.Vector(0, 0, 2000))
    sky.root_component.set_mobility(unreal.ComponentMobility.MOVABLE)
    sky.light_component.set_editor_property("real_time_capture", True)
    spawn(eas, unreal.VolumetricCloud)
    fog = spawn(eas, unreal.ExponentialHeightFog)
    fog.component.set_editor_property("fog_density", 0.01)
    ppv = spawn(eas, unreal.PostProcessVolume)
    ppv.set_editor_property("unbound", True)


def set_default_map(level_path):
    ini = os.path.join(PROJECT, "Config", "DefaultEngine.ini")
    with open(ini) as f:
        text = f.read()
    obj = "%s.%s" % (level_path, level_path.rsplit("/", 1)[-1])
    for key in ("EditorStartupMap", "GameDefaultMap"):
        text = re.sub(r"^%s=.*$" % key, "%s=%s" % (key, obj), text, flags=re.M)
    with open(ini, "w") as f:
        f.write(text)


def build(folder):
    track_id, meshes = import_track(folder)
    level_path = "/Game/Maps/" + track_id
    les, eas = open_fresh_level(level_path)
    markers = {}
    for mesh in meshes:
        name = mesh.get_name()
        upper = name.upper()
        if upper.endswith("START_AHEAD") or upper.endswith("START"):
            markers["ahead" if upper.endswith("START_AHEAD") else "start"] = mesh.get_bounds().origin
            continue
        # Trees and painted lines are visual only (lines sit 1 cm above the road).
        visual_only = "TREE" in upper or "LINE" in upper
        set_collision(mesh, not visual_only)
        actor = eas.spawn_actor_from_object(mesh, unreal.Vector(0, 0, 0), unreal.Rotator())
        actor.set_actor_label(name)
        comp = actor.static_mesh_component
        if visual_only:
            # The profile, not set_collision_enabled: StaticMeshActors re-apply
            # their BlockAll profile on load. Merged tree meshes otherwise get one
            # auto box over the whole forest and the car drives on top of it.
            comp.set_collision_profile_name("NoCollision")
            remove_simple_collision(mesh)
            continue
        surf = surface_of(name)
        if surf:
            comp.component_tags = [unreal.Name(surf)]
            mat = material_for(surf)
            if mat:
                for i in range(comp.get_num_materials()):
                    comp.set_material(i, mat)
    if "start" in markers and "ahead" in markers:
        s, a = markers["start"], markers["ahead"]
        yaw = math.degrees(math.atan2(a.y - s.y, a.x - s.x))
        spawn(eas, unreal.PlayerStart, unreal.Vector(s.x, s.y, s.z + 60.0), yaw=yaw)
    else:
        unreal.log_warning("[DriftGame] no START markers - placing PlayerStart at the origin")
        spawn(eas, unreal.PlayerStart, unreal.Vector(0, 0, 200))
    add_sky(eas)
    try:
        world = unreal.get_editor_subsystem(unreal.UnrealEditorSubsystem).get_editor_world()
        world.get_world_settings().set_editor_property("default_game_mode", unreal.load_class(None, "/Script/DriftGame.DriftGameMode"))
    except Exception as e:
        unreal.log_warning("[DriftGame] couldn't set the level's game mode (%s); the project default is DriftGameMode anyway" % e)
    les.save_current_level()
    set_default_map(level_path)
    log("built %s - press Play." % level_path)


def main():
    args = [a for a in sys.argv[1:] if a.strip()]
    folders = args
    if not folders:
        root = os.path.join(PROJECT, "Tracks")
        if os.path.isdir(root):
            folders = [os.path.join(root, d) for d in sorted(os.listdir(root)) if os.path.isdir(os.path.join(root, d))]
    if not folders:
        unreal.log_error("[DriftGame] pass a Drift Lab export folder, or put export folders in %s" % os.path.join(PROJECT, "Tracks"))
        return
    for folder in folders:
        build(folder)


main()
