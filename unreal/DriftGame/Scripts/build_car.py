"""Imports a car exported by Drift Lab (tools/export_unreal_car.gd) and makes it the car you drive.

In the Unreal editor: Output Log -> "Cmd" box:
    py "<project>/Scripts/build_car.py" bdc_streetspec_miata_v4
(no argument = every folder in <project>/Cars/; the last one becomes the selected car). It:
  - imports Cars/<id>/<id>.glb into /Game/Cars/<id> (BODY, WHEEL_*, SUSP_*, STEER meshes,
    with the car's own textures), no collision - DriftCore does the car's collision
  - writes Content/DriftData/SelectedCar.txt = ac_<id>, the spec the car pawn loads
Re-running replaces the imported meshes (handy after re-exporting with another skin).
"""
import os
import sys

import unreal

PROJECT = unreal.Paths.convert_relative_path_to_full(unreal.Paths.project_dir())
eal = unreal.EditorAssetLibrary
asset_tools = unreal.AssetToolsHelpers.get_asset_tools()


def log(msg):
    unreal.log("[DriftGame] " + msg)


def import_car(car_id):
    glb = os.path.join(PROJECT, "Cars", car_id, car_id + ".glb")
    if not os.path.exists(glb):
        raise RuntimeError("%s not found - export the car from Drift Lab first" % glb)
    spec = os.path.join(PROJECT, "Content", "DriftData", "Cars", "ac_%s.json" % car_id)
    if not os.path.exists(spec):
        raise RuntimeError("%s not found - export the car from Drift Lab first" % spec)
    dest = "/Game/Cars/" + car_id
    if eal.does_directory_exist(dest):
        eal.delete_directory(dest)
    task = unreal.AssetImportTask()
    task.set_editor_property("filename", glb)
    task.set_editor_property("destination_path", dest)
    task.set_editor_property("automated", True)
    task.set_editor_property("replace_existing", True)
    task.set_editor_property("save", True)
    asset_tools.import_asset_tasks([task])
    meshes = {}
    for p in eal.list_assets(dest, recursive=True):
        a = eal.load_asset(p)
        if isinstance(a, unreal.StaticMesh):
            meshes[a.get_name()] = a
    if "BODY" not in meshes:
        raise RuntimeError("import of %s made no BODY mesh (got %s)" % (glb, sorted(meshes)))
    # The pawn finds the parts at /Game/Cars/<id>/<PART>; move any nested ones there.
    for name, mesh in meshes.items():
        want = "%s/%s" % (dest, name)
        if mesh.get_path_name().split(".")[0] != want:
            eal.rename_asset(mesh.get_path_name().split(".")[0], want)
        body = mesh.get_editor_property("body_setup")
        if body:
            body.set_editor_property("collision_trace_flag", unreal.CollisionTraceFlag.CTF_USE_SIMPLE_AS_COMPLEX)
        try:
            unreal.get_editor_subsystem(unreal.StaticMeshEditorSubsystem).remove_collisions(mesh)
        except Exception:
            pass
        eal.save_loaded_asset(mesh)
    log("imported %s: %s" % (car_id, ", ".join(sorted(meshes))))


def select(car_id):
    path = os.path.join(PROJECT, "Content", "DriftData", "SelectedCar.txt")
    with open(path, "w") as f:
        f.write("ac_%s\n" % car_id)
    log("selected car ac_%s - press Play." % car_id)


def main():
    ids = [a.strip() for a in sys.argv[1:] if a.strip()]
    if not ids:
        root = os.path.join(PROJECT, "Cars")
        if os.path.isdir(root):
            ids = sorted(d for d in os.listdir(root) if os.path.isdir(os.path.join(root, d)))
    if not ids:
        unreal.log_error("[DriftGame] no cars in %s - export one from Drift Lab first" % os.path.join(PROJECT, "Cars"))
        return
    for car_id in ids:
        import_car(car_id)
    select(ids[-1])


main()
