"""
Metashape Pro Automation Script
================================
Runs the full photogrammetry pipeline on the captured dataset:
  photos → calibration → masks → align → scale → dense cloud → mesh → export

How to run (two options):

  Option A — inside Metashape:
      Tools > Run Script > select this file

  Option B — command line (headless):
      /path/to/metashape.sh -r /home/orin/MATE-ROV/scripts/metashape_process.py

Physical setup for scale bars (do this BEFORE capturing):
  1. Download Agisoft coded targets:
       https://www.agisoft.com/downloads/installer/
     (scroll to "Targets" — print the 12-bit circular targets)
  2. Attach at least TWO targets to the PVC structure at a known distance apart.
  3. Measure the real-world distance between target centres (in metres).
  4. Update SCALE_BARS below with the target IDs and that distance.
  5. Re-capture your dataset with the targets visible in the shots.
"""

import os
import Metashape

# ── Configuration — edit these paths and scale bar values ────────────────────

DATASET_DIR  = os.path.expanduser("~/MATE-ROV/dataset")
PROJECT_FILE = os.path.join(DATASET_DIR, "project.psx")
EXPORT_OBJ   = os.path.join(DATASET_DIR, "model.obj")

# Map of (target_id_A, target_id_B) → distance in METRES.
# Metashape names coded targets as "target 0", "target 1", etc.
# Set to {} to skip scale bars (model will have arbitrary scale).
SCALE_BARS = {
    (0, 1): 0.30,   # <-- change 0.30 to your measured distance in metres
}

# Dense cloud quality: 1=Ultra, 2=High, 4=Medium, 8=Low
DEPTH_DOWNSCALE = 2

# ── Helpers ───────────────────────────────────────────────────────────────────

def load_photos(images_dir):
    exts = (".jpg", ".jpeg", ".png", ".tif", ".tiff")
    return sorted(
        os.path.join(images_dir, f)
        for f in os.listdir(images_dir)
        if f.lower().endswith(exts)
    )


def import_calibration(chunk, calib_xml):
    calib = Metashape.Calibration()
    calib.load(calib_xml)
    for sensor in chunk.sensors:
        sensor.user_calib = calib
        sensor.fixed_calibration = True
    print(f"  Calibration imported and fixed (fx={calib.f:.2f})")


def import_masks(chunk, masks_dir):
    # {filename} is replaced by Metashape with each photo's stem (no extension)
    mask_template = os.path.join(masks_dir, "{filename}.png")
    chunk.generateMasks(
        path=mask_template,
        masking_mode=Metashape.MaskingModeFile
    )
    masked = sum(1 for cam in chunk.cameras if cam.mask is not None)
    print(f"  Masks loaded for {masked}/{len(chunk.cameras)} cameras")


def align_photos(chunk):
    chunk.matchPhotos(
        downscale=1,                   # Highest accuracy keypoint detection
        generic_preselection=True,
        reference_preselection=False,
        filter_mask=True,              # Ignore masked regions when finding tie points
        mask_tiepoints=True,
        keypoint_limit=40000,
        tiepoint_limit=4000
    )
    chunk.alignCameras()
    aligned = sum(1 for c in chunk.cameras if c.transform is not None)
    print(f"  {aligned}/{len(chunk.cameras)} cameras aligned")


def apply_scale_bars(chunk, scale_bars):
    if not scale_bars:
        return

    chunk.detectMarkers(
        target_type=Metashape.CircularTarget12bit,
        tolerance=50,
        filter_mask=True
    )
    print(f"  Detected {len(chunk.markers)} coded targets: "
          f"{[m.label for m in chunk.markers]}")

    marker_map = {m.label: m for m in chunk.markers}
    added = 0
    for (id_a, id_b), dist_m in scale_bars.items():
        label_a, label_b = f"target {id_a}", f"target {id_b}"
        if label_a in marker_map and label_b in marker_map:
            sb = chunk.addScalebar(marker_map[label_a], marker_map[label_b])
            sb.reference.distance = dist_m
            added += 1
            print(f"  Scale bar: {label_a} ↔ {label_b} = {dist_m} m")
        else:
            missing = [l for l in (label_a, label_b) if l not in marker_map]
            print(f"  WARNING: marker(s) not found: {missing}")

    if added:
        chunk.updateTransform()
        print(f"  Scale applied ({added} scale bar(s))")


def build_dense_cloud(chunk, downscale):
    chunk.buildDepthMaps(
        downscale=downscale,
        filter_mode=Metashape.MildFiltering
    )
    chunk.buildPointCloud()
    pts = chunk.point_cloud.point_count if chunk.point_cloud else 0
    print(f"  Dense cloud: {pts:,} points")


def build_mesh(chunk):
    chunk.buildModel(
        source_data=Metashape.PointCloudData,
        surface_type=Metashape.Arbitrary,
        interpolation=Metashape.EnabledInterpolation,
        face_count=Metashape.MediumFaceCount
    )
    faces = chunk.model.face_count if chunk.model else 0
    print(f"  Mesh: {faces:,} faces")


def export_model(chunk, path):
    chunk.exportModel(
        path,
        binary=False,
        save_texture=True,
        save_uv=True,
        save_normals=True,
        format=Metashape.ModelFormatOBJ
    )
    print(f"  Exported: {path}")


# ── Main pipeline ─────────────────────────────────────────────────────────────

def run():
    images_dir = os.path.join(DATASET_DIR, "images")
    masks_dir  = os.path.join(DATASET_DIR, "masks")
    calib_xml  = os.path.join(DATASET_DIR, "calibration.xml")

    photos = load_photos(images_dir)
    if not photos:
        print(f"ERROR: No images in {images_dir}")
        return
    print(f"Found {len(photos)} photos in {images_dir}")

    doc = Metashape.Document()
    doc.save(PROJECT_FILE)
    chunk = doc.addChunk()

    print("\n[1/6] Adding photos ...")
    chunk.addPhotos(photos)
    doc.save()

    print("\n[2/6] Importing calibration ...")
    if os.path.exists(calib_xml):
        import_calibration(chunk, calib_xml)
        doc.save()
    else:
        print(f"  WARNING: {calib_xml} not found — using Metashape auto-calibration")

    print("\n[3/6] Importing masks ...")
    if os.path.exists(masks_dir):
        import_masks(chunk, masks_dir)
        doc.save()
    else:
        print(f"  WARNING: {masks_dir} not found — no masking applied")

    print("\n[4/6] Aligning photos ...")
    align_photos(chunk)
    doc.save()

    print("\n[5/6] Applying scale bars ...")
    apply_scale_bars(chunk, SCALE_BARS)
    doc.save()

    print("\n[6/6] Building dense cloud ...")
    build_dense_cloud(chunk, DEPTH_DOWNSCALE)
    doc.save()

    print("\n[+]   Building mesh ...")
    build_mesh(chunk)
    doc.save()

    print("\n[+]   Exporting model ...")
    export_model(chunk, EXPORT_OBJ)

    print("\nDone.")
    print(f"  Project : {PROJECT_FILE}")
    print(f"  Model   : {EXPORT_OBJ}")


run()
