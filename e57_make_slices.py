import os
import shutil
import numpy as np
import pye57
import ezdxf
import json
from ezdxf.addons import Importer
import time
from datetime import timedelta
import open3d as o3d

INPUT_E57 = "model.e57"
OUTPUT_DIR = "output_slices_dxf"
CLEANED_E57 = "model_cleaned.e57"

LEVEL_COUNT = 5
VERTICAL_COUNT = 5

# THICKNESS_DIVIDER = 800  # grubość slice = zakres / 200
# SLICE_THICKNESS = 0.03   # 10 cm
SLICE_THICKNESS_BY_AXIS = {
    "X": 0.03,
    "Y": 0.03,
    "Z": 0.03,
}
# SLICE_STEP = 12.0         # co 1 metr
SLICE_STEP_BY_AXIS = {
    "X": 20.0,   # pionowe przekroje po X co 1 m
    "Y": 10.0,   # pionowe przekroje po Y co 1 m
    "Z": 2.0,   # poziome przekroje po Z co 0.5 m
}

POINT_SIZE = 0.005

AXIS_ID = {
    "X": 0,
    "Y": 1,
    "Z": 2,
}

ENABLE_SOR = True
ENABLE_RADIUS = True
ENABLE_VOXEL = False

ENABLE_CLEANING = True

VOXEL_SIZE = 0.02          # 2 cm
SOR_NB_NEIGHBORS = 20
SOR_STD_RATIO = 1.5

SOR_NB_NEIGHBORS = 6
SOR_STD_RATIO = 1.0

ENABLE_VOXEL = False
ENABLE_SOR = True
ENABLE_RADIUS = True

RADIUS_NB_POINTS = 3
RADIUS_VALUE = 0.04

def save_e57_points(points, path):
    print()
    print(f"Zapisuję oczyszczony E57: {path}")

    data = {
        "cartesianX": points[:, 0],
        "cartesianY": points[:, 1],
        "cartesianZ": points[:, 2],
    }

    e57 = pye57.E57(path, mode="w")
    e57.write_scan_raw(data)

    print(f"Zapisano E57: {path}")

def load_or_create_cleaned_points():
    # if os.path.exists(CLEANED_E57):
    #     print(f"Znaleziono oczyszczony plik: {CLEANED_E57}")
    #     print("Czytam oczyszczony E57...")
    #     points = read_e57_points(CLEANED_E57)
    #     print(f"Liczba punktów z cleaned E57: {len(points):,}")
    #     return points

    # print(f"Brak oczyszczonego pliku: {CLEANED_E57}")
    print("Czytam źródłowy E57...")
    points = read_e57_points(INPUT_E57)

    print(f"Liczba punktów źródłowych: {len(points):,}")

    if ENABLE_CLEANING:
        points = clean_points(points)
        # save_e57_points(points, CLEANED_E57)

    return points

def clean_points(points):

    print()
    print("Czyszczenie chmury punktów...")

    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points)

    print(f"Przed czyszczeniem: {len(points):,} pkt")

    if ENABLE_SOR:
        pcd, ind = pcd.remove_statistical_outlier(
            nb_neighbors=SOR_NB_NEIGHBORS,
            std_ratio=SOR_STD_RATIO
        )
        print(f"Po SOR: {len(pcd.points):,} pkt")

    if ENABLE_RADIUS:
        pcd, ind = pcd.remove_radius_outlier(
            nb_points=RADIUS_NB_POINTS,
            radius=RADIUS_VALUE
        )
        print(f"Po Radius Filter: {len(pcd.points):,} pkt")

    if ENABLE_VOXEL:
        pcd = pcd.voxel_down_sample(
            voxel_size=VOXEL_SIZE
        )
        print(f"Po Voxel: {len(pcd.points):,} pkt")

    cleaned = np.asarray(pcd.points)
    print(f"Po czyszczeniu finalnie: {len(cleaned):,} pkt")

    return cleaned

def clear_output_dir(path):
    if os.path.exists(path):
        shutil.rmtree(path)
    os.makedirs(path, exist_ok=True)


def read_e57_points(path):
    e57 = pye57.E57(path)
    all_points = []

    for scan_index in range(e57.scan_count):
        print(f"Czytam scan {scan_index + 1}/{e57.scan_count}...")

        data = e57.read_scan(scan_index, ignore_missing_fields=True)

        x = data["cartesianX"]
        y = data["cartesianY"]
        z = data["cartesianZ"]

        points = np.vstack((x, y, z)).T
        points = points[np.isfinite(points).all(axis=1)]

        all_points.append(points)

    return np.vstack(all_points)


def save_dxf_points(points, path, meta):
    if len(points) == 0:
        print(f"Pusty slice, pomijam: {path}")
        return

    doc = ezdxf.new("R2010")
    doc.header["$PDSIZE"] = POINT_SIZE
    doc.header["$PDMODE"] = 0

    msp = doc.modelspace()

    for layer_name in ["POINT_CLOUD", "REF_FRAME", "REF_AXIS", "REF_TEXT"]:
        if layer_name not in doc.layers:
            doc.layers.add(layer_name)

    for x, y, z in points:
        msp.add_point(
            (float(x), float(y), float(z)),
            dxfattribs={"layer": "POINT_CLOUD"}
        )

    x_min, y_min, z_min = np.min(points, axis=0)
    x_max, y_max, z_max = np.max(points, axis=0)
    x_mid, y_mid, z_mid = np.mean(points, axis=0)

    # punkty referencyjne
    ref_points = {
        "REF_MIN": (x_min, y_min, z_min),
        "REF_MAX": (x_max, y_max, z_max),
        "REF_CENTER": (x_mid, y_mid, z_mid),
    }

    for name, pt in ref_points.items():
        msp.add_point(pt, dxfattribs={"layer": "REF_FRAME"})
        msp.add_text(
            name,
            dxfattribs={"height": 0.15, "layer": "REF_TEXT"}
        ).set_placement(pt)

    # linie osi pomocniczych
    msp.add_line(
        (x_min, y_mid, z_mid),
        (x_max, y_mid, z_mid),
        dxfattribs={"layer": "REF_AXIS"}
    )

    msp.add_line(
        (x_mid, y_min, z_mid),
        (x_mid, y_max, z_mid),
        dxfattribs={"layer": "REF_AXIS"}
    )

    msp.add_line(
        (x_mid, y_mid, z_min),
        (x_mid, y_mid, z_max),
        dxfattribs={"layer": "REF_AXIS"}
    )

    # ramka bounding box
    corners = [
        (x_min, y_min, z_min),
        (x_max, y_min, z_min),
        (x_max, y_max, z_min),
        (x_min, y_max, z_min),
        (x_min, y_min, z_min),

        (x_min, y_min, z_max),
        (x_max, y_min, z_max),
        (x_max, y_max, z_max),
        (x_min, y_max, z_max),
        (x_min, y_min, z_max),
    ]

    # for i in range(4):
    #     msp.add_line(corners[i], corners[i + 1], dxfattribs={"layer": "REF_FRAME"})

    # for i in range(5, 9):
    #     msp.add_line(corners[i], corners[i + 1], dxfattribs={"layer": "REF_FRAME"})

    # for i in range(4):
    #     msp.add_line(corners[i], corners[i + 5], dxfattribs={"layer": "REF_FRAME"})

    # tekst z opisem przekroju
    label = (
        f"{meta['name']} | axis={meta['axis']} | "
        f"center={meta['center']:.3f} | "
        f"thickness={meta['thickness']:.3f} | "
        f"points={len(points)}"
    )

    msp.add_text(
        label,
        dxfattribs={"height": 0.2, "layer": "REF_TEXT"}
    ).set_placement((x_min, y_min, z_max))

    doc.saveas(path)

    json_path = path.replace(".dxf", ".json")

    meta_out = {
        **meta,
        "dxf_file": os.path.basename(path),
        "point_count": int(len(points)),
        "bounds": {
            "x_min": float(x_min),
            "x_max": float(x_max),
            "y_min": float(y_min),
            "y_max": float(y_max),
            "z_min": float(z_min),
            "z_max": float(z_max),
        },
        "center_xyz": {
            "x": float(x_mid),
            "y": float(y_mid),
            "z": float(z_mid),
        }
    }

    with open(json_path, "w", encoding="utf-8") as f:
        json.dump(meta_out, f, indent=2, ensure_ascii=False)


def make_slice(points, axis, center, thickness):
    axis_index = AXIS_ID[axis]
    values = points[:, axis_index]

    low = center - thickness / 2
    high = center + thickness / 2

    mask = (values >= low) & (values <= high)

    return points[mask], low, high


def get_slice_centers(points, axis):
    axis_index = AXIS_ID[axis]

    v_min = points[:, axis_index].min()
    v_max = points[:, axis_index].max()

    thickness = SLICE_THICKNESS_BY_AXIS[axis]
    step = SLICE_STEP_BY_AXIS[axis]

    centers = np.arange(
        v_min + thickness / 2,
        v_max - thickness / 2,
        step
    )

    return centers, thickness, v_min, v_max


def export_slices_for_axis(points, axis, output_dir, prefix):
    os.makedirs(output_dir, exist_ok=True)

    centers, thickness, v_min, v_max = get_slice_centers(points, axis)

    print()
    print(f"=== Przekroje {prefix} po osi {axis} ===")
    print(f"Zakres {axis}: {v_min:.3f} → {v_max:.3f}")
    print(f"Grubość slice: {thickness:.3f}")
    print(f"Krok slice: {SLICE_STEP_BY_AXIS[axis]:.3f}")

    for i, center in enumerate(centers, start=1):
        slice_points, low, high = make_slice(points, axis, center, thickness)

        out_file = os.path.join(
            output_dir,
            f"{prefix}_{i:02d}_{axis}_{center:.3f}.dxf"
        )

        save_dxf_points(
            slice_points,
            out_file,
            {
                "name": f"{prefix}_{i:02d}",
                "axis": axis,
                "center": float(center),
                "low": float(low),
                "high": float(high),
                "thickness": float(thickness),
            }
        )

        print(
            f"{prefix} {i}: {axis} {low:.3f} → {high:.3f}, "
            f"punktów: {len(slice_points):,}"
        )
def safe_name(name, max_len=31):
    bad = ['/', '\\', ':', '*', '?', '"', '<', '>', '|', '.', ' ']
    for ch in bad:
        name = name.replace(ch, '_')
    return name[:max_len]


def create_master_dxf(output_dir):
    master_path = os.path.join(output_dir, "_MASTER_ALL_SLICES.dxf")

    print()
    print("Tworzę master DXF...")

    master_doc = ezdxf.new("R2010")
    master_doc.header["$PDSIZE"] = POINT_SIZE
    master_doc.header["$PDMODE"] = 0

    master_msp = master_doc.modelspace()

    if "ALL_SLICES" not in master_doc.layers:
        master_doc.layers.add("ALL_SLICES")

    dxf_files = []

    for root, dirs, files in os.walk(output_dir):
        for file in files:
            if file.lower().endswith(".dxf") and not file.startswith("_MASTER"):
                dxf_files.append(os.path.join(root, file))

    dxf_files.sort()

    for index, dxf_path in enumerate(dxf_files, start=1):
        rel_name = os.path.relpath(dxf_path, output_dir)
        block_name = safe_name(f"S{index:03d}_{rel_name}")

        print(f"  Dodaję: {rel_name}")

        try:
            source_doc = ezdxf.readfile(dxf_path)

            if block_name in master_doc.blocks:
                block_name = safe_name(f"{block_name}_{index}")

            block = master_doc.blocks.new(name=block_name)

            importer = Importer(source_doc, master_doc)
            importer.import_entities(source_doc.modelspace(), block)
            importer.finalize()

            # Wstawiamy blok w oryginalnych współrzędnych
            master_msp.add_blockref(
                block_name,
                insert=(0, 0, 0),
                dxfattribs={"layer": "ALL_SLICES"}
            )

            # Osobny Layout dla każdego DXF
            layout_name = safe_name(f"L{index:03d}_{os.path.basename(dxf_path)}")

            if layout_name not in master_doc.layouts:
                layout = master_doc.layouts.new(layout_name)
                vp = layout.add_viewport(
                    center=(148.5, 105),   # środek kartki A4 landscape
                    size=(270, 180),
                    view_center_point=(0, 0),
                    view_height=20,
                )
                # ukrycie warstw pomocniczych
                vp.frozen_layers = [
                    "REF_FRAME",
                    "REF_AXIS",
                    "REF_TEXT",
                ]

        except Exception as e:
            print(f"  Błąd przy {rel_name}: {e}")

    master_doc.saveas(master_path)

    print(f"Master DXF zapisany: {master_path}")

def main():
    start_time = time.time()

    print("=" * 60)
    print("START:", time.strftime("%Y-%m-%d %H:%M:%S"))
    print("=" * 60)

    clear_output_dir(OUTPUT_DIR)

    points = load_or_create_cleaned_points()

    print(f"Liczba punktów do cięcia: {len(points):,}")

    # 1. Poziome przekroje całego modelu po Z
    horizontal_dir = os.path.join(OUTPUT_DIR, "01_horizontal_Z")
    export_slices_for_axis(
        points=points,
        axis="Z",
        # count=LEVEL_COUNT,
        output_dir=horizontal_dir,
        prefix="horizontal"
    )

    # 2. Pionowe przekroje całego modelu po X
    vertical_x_dir = os.path.join(OUTPUT_DIR, "02_vertical_X_full_model")
    export_slices_for_axis(
        points=points,
        axis="X",
        # count=VERTICAL_COUNT,
        output_dir=vertical_x_dir,
        prefix="vertical_X_full"
    )

    # 3. Pionowe przekroje całego modelu po Y
    vertical_y_dir = os.path.join(OUTPUT_DIR, "03_vertical_Y_full_model")
    export_slices_for_axis(
        points=points,
        axis="Y",
        # count=VERTICAL_COUNT,
        output_dir=vertical_y_dir,
        prefix="vertical_Y_full"
    )

    # 4. Najpierw poziom Z, potem pion X w każdym poziomie
    levels_dir = os.path.join(OUTPUT_DIR, "04_levels_Z_then_vertical_X")
    os.makedirs(levels_dir, exist_ok=True)

    z_centers, z_thickness, z_min, z_max = get_slice_centers(
        points,
        "Z"
        # LEVEL_COUNT
    )

    print()
    print("=== Najpierw poziom Z, potem pion X ===")

    for level_num, z_center in enumerate(z_centers, start=1):
        level_points, z_low, z_high = make_slice(
            points,
            "Z",
            z_center,
            z_thickness
        )

        level_dir = os.path.join(
            levels_dir,
            f"level_{level_num:02d}_Z_{z_center:.3f}"
        )
        os.makedirs(level_dir, exist_ok=True)

        horizontal_file = os.path.join(
            level_dir,
            f"level_{level_num:02d}_horizontal_Z.dxf"
        )

        save_dxf_points(
            level_points,
            horizontal_file,
            {
                "name": f"level_{level_num:02d}_horizontal_Z",
                "axis": "Z",
                "center": float(z_center),
                "low": float(z_low),
                "high": float(z_high),
                "thickness": float(z_thickness),
            }
        )

        print()
        print(f"Poziom {level_num}: Z {z_low:.3f} → {z_high:.3f}")
        print(f"Punktów poziomu: {len(level_points):,}")

        if len(level_points) == 0:
            continue

        export_slices_for_axis(
            points=level_points,
            axis="X",
            # count=VERTICAL_COUNT,
            output_dir=level_dir,
            prefix=f"level_{level_num:02d}_vertical_X"
        )
    create_master_dxf(OUTPUT_DIR)

    end_time = time.time()
    duration = end_time - start_time

    print()
    print("=" * 60)
    print("END:", time.strftime("%Y-%m-%d %H:%M:%S"))
    print(f"CZAS TRWANIA: {timedelta(seconds=int(duration))}")
    print("=" * 60)

    print()
    print("Gotowe.")
    print(f"Wynik: {OUTPUT_DIR}")


if __name__ == "__main__":
    main()