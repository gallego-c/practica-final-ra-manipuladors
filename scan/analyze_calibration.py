#!/usr/bin/env python3
"""
analyze_calibration.py — Geometric analysis of cube scan calibration.

Compares BASELINE preprocessing (raw scan step → same cube face, identity stickers)
against a calibrate.html export and reports the minimal CUBE_INTERPRET delta.

Sticker layout (viewed from outside each face):
  [0=TL] [1=TR]
  [2=BL] [3=BR]

Scan order when capturing: U, F, R, B, L, D (faceData keyed by scan step label).

Usage:
  python3 scan/analyze_calibration.py path/to/calibration.json
  python3 scan/analyze_calibration.py   # reads stdin JSON
  echo '{...}' | python3 scan/analyze_calibration.py
"""

from __future__ import annotations

import json
import sys
from pathlib import Path
from typing import Dict, List, Optional, Tuple

FACES = ["U", "F", "R", "B", "L", "D"]
LABELS = ["TL", "TR", "BL", "BR"]

# Primitive sticker remaps (same as calibrate.html TX).
#
# A remap is read as:
#   output[i] = input[remap[i]]
#
# Sticker positions:
#   0 TL   1 TR
#   2 BL   3 BR
IDENTITY = [0, 1, 2, 3]
FLIP_H = [1, 0, 3, 2]
FLIP_V = [2, 3, 0, 1]
ROT_CW = [2, 0, 3, 1]
ROT_CCW = [1, 3, 0, 2]
ROT_180 = [3, 2, 1, 0]
DIAG_TL_BR = [0, 2, 1, 3]  # reflection across TL→BR diagonal (transpose)
DIAG_TR_BL = [3, 1, 2, 0]  # reflection across TR→BL diagonal

PRIMITIVES: Dict[str, List[int]] = {
    "identity": IDENTITY,
    "flip_h": FLIP_H,
    "flip_v": FLIP_V,
    "rot_cw": ROT_CW,
    "rot_ccw": ROT_CCW,
    "rot_180": ROT_180,
    "diag_tl_br": DIAG_TL_BR,
    "diag_tr_bl": DIAG_TR_BL,
}

D4_NAMES = {tuple(v): k for k, v in PRIMITIVES.items()}

SCAN_CORNERS = {
    "U": ["ubl", "ubr", "ufl", "ufr"],
    "F": ["ufl", "ufr", "dfl", "dfr"],
    "R": ["ufr", "ubr", "dfr", "dbr"],
    "B": ["ubr", "ubl", "dbr", "dbl"],
    "L": ["ubl", "ufl", "dbl", "dfl"],
    "D": ["dfl", "dfr", "dbl", "dbr"],
}

# Baseline = no interpretation layer (what index.html would do without CUBE_INTERPRET)
BASELINE = {f: {"scanKey": f, "remap": list(IDENTITY)} for f in FACES}


def compose(remap_a: List[int], remap_b: List[int]) -> List[int]:
    """Apply remap_b after remap_a: out[i] = in[remap_a[remap_b[i]]]."""
    return [remap_a[remap_b[i]] for i in range(4)]


def apply_remap(colors: List[str], remap: List[int]) -> List[str]:
    return [colors[remap[i]] for i in range(4)]


def remap_equal(a: List[int], b: List[int]) -> bool:
    return list(a) == list(b)


def decompose_remap(target: List[int], max_depth: int = 3) -> Optional[List[str]]:
    """Find shortest sequence of primitive names whose composition equals target."""
    exact = D4_NAMES.get(tuple(target))
    if exact is not None:
        return [] if exact == "identity" else [exact]

    names = list(PRIMITIVES.keys())

    def search(current: List[int], path: List[str]) -> Optional[List[str]]:
        if remap_equal(current, target):
            return path
        if len(path) >= max_depth:
            return None
        for name in names:
            if name == "identity":
                continue
            nxt = compose(current, PRIMITIVES[name])
            found = search(nxt, path + [name])
            if found is not None:
                return found
        return None

    return search(IDENTITY, [])


def is_geometric_face_transform(remap: List[int]) -> bool:
    """True if remap is one of the 8 square symmetries (D4 group)."""
    return tuple(remap) in D4_NAMES


def describe_remap(remap: List[int]) -> str:
    name = D4_NAMES.get(tuple(remap))
    if name == "identity":
        return "no rotation/reflection"
    if name == "flip_h":
        return "mirror left↔right"
    if name == "flip_v":
        return "mirror top↔bottom"
    if name == "rot_cw":
        return "rotate face 90° clockwise, viewed from outside"
    if name == "rot_ccw":
        return "rotate face 90° counter-clockwise, viewed from outside"
    if name == "rot_180":
        return "rotate face 180°"
    if name == "diag_tl_br":
        return "reflect across TL→BR diagonal (transpose; swaps TR↔BL)"
    if name == "diag_tr_bl":
        return "reflect across TR→BL diagonal (swaps TL↔BR)"
    return "NOT a square symmetry; check calibration"


def load_calibration(path: Optional[str]) -> dict:
    if path:
        text = Path(path).read_text(encoding="utf-8")
    else:
        text = sys.stdin.read()
    # Strip JS comment block if pasted from calibrate export
    if "// ---" in text:
        text = text.split("// ---")[0]
    return json.loads(text.strip())


def extract_mapping(data: dict) -> Dict[str, dict]:
    if "mapping" in data:
        m = data["mapping"]
        return {
            f: {
                "scanKey": m[f].get("source", m[f].get("scanKey", f)),
                "remap": list(m[f]["remap"] if "remap" in m[f] else _remap_from_flips(m[f])),
            }
            for f in FACES
        }
    if "suggestedIndexRemap" in data:
        out = {f: dict(BASELINE[f]) for f in FACES}
        for f, spec in data["suggestedIndexRemap"].items():
            out[f] = {
                "scanKey": spec["sourceCapture"],
                "remap": list(spec["remap"]),
            }
        return out
    raise ValueError("JSON must contain 'mapping' or 'suggestedIndexRemap'")


def _remap_from_flips(entry: dict) -> List[int]:
    r = list(IDENTITY)
    if entry.get("flipH"):
        r = compose(r, FLIP_H)
    if entry.get("flipV"):
        r = compose(r, FLIP_V)
    return r


def build_solver_face_data(captures: dict, mapping: Dict[str, dict]) -> Dict[str, List[str]]:
    out = {}
    for face in FACES:
        spec = mapping[face]
        key = spec["scanKey"]
        if key not in captures:
            continue
        colors = captures[key]["colors"] if isinstance(captures[key], dict) else captures[key]
        out[face] = apply_remap(colors, spec["remap"])
    return out


def analyze_scan_step_geometry(mapping: Dict[str, dict]) -> None:
    """Explain which scan step's raw buffer feeds each physical face."""
    print("\n── Scan step → physical face (inverse of CUBE_INTERPRET) ──")
    # For each scan step S, which physical faces read from faceData[S]?
    by_scan: Dict[str, List[str]] = {s: [] for s in FACES}
    for cube_face, spec in mapping.items():
        by_scan[spec["scanKey"]].append(cube_face)

    for scan in FACES:
        targets = by_scan[scan]
        if len(targets) == 1 and targets[0] == scan:
            print(f"  faceData[{scan}]  →  physical {targets[0]}  (direct)")
        elif targets:
            print(f"  faceData[{scan}]  →  physical {', '.join(targets)}")
        else:
            print(f"  faceData[{scan}]  →  (unused in 3D/solver)")


def detect_global_patterns(mapping: Dict[str, dict]) -> None:
    print("\n── Global patterns ──")
    flip_h_faces = [f for f in FACES if remap_equal(mapping[f]["remap"], FLIP_H)]
    flip_v_faces = [f for f in FACES if remap_equal(mapping[f]["remap"], FLIP_V)]
    rot_ccw_faces = [f for f in FACES if remap_equal(mapping[f]["remap"], ROT_CCW)]
    diag_tl_br_faces = [f for f in FACES if remap_equal(mapping[f]["remap"], DIAG_TL_BR)]
    id_faces = [f for f in FACES if remap_equal(mapping[f]["remap"], IDENTITY)]

    if flip_h_faces:
        print(f"  flip_h [1,0,3,2] on: {', '.join(flip_h_faces)}")
    if flip_v_faces:
        print(f"  flip_v [2,3,0,1] on: {', '.join(flip_v_faces)}")
    if rot_ccw_faces:
        print(f"  rot_ccw [1,3,0,2] on: {', '.join(rot_ccw_faces)}")
    if diag_tl_br_faces:
        print(f"  diag_tl_br [0,2,1,3] on: {', '.join(diag_tl_br_faces)}")
    if id_faces:
        print(f"  identity on: {', '.join(id_faces)}")

    swaps = [(f, mapping[f]["scanKey"]) for f in FACES if mapping[f]["scanKey"] != f]
    if swaps:
        print("  scanKey ≠ cube face (source swap):")
        for cube, src in swaps:
            print(f"    physical {cube} ← faceData[{src}]")

    # F/B cycle detection
    if mapping["F"]["scanKey"] == "B" and mapping["B"]["scanKey"] == "F":
        print("  F ↔ B capture swap (scan steps F and B exchange roles for front/back)")


def diff_against_baseline(mapping: Dict[str, dict]) -> Dict[str, dict]:
    """Minimal delta from baseline preprocessing."""
    delta = {}
    for f in FACES:
        b = BASELINE[f]
        m = mapping[f]
        if m["scanKey"] != b["scanKey"] or not remap_equal(m["remap"], b["remap"]):
            delta[f] = {"scanKey": m["scanKey"], "remap": list(m["remap"])}
    return delta


def emit_js_cube_interpret(mapping: Dict[str, dict]) -> str:
    lines = ["const CUBE_INTERPRET = {"]
    for f in FACES:
        m = mapping[f]
        r = ", ".join(str(x) for x in m["remap"])
        lines.append(f'  {f}: {{ scanKey: "{m["scanKey"]}", remap: [{r}] }},')
    lines.append("};")
    return "\n".join(lines)


def print_report(data: dict) -> None:
    mapping = extract_mapping(data)
    captures = data.get("captures", {})

    print("=" * 60)
    print("  CALIBRATION GEOMETRY ANALYSIS")
    print("=" * 60)

    print("\n── Baseline preprocessing (current capture, no interpret layer) ──")
    print("  faceData[scanStep] stored as captured at that scan step")
    print("  cube[face][i] = faceData[face][i]   (identity)")

    print("\n── Per-face delta (baseline → calibrated) ──")
    print(f"  {'Face':<4} {'scanKey':<8} {'remap':<18} {'ops':<30} corners (TL..BR)")
    print("  " + "-" * 72)
    all_geometric = True
    for f in FACES:
        b = BASELINE[f]
        m = mapping[f]
        ops = decompose_remap(m["remap"]) or ["custom"]
        ops_str = " · ".join(ops) if ops else "identity"
        if not is_geometric_face_transform(m["remap"]):
            all_geometric = False
            ops_str = "INVALID: not D4"
        changed = m["scanKey"] != b["scanKey"] or not remap_equal(m["remap"], b["remap"])
        mark = "*" if changed else " "
        rstr = str(m["remap"]).replace(" ", "")
        print(f"  {mark}{f:<3} {m['scanKey']:<8} {rstr:<18} {ops_str:<30} {SCAN_CORNERS[f]}")

    print("\n── Geometric validity ──")
    print(f"  All remaps are D4 square symmetries: {'YES' if all_geometric else 'NO'}")
    for f in FACES:
        print(f"  {f}: {describe_remap(mapping[f]['remap'])}")

    delta = diff_against_baseline(mapping)
    print(f"\n── Minimum CUBE_INTERPRET ({len(delta)}/{len(FACES)} faces differ from baseline) ──")
    print(emit_js_cube_interpret(mapping))

    detect_global_patterns(mapping)
    analyze_scan_step_geometry(mapping)

    if captures:
        computed = build_solver_face_data(captures, mapping)
        expected = data.get("solverFaceData", {})
        print("\n── solverFaceData verification ──")
        if expected:
            ok = all(computed.get(f) == expected.get(f) for f in FACES if f in expected)
            print(f"  Recomputed matches export: {'YES' if ok else 'NO'}")
            if not ok:
                for f in FACES:
                    if f in expected and computed.get(f) != expected.get(f):
                        print(f"    {f}: got {computed.get(f)} expected {expected.get(f)}")
        else:
            print("  (no solverFaceData in JSON — skipped)")

    print("\n── Implementation note (index.html) ──")
    print("  Capture unchanged. After all 6 faceData[step] buffers are filled:")
    print("  getCubeFaceColors(cubeFace):")
    print("    raw = faceData[ CUBE_INTERPRET[cubeFace].scanKey ]")
    print("    return remap(raw, CUBE_INTERPRET[cubeFace].remap)")
    print("=" * 60)


def main() -> None:
    path = sys.argv[1] if len(sys.argv) > 1 else None
    data = load_calibration(path)
    print_report(data)


if __name__ == "__main__":
    main()
