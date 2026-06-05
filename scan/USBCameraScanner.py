#!/usr/bin/env python3
"""
camera_scan.py - Scan a 2x2 Rubik with a USB camera and solve it 
with the same pipeline the web scanner uses (robot.solver BFS for the
cube, then Fast Downward for the physical robot plan).

HOW TO RUN 
-----
  # 1) do some color calibration:
  python3 scan/USBCameraScanner.py --calibrate

  # 2) scan a cube and solve it:
  python3 scan/USBCameraScanner.py
  python3 scan/USBCameraScanner.py --device 2          # pick another camera index

CONTROLS (during scan / calibration window)
  SPACE / ENTER : capture the face shown in the title
  r             : redo the last captured face
  c             : (re)run color calibration
  q / ESC       : quit
"""

import sys
import os
import json
import time
import argparse
from pathlib import Path

import numpy as np
import cv2

_SCAN_DIR = Path(__file__).resolve().parent
_REPO_ROOT = _SCAN_DIR.parent
if str(_REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(_REPO_ROOT))

from robot.solver import COLOR_IDX, bfs_solve, print_cube, SOLVED_STATE  # noqa: E402

FACE_ORDER = ["U", "F", "R", "B", "L", "D"]
CALIB_FILE = _SCAN_DIR / "color_calib.json"
LAST_SCAN_FILE = _SCAN_DIR / "last_scan.json"

# Letters used internally: pink is folded into R, matching 2x2scaner.py.
LETTERS = ["W", "Y", "R", "O", "B", "G"]
LETTER_NAME = {"W": "white", "Y": "yellow", "R": "pink/red",
               "O": "orange", "B": "blue", "G": "green"}
# BGR swatches just for drawing the on-screen overlay.
LETTER_BGR = {"W": (240, 240, 240), "Y": (0, 215, 255), "R": (170, 110, 255),
              "O": (0, 140, 255), "B": (200, 90, 0), "G": (60, 170, 60)}

# Default HSV centroids (OpenCV ranges H:0-179 S:0-255 V:0-255).
# These are a starting point; --calibrate overwrites them for your lighting.
DEFAULT_CENTROIDS = {
    "W": (0,   0,   235),
    "Y": (30,  200, 225),
    "R": (167, 150, 240),   # pink reads as magenta-ish hue with high V
    "O": (13,  220, 235),
    "B": (112, 200, 185),
    "G": (68,  170, 175),
}
WHITE_S_MAX = 70      # below this saturation (and bright) -> white
WHITE_V_MIN = 140


# ===========================================================================
# Colour classification
# ===========================================================================
def _hue_dist(h1, h2):
    d = abs(int(h1) - int(h2)) % 180
    return min(d, 180 - d)


def classify_hsv(hsv, centroids):
    """Map a single (H,S,V) sample to one of W/Y/R/O/B/G."""
    h, s, v = int(hsv[0]), int(hsv[1]), int(hsv[2])
    if s < WHITE_S_MAX and v > WHITE_V_MIN:
        return "W"
    best, best_d = "W", 1e9
    for letter, (ch, cs, cv) in centroids.items():
        if letter == "W":
            continue
        # hue dominates; small saturation/value terms break ties (pink vs orange)
        d = _hue_dist(h, ch) * 3.0 + abs(s - cs) * 0.15 + abs(v - cv) * 0.05
        if d < best_d:
            best, best_d = letter, d
    return best


def bgr_to_hsv(bgr):
    px = np.uint8([[[int(bgr[0]), int(bgr[1]), int(bgr[2])]]])
    return cv2.cvtColor(px, cv2.COLOR_BGR2HSV)[0, 0]


def patch_median_bgr(frame, cx, cy, half):
    """Robust colour of a square patch centred at (cx,cy): per-channel median."""
    y0, y1 = max(cy - half, 0), min(cy + half, frame.shape[0])
    x0, x1 = max(cx - half, 0), min(cx + half, frame.shape[1])
    patch = frame[y0:y1, x0:x1].reshape(-1, 3)
    return np.median(patch, axis=0)


def facelet_regions(frame_w, frame_h):
    """Return the 4 facelet centres in [TL,TR,BL,BR] order + patch half-size.

    This [TL,TR,BL,BR] ordering MUST match build_solver_state below
    (it mirrors SCAN_CORNERS / build_solver_state in 2x2scaner.py).
    """
    box = int(min(frame_w, frame_h) * 0.42)
    cx, cy = frame_w // 2, frame_h // 2
    off = box // 4
    half = max(box // 8, 6)
    centres = [
        (cx - off, cy - off),  # TL
        (cx + off, cy - off),  # TR
        (cx - off, cy + off),  # BL
        (cx + off, cy + off),  # BR
    ]
    return centres, half, box


# ===========================================================================
# 6 faces x 4 facelets  ->  24-sticker solver state
# (Identical mapping to 2x2scaner.build_solver_state. If you later extract a
#  shared module, both files should import it instead of duplicating.)
# ===========================================================================
def build_solver_state(face_data):
    c = {
        "W": COLOR_IDX["white"], "Y": COLOR_IDX["yellow"], "R": COLOR_IDX["red"],
        "O": COLOR_IDX["orange"], "B": COLOR_IDX["blue"], "G": COLOR_IDX["green"],
    }
    s = [0] * 24
    s[0] = c[face_data["U"][3]]; s[1] = c[face_data["U"][2]]
    s[2] = c[face_data["U"][1]]; s[3] = c[face_data["U"][0]]
    s[4] = c[face_data["D"][1]]; s[5] = c[face_data["D"][0]]
    s[6] = c[face_data["D"][3]]; s[7] = c[face_data["D"][2]]
    s[8] = c[face_data["F"][1]]; s[9] = c[face_data["F"][0]]
    s[10] = c[face_data["F"][3]]; s[11] = c[face_data["F"][2]]
    s[12] = c[face_data["B"][0]]; s[13] = c[face_data["B"][1]]
    s[14] = c[face_data["B"][2]]; s[15] = c[face_data["B"][3]]
    s[16] = c[face_data["L"][1]]; s[17] = c[face_data["L"][0]]
    s[18] = c[face_data["L"][3]]; s[19] = c[face_data["L"][2]]
    s[20] = c[face_data["R"][0]]; s[21] = c[face_data["R"][1]]
    s[22] = c[face_data["R"][2]]; s[23] = c[face_data["R"][3]]
    return tuple(s)


def validate_state(face_data):
    """A real 2x2 must show exactly 4 of each of the 6 colours."""
    counts = {l: 0 for l in LETTERS}
    for f in FACE_ORDER:
        for l in face_data[f]:
            counts[l] += 1
    bad = {l: n for l, n in counts.items() if n != 4}
    return (len(bad) == 0), counts, bad


# ===========================================================================
# Calibration  (capture one reference HSV per colour)
# ===========================================================================
def load_centroids():
    if CALIB_FILE.exists():
        try:
            data = json.loads(CALIB_FILE.read_text())
            return {k: tuple(v) for k, v in data.items()}
        except Exception as e:
            print(f"[calib] could not read {CALIB_FILE}: {e}; using defaults")
    return dict(DEFAULT_CENTROIDS)


def save_centroids(centroids):
    CALIB_FILE.write_text(json.dumps({k: list(v) for k, v in centroids.items()}, indent=2))
    print(f"[calib] saved -> {CALIB_FILE}")


def run_calibration(cap):
    print("\n[calib] Fill the CENTRE patch fully with each colour, press SPACE.")
    centroids = dict(DEFAULT_CENTROIDS)
    for letter in LETTERS:
        while True:
            ok, frame = cap.read()
            if not ok:
                print("[calib] camera read failed"); return None
            frame = cv2.flip(frame, 1)
            h, w = frame.shape[:2]
            cx, cy = w // 2, h // 2
            half = max(min(h, w) // 16, 8)
            cv2.rectangle(frame, (cx - half, cy - half), (cx + half, cy + half), (0, 0, 0), 2)
            cv2.putText(frame, f"Show {LETTER_NAME[letter]} ({letter})  SPACE=capture  q=abort",
                        (20, 35), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 0), 3)
            cv2.putText(frame, f"Show {LETTER_NAME[letter]} ({letter})  SPACE=capture  q=abort",
                        (20, 35), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 1)
            cv2.imshow("scan", frame)
            k = cv2.waitKey(1) & 0xFF
            if k in (ord(" "), 13):
                bgr = patch_median_bgr(frame, cx, cy, half)
                centroids[letter] = tuple(int(x) for x in bgr_to_hsv(bgr))
                print(f"[calib] {letter} = HSV {centroids[letter]}")
                break
            if k in (ord("q"), 27):
                print("[calib] aborted"); return None
    save_centroids(centroids)
    return centroids


# ===========================================================================
# Scan the 6 faces
# ===========================================================================
def read_face(cap, centroids):
    """Sample the 4 facelets of the currently-shown face."""
    ok, frame = cap.read()
    if not ok:
        return None
    frame = cv2.flip(frame, 1)
    h, w = frame.shape[:2]
    centres, half, _ = facelet_regions(w, h)
    letters = []
    for (cx, cy) in centres:
        bgr = patch_median_bgr(frame, cx, cy, half)
        letters.append(classify_hsv(bgr_to_hsv(bgr), centroids))
    return letters


def draw_overlay(frame, face_label, captured, preview_letters):
    h, w = frame.shape[:2]
    centres, half, box = facelet_regions(w, h)
    cx, cy = w // 2, h // 2
    cv2.rectangle(frame, (cx - box // 2, cy - box // 2),
                  (cx + box // 2, cy + box // 2), (255, 255, 255), 1)
    for (px, py), letter in zip(centres, preview_letters or [""] * 4):
        col = LETTER_BGR.get(letter, (200, 200, 200))
        cv2.rectangle(frame, (px - half, py - half), (px + half, py + half), col, -1)
        cv2.rectangle(frame, (px - half, py - half), (px + half, py + half), (0, 0, 0), 2)
        if letter:
            cv2.putText(frame, letter, (px - 8, py + 6),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 2)
    title = f"Face {face_label}  [{captured}/6]   SPACE=capture  r=redo  c=calib  q=quit"
    cv2.putText(frame, title, (15, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 3)
    cv2.putText(frame, title, (15, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)


def run_scan(cap, centroids):
    face_data = {}
    i = 0
    while i < len(FACE_ORDER):
        face = FACE_ORDER[i]
        ok, frame = cap.read()
        if not ok:
            print("[scan] camera read failed"); return None
        frame = cv2.flip(frame, 1)
        preview = read_face_from_frame(frame, centroids)
        draw_overlay(frame, face, i, preview)
        cv2.imshow("scan", frame)
        k = cv2.waitKey(1) & 0xFF
        if k in (ord(" "), 13):
            face_data[face] = preview
            print(f"[scan] {face} = {preview}")
            i += 1
        elif k == ord("r") and i > 0:
            i -= 1
            face_data.pop(FACE_ORDER[i], None)
            print(f"[scan] redo {FACE_ORDER[i]}")
        elif k == ord("c"):
            new_c = run_calibration(cap)
            if new_c:
                centroids = new_c
        elif k in (ord("q"), 27):
            print("[scan] aborted"); return None
    return face_data


def read_face_from_frame(frame, centroids):
    h, w = frame.shape[:2]
    centres, half, _ = facelet_regions(w, h)
    out = []
    for (cx, cy) in centres:
        bgr = patch_median_bgr(frame, cx, cy, half)
        out.append(classify_hsv(bgr_to_hsv(bgr), centroids))
    return out


# ===========================================================================
# Solve hand-off  (same path as 2x2scaner.py)
# ===========================================================================
def solve(face_data):
    ok, counts, bad = validate_state(face_data)
    print("\n[solve] colour counts:", counts)
    if not ok:
        print(f"[solve] INVALID scan (each colour must appear 4x). Off: {bad}")
        print("[solve] re-scan or recalibrate (--calibrate). Not solving.")
        return False

    state = build_solver_state(face_data)
    print("\n[solve] scanned cube:")
    print_cube(state)

    # persist the scan so the motion pipeline can consume it (replaces the
    # random scramble in generate_taskfile.main()).
    LAST_SCAN_FILE.write_text(json.dumps(
        {"face_data": face_data, "state": list(state)}, indent=2))
    print(f"[solve] saved scan -> {LAST_SCAN_FILE}")

    if state == SOLVED_STATE:
        print("[solve] cube already solved; nothing to do.")
        return True

    print("[solve] Level 1: solving cube (robot.solver BFS)...")
    t0 = time.perf_counter()
    solution = bfs_solve(state)
    print(f"[solve] cube solution ({len(solution)} moves, {time.perf_counter()-t0:.3f}s): {solution}")

    # Level 2: Fast Downward physical plan (optional; needs ROS ws + fast-downward)
    try:
        from robot.generate_taskfile import generate_manipulation_problem, run_fast_downward
    except Exception as e:
        print(f"[solve] Level 2 skipped (Fast Downward helpers unavailable): {e}")
        return True

    prob = _REPO_ROOT / "robot" / "manipulation_problem.pddl"
    dom = _REPO_ROOT / "robot" / "manipulation_domain.pddl"
    generate_manipulation_problem(solution, filename=str(prob))
    plan = run_fast_downward(domain_path=str(dom), problem_path=str(prob))
    if plan is None:
        print("[solve] Fast Downward failed to produce a physical plan.")
        return False
    print(f"[solve] Level 2 physical plan ({len(plan)} actions):")
    for i, a in enumerate(plan, 1):
        print(f"  {i:2d}. {a}")
    (_REPO_ROOT / "robot" / "robot_plan.txt").write_text("\n".join(plan) + "\n")
    print(f"[solve] wrote robot/robot_plan.txt")
    return True


# ===========================================================================
# Self-test (no camera): feed a known scramble through the solve hand-off.
# ===========================================================================
def selftest():
    from robot.solver import scramble
    # Build face_data that decodes to a known scrambled state by reusing the
    # solved layout per face and letting the solver exercise the hand-off.
    # NOTE: solver.SOLVED_STATE has L=color-index 4 and R=5. Per COLOR_IDX that
    # is blue on L and green on R (solver.py's inline comments are swapped).
    # Decode order: 0=W 1=Y 2=R 3=O 4=B 5=G.
    solved = {
        "U": ["W", "W", "W", "W"], "D": ["Y", "Y", "Y", "Y"],
        "F": ["R", "R", "R", "R"], "B": ["O", "O", "O", "O"],
        "L": ["B", "B", "B", "B"], "R": ["G", "G", "G", "G"],
    }
    assert build_solver_state(solved) == SOLVED_STATE, "solved mapping mismatch"
    ok, counts, bad = validate_state(solved)
    assert ok, f"validation failed: {bad}"
    print("[selftest] solved face_data -> SOLVED_STATE OK; counts", counts)
    print("[selftest] classify checks:")
    for letter, hsv in DEFAULT_CENTROIDS.items():
        got = classify_hsv(hsv, DEFAULT_CENTROIDS)
        print(f"   centroid {letter} -> {got}", "OK" if got == letter else "**MISMATCH**")
    print("[selftest] running solve() on the solved cube (no camera):")
    return solve(solved)


# ===========================================================================
def open_camera(device):
    cap = cv2.VideoCapture(device)
    if not cap.isOpened():
        print(f"ERROR: could not open camera index {device}.")
        print("  - check the cable / try another index (--device 1, 2, ...)")
        print("  - on Linux verify with:  v4l2-ctl --list-devices")
        return None
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
    return cap


def main():
    ap = argparse.ArgumentParser(description="USB-camera 2x2 cube scanner + solver")
    ap.add_argument("--device", type=int, default=0, help="camera index (default 0)")
    ap.add_argument("--calibrate", action="store_true", help="run colour calibration then exit")
    ap.add_argument("--selftest", action="store_true", help="exercise solve hand-off without a camera")
    args = ap.parse_args()

    if args.selftest:
        sys.exit(0 if selftest() else 1)

    cap = open_camera(args.device)
    if cap is None:
        sys.exit(1)
    cv2.namedWindow("scan", cv2.WINDOW_NORMAL)

    centroids = load_centroids()
    try:
        if args.calibrate:
            run_calibration(cap)
            return
        print(f"[scan] using centroids from {'calibration' if CALIB_FILE.exists() else 'defaults'}")
        print("[scan] present faces in order:", " ".join(FACE_ORDER))
        face_data = run_scan(cap, centroids)
        if face_data:
            solve(face_data)
    finally:
        cap.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()