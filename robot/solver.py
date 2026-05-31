#!/usr/bin/env python3
"""
solver.py — BFS solver for the robot-constrained 2x2 Rubik's Cube.

Available robot actions:
  - rotate_top_cw   : spin top layer CW  (U move, robot holds cube)
  - rotate_top_ccw  : spin top layer CCW (U' move, robot holds cube)
  - tilt_x_pos      : tip cube forward   (F→top, pick+reorient+place)
  - tilt_x_neg      : tip cube backward  (B→top, pick+reorient+place)
  - tilt_y_pos      : tip cube rightward (R→top, pick+reorient+place)
  - tilt_y_neg      : tip cube leftward  (L→top, pick+reorient+place)

Note: rotate_top_180 (U2) is NOT included as an atomic action.
The BFS will find it naturally as two rotate_top_cw/ccw moves.
You can add it to ROBOT_MOVES for efficiency if desired.

State: tuple of 24 integers (one per sticker position), color index 0-5.
"""

from collections import deque


# ── Color & Position definitions ──────────────────────────────────────────────

COLORS = ['white', 'yellow', 'red', 'orange', 'blue', 'green']
COLOR_ABBR = {c: c[0].upper() for c in COLORS}  # W Y R O B G

POSITIONS = [
    # U face (4 stickers)
    'u-ufr', 'u-ufl', 'u-ubr', 'u-ubl',
    # D face (4 stickers)
    'd-dfr', 'd-dfl', 'd-dbr', 'd-dbl',
    # F face (4 stickers)
    'f-ufr', 'f-ufl', 'f-dfr', 'f-dfl',
    # B face (4 stickers)
    'b-ubr', 'b-ubl', 'b-dbr', 'b-dbl',
    # L face (4 stickers)
    'l-ufl', 'l-ubl', 'l-dfl', 'l-dbl',
    # R face (4 stickers)
    'r-ufr', 'r-ubr', 'r-dfr', 'r-dbr',
]

POS_IDX = {p: i for i, p in enumerate(POSITIONS)}
COLOR_IDX = {c: i for i, c in enumerate(COLORS)}


# ── Solved state (color index per position) ────────────────────────────────────

SOLVED_STATE = (
    # u-ufr, u-ufl, u-ubr, u-ubl  →  white (0)
    0, 0, 0, 0,
    # d-dfr, d-dfl, d-dbr, d-dbl  →  yellow (1)
    1, 1, 1, 1,
    # f-ufr, f-ufl, f-dfr, f-dfl  →  red (2)
    2, 2, 2, 2,
    # b-ubr, b-ubl, b-dbr, b-dbl  →  orange (3)
    3, 3, 3, 3,
    # l-ufl, l-ubl, l-dfl, l-dbl  →  green (4)
    4, 4, 4, 4,
    # r-ufr, r-ubr, r-dfr, r-dbr  →  blue (5)
    5, 5, 5, 5,
)


# ── Move permutations ─────────────────────────────────────────────────────────
# Each move is a list of 4-cycles: (p0, p1, p2, p3) means p0→p1→p2→p3→p0
# i.e., state[p1] ← state[p0], state[p2] ← state[p1], etc.

def build_perm(cycles):
    """Build permutation array from list of 4-cycles (CW direction)."""
    n = len(POSITIONS)
    perm = list(range(n))
    for cycle in cycles:
        idxs = [POS_IDX[p] for p in cycle]
        m = len(idxs)
        for i in range(m):
            perm[idxs[(i + 1) % m]] = idxs[i]
    return tuple(perm)


def invert_perm(perm):
    """Invert a permutation."""
    inv = list(range(len(perm)))
    for i, p in enumerate(perm):
        inv[p] = i
    return tuple(inv)


# ── rotate_top_cw  (U move) ───────────────────────────────────────────────────
ROTATE_TOP_CW_CYCLES = [
    ('u-ubl', 'u-ubr', 'u-ufr', 'u-ufl'),   # U face
    ('b-ubl', 'r-ubr', 'f-ufr', 'l-ufl'),   # top belt group 1
    ('l-ubl', 'b-ubr', 'r-ufr', 'f-ufl'),   # top belt group 2
]

# ── tilt_x_pos  (tip forward: F→top) ─────────────────────────────────────────
# Face mapping: U←F, F←D, D←B, B←U  (orientation-corrected)
# Corners involved:
#   [f-ufr → u-ufr → b-dbr → d-dfr]  (cycle of 4)
#   [f-ufl → u-ufl → b-dbl → d-dfl]
#   [f-dfr → u-ubr → b-ubr → d-dbr]
#   [f-dfl → u-ubl → b-ubl → d-dbl]
#   L face CW: [l-ufl → l-ubl → l-dbl → l-dfl]
#   R face CCW: [r-ufr → r-dfr → r-dbr → r-ubr]
TILT_X_POS_CYCLES = [
    ('f-ufr', 'u-ufr', 'b-dbr', 'd-dfr'),
    ('f-ufl', 'u-ufl', 'b-dbl', 'd-dfl'),
    ('f-dfr', 'u-ubr', 'b-ubr', 'd-dbr'),
    ('f-dfl', 'u-ubl', 'b-ubl', 'd-dbl'),
    ('l-ufl', 'l-ubl', 'l-dbl', 'l-dfl'),   # L face CW from left
    ('r-ufr', 'r-dfr', 'r-dbr', 'r-ubr'),   # R face CCW from right (same cycle order = CCW)
]

# ── tilt_y_pos  (tip rightward: R→top) ───────────────────────────────────────
# Face mapping: U←R, R←D, D←L, L←U
# Corners (4-cycles derived from Y+ rotation):
#   [r-ufr → u-ufr → l-dfl → d-dfr]
#   [r-ubr → u-ubr → l-dbl → d-dbr]
#   [r-dfr → u-ufl → l-ufl → d-dfl]
#   [r-dbr → u-ubl → l-ubl → d-dbl]
#   F face CW: [f-ufl → f-ufr → f-dfr → f-dfl]
#   B face CCW: [b-ubl → b-ubr → b-dbr → b-dbl]
TILT_Y_POS_CYCLES = [
    ('r-ufr', 'u-ufr', 'l-dfl', 'd-dfr'),
    ('r-ubr', 'u-ubr', 'l-dbl', 'd-dbr'),
    ('r-dfr', 'u-ufl', 'l-ufl', 'd-dfl'),
    ('r-dbr', 'u-ubl', 'l-ubl', 'd-dbl'),
    ('f-ufl', 'f-ufr', 'f-dfr', 'f-dfl'),   # F face CW from front
    ('b-ubl', 'b-dbl', 'b-dbr', 'b-ubr'),   # B face CW from back (= CCW from front)
]


# Build all robot move permutations
_rtcw = build_perm(ROTATE_TOP_CW_CYCLES)
_txp  = build_perm(TILT_X_POS_CYCLES)
_typ  = build_perm(TILT_Y_POS_CYCLES)

ROBOT_MOVES = {
    'rotate_top_cw':  _rtcw,
    'rotate_top_ccw': invert_perm(_rtcw),
    'tilt_x_pos':     _txp,
    'tilt_x_neg':     invert_perm(_txp),
    'tilt_y_pos':     _typ,
    'tilt_y_neg':     invert_perm(_typ),
}

INVERSE_MOVE = {
    'rotate_top_cw':  'rotate_top_ccw',
    'rotate_top_ccw': 'rotate_top_cw',
    'tilt_x_pos':     'tilt_x_neg',
    'tilt_x_neg':     'tilt_x_pos',
    'tilt_y_pos':     'tilt_y_neg',
    'tilt_y_neg':     'tilt_y_pos',
}


# ── State manipulation ─────────────────────────────────────────────────────────

def apply_move(state, perm):
    """Apply a permutation to a state tuple."""
    return tuple(state[perm[i]] for i in range(len(state)))


def scramble(move_names):
    """Apply a sequence of named robot moves to the solved state."""
    state = SOLVED_STATE
    for name in move_names:
        state = apply_move(state, ROBOT_MOVES[name])
    return state


def is_solved_monochromatic(state):
    """Returns True if each of the 6 faces is monochromatic (solved in some orientation)."""
    for face_idx in range(6):
        face_stickers = state[face_idx*4 : (face_idx+1)*4]
        if len(set(face_stickers)) > 1:
            return False
    return True


# ── BFS Solver ────────────────────────────────────────────────────────────────

def get_all_solved_states():
    """Generates all 24 physically reachable monochromatic states of the 2x2 cube."""
    solved_states = set()
    queue = deque([SOLVED_STATE])
    solved_states.add(SOLVED_STATE)
    while queue:
        curr = queue.popleft()
        for name in ['tilt_x_pos', 'tilt_x_neg', 'tilt_y_pos', 'tilt_y_neg']:
            perm = ROBOT_MOVES[name]
            ns = apply_move(curr, perm)
            if ns not in solved_states:
                solved_states.add(ns)
                queue.append(ns)
    return list(solved_states)


def bfs_solve(init_state):
    """
    Bidirectional BFS to find the shortest robot action sequence.
    Returns a list of move names, or None if no solution found.
    """
    if is_solved_monochromatic(init_state):
        return []

    solved_states = get_all_solved_states()
    fwd = {init_state: []}
    bwd = {s: [] for s in solved_states}
    fwd_q = deque([init_state])
    bwd_q = deque(solved_states)

    def expand(queue, visited):
        """Expand one level of BFS."""
        if not queue:
            return
        current_depth = len(visited[queue[0]])
        next_visited = {}
        while queue:
            s = queue[0]
            if len(visited[s]) > current_depth:
                break
            queue.popleft()
            for name, perm in ROBOT_MOVES.items():
                ns = apply_move(s, perm)
                if ns not in visited and ns not in next_visited:
                    next_visited[ns] = visited[s] + [name]
        visited.update(next_visited)
        for s in next_visited:
            queue.append(s)

    for depth in range(30):
        expand(fwd_q, fwd)
        # Check intersection
        intersect = set(fwd.keys()) & set(bwd.keys())
        if intersect:
            meet = intersect.pop()
            bwd_path = [INVERSE_MOVE[m] for m in reversed(bwd[meet])]
            return fwd[meet] + bwd_path

        expand(bwd_q, bwd)
        # Check intersection
        intersect = set(fwd.keys()) & set(bwd.keys())
        if intersect:
            meet = intersect.pop()
            bwd_path = [INVERSE_MOVE[m] for m in reversed(bwd[meet])]
            return fwd[meet] + bwd_path

    return None


# ── Pretty printer ────────────────────────────────────────────────────────────

def print_cube(state):
    """Pretty-print the 2x2 cube in cross layout."""
    A = COLOR_ABBR
    s = {POSITIONS[i]: COLORS[state[i]] for i in range(len(POSITIONS))}
    S = {p: A[c] for p, c in s.items()}

    print("         ┌────┐")
    print(f"         │{S['u-ubl']}{S['u-ubr']}  │")
    print(f"         │{S['u-ufl']}{S['u-ufr']}  │")
    print("┌────┬───┴┬───┴┬────┐")
    print(f"│{S['l-ubl']}{S['l-ufl']}  │{S['f-ufl']}{S['f-ufr']}  │{S['r-ufr']}{S['r-ubr']}  │{S['b-ubr']}{S['b-ubl']}  │")
    print(f"│{S['l-dbl']}{S['l-dfl']}  │{S['f-dfl']}{S['f-dfr']}  │{S['r-dfr']}{S['r-dbr']}  │{S['b-dbr']}{S['b-dbl']}  │")
    print("└────┴───┬┴───┬┴────┘")
    print(f"         │{S['d-dfl']}{S['d-dfr']}  │")
    print(f"         │{S['d-dbl']}{S['d-dbr']}  │")
    print("         └────┘")
    print()
    print("Legend: W=white  Y=yellow  R=red  O=orange  B=blue  G=green")
    print("Layout: [L][F][R][B], U=top, D=bottom")


# ── PDDL Problem Generator ─────────────────────────────────────────────────────

def generate_pddl_problem(state, filename="/home/barrendeiro/robotica/cub/robot/problem.pddl"):
    """Write the PDDL problem file for the given scrambled state."""
    lines = []
    lines.append(";;; ============================================================")
    lines.append(";;; PROBLEM: rubik-robot-scrambled")
    lines.append(";;; ============================================================")
    lines.append("(define (problem rubik-robot-scrambled)")
    lines.append("  (:domain rubik-robot)")
    lines.append("  (:objects")
    lines.append("    white yellow red orange blue green - color")
    lines.append("    " + " ".join(POSITIONS) + " - position")
    lines.append("  )")
    lines.append("  (:init")
    lines.append("    (cube-on-fixture)")
    for i, pos in enumerate(POSITIONS):
        lines.append(f"    (color-at {pos} {COLORS[state[i]]})")
    lines.append("  )")
    lines.append("  (:goal (and")
    lines.append("    (cube-on-fixture)")
    
    # Universal monochromatic goal constraints for all 6 faces
    faces = {
        'u': ['u-ufr', 'u-ufl', 'u-ubr', 'u-ubl'],
        'd': ['d-dfr', 'd-dfl', 'd-dbr', 'd-dbl'],
        'f': ['f-ufr', 'f-ufl', 'f-dfr', 'f-dfl'],
        'b': ['b-ubr', 'b-ubl', 'b-dbr', 'b-dbl'],
        'l': ['l-ufl', 'l-ubl', 'l-dfl', 'l-dbl'],
        'r': ['r-ufr', 'r-ubr', 'r-dfr', 'r-dbr']
    }
    
    for face_name, positions in faces.items():
        lines.append(f"    ;; Face {face_name.upper()} must be monochromatic")
        lines.append("    (or")
        for color in COLORS:
            conds = " ".join([f"(color-at {pos} {color})" for pos in positions])
            lines.append(f"      (and {conds})")
        lines.append("    )")
        
    lines.append("  ))")
    lines.append(")")
    
    with open(filename, 'w') as f:
        f.write("\n".join(lines))
    print(f"✓ PDDL problem file written to {filename}")


# ── Main ──────────────────────────────────────────────────────────────────────

if __name__ == '__main__':
    import sys
    import time

    valid_moves = list(ROBOT_MOVES.keys())

    if len(sys.argv) > 1:
        scramble_seq = sys.argv[1:]
    else:
        # Default: a simple 4-move scramble
        scramble_seq = ['tilt_x_pos', 'rotate_top_cw', 'tilt_y_pos', 'rotate_top_ccw']

    # Validate moves
    for m in scramble_seq:
        if m not in ROBOT_MOVES:
            print(f"ERROR: Unknown move '{m}'")
            print(f"Valid moves: {valid_moves}")
            sys.exit(1)

    print("=" * 60)
    print("  UR3 Robot — 2×2 Rubik's Cube Solver")
    print("=" * 60)
    print(f"\nScramble sequence ({len(scramble_seq)} moves):")
    for i, m in enumerate(scramble_seq):
        print(f"  {i+1}. {m}")

    init_state = scramble(scramble_seq)
    generate_pddl_problem(init_state)

    print("\n── Scrambled Cube ──")
    print_cube(init_state)

    if is_solved_monochromatic(init_state):
        print("Cube is already solved!")
        sys.exit(0)

    print("Solving with bidirectional BFS...")
    t0 = time.time()
    solution = bfs_solve(init_state)
    t1 = time.time()

    if solution is None:
        print("No solution found (this should not happen for a valid 2x2 state).")
        sys.exit(1)

    print(f"\n✓ OPTIMAL SOLUTION  ({len(solution)} robot actions)  [{t1-t0:.3f}s]")
    print()
    for i, action in enumerate(solution):
        print(f"  Step {i+1:2d}: {action}")

    # Verify
    state = init_state
    for action in solution:
        state = apply_move(state, ROBOT_MOVES[action])
    assert is_solved_monochromatic(state), "BUG: Solution verification failed!"
    print("\n✓ Verification passed — cube is solved.")
    print()
    print("── Solved Cube ──")
    print_cube(state)
