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
    ('r-ufr', 'u-ufl', 'l-dfl', 'd-dfr'),
    ('r-dfr', 'u-ufr', 'l-ufl', 'd-dfl'),
    ('r-ubr', 'u-ubl', 'l-dbl', 'd-dbr'),
    ('r-dbr', 'u-ubr', 'l-ubl', 'd-dbl'),
    ('f-ufl', 'f-dfl', 'f-dfr', 'f-ufr'),   # F face CCW from front
    ('b-ubr', 'b-ubl', 'b-dbl', 'b-dbr'),   # B face CW from back
]


# Build all robot move permutations
_rtcw = build_perm(ROTATE_TOP_CW_CYCLES)
_rtccw = invert_perm(_rtcw)
_txp  = build_perm(TILT_X_POS_CYCLES)
_txn  = invert_perm(_txp)
_typ  = build_perm(TILT_Y_POS_CYCLES)
_tyn  = invert_perm(_typ)

def compose_perms(*perms):
    """Composes multiple permutations in sequence."""
    p = list(range(len(perms[0])))
    for perm in perms:
        p = [p[perm[i]] for i in range(len(p))]
    return tuple(p)

# Compose standard Rubik's moves (U, D, R, L, F, B)
ROBOT_MOVES = {
    'U':       _rtcw,
    'U_PRIME': _rtccw,
    'D':       compose_perms(_txp, _txp, _rtcw, _txp, _txp),
    'D_PRIME': compose_perms(_txp, _txp, _rtccw, _txp, _txp),
    'R':       compose_perms(_typ, _rtcw, _tyn),
    'R_PRIME': compose_perms(_typ, _rtccw, _tyn),
    'L':       compose_perms(_tyn, _rtcw, _typ),
    'L_PRIME': compose_perms(_tyn, _rtccw, _typ),
    'F':       compose_perms(_txp, _rtcw, _txn),
    'F_PRIME': compose_perms(_txp, _rtccw, _txn),
    'B':       compose_perms(_txn, _rtcw, _txp),
    'B_PRIME': compose_perms(_txn, _rtccw, _txp),
}

INVERSE_MOVE = {
    'U':       'U_PRIME',
    'U_PRIME': 'U',
    'D':       'D_PRIME',
    'D_PRIME': 'D',
    'R':       'R_PRIME',
    'R_PRIME': 'R',
    'L':       'L_PRIME',
    'L_PRIME': 'L',
    'F':       'F_PRIME',
    'F_PRIME': 'F',
    'B':       'B_PRIME',
    'B_PRIME': 'B',
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


# ── IDA* Solver ───────────────────────────────────────────────────────────────

def get_all_solved_states():
    """Generates all 24 physically reachable monochromatic states of the 2x2 cube."""
    solved_states = set()
    queue = deque([SOLVED_STATE])
    solved_states.add(SOLVED_STATE)
    while queue:
        curr = queue.popleft()
        for perm in [_txp, _txn, _typ, _tyn]:
            ns = apply_move(curr, perm)
            if ns not in solved_states:
                solved_states.add(ns)
                queue.append(ns)
    return list(solved_states)

_HEURISTIC_DB = {}

def init_heuristic_db(max_depth=3):
    """Precompute a small distance database at startup to serve as an admissible heuristic."""
    global _HEURISTIC_DB
    if _HEURISTIC_DB:
        return
    solved = get_all_solved_states()
    queue = deque()
    for s in solved:
        _HEURISTIC_DB[s] = 0
        queue.append((s, 0))
    while queue:
        s, d = queue.popleft()
        if d >= max_depth:
            continue
        for name, perm in ROBOT_MOVES.items():
            ns = apply_move(s, perm)
            if ns not in _HEURISTIC_DB:
                _HEURISTIC_DB[ns] = d + 1
                queue.append((ns, d + 1))

def get_heuristic(state):
    """Returns an admissible lower bound of moves to solve the state."""
    return _HEURISTIC_DB.get(state, 4)

def bfs_solve(init_state):
    """
    IDA* (Iterative Deepening A*) solver for 2x2 Rubik's Cube.
    Keeps the name 'bfs_solve' for seamless backward compatibility.
    """
    init_heuristic_db(max_depth=3)
    
    if is_solved_monochromatic(init_state):
        return []

    def search(path, g, bound):
        state = path[-1]
        f = g + get_heuristic(state)
        if f > bound:
            return f
        if is_solved_monochromatic(state):
            return True
        
        min_val = float('inf')
        last_move = path_moves[-1] if path_moves else None
        
        for name, perm in ROBOT_MOVES.items():
            # Prune inverse moves
            if last_move and INVERSE_MOVE[name] == last_move:
                continue
            
            # Prune redundant identical moves (e.g. U U U is equivalent to U_PRIME)
            if len(path_moves) >= 2 and path_moves[-1] == name and path_moves[-2] == name:
                continue
                
            ns = apply_move(state, perm)
            if ns not in path:
                path.append(ns)
                path_moves.append(name)
                
                t = search(path, g + 1, bound)
                if t is True:
                    return True
                if t < min_val:
                    min_val = t
                    
                path_moves.pop()
                path.pop()
        return min_val

    bound = get_heuristic(init_state)
    path = [init_state]
    path_moves = []
    
    # 2x2 Rubik's Cube max distance is 11 in HTM, 14 in QTM
    while bound <= 14:
        t = search(path, 0, bound)
        if t is True:
            return path_moves
        if t == float('inf'):
            break
        bound = t
        
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


# ── Main ──────────────────────────────────────────────────────────────────────

if __name__ == '__main__':
    import sys
    import time

    valid_moves = list(ROBOT_MOVES.keys())

    if len(sys.argv) > 1:
        scramble_seq = sys.argv[1:]
    else:
        # Default: a simple 4-move scramble using standard Rubik's moves
        scramble_seq = ['R', 'U', 'L_PRIME', 'F']

    # Validate moves
    for m in scramble_seq:
        if m not in ROBOT_MOVES:
            print(f"ERROR: Unknown move '{m}'")
            print(f"Valid moves: {valid_moves}")
            sys.exit(1)

    print("=" * 60)
    print("  UR3 Robot — 2×2 Rubik's Cube Solver (Standard Moves)")
    print("=" * 60)
    print(f"\nScramble sequence ({len(scramble_seq)} moves):")
    for i, m in enumerate(scramble_seq):
        print(f"  {i+1}. {m}")

    init_state = scramble(scramble_seq)

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

    print(f"\n✓ OPTIMAL SOLUTION  ({len(solution)} standard Rubik moves)  [{t1-t0:.3f}s]")
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
