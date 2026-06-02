#!/usr/bin/env python3
"""
solver.py — IDA* solver for the robot-constrained 2x2 Rubik's Cube.

Available robot actions:
  - rotate_top_cw   : spin top layer CW  (U move, robot holds cube)
  - rotate_top_ccw  : spin top layer CCW (U' move, robot holds cube)
  - tilt_x_pos      : tip cube forward   (F→top, pick+reorient+place)
  - tilt_x_neg      : tip cube backward  (B→top, pick+reorient+place)
  - tilt_y_pos      : tip cube rightward (R→top, pick+reorient+place)
  - tilt_y_neg      : tip cube leftward  (L→top, pick+reorient+place)

Note: rotate_top_180 (U2) is NOT included as an atomic action.
IDA* will find it naturally as two rotate_top_cw/ccw moves.
You can add it to ROBOT_MOVES for efficiency if desired.

State: tuple of 24 integers (one per sticker position), color index 0-5.
"""

from collections import Counter, deque


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


# ── Physical face-turn permutations ───────────────────────────────────────────

FACE_NORMALS = {
    'u': (0, 1, 0),
    'd': (0, -1, 0),
    'f': (0, 0, 1),
    'b': (0, 0, -1),
    'l': (-1, 0, 0),
    'r': (1, 0, 0),
}


def _corner_coord(corner_name):
    return (
        1 if 'r' in corner_name else -1,
        1 if 'u' in corner_name else -1,
        1 if 'f' in corner_name else -1,
    )


def _position_geometry(position):
    face, corner_name = position.split('-', 1)
    return _corner_coord(corner_name), FACE_NORMALS[face]


def _rotate_vec(vec, axis, quarter_turns):
    x, y, z = vec
    ax, ay, az = axis
    for _ in range(quarter_turns % 4):
        if ax:
            y, z = -ax * z, ax * y
        elif ay:
            x, z = ay * z, -ay * x
        else:
            x, y = -az * y, az * x
    return (x, y, z)


_GEOM_TO_POS = {_position_geometry(pos): idx for idx, pos in enumerate(POSITIONS)}


def _build_face_turn(face):
    axis = FACE_NORMALS[face.lower()]
    perm = list(range(len(POSITIONS)))
    for src_idx, position in enumerate(POSITIONS):
        coord, normal = _position_geometry(position)
        if sum(coord[i] * axis[i] for i in range(3)) != 1:
            continue
        dst = (
            _rotate_vec(coord, axis, -1),
            _rotate_vec(normal, axis, -1),
        )
        perm[_GEOM_TO_POS[dst]] = src_idx
    return tuple(perm)


ROBOT_MOVES = {
    'U': _build_face_turn('u'),
    'D': _build_face_turn('d'),
    'R': _build_face_turn('r'),
    'L': _build_face_turn('l'),
    'F': _build_face_turn('f'),
    'B': _build_face_turn('b'),
}
ROBOT_MOVES.update({
    f'{name}_PRIME': invert_perm(perm)
    for name, perm in list(ROBOT_MOVES.items())
})

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

MAX_QTM_DEPTH = 14
PRUNING_TABLE_DEPTH = 4



def _build_whole_cube_turn(axis):
    perm = list(range(len(POSITIONS)))
    for src_idx, position in enumerate(POSITIONS):
        coord, normal = _position_geometry(position)
        dst = (_rotate_vec(coord, axis, -1), _rotate_vec(normal, axis, -1))
        perm[_GEOM_TO_POS[dst]] = src_idx
    return tuple(perm)


_CUBE_ROT_X = _build_whole_cube_turn((1, 0, 0))
_CUBE_ROT_Y = _build_whole_cube_turn((0, 1, 0))
_CUBE_ROT_Z = _build_whole_cube_turn((0, 0, 1))


def get_all_solved_states():
    """Generate all 24 solved color orientations of the cube."""
    rotations = [_CUBE_ROT_X, invert_perm(_CUBE_ROT_X), _CUBE_ROT_Y, invert_perm(_CUBE_ROT_Y), _CUBE_ROT_Z, invert_perm(_CUBE_ROT_Z)]
    solved_states = {SOLVED_STATE}
    queue = deque([SOLVED_STATE])
    while queue:
        state = queue.popleft()
        for perm in rotations:
            next_state = apply_move(state, perm)
            if next_state not in solved_states:
                solved_states.add(next_state)
                queue.append(next_state)
    return list(solved_states)


_PRUNING_DB = {}
_REVERSE_NEXT_MOVE = {}


def _state_key(state):
    return bytes(state)


def _apply_perm_key(state_key, perm):
    return bytes(state_key[perm[i]] for i in range(len(perm)))


def init_heuristic_db(max_depth=PRUNING_TABLE_DEPTH):
    """Build a small admissible reverse pruning table for IDA*."""
    if _PRUNING_DB:
        return
    queue = deque()
    for state in get_all_solved_states():
        key = _state_key(state)
        _PRUNING_DB[key] = 0
        _REVERSE_NEXT_MOVE[key] = None
        queue.append((key, 0))

    while queue:
        state_key, depth = queue.popleft()
        if depth >= max_depth:
            continue
        for name, perm in ROBOT_MOVES.items():
            next_key = _apply_perm_key(state_key, perm)
            if next_key in _PRUNING_DB:
                continue
            _PRUNING_DB[next_key] = depth + 1
            _REVERSE_NEXT_MOVE[next_key] = INVERSE_MOVE[name]
            queue.append((next_key, depth + 1))


def get_heuristic(state):
    """Return an admissible lower bound for IDA*."""
    init_heuristic_db()
    dist = _PRUNING_DB.get(_state_key(state))
    if dist is not None:
        return dist
    return PRUNING_TABLE_DEPTH + 1


def has_valid_color_counts(state):
    """True when the sticker state contains exactly four stickers of each color."""
    counts = Counter(state)
    return len(counts) == len(COLORS) and all(counts[color] == 4 for color in range(len(COLORS)))


def is_reachable_state(state):
    """Fast pre-check before solving. Full reachability is proven by finding a solution."""
    return has_valid_color_counts(state)


def _reverse_solution_from_table(state):
    init_heuristic_db()
    key = _state_key(state)
    if key not in _PRUNING_DB:
        return None

    suffix = []
    current = tuple(state)
    while True:
        move = _REVERSE_NEXT_MOVE[_state_key(current)]
        if move is None:
            return suffix
        suffix.append(move)
        current = apply_move(current, ROBOT_MOVES[move])


def bfs_solve(init_state):
    """
    IDA* (Iterative Deepening A*) solver for 2x2 Rubik's Cube.
    Keeps the name 'bfs_solve' for backward compatibility with existing callers.
    """
    if not is_reachable_state(init_state):
        return None
    if is_solved_monochromatic(init_state):
        return []

    init_heuristic_db()
    table_solution = _reverse_solution_from_table(init_state)
    if table_solution is not None:
        return table_solution

    path = [init_state]
    path_moves = []

    def search(g, bound):
        state = path[-1]
        suffix = _reverse_solution_from_table(state)
        if suffix is not None and g + len(suffix) <= bound:
            path_moves.extend(suffix)
            return True

        f_score = g + get_heuristic(state)
        if f_score > bound:
            return f_score
        if is_solved_monochromatic(state):
            return True

        min_overflow = float('inf')
        last_move = path_moves[-1] if path_moves else None
        for name, perm in ROBOT_MOVES.items():
            if last_move and INVERSE_MOVE[name] == last_move:
                continue
            if len(path_moves) >= 2 and path_moves[-1] == name and path_moves[-2] == name:
                continue

            next_state = apply_move(state, perm)
            if next_state in path:
                continue

            path.append(next_state)
            path_moves.append(name)
            result = search(g + 1, bound)
            if result is True:
                return True
            if result < min_overflow:
                min_overflow = result
            path_moves.pop()
            path.pop()
        return min_overflow

    bound = get_heuristic(init_state)
    while bound <= MAX_QTM_DEPTH:
        result = search(0, bound)
        if result is True:
            return path_moves
        if result == float('inf'):
            break
        bound = result
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

    print("Solving with IDA*...")
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
