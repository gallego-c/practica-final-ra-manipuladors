#!/usr/bin/env python3
"""
solver.py — bidirectional BFS solver for the robot-constrained 2x2 Rubik's Cube.

Available standard moves: U, D, R, L, F, B, their _PRIME inverses, and 180-degree moves (U2, D2, R2, L2, F2, B2).

Half turns (U2, D2, R2, L2, F2, B2) are first-class moves.

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

def invert_perm(perm):
    """Invert a permutation."""
    inv = list(range(len(perm)))
    for i, p in enumerate(perm):
        inv[p] = i
    return tuple(inv)


def compose_perms(*perms):
    """Compose permutations in application order."""
    p = list(range(len(perms[0])))
    for perm in perms:
        p = [p[perm[i]] for i in range(len(p))]
    return tuple(p)


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
ROBOT_MOVES.update({
    f'{name}2': compose_perms(perm, perm)
    for name, perm in list(ROBOT_MOVES.items())
    if not name.endswith('_PRIME')
})

INVERSE_MOVE = {
    'U':       'U_PRIME',
    'U_PRIME': 'U',
    'U2':      'U2',
    'D':       'D_PRIME',
    'D_PRIME': 'D',
    'D2':      'D2',
    'R':       'R_PRIME',
    'R_PRIME': 'R',
    'R2':      'R2',
    'L':       'L_PRIME',
    'L_PRIME': 'L',
    'L2':      'L2',
    'F':       'F_PRIME',
    'F_PRIME': 'F',
    'F2':      'F2',
    'B':       'B_PRIME',
    'B_PRIME': 'B',
    'B2':      'B2',
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


# ── Bidirectional BFS solver with fixed DBL corner ────────────────────────────

CORNERS = [
    ('u-ufr', 'f-ufr', 'r-ufr'),
    ('u-ufl', 'l-ufl', 'f-ufl'),
    ('u-ubr', 'r-ubr', 'b-ubr'),
    ('u-ubl', 'b-ubl', 'l-ubl'),
    ('d-dfr', 'r-dfr', 'f-dfr'),
    ('d-dfl', 'f-dfl', 'l-dfl'),
    ('d-dbr', 'b-dbr', 'r-dbr'),
    ('d-dbl', 'l-dbl', 'b-dbl'),
]
CORNER_ID_DBL = 7
CORNERS_IDX = tuple(tuple(POS_IDX[pos] for pos in corner) for corner in CORNERS)
SOLVED_CORNER_COLORS = tuple(tuple(SOLVED_STATE[idx] for idx in corner) for corner in CORNERS_IDX)
COLOR_SET_TO_CUBIE = {frozenset(colors): idx for idx, colors in enumerate(SOLVED_CORNER_COLORS)}
ORIENTATION_MAPS = (
    (0, 1, 2),
    (1, 2, 0),
    (2, 0, 1),
)
MAX_HTM_DEPTH = 11



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


def _generate_cube_rotations():
    rotations = []
    seen = set()
    queue = deque([tuple(range(len(POSITIONS)))])
    generators = (_CUBE_ROT_X, invert_perm(_CUBE_ROT_X), _CUBE_ROT_Y, invert_perm(_CUBE_ROT_Y), _CUBE_ROT_Z, invert_perm(_CUBE_ROT_Z))
    while queue:
        rot = queue.popleft()
        if rot in seen:
            continue
        seen.add(rot)
        rotations.append(rot)
        for gen in generators:
            next_rot = compose_perms(rot, gen)
            if next_rot not in seen:
                queue.append(next_rot)
    return tuple(rotations)


CUBE_ROTATIONS = _generate_cube_rotations()


def get_all_solved_states():
    """Generate all 24 solved color orientations of the cube."""
    return [apply_move(SOLVED_STATE, rotation) for rotation in CUBE_ROTATIONS]


def _cubie_move_effect(perm):
    labels = [None] * len(POSITIONS)
    for corner_idx, corner in enumerate(CORNERS_IDX):
        for orient_idx, position_idx in enumerate(corner):
            labels[position_idx] = (corner_idx, orient_idx)

    moved = apply_move(tuple(labels), perm)
    effect = []
    for corner in CORNERS_IDX:
        labels_at_corner = [moved[position_idx] for position_idx in corner]
        source_corners = {label[0] for label in labels_at_corner}
        if len(source_corners) != 1:
            raise ValueError('Move permutation splits a physical corner')
        source_corner = labels_at_corner[0][0]
        orientation_map = [0, 0, 0]
        for dst_orient, (_, src_orient) in enumerate(labels_at_corner):
            orientation_map[src_orient] = dst_orient
        effect.append((source_corner, tuple(orientation_map)))
    return tuple(effect)


MOVE_EFFECTS = {name: _cubie_move_effect(perm) for name, perm in ROBOT_MOVES.items()}
ROTATION_EFFECTS = tuple((rotation, _cubie_move_effect(rotation)) for rotation in CUBE_ROTATIONS)


def _apply_cubie_effect(cubie_state, effect):
    corner_perm, corner_orient = cubie_state
    next_perm = [0] * len(CORNERS)
    next_orient = [0] * len(CORNERS)
    for dst_corner, (src_corner, orientation_map) in enumerate(effect):
        next_perm[dst_corner] = corner_perm[src_corner]
        next_orient[dst_corner] = orientation_map[corner_orient[src_corner]]
    return tuple(next_perm), tuple(next_orient)


def _oriented_corner_colors(cubie, orient):
    colors = SOLVED_CORNER_COLORS[cubie]
    oriented = [None, None, None]
    for src_idx, dst_idx in enumerate(ORIENTATION_MAPS[orient]):
        oriented[dst_idx] = colors[src_idx]
    return tuple(oriented)


def _state_to_cubies(state):
    if not has_valid_color_counts(state):
        return None

    corner_perm = []
    corner_orient = []
    seen_cubies = set()
    for corner in CORNERS_IDX:
        colors = tuple(state[position_idx] for position_idx in corner)
        cubie = COLOR_SET_TO_CUBIE.get(frozenset(colors))
        if cubie is None or cubie in seen_cubies:
            return None
        orient = None
        for candidate in range(3):
            if colors == _oriented_corner_colors(cubie, candidate):
                orient = candidate
                break
        if orient is None:
            return None
        seen_cubies.add(cubie)
        corner_perm.append(cubie)
        corner_orient.append(orient)
    return tuple(corner_perm), tuple(corner_orient)


def _cubie_key(cubie_state):
    corner_perm, corner_orient = cubie_state
    return bytes(corner_perm + corner_orient)


def _canonicalize_fixed_corner(cubie_state):
    """Rotate the whole cube so the DBL cubie is fixed and oriented."""
    for rotation_perm, effect in ROTATION_EFFECTS:
        rotated = _apply_cubie_effect(cubie_state, effect)
        corner_perm, corner_orient = rotated
        if corner_perm[CORNER_ID_DBL] == CORNER_ID_DBL and corner_orient[CORNER_ID_DBL] == 0:
            return rotated, rotation_perm
    return None, None


def _move_to_original(move_name, canonical_rotation):
    inverse_rotation = invert_perm(canonical_rotation)
    move_perm = ROBOT_MOVES[move_name]
    conjugated = [0] * len(POSITIONS)
    for pos in range(len(POSITIONS)):
        conjugated[pos] = canonical_rotation[move_perm[inverse_rotation[pos]]]
    move_by_perm = {perm: name for name, perm in ROBOT_MOVES.items()}
    return move_by_perm[tuple(conjugated)]


BIDIR_MOVES = []
for face in ('U', 'R', 'F'):
    BIDIR_MOVES.append((face, MOVE_EFFECTS[face]))
    prime = f'{face}_PRIME'
    BIDIR_MOVES.append((prime, MOVE_EFFECTS[prime]))
    double = f'{face}2'
    BIDIR_MOVES.append((double, MOVE_EFFECTS[double]))

INVERSE_BIDIR_MOVE = {
    'U': 'U_PRIME', 'U_PRIME': 'U', 'U2': 'U2',
    'R': 'R_PRIME', 'R_PRIME': 'R', 'R2': 'R2',
    'F': 'F_PRIME', 'F_PRIME': 'F', 'F2': 'F2',
}
BIDIR_MOVE_EFFECT = {name: effect for name, effect in BIDIR_MOVES}
SOLVED_FIXED_KEY = _cubie_key((tuple(range(len(CORNERS))), (0,) * len(CORNERS)))


def has_valid_color_counts(state):
    """True when the sticker state contains exactly four stickers of each color."""
    counts = Counter(state)
    return len(counts) == len(COLORS) and all(counts[color] == 4 for color in range(len(COLORS)))


def is_reachable_state(state):
    """True when the sticker state is a valid physical 2x2 corner state."""
    cubies = _state_to_cubies(state)
    if cubies is None:
        return False
    canonical, _ = _canonicalize_fixed_corner(cubies)
    return canonical is not None


def _reconstruct_bidirectional(meet_key, parents_from_start, parents_from_goal):
    first_half = []
    key = meet_key
    while parents_from_start[key][0] is not None:
        key, move_name = parents_from_start[key]
        first_half.append(move_name)
    first_half.reverse()

    second_half = []
    key = meet_key
    while parents_from_goal[key][0] is not None:
        key, move_name = parents_from_goal[key]
        second_half.append(INVERSE_BIDIR_MOVE[move_name])
    return first_half + second_half


def _expand_frontier(frontier, own_parents, other_parents):
    next_frontier = set()
    for key in frontier:
        cubie_state = (tuple(key[:8]), tuple(key[8:]))
        for move_name, effect in BIDIR_MOVES:
            next_state = _apply_cubie_effect(cubie_state, effect)
            next_key = _cubie_key(next_state)
            if next_key in own_parents:
                continue
            own_parents[next_key] = (key, move_name)
            if next_key in other_parents:
                return next_frontier, next_key
            next_frontier.add(next_key)
    return next_frontier, None


def _bidirectional_solve_fixed_corner(canonical_state):
    start_key = _cubie_key(canonical_state)
    if start_key == SOLVED_FIXED_KEY:
        return []

    parents_from_start = {start_key: (None, None)}
    parents_from_goal = {SOLVED_FIXED_KEY: (None, None)}
    frontier_start = {start_key}
    frontier_goal = {SOLVED_FIXED_KEY}
    depth_start = 0
    depth_goal = 0

    while frontier_start and frontier_goal and depth_start + depth_goal < MAX_HTM_DEPTH:
        if len(frontier_start) <= len(frontier_goal):
            frontier_start, meet_key = _expand_frontier(frontier_start, parents_from_start, parents_from_goal)
            depth_start += 1
        else:
            frontier_goal, meet_key = _expand_frontier(frontier_goal, parents_from_goal, parents_from_start)
            depth_goal += 1
        if meet_key is not None:
            return _reconstruct_bidirectional(meet_key, parents_from_start, parents_from_goal)
    return None


def bfs_solve(init_state):
    """Optimal bidirectional BFS solver for the 2x2 cube, fixing the DBL corner."""
    cubies = _state_to_cubies(init_state)
    if cubies is None:
        return None

    canonical_state, canonical_rotation = _canonicalize_fixed_corner(cubies)
    if canonical_state is None:
        return None

    canonical_solution = _bidirectional_solve_fixed_corner(canonical_state)
    if canonical_solution is None:
        return None

    return [_move_to_original(move_name, canonical_rotation) for move_name in canonical_solution]


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
