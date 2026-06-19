#!/usr/bin/env python3
import sys
import os

_SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, _SCRIPT_DIR)

from solver import apply_move, ROBOT_MOVES, SOLVED_STATE, is_solved_monochromatic, print_cube, CUBE_ROTATIONS, _CUBE_ROT_Y

def state_equiv(s1, s2):
    for rot in CUBE_ROTATIONS:
        if apply_move(s1, rot) == s2:
            return True
    return False

# Scramble: ['U2', 'U', 'F2', 'D', 'D_PRIME', 'U', 'L_PRIME', 'U_PRIME', 'R2', 'F2', 'L2', 'B2']
# BFS solution: ['L2', 'B2', 'D', 'L', 'B_PRIME', 'D2', 'B']
# Simulator moves: ['L2', 'F2', 'D', 'R_PRIME', 'B_PRIME', 'U2', 'F_PRIME']

s = SOLVED_STATE
# apply scramble
scramble_seq = ['U2', 'U', 'F2', 'D', 'D_PRIME', 'U', 'L_PRIME', 'U_PRIME', 'R2', 'F2', 'L2', 'B2']
for m in scramble_seq:
    s = apply_move(s, ROBOT_MOVES[m])

s_bfs = s
s_sim = s

bfs_seq = ['L2', 'B2', 'D', 'L', 'B_PRIME', 'D2', 'B']
sim_seq = ['L2', 'F2', 'D', 'R_PRIME', 'B_PRIME', 'U2', 'F_PRIME']

# Let's find y2
y2 = None
for rot in CUBE_ROTATIONS:
    if apply_move(SOLVED_STATE, rot) == apply_move(apply_move(SOLVED_STATE, ROBOT_MOVES['Y2'] if 'Y2' in ROBOT_MOVES else _CUBE_ROT_Y), _CUBE_ROT_Y):
        pass

# Since CUBE_ROTATIONS maps solved state to oriented solved states, let's find the rotation permutation that corresponds to 180 degrees around Y.
# We can find it by looking for the rotation permutation that maps front-face to back-face, left-face to right-face, etc.
for rot in CUBE_ROTATIONS:
    # check if rot maps Left to Right and Front to Back
    test_state = apply_move(SOLVED_STATE, rot)
    # in SOLVED_STATE, Left is 4 (Green), Right is 5 (Blue), Front is 2 (Red), Back is 3 (Orange)
    # We want a rotation that maps Green to Blue, Blue to Green, Red to Orange, Orange to Red, White to White, Yellow to Yellow.
    # index 0-3 is U (White=0), index 4-7 is D (Yellow=1)
    if (test_state[0] == 0 and test_state[4] == 1 and
        test_state[16] == 3 and test_state[8] == 3 and # Front and Back are Orange (3)? Wait, let's check
        test_state[16] == 3): # Left is green, Right is blue
        pass

# Let's just find the rotation permutation rot such that rot(s_sim) == s_bfs at step 3.
s_bfs_3 = s
s_sim_3 = s
for m in ['L2', 'B2', 'D']:
    s_bfs_3 = apply_move(s_bfs_3, ROBOT_MOVES[m])
for m in ['L2', 'F2', 'D']:
    s_sim_3 = apply_move(s_sim_3, ROBOT_MOVES[m])

matching_rot = None
for rot in CUBE_ROTATIONS:
    if apply_move(s_sim_3, rot) == s_bfs_3:
        matching_rot = rot
        break

print("Found matching rotation:", matching_rot is not None)
if matching_rot:
    # Let's find which face each face is mapped to by matching_rot
    # We can apply matching_rot to SOLVED_STATE and print the monochromatic colors of each face.
    # SOLVED_STATE colors: U=white(0), D=yellow(1), F=red(2), B=orange(3), L=green(4), R=blue(5)
    rotated_solved = apply_move(SOLVED_STATE, matching_rot)
    face_names = ['U', 'D', 'F', 'B', 'L', 'R']
    color_names = ['White(U)', 'Yellow(D)', 'Red(F)', 'Orange(B)', 'Green(L)', 'Blue(R)']
    print("Face mappings under matching_rot:")
    for f_idx in range(6):
        face_color = rotated_solved[f_idx*4]
        print(f"  Face {face_names[f_idx]} maps to color of: {color_names[face_color]}")
        
    # Now let's apply L to s_bfs_3
    s_bfs_4 = apply_move(s_bfs_3, ROBOT_MOVES['L'])
    # Let's see which move applied to s_sim_3, and then rotated by matching_rot, gives s_bfs_4
    for move_name in ROBOT_MOVES:
        if apply_move(apply_move(s_sim_3, ROBOT_MOVES[move_name]), matching_rot) == s_bfs_4:
            print(f"L on BFS corresponds to {move_name} on Sim!")



