#!/usr/bin/env python3
import sys
import os
import random
import time

# Ensure imports work from workspace root
_SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, _SCRIPT_DIR)

from solver import scramble, bfs_solve, ROBOT_MOVES, SOLVED_STATE, apply_move, is_solved_monochromatic, CUBE_ROTATIONS, POSITIONS
from generate_taskfile import generate_manipulation_problem, run_fast_downward

FACES = ['u', 'd', 'f', 'b', 'l', 'r']

def find_tilt_perm(face_map):
    for rot in CUBE_ROTATIONS:
        match = True
        for idx, pos in enumerate(POSITIONS):
            face = pos.split('-')[0]
            dst_pos = POSITIONS[rot[idx]]
            dst_face = dst_pos.split('-')[0]
            if dst_face != face_map[face]:
                match = False
                break
        if match:
            return rot
    return None

M_x = {'u': 'l', 'l': 'd', 'd': 'r', 'r': 'u', 'f': 'f', 'b': 'b'}
M_y = {'u': 'f', 'f': 'd', 'd': 'b', 'b': 'u', 'l': 'l', 'r': 'r'}

TILT_X_PERM = find_tilt_perm(M_x)
TILT_Y_PERM = find_tilt_perm(M_y)


class CubeSimulator:
    def __init__(self, initial_stickers, bfs_solution=None):
        self.stickers = initial_stickers
        # Spatial orientation map: Env Direction -> PDDL Face
        self.orientation = {
            'top': 'face_u',
            'bottom': 'face_d',
            'front': 'face_f',
            'back': 'face_b',
            'left': 'face_l',
            'right': 'face_r'
        }
        self.robot_holding = False
        self.holding_x = False
        self.holding_y = False
        self.cube_on_fixture = True
        self.bfs_solution = bfs_solution if bfs_solution is not None else []
        self.bfs_step_idx = 0

    def rotate_orientation_y_cw(self):
        new_orient = self.orientation.copy()
        new_orient['left'] = self.orientation['front']
        new_orient['back'] = self.orientation['left']
        new_orient['right'] = self.orientation['back']
        new_orient['front'] = self.orientation['right']
        self.orientation = new_orient

    def rotate_orientation_y_ccw(self):
        new_orient = self.orientation.copy()
        new_orient['right'] = self.orientation['front']
        new_orient['back'] = self.orientation['right']
        new_orient['left'] = self.orientation['back']
        new_orient['front'] = self.orientation['left']
        self.orientation = new_orient

    def rotate_orientation_y_180(self):
        new_orient = self.orientation.copy()
        new_orient['back'] = self.orientation['front']
        new_orient['front'] = self.orientation['back']
        new_orient['right'] = self.orientation['left']
        new_orient['left'] = self.orientation['right']
        self.orientation = new_orient

    def rotate_orientation_x_cw(self):
        new_orient = self.orientation.copy()
        new_orient['front'] = self.orientation['top']
        new_orient['bottom'] = self.orientation['front']
        new_orient['back'] = self.orientation['bottom']
        new_orient['top'] = self.orientation['back']
        self.orientation = new_orient

    def rotate_orientation_x_ccw(self):
        new_orient = self.orientation.copy()
        new_orient['back'] = self.orientation['top']
        new_orient['bottom'] = self.orientation['back']
        new_orient['front'] = self.orientation['bottom']
        new_orient['top'] = self.orientation['front']
        self.orientation = new_orient

    def rotate_orientation_x_180(self):
        new_orient = self.orientation.copy()
        new_orient['bottom'] = self.orientation['top']
        new_orient['top'] = self.orientation['bottom']
        new_orient['back'] = self.orientation['front']
        new_orient['front'] = self.orientation['back']
        self.orientation = new_orient

    def rotate_orientation_z_cw(self):
        new_orient = self.orientation.copy()
        new_orient['right'] = self.orientation['top']
        new_orient['bottom'] = self.orientation['right']
        new_orient['left'] = self.orientation['bottom']
        new_orient['top'] = self.orientation['left']
        self.orientation = new_orient

    def rotate_orientation_z_ccw(self):
        new_orient = self.orientation.copy()
        new_orient['left'] = self.orientation['top']
        new_orient['bottom'] = self.orientation['left']
        new_orient['right'] = self.orientation['bottom']
        new_orient['top'] = self.orientation['right']
        self.orientation = new_orient

    def rotate_orientation_z_180(self):
        new_orient = self.orientation.copy()
        new_orient['bottom'] = self.orientation['top']
        new_orient['top'] = self.orientation['bottom']
        new_orient['right'] = self.orientation['left']
        new_orient['left'] = self.orientation['right']
        self.orientation = new_orient

    def execute_action(self, action):
        action = action.lower().strip()
        
        if action == 'pick_x':
            self.robot_holding = True
            self.holding_x = True
            self.cube_on_fixture = False
        elif action == 'pick_y':
            self.robot_holding = True
            self.holding_y = True
            self.cube_on_fixture = False
        elif action == 'place':
            self.robot_holding = False
            self.holding_x = False
            self.holding_y = False
            self.cube_on_fixture = True
        elif action == 'change_pick':
            self.holding_x, self.holding_y = self.holding_y, self.holding_x
        elif action == 'tilt_x':
            # Left -> Top, Top -> Right, Right -> Bottom, Bottom -> Left
            new_orient = self.orientation.copy()
            new_orient['top'] = self.orientation['left']
            new_orient['right'] = self.orientation['top']
            new_orient['bottom'] = self.orientation['right']
            new_orient['left'] = self.orientation['bottom']
            self.orientation = new_orient
            self.stickers = apply_move(self.stickers, TILT_X_PERM)
            self.robot_holding = False
            self.holding_x = False
            self.cube_on_fixture = True
        elif action == 'tilt_y':
            # Front -> Top, Top -> Back, Back -> Bottom, Bottom -> Front
            new_orient = self.orientation.copy()
            new_orient['top'] = self.orientation['front']
            new_orient['back'] = self.orientation['top']
            new_orient['bottom'] = self.orientation['back']
            new_orient['front'] = self.orientation['bottom']
            self.orientation = new_orient
            self.stickers = apply_move(self.stickers, TILT_Y_PERM)
            self.robot_holding = False
            self.holding_y = False
            self.cube_on_fixture = True
            
        elif action.startswith('execute_'):
            # Parse standard action name e.g. execute_u, execute_d_prime, execute_r2
            clean = action.replace('execute_', '')
            move_name = clean.upper()
            
            # The robot always turns the physical top layer.
            # So we apply the physical top layer turn (U/U_PRIME/U2) to the stickers.
            if 'PRIME' in move_name or '-PRIME' in move_name or '\'' in move_name:
                self.stickers = apply_move(self.stickers, ROBOT_MOVES['U_PRIME'])
            elif '2' in move_name:
                self.stickers = apply_move(self.stickers, ROBOT_MOVES['U2'])
            else:
                self.stickers = apply_move(self.stickers, ROBOT_MOVES['U'])
            
            # If we have BFS solution steps, track and apply any coordinate rotation
            # that occurs when a turn is executed as an opposite turn.
            if self.bfs_step_idx < len(self.bfs_solution):
                bfs_move = self.bfs_solution[self.bfs_step_idx].upper().replace('\'', '_PRIME').replace('-', '_')
                self.bfs_step_idx += 1
                
                top_phys = self.orientation['top']
                
                # A turn is an opposite turn if the target face is at the bottom of the fixture.
                # In top-only execution, if the top_phys face is not the face of the logical move,
                # then the target face of the logical move is at the bottom.
                is_opposite = False
                logical_face = bfs_move[0].lower() # u, d, l, r, f, b
                if top_phys == 'face_u' and logical_face == 'd': is_opposite = True
                elif top_phys == 'face_d' and logical_face == 'u': is_opposite = True
                elif top_phys == 'face_l' and logical_face == 'r': is_opposite = True
                elif top_phys == 'face_r' and logical_face == 'l': is_opposite = True
                elif top_phys == 'face_f' and logical_face == 'b': is_opposite = True
                elif top_phys == 'face_b' and logical_face == 'f': is_opposite = True
                
                if is_opposite:
                    if 'PRIME' in bfs_move or '_PRIME' in bfs_move or '\'' in bfs_move:
                        self.rotate_orientation_y_ccw()
                    elif '2' in bfs_move:
                        self.rotate_orientation_y_180()
                    else:
                        self.rotate_orientation_y_cw()

def simulate_plan(scramble_seq, plan, bfs_solution=None):
    stickers = scramble(scramble_seq)
    sim = CubeSimulator(stickers, bfs_solution)
    for act in plan:
        sim.execute_action(act)
    return is_solved_monochromatic(sim.stickers)

def run_tests(num_tests=50):
    valid_moves = list(ROBOT_MOVES.keys())
    # Filter out PRIME and 2 moves for scramble to keep it simple but diverse
    base_moves = [m for m in valid_moves if not m.endswith('_PRIME') and not m.endswith('2')]
    
    success_count = 0
    failure_details = []
    
    print(f"Starting {num_tests} random TAMP validation tests...")
    
    for i in range(num_tests):
        # Generate random 12-move scramble
        scramble_seq = [random.choice(base_moves) for _ in range(12)]
        # Add random primes or doubles to make it interesting
        scramble_seq = [m + random.choice(['', '_PRIME', '2']) for m in scramble_seq]
        
        # Level 1 solve
        init_state = scramble(scramble_seq)
        solution = bfs_solve(init_state)
        if solution is None:
            print(f"Test {i+1}: BFS Solver failed to solve scramble: {scramble_seq}")
            continue
            
        # Level 2 solve
        prob_file = os.path.join(_SCRIPT_DIR, "manipulation_problem_test.pddl")
        dom_file = os.path.join(_SCRIPT_DIR, "manipulation_domain.pddl")
        generate_manipulation_problem(solution, filename=prob_file)
        
        plan = run_fast_downward(domain_path=dom_file, problem_path=prob_file)
        if os.path.exists(prob_file):
            os.remove(prob_file)
            
        if plan is None:
            print(f"Test {i+1}: Fast Downward failed to plan scramble: {scramble_seq}")
            failure_details.append((scramble_seq, solution, "PDDL Planning failed"))
            continue
            
        # Run simulation
        is_solved = simulate_plan(scramble_seq, plan, solution)
        if is_solved:
            success_count += 1
        else:
            print(f"Test {i+1}: FAILURE!")
            print(f"  Scramble: {scramble_seq}")
            print(f"  BFS Solution: {solution}")
            print(f"  PDDL Plan: {plan}")
            failure_details.append((scramble_seq, solution, plan))
            
    print("\n" + "="*50)
    print(f"Validation completed: {success_count}/{num_tests} tests PASSED.")
    print("="*50)
    
    return success_count == num_tests

if __name__ == "__main__":
    run_tests(30)
