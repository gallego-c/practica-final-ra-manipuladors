#!/usr/bin/env python3
"""
solver_moves.py - Standalone movements-only solver.
Reads the dynamically generated PDDL problem.pddl, solves it optimally
using BFS, and prints ONLY active robot reorientation and layer turns.
"""
import sys
import re
from pathlib import Path

# Add workspace root to system path for imports
workspace_root = Path(__file__).resolve().parent.parent
if str(workspace_root) not in sys.path:
    sys.path.insert(0, str(workspace_root))

try:
    from robot.solver import bfs_solve, COLOR_IDX, POSITIONS, SOLVED_STATE, print_cube, scramble, is_solved_monochromatic
    SOLVER_AVAILABLE = True
except ImportError:
    SOLVER_AVAILABLE = False

def parse_pddl_state(pddl_file):
    """Parses a PDDL problem file to reconstruct the 24-sticker state tuple."""
    if not pddl_file.exists():
        return None
        
    with open(pddl_file, "r", encoding="utf-8") as f:
        content = f.read()

    # Extract only the (:init ...) block to prevent goal conditions from polluting
    init_match = re.search(r"\(:init(.*?)\(:goal", content, re.DOTALL)
    if not init_match:
        return None
    init_block = init_match.group(1)

    color_at_re = re.compile(r"\(color-at\s+([\w\-]+)\s+(\w+)\)")
    matches = color_at_re.findall(init_block)
    
    if not matches:
        return None
        
    state_dict = {pos: col for pos, col in matches}
    color_map = {
        "white": 0, "yellow": 1, "red": 2, "orange": 3, "blue": 4, "green": 5
    }
    
    state = [0] * 24
    for pos, col in state_dict.items():
        if pos in POSITIONS:
            idx = POSITIONS.index(pos)
            state[idx] = color_map.get(col, 0)
            
    return tuple(state)

def main():
    print("=" * 60)
    print("  STANDALONE CUBE SOLVER (ACTIVE MOVEMENTS ONLY)")
    print("=" * 60)
    
    if not SOLVER_AVAILABLE:
        print("Error: The robot.solver module is not available.")
        sys.exit(1)
        
    pddl_path = workspace_root / "robot" / "problem.pddl"
    state = None
    
    if pddl_path.exists():
        print(f"Loading scrambled state from: {pddl_path.relative_to(workspace_root)}")
        state = parse_pddl_state(pddl_path)
        
    if state is None:
        print("Warning: problem.pddl not found or empty. Generating default scramble.")
        default_scramble = ['tilt_x_pos', 'rotate_top_cw', 'tilt_y_pos', 'rotate_top_ccw']
        state = scramble(default_scramble)
        
    print("\n── Scrambled Cube State ──")
    print_cube(state)
    
    if is_solved_monochromatic(state):
        print("Cube is already solved!")
        sys.exit(0)
        
    print("Solving optimally using BFS...")
    solution = bfs_solve(state)
    
    if solution is None:
        print("Error: No solution found. Check that the color configuration is valid.")
        sys.exit(1)
        
    print(f"\n✓ OPTIMAL SOLUTION ({len(solution)} active movements)")
    print("-" * 60)
    for idx, move in enumerate(solution):
        friendly = move
        if move == "rotate_top_cw": friendly = "Rotate top layer Clockwise (Wrist CW)"
        elif move == "rotate_top_ccw": friendly = "Rotate top layer Counter-Clockwise (Wrist CCW)"
        elif move == "tilt_x_pos": friendly = "Tilt cube Forward (Front to Top)"
        elif move == "tilt_x_neg": friendly = "Tilt cube Backward (Back to Top)"
        elif move == "tilt_y_pos": friendly = "Tilt cube Right (Right to Top)"
        elif move == "tilt_y_neg": friendly = "Tilt cube Left (Left to Top)"
        
        print(f"  Step {idx + 1:2d}: {friendly}")
    print("-" * 60)
    print()

if __name__ == "__main__":
    main()
