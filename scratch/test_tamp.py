#!/usr/bin/env python3
import sys
import os

# Incorporar el path del robot
_SCRIPT_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(_SCRIPT_DIR, "robot"))

from solver import scramble, bfs_solve, SOLVED_STATE
from generate_taskfile import generate_manipulation_problem, run_fast_downward

def main():
    print("=" * 60)
    print("  VERIFICADOR DE ARQUITECTURA TAMP DE TAREAS SIMBÓLICAS")
    print("=" * 60)
    
    # 1. Definir scramble de prueba
    scramble_seq = ['tilt_x_pos', 'rotate_top_cw', 'tilt_y_pos', 'rotate_top_ccw']
    print(f"\nScramble de prueba: {scramble_seq}")
    
    # 2. BFS Nivel 1
    init_state = scramble(scramble_seq)
    print("\n[Nivel 1] Buscando solución abstracta óptima con BFS...")
    solution = bfs_solve(init_state)
    print(f"✓ Solución del cubo encontrada: {solution}")
    
    # 3. Fast Downward Nivel 2
    print("\n[Nivel 2] Generando problema PDDL y planificando con Fast Downward...")
    generate_manipulation_problem(solution)
    manipulation_plan = run_fast_downward()
    
    print("\n" + "=" * 60)
    if manipulation_plan:
        print("✓ ¡ÉXITO! Fast Downward resolvió la tarea simbólica:")
        for i, act in enumerate(manipulation_plan):
            print(f"  Acción {i+1:2d}: {act}")
    else:
        print("❌ ERROR: La planificación con Fast Downward falló.")
    print("=" * 60)

if __name__ == "__main__":
    main()
