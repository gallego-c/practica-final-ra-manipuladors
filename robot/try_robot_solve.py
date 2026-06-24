#!/usr/bin/env python3
"""
try_robot_solve.py — Resuelve el cubo de Rubik 2x2 en dos niveles simbólicos (TAMP Jerárquico).
1. BFS: Calcula la solución óptima de movimientos abstractos del cubo.
2. Fast Downward: Calcula los movimientos físicos de alto nivel del robot (pick, place, rotate, tilt).

Este script funciona de forma 100% independiente, sin requerir ROS 2 ni Kautham activo.
"""

import sys
import os
import time
import random

# Ajustar paths
_SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, _SCRIPT_DIR)

from solver import scramble, bfs_solve, ROBOT_MOVES, SOLVED_STATE
from generate_taskfile import generate_manipulation_problem, run_fast_downward

def main():
    print("=" * 70)
    print("  TRY ROBOT SOLVE — PLANIFICACIÓN SIMBÓLICA TAMP COMPLETA")
    print("=" * 70)
    
    # 1. Generar mezcla aleatoria (ej: 4 movimientos)
    # Movimientos posibles
    valid_moves = list(ROBOT_MOVES.keys())
    scramble_seq = []
    
    # Asegurar reproducibilidad de mezcla pero variada
    random.seed(time.time())
    for _ in range(4):
        scramble_seq.append(random.choice(valid_moves))
        
    print(f"\n[+] Mezcla Scramble aleatoria: {' '.join(scramble_seq)}")

    # 2. [Nivel 1] Resolver cubo con BFS
    init_state = scramble(scramble_seq)
    
    t0_bfs = time.time()
    solution = bfs_solve(init_state)
    t1_bfs = time.time()
    
    if solution is None:
        print("Error: No se encontró solución para el cubo.")
        sys.exit(1)
        
    print(f"\n[Nivel 1] Solución BFS del Cubo ({len(solution)} movimientos abstractos) [{t1_bfs - t0_bfs:.4f}s]:")
    for i, action in enumerate(solution):
        print(f"  Movimiento {i+1:2d}: {action}")

    # 3. [Nivel 2] Plan simbólico con Fast Downward (Acciones físicas del Robot)
    print("\n[+] [Nivel 2] Planificando tareas físicas con Fast Downward...")
    
    prob_file = os.path.join(_SCRIPT_DIR, "manipulation_problem.pddl")
    dom_file = os.path.join(_SCRIPT_DIR, "manipulation_domain.pddl")
    
    generate_manipulation_problem(solution, filename=prob_file)
    
    t0_fd = time.time()
    manipulation_plan = run_fast_downward(domain_path=dom_file, problem_path=prob_file)
    t1_fd = time.time()

    print("\n" + "=" * 70)
    if manipulation_plan is not None:
        print(f"Plan de manipulación robótica simbólica [{t1_fd - t0_fd:.4f}s]:")
        for i, act in enumerate(manipulation_plan):
            print(f"  Acción física {i+1:2d}: {act}")
            
        # Escribir el plan de texto resultante para que sea importable
        plan_output_path = os.path.join(_SCRIPT_DIR, "robot_plan.txt")
        with open(plan_output_path, "w") as f:
            for act in manipulation_plan:
                f.write(f"{act}\n")
        print("\n" + "=" * 70)
        print(f"Fichero de plan de alto nivel exportado a: {plan_output_path}")
        
        # Generar automáticamente el orquestrador ejecutable en Python
        import orchestrator
        orchestrator.generate_execution_script(manipulation_plan)
    else:
        print("Error: Fast Downward falló en la planificación física.")
        sys.exit(1)
    print("=" * 70)

if __name__ == "__main__":
    main()
