#!/usr/bin/env python3
"""
run_experiments.py — Ejecuta pruebas con 20 mezclas (scrambles) aleatorias del cubo de Rubik 2x2.
Mide tiempos de BFS (Nivel 1), tiempos de Fast Downward (Nivel 2) y cuenta los movimientos abstractos y físicos.
Guarda los resultados en experiments/results.csv y muestra un reporte formateado.
"""

import sys
import os
import time
import random
import csv
import subprocess

# Ajustar paths de imports
_SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
_REPO_ROOT = os.path.dirname(_SCRIPT_DIR)
sys.path.insert(0, os.path.join(_REPO_ROOT, "robot"))

from solver import scramble, bfs_solve, ROBOT_MOVES, SOLVED_STATE
from generate_taskfile import generate_manipulation_problem, run_fast_downward

# Configuración de Salida
CSV_OUTPUT_PATH = os.path.join(_SCRIPT_DIR, "results.csv")

# Lista de movimientos posibles
VALID_MOVES = list(ROBOT_MOVES.keys())

def generate_random_scramble(depth):
    """Genera una secuencia aleatoria de movimientos sin cancelaciones inmediatas."""
    moves = []
    opposite_moves = {
        'U': 'U_PRIME', 'U_PRIME': 'U', 'U2': 'U2',
        'D': 'D_PRIME', 'D_PRIME': 'D', 'D2': 'D2',
        'R': 'R_PRIME', 'R_PRIME': 'R', 'R2': 'R2',
        'L': 'L_PRIME', 'L_PRIME': 'L', 'L2': 'L2',
        'F': 'F_PRIME', 'F_PRIME': 'F', 'F2': 'F2',
        'B': 'B_PRIME', 'B_PRIME': 'B', 'B2': 'B2',
    }
    
    last_move = None
    while len(moves) < depth:
        m = random.choice(VALID_MOVES)
        # Evitar cancelar el último movimiento inmediatamente
        if last_move and opposite_moves.get(last_move) == m:
            continue
        moves.append(m)
        last_move = m
    return moves

def main():
    print("=" * 70)
    print("  UR3 Rubik 2x2 TAMP — EJECUTANDO EXPERIMENTOS (20 Scrambles)")
    print("=" * 70)
    
    results = []
    
    # Asegurar que el directorio de salida existe
    os.makedirs(_SCRIPT_DIR, exist_ok=True)
    
    # Generar 20 scrambles de profundidades variables (entre 2 y 8 movimientos)
    random.seed(42)  # Semilla fija para reproducibilidad
    scramble_depths = [2, 2, 3, 3, 4, 4, 4, 5, 5, 5, 6, 6, 6, 6, 7, 7, 7, 8, 8, 8]
    
    print(f"\n[+] Generando y ejecutando {len(scramble_depths)} experimentos simbólicos...")
    
    for idx, depth in enumerate(scramble_depths):
        scramble_seq = generate_random_scramble(depth)
        scramble_str = " ".join(scramble_seq)
        
        # 1. Nivel de Tarea Abstracta (Python BFS)
        init_state = scramble(scramble_seq)
        
        t0_bfs = time.time()
        bfs_solution = bfs_solve(init_state)
        t1_bfs = time.time()
        
        bfs_time = t1_bfs - t0_bfs
        num_bfs_moves = len(bfs_solution) if bfs_solution is not None else 0
        
        # 2. Nivel Simbólico de Manipulación (PDDL + Fast Downward)
        fd_time = 0.0
        num_fd_moves = 0
        fd_status = "FAILED"
        
        if bfs_solution is not None:
            # Generar problema PDDL simplificado de manipulación
            prob_file = os.path.join(_REPO_ROOT, "robot", "manipulation_problem.pddl")
            generate_manipulation_problem(bfs_solution, filename=prob_file)
            
            t0_fd = time.time()
            fd_plan = run_fast_downward(
                domain_path=os.path.join(_REPO_ROOT, "robot", "manipulation_domain.pddl"),
                problem_path=prob_file
            )
            t1_fd = time.time()
            
            fd_time = t1_fd - t0_fd
            if fd_plan is not None:
                num_fd_moves = len(fd_plan)
                fd_status = "SUCCESS"
            else:
                fd_status = "FAILED"
                
        # Guardar métricas
        results.append({
            "ID": idx + 1,
            "Profundidad Scramble": depth,
            "Secuencia Mezcla": scramble_str,
            "BFS Tiempo (s)": f"{bfs_time:.4f}",
            "BFS Movs Cubo": num_bfs_moves,
            "Downward Tiempo (s)": f"{fd_time:.4f}",
            "Downward Movs Robot": num_fd_moves,
            "Estado Downward": fd_status
        })
        
        print(f"Exp {idx+1:2d}/20 | Prof: {depth} | BFS Movs: {num_bfs_moves} ({bfs_time:.3f}s) | FD Movs: {num_fd_moves} ({fd_time:.3f}s)")

    # Escribir a CSV
    headers = [
        "ID", "Profundidad Scramble", "Secuencia Mezcla", 
        "BFS Tiempo (s)", "BFS Movs Cubo", 
        "Downward Tiempo (s)", "Downward Movs Robot", "Estado Downward"
    ]
    
    with open(CSV_OUTPUT_PATH, mode="w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=headers)
        writer.writeheader()
        writer.writerows(results)
        
    print("\n" + "=" * 70)
    print(f"Experimentos completados con éxito!")
    print(f"Fichero CSV guardado en: {CSV_OUTPUT_PATH}")
    print("=" * 70)
    
    # Mostrar resumen en consola
    print("\nResumen Promedios:")
    total_bfs_time = sum(float(r["BFS Tiempo (s)"]) for r in results)
    total_fd_time = sum(float(r["Downward Tiempo (s)"]) for r in results)
    total_bfs_moves = sum(r["BFS Movs Cubo"] for r in results)
    total_fd_moves = sum(r["Downward Movs Robot"] for r in results)
    
    n = len(results)
    print(f"  • Tiempo promedio BFS Cubo: {total_bfs_time/n:.4f} segundos")
    print(f"  • Tiempo promedio Fast Downward: {total_fd_time/n:.4f} segundos")
    print(f"  • Movimientos promedio del Cubo: {total_bfs_moves/n:.1f}")
    print(f"  • Movimientos promedio del Robot (acciones físicas): {total_fd_moves/n:.1f}")
    print()

if __name__ == "__main__":
    main()
