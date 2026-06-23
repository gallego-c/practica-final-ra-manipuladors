#!/usr/bin/env python3
"""
execute_plan.py - Script de ejecución autogenerado para el robot UR3.
Generado el: 2026-06-23 11:46:22
"""
import sys
import os
import time
import importlib

# Configurar el path para incluir la carpeta 'scripts'
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
SCRIPTS_PATH = os.path.join(os.path.dirname(SCRIPT_DIR), 'scripts')
if SCRIPTS_PATH not in sys.path:
    sys.path.insert(0, SCRIPTS_PATH)

# Lista secuencial de acciones generada del plan PDDL
PLAN_ACTIONS = [
    ('place', 'place'),
    ('pick_y', 'pick_y'),
    ('tilt_y', 'tilt_y_ur3'),
    ('pick_x', 'pick_x'),
    ('execute_f', 'turn_clockwise'),
    ('tilt_y', 'tilt_y_ur3'),
    ('pick_y', 'pick_y'),
    ('execute_r', 'turn_clockwise'),
    ('tilt_x', 'tilt_x_ur3'),
    ('pick_y', 'pick_y'),
    ('execute_b', 'turn_clockwise'),
    ('tilt_x', 'tilt_x_ur3'),
    ('pick_x', 'pick_x'),
    ('execute_l_prime', 'turn_counterclockwise'),
    ('tilt_y', 'tilt_y_ur3'),
    ('pick_y', 'pick_y'),
    ('execute_u_prime', 'turn_counterclockwise'),
    ('tilt_x', 'tilt_x_ur3'),
    ('pick_y', 'pick_y'),
    ('execute_l', 'turn_clockwise'),
    ('tilt_x', 'tilt_x_ur3'),
    ('pick_x', 'pick_x'),
    ('execute_d', 'turn_clockwise'),
    ('tilt_y', 'tilt_y_ur3'),
    ('pick_x', 'pick_x'),
    ('execute_f_prime', 'turn_counterclockwise'),
    ('place', 'place')
]

def run_action(action_name, module_name):
    print("\n" + "=" * 60)
    print(f"Ejecutando: {action_name} (modulo: {module_name}.py)")
    print("=" * 60)
    t0 = time.time()
    try:
        if module_name in sys.modules:
            importlib.reload(sys.modules[module_name])
        else:
            importlib.import_module(module_name)
        print(f"✓ {action_name} finalizado con éxito en {time.time() - t0:.2f}s.")
    except Exception as e:
        print(f"❌ Error al ejecutar {action_name}: {e}")
        raise e

def main():
    print(f"Iniciando ejecución del plan ({len(PLAN_ACTIONS)} pasos)...")
    
    t_start = time.time()
    for i, (action, module) in enumerate(PLAN_ACTIONS):
        print(f"\n[Paso {i+1}/{len(PLAN_ACTIONS)}]")
        run_action(action, module)
        
    print(f"\n✓ ¡Plan completado con éxito en {time.time() - t_start:.2f}s!")

if __name__ == "__main__":
    main()
