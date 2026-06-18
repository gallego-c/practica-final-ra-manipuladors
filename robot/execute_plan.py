#!/usr/bin/env python3
"""
execute_plan.py - Script de ejecución autogenerado para el robot UR3.
Generado el: 2026-06-18 12:37:25
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
    ('pick_x', 'pick_x'),
    ('tilt_x', 'tilt_x_ur3'),
    ('pick_x', 'pick_x'),
    ('execute_l', 'turn_clockwise'),
    ('tilt_y', 'tilt_y_ur3'),
    ('pick_x', 'pick_x'),
    ('execute_f_prime', 'turn_counterclockwise'),
    ('tilt_y', 'tilt_y_ur3')
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
    from go_home import go_home
    print(f"Iniciando ejecución del plan ({len(PLAN_ACTIONS)} pasos)...")
    
    print("\nLlevando el robot a la posición HOME antes de iniciar...")
    go_home(open_gripper=True)
    
    t_start = time.time()
    for i, (action, module) in enumerate(PLAN_ACTIONS):
        print(f"\n[Paso {i+1}/{len(PLAN_ACTIONS)}]")
        run_action(action, module)
        
    print("\nLlevando el robot a la posición HOME tras finalizar...")
    go_home()
        
    print(f"\n✓ ¡Plan completado con éxito en {time.time() - t_start:.2f}s!")

if __name__ == "__main__":
    main()
