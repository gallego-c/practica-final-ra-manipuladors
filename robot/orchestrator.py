#!/usr/bin/env python3
import os
import sys
import time
import subprocess

# ---------------------------------------------------------------------------
# Mapeo de Acciones PDDL -> Nombres de Módulos Python en 'scripts/'
# ---------------------------------------------------------------------------
ACTION_MAPPING = {
    'pick_x': 'pick_x',
    'pick_y': 'pick_y',
    'place': 'place',
    'change_pick': 'turn_90_regrasp',
    'tilt_x': 'tilt_x_ur3',
    'tilt_y': 'tilt_y_ur3',
    'tilt_x_pos': 'tilt_x_ur3',
    'tilt_x_neg': 'tilt_x_ur3',
    'tilt_y_pos': 'tilt_y_ur3',
    'tilt_y_neg': 'tilt_y_ur3',
}

def get_script_for_action(action):
    """Determina qué script de la carpeta 'scripts/' corresponde a la acción PDDL."""
    act_lower = action.lower().strip()
    if act_lower in ACTION_MAPPING:
        return ACTION_MAPPING[act_lower]
        
    if act_lower.startswith('execute_'):
        # Rotaciones de capa (wrist_3)
        if act_lower.endswith('2'):
            return 'turn_180'
        elif act_lower.endswith('_prime') or act_lower.endswith('-prime') or act_lower.endswith('\''):
            return 'turn_counterclockwise'
        else:
            return 'turn_clockwise'
            
    return None

def generate_execution_script(manipulation_plan, output_file=None):
    """Genera un archivo Python ejecutable con la secuencia de imports correspondiente."""
    if output_file is None:
        robot_dir = os.path.dirname(os.path.abspath(__file__))
        output_file = os.path.join(robot_dir, 'execute_plan.py')
        
    # Construir la lista de tuplas (acción_pddl, nombre_modulo)
    steps = [('place', 'place')]
    for action in manipulation_plan:
        module_name = get_script_for_action(action)
        if module_name is None:
            print(f"[Orchestrator] Warning: Acción PDDL '{action}' no tiene script asignado. Saltando.")
            continue
        # Si el plan ya empieza por place, no lo duplicamos
        if len(steps) == 1 and module_name == 'place':
            continue
        steps.append((action, module_name))
        
    # Añadir un place final de forma fija
    if not steps or steps[-1][1] != 'place':
        steps.append(('place', 'place'))
        
    # Plantilla del código generado
    timestamp = time.strftime('%Y-%m-%d %H:%M:%S')
    steps_repr = "[\n" + ",\n".join(f"    ({repr(act)}, {repr(mod)})" for act, mod in steps) + "\n]"
    
    code = '''#!/usr/bin/env python3
"""
execute_plan.py - Script de ejecución autogenerado para el robot UR3.
Generado el: {TIMESTAMP}
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
PLAN_ACTIONS = {STEPS}

def run_action(action_name, module_name):
    print("\\n" + "=" * 60)
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
        print(f"\\n[Paso {i+1}/{len(PLAN_ACTIONS)}]")
        run_action(action, module)
        
    print(f"\\n✓ ¡Plan completado con éxito en {time.time() - t_start:.2f}s!")

if __name__ == "__main__":
    main()
'''
    code = code.replace('{TIMESTAMP}', timestamp).replace('{STEPS}', steps_repr)
    with open(output_file, 'w', encoding='utf-8') as f:
        f.write(code)
        
    # Hacer el archivo ejecutable (chmod +x)
    try:
        os.chmod(output_file, 0o755)
    except OSError:
        pass
        
    print(f"[Orchestrator] Script de ejecución generado con éxito en: {output_file}")
    return output_file

def run_plan_script_iter(script_path):
    """Ejecuta el script de plan en un subproceso y rinde su salida línea a línea en tiempo real."""
    cmd = [sys.executable, script_path]
    process = subprocess.Popen(
        cmd,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        bufsize=1
    )
    for line in process.stdout:
        yield line
    process.wait()
    if process.returncode != 0:
        raise subprocess.CalledProcessError(process.returncode, cmd)
