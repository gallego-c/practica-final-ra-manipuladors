#!/usr/bin/env python3
"""
generate_taskfile.py — Genera el taskfile XML de Kautham para el proyecto UR3 Rubik 2×2.

Utiliza la interfaz ROS 2 de Kautham (kautham_ros) para planificar cada tramo del movimiento
usando RRT-Connect, asegurando trayectorias libres de colisiones con la mesa y el suelo.

Uso:
    python3 robot/generate_taskfile.py
    python3 robot/generate_taskfile.py tilt_x_pos rotate_top_cw tilt_y_pos rotate_top_ccw

El fichero de salida se escribe en: kautham/taskfile_rubik_ur3.xml
"""

import sys
import os
import xml.etree.ElementTree as ET
from xml.dom import minidom
import time

# ── Importar el solver para acceder a las funciones BFS ───────────────────────
_SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, _SCRIPT_DIR)
from solver import scramble, bfs_solve, ROBOT_MOVES, SOLVED_STATE

# ── Importar rclpy y la interfaz ROS 2 de Kautham ─────────────────────────────
import rclpy
from rclpy.node import Node
from kautham_ros_interfaces.srv import OpenProblem

# Añadir el path local del módulo kautham_ros
_KAUTHAM_ROS_PATH = "/home/barrendeiro/robotica/ws_tamp/src/task_and_motion_planning2/kautham_ros/kautham_ros/kautham_ros"
if _KAUTHAM_ROS_PATH not in sys.path:
    sys.path.append(_KAUTHAM_ROS_PATH)

import kautham_ros_interface_python as kautham

# ── Ruta de salida ────────────────────────────────────────────────────────────
_REPO_ROOT = os.path.dirname(_SCRIPT_DIR)
OUTPUT_FILE = os.path.join(_REPO_ROOT, "kautham", "taskfile_rubik_ur3.xml")

# ═══════════════════════════════════════════════════════════════════════════════
# CONFIGURACIONES CLAVE DEL ROBOT (espacio de control Kautham, normalizado [0,1])
# ═══════════════════════════════════════════════════════════════════════════════

# Gripper normalizado: 0.5 = abierto, 0.680 = cerrado sobre el cubo de 5 cm
GRIPPER_OPEN   = 0.500
GRIPPER_CLOSED = 0.680

# Posición Home
HOME = [0.500, 0.375, 0.500, 0.375, 0.500, 0.500, GRIPPER_OPEN]

# Posición de Grasp (sobre el cubo)
GRASP_OPEN = [0.545, 0.380, 0.835, 0.327, 0.375, 0.425, GRIPPER_OPEN]
GRASP      = [0.545, 0.380, 0.835, 0.327, 0.375, 0.425, GRIPPER_CLOSED]

# Posición elevada (Lift)
LIFT = [0.545, 0.300, 0.650, 0.280, 0.375, 0.425, GRIPPER_CLOSED]

# Picos de inclinación (Tilt Peak)
TILT_X_POS_PEAK = [0.545, 0.220, 0.750, 0.180, 0.375, 0.425, GRIPPER_CLOSED]
TILT_X_NEG_PEAK = [0.545, 0.440, 0.600, 0.430, 0.375, 0.425, GRIPPER_CLOSED]
TILT_Y_POS_PEAK = [0.620, 0.300, 0.700, 0.280, 0.400, 0.425, GRIPPER_CLOSED]
TILT_Y_NEG_PEAK = [0.470, 0.300, 0.700, 0.280, 0.350, 0.425, GRIPPER_CLOSED]

# Picos de rotación de muñeca (Rotate Peak)
ROTATE_CW_PEAK  = [0.545, 0.380, 0.835, 0.327, 0.375, 0.675, GRIPPER_CLOSED]
ROTATE_CCW_PEAK = [0.545, 0.380, 0.835, 0.327, 0.375, 0.175, GRIPPER_CLOSED]

# Poses auxiliares con pinza abierta para reinicios de giro
ROTATE_CW_PEAK_OPEN  = [0.545, 0.380, 0.835, 0.327, 0.375, 0.675, GRIPPER_OPEN]
ROTATE_CCW_PEAK_OPEN = [0.545, 0.380, 0.835, 0.327, 0.375, 0.175, GRIPPER_OPEN]

# Pose inicial del cubo en el simulador (X Y Z WX WY WZ TH)
CUBE_INITIAL_POSE = "0.0 0.4 0.046 0.0 0.0 0.0 1.0"

# ═══════════════════════════════════════════════════════════════════════════════
# GENERADOR TAMP DE TASKFILE
# ═══════════════════════════════════════════════════════════════════════════════

class RRTTaskfileGenerator(Node):
    def __init__(self):
        super().__init__("rrt_taskfile_generator")
        
        # 1. Abrir escena del problema en Kautham
        kautham_problem = os.path.join(_REPO_ROOT, "kautham", "ur3_rubik_kautham.xml")
        model_folders = [
            "/usr/share/kautham/demos/models/",
            "/home/barrendeiro/robotica/cub/kautham/",
            "/usr/share/kautham/demos/OMPL_geometric_demos/chess_ur3_robotiq/"
        ]
        
        self.get_logger().info("Conectando con Kautham ROS 2...")
        if not self.open_kautham_problem(model_folders, kautham_problem):
            self.get_logger().error("No se pudo abrir el problema en Kautham.")
            sys.exit(1)
        
        # Establecer el planificador OMPL RRT-Connect
        kautham.kSetPlannerByName(self, "omplRRTConnect")
        kautham.kSetPlannerParameter(self, "_Incremental (0/1)", "0")
        kautham.kSetPlannerParameter(self, "_Max Planning Time", "10")

    def open_kautham_problem(self, model_folders, problem_file):
        """Llama al servicio OpenProblem directamente con múltiples carpetas de búsqueda."""
        kthopenproblem_client = self.create_client(OpenProblem, '/kautham_node/OpenProblem')

        while not kthopenproblem_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('OpenProblem service not available, waiting again...')

        req = OpenProblem.Request()
        req.problem = problem_file
        req.dir = model_folders
        future = kthopenproblem_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)

        result = future.result()
        if result and result.response is True:
            self.get_logger().info("Kautham Problem opened correctly")
            return True
        else:
            self.get_logger().info("ERROR Opening Kautham Problem")
            self.get_logger().info(f"models folders: {req.dir}")
            self.get_logger().info(f"problem file: {req.problem}")
            return False

    def plan_trajectory(self, start, goal):
        """Llama a Kautham para resolver una query de RRT-Connect y devuelve el camino."""
        self.get_logger().info(f"Planificando trayecto RRT de {start} a {goal}...")
        kautham.kSetQuery(self, start, goal)
        if kautham.kSolve(self):
            path = kautham.kGetPath(self, False, 0)
            if path:
                self.get_logger().info("¡Trayectoria RRT encontrada con éxito!")
                return path
        self.get_logger().error("RRT falló al encontrar trayectoria.")
        return None

    def write_path_to_xml(self, parent_elem, path):
        """Escribe las configuraciones densas del path RRT en el elemento XML."""
        k = sorted(list(path.keys()))[-1][1] + 1
        p = sorted(list(path.keys()))[-1][0] + 1
        for i in range(p):
            tex = " " + " ".join(f"{path[i, j]:.6f}" for j in range(k)) + " "
            c = ET.SubElement(parent_elem, "Conf")
            c.text = tex

    def run(self, solution):
        self.get_logger().info(f"Generando taskfile RRT para {len(solution)} acciones...")
        
        # Estructura raíz XML
        root = ET.Element("Task", name="UR3_Rubik_RRT_Solver")
        
        # Estado inicial
        init = ET.SubElement(root, "Initialstate")
        obj = ET.SubElement(init, "Object", object="rubik_cube")
        obj.text = f" {CUBE_INITIAL_POSE} "

        # ── 1. Primer Transit: HOME -> GRASP_OPEN (Acercamiento inicial) ───────────────────
        transit_elem = ET.SubElement(root, "Transit")
        path = self.plan_trajectory(HOME, GRASP_OPEN)
        if not path:
            self.get_logger().error("Fallo crítico en el tránsito inicial.")
            return False
        self.write_path_to_xml(transit_elem, path)

        # Grasp: Cerrar pinza sobre el cubo en simulación
        kautham.kMoveRobot(self, GRASP_OPEN)
        kautham.kAttachObject(self, "ur3_right", "robotiq_85_base_link", "rubik_cube")
        
        # Guardar la pose de agarre actual
        current_grasp = list(GRASP)

        # ── 2. Resolver y planificar cada acción ─────────────────────────────────────
        for idx, action in enumerate(solution):
            self.get_logger().info(f"── PLANIFICANDO PASO {idx+1}/{len(solution)}: {action} ──")
            
            if action in ('rotate_top_cw', 'rotate_top_ccw'):
                # Acción de rotación:
                # 1. Girar la muñeca con el cubo sujeto (Transfer)
                peak_closed = list(current_grasp)
                peak_open = list(current_grasp)
                
                if action == 'rotate_top_cw':
                    peak_closed[5] = ROTATE_CW_PEAK[5]
                    peak_open[5] = ROTATE_CW_PEAK_OPEN[5]
                else:
                    peak_closed[5] = ROTATE_CCW_PEAK[5]
                    peak_open[5] = ROTATE_CCW_PEAK_OPEN[5]

                # Transfer de rotación
                transfer_elem = ET.SubElement(root, "Transfer",
                                             attrib={
                                                 "object": "rubik_cube",
                                                 "robot": "ur3_right",
                                                 "link": "robotiq_85_base_link"
                                             })
                path = self.plan_trajectory(current_grasp, peak_closed)
                if not path:
                    return False
                self.write_path_to_xml(transfer_elem, path)

                # 2. Soltar el cubo para reiniciar la muñeca (Transit)
                kautham.kMoveRobot(self, peak_closed)
                kautham.kDetachObject(self, "rubik_cube")
                
                # Tránsito de regreso con la pinza abierta para no desarmar el cubo
                transit_elem = ET.SubElement(root, "Transit")
                # De peak_open de vuelta a GRASP_OPEN
                path = self.plan_trajectory(peak_open, GRASP_OPEN)
                if not path:
                    return False
                self.write_path_to_xml(transit_elem, path)

                # Volver a sujetar el cubo
                kautham.kMoveRobot(self, GRASP_OPEN)
                kautham.kAttachObject(self, "ur3_right", "robotiq_85_base_link", "rubik_cube")
                current_grasp = list(GRASP)

            elif action in ('tilt_x_pos', 'tilt_x_neg', 'tilt_y_pos', 'tilt_y_neg'):
                # Acción de inclinación (Tilt):
                # 1. Lift: Grasp -> LIFT (Transfer)
                transfer_elem = ET.SubElement(root, "Transfer",
                                             attrib={
                                                 "object": "rubik_cube",
                                                 "robot": "ur3_right",
                                                 "link": "robotiq_85_base_link"
                                             })
                path = self.plan_trajectory(current_grasp, LIFT)
                if not path:
                    return False
                self.write_path_to_xml(transfer_elem, path)

                # 2. Tilt Peak: LIFT -> TILT_PEAK (Transfer)
                if action == 'tilt_x_pos':
                    peak = TILT_X_POS_PEAK
                elif action == 'tilt_x_neg':
                    peak = TILT_X_NEG_PEAK
                elif action == 'tilt_y_pos':
                    peak = TILT_Y_POS_PEAK
                else:
                    peak = TILT_Y_NEG_PEAK

                path = self.plan_trajectory(LIFT, peak)
                if not path:
                    return False
                self.write_path_to_xml(transfer_elem, path)

                # 3. Lift after tilt: TILT_PEAK -> LIFT (Transfer)
                path = self.plan_trajectory(peak, LIFT)
                if not path:
                    return False
                self.write_path_to_xml(transfer_elem, path)

                # 4. Place onto fixture: LIFT -> GRASP (Transfer)
                path = self.plan_trajectory(LIFT, GRASP)
                if not path:
                    return False
                self.write_path_to_xml(transfer_elem, path)

                # 5. Let go of the cube (Transit: GRASP -> GRASP_OPEN -> HOME)
                kautham.kMoveRobot(self, GRASP)
                kautham.kDetachObject(self, "rubik_cube")
                
                transit_elem = ET.SubElement(root, "Transit")
                path = self.plan_trajectory(GRASP_OPEN, HOME)
                if not path:
                    return False
                self.write_path_to_xml(transit_elem, path)

                # Si queda otra acción, volvemos a coger el cubo
                if idx + 1 < len(solution):
                    transit_elem2 = ET.SubElement(root, "Transit")
                    path = self.plan_trajectory(HOME, GRASP_OPEN)
                    if not path:
                        return False
                    self.write_path_to_xml(transit_elem2, path)
                    
                    kautham.kMoveRobot(self, GRASP_OPEN)
                    kautham.kAttachObject(self, "ur3_right", "robotiq_85_base_link", "rubik_cube")
                    current_grasp = list(GRASP)

        # ── 3. Tránsito final a HOME si no se ha vuelto ────────────────────────────────
        # Si la última acción fue un rotate, el robot sigue sujetando el cubo
        if solution[-1] in ('rotate_top_cw', 'rotate_top_ccw'):
            kautham.kMoveRobot(self, GRASP)
            kautham.kDetachObject(self, "rubik_cube")
            transit_elem = ET.SubElement(root, "Transit")
            path = self.plan_trajectory(GRASP_OPEN, HOME)
            if path:
                self.write_path_to_xml(transit_elem, path)

        # Guardar y formatear el XML resultante
        rough = ET.tostring(root, encoding="unicode")
        reparsed = minidom.parseString(rough)
        xml_str = reparsed.toprettyxml(indent="\t")

        os.makedirs(os.path.dirname(OUTPUT_FILE), exist_ok=True)
        with open(OUTPUT_FILE, "w", encoding="utf-8") as f:
            f.write(xml_str)
            
        self.get_logger().info(f"✓ ¡Taskfile planificado con RRT guardado con éxito en: {OUTPUT_FILE}!")
        
        # Cerrar el problema en Kautham
        kautham.kCloseProblem(self)
        return True


# ═══════════════════════════════════════════════════════════════════════════════
# MAIN
# ═══════════════════════════════════════════════════════════════════════════════

# ── Generador PDDL y Planificador Fast Downward ───────────────────────────────

def generate_manipulation_problem(solution, filename="/home/barrendeiro/robotica/cub/robot/manipulation_problem.pddl"):
    """Genera el problema PDDL simplificado de manipulación para la secuencia de solución."""
    lines = []
    lines.append(";;; ============================================================")
    lines.append(";;; PROBLEM: solve-manipulation-sequence")
    lines.append(";;; ============================================================")
    lines.append("(define (problem solve-manipulation-sequence)")
    lines.append("  (:domain robot-manipulation)")
    
    # Declarar los objetos paso (step1, step2, ..., stepN)
    steps = [f"step{i+1}" for i in range(len(solution) + 1)]
    lines.append("  (:objects")
    lines.append("    " + " ".join(steps) + " - step")
    lines.append("  )")
    
    lines.append("  (:init")
    lines.append("    ;; Estado físico inicial del robot y el cubo")
    lines.append("    (cube-on-fixture)")
    lines.append("    (not (robot-holding))")
    lines.append("")
    lines.append("    ;; Orientación 3D inicial del cubo en la base")
    lines.append("    (top-face face_u)")
    lines.append("    (bottom-face face_d)")
    lines.append("    (front-face face_f)")
    lines.append("    (back-face face_b)")
    lines.append("    (left-face face_l)")
    lines.append("    (right-face face_r)")
    lines.append("")
    lines.append("    ;; Paso inicial de la receta")
    lines.append("    (current-step step1)")
    lines.append("")
    
    # Secuencia y tipos de pasos
    for i, action in enumerate(solution):
        lines.append(f"    (next-step step{i+1} step{i+2})")
        # Asignar tipo de paso según la nomenclatura estándar
        act_pddl = action.replace("_PRIME", "-prime")
        lines.append(f"    (step-type-{act_pddl} step{i+1})")
            
    lines.append("  )")
    
    # Meta: todos los pasos completados, cubo en fixture y gripper libre
    lines.append("  (:goal (and")
    for i in range(len(solution)):
        lines.append(f"    (step-completed step{i+1})")
    lines.append("    (cube-on-fixture)")
    lines.append("    (not (robot-holding))")
    lines.append("  ))")
    lines.append(")")
    
    with open(filename, 'w') as f:
        f.write("\n".join(lines))
    print(f"✓ PDDL manipulation problem written to {filename}")


def run_fast_downward(domain_path="/home/barrendeiro/robotica/cub/robot/manipulation_domain.pddl", 
                      problem_path="/home/barrendeiro/robotica/cub/robot/manipulation_problem.pddl"):
    """Ejecuta Fast Downward para resolver la tarea física de manipulación robótica."""
    import subprocess
    print("Invocando a Fast Downward...")
    
    # Usamos el wrapper fast-downward ejecutado con el python del sistema para evitar incompatibilidades
    cmd = [
        "/usr/bin/python3",
        "/usr/bin/fast-downward",
        domain_path,
        problem_path,
        "--search", "astar(blind())"
    ]
    
    result = subprocess.run(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
    
    plan_file = "sas_plan"
    if os.path.exists(plan_file):
        actions = []
        with open(plan_file, "r") as f:
            for line in f:
                line = line.strip()
                if line.startswith(";"):
                    break
                # Extrae el nombre de la acción PDDL
                action_name = line.replace("(", "").replace(")", "").split()[0]
                actions.append(action_name)
        os.remove(plan_file)
        return actions
    else:
        print("ERROR: Fast Downward no generó ningún plan.")
        print("Salida de FD:", result.stdout)
        print("Errores de FD:", result.stderr)
        return None


# ═══════════════════════════════════════════════════════════════════════════════
# MAIN
# ═══════════════════════════════════════════════════════════════════════════════

def main():
    rclpy.init()

    # ── Obtener scramble desde args o usar el por defecto ─────────────────────
    if len(sys.argv) > 1:
        scramble_seq = sys.argv[1:]
    else:
        scramble_seq = ['tilt_x_pos', 'rotate_top_cw', 'tilt_y_pos', 'rotate_top_ccw']

    for m in scramble_seq:
        if m not in ROBOT_MOVES:
            print(f"ERROR: Movimiento desconocido '{m}'")
            print(f"Movimientos válidos: {list(ROBOT_MOVES.keys())}")
            sys.exit(1)

    print("=" * 60)
    print("  Generador RRT de Taskfile — UR3 Rubik 2×2 (Arquitectura Jerárquica TAMP)")
    print("=" * 60)
    print(f"\nScramble ({len(scramble_seq)} movimientos):")
    for i, m in enumerate(scramble_seq):
        print(f"  {i+1}. {m}")

    # ── 1. Nivel de Tarea Abstracta: Resolver con BFS ─────────────────────────
    init_state = scramble(scramble_seq)

    if init_state == SOLVED_STATE:
        print("\nEl cubo ya está resuelto. No se genera taskfile.")
        sys.exit(0)

    print("\n[Nivel 1] Resolviendo cubo con BFS óptimo...")
    solution = bfs_solve(init_state)

    if solution is None:
        print("ERROR: No se encontró solución.")
        sys.exit(1)

    print(f"\n✓ SOLUCIÓN DEL CUBO ({len(solution)} movimientos abstractos):")
    for i, action in enumerate(solution):
        print(f"  Movimiento {i+1:2d}: {action}")

    # ── 2. Nivel Simbólico de Manipulación: Fast Downward ─────────────────────
    print("\n[Nivel 2] Planificando tareas físicas con Fast Downward...")
    generate_manipulation_problem(solution)
    manipulation_plan = run_fast_downward()
    
    if manipulation_plan is None:
        print("ERROR: Fast Downward falló en la planificación física.")
        sys.exit(1)
        
    print(f"\n✓ PLAN DE MANIPULACIÓN ROBÓTICA SIMBÓLICA (Fast Downward):")
    for i, act in enumerate(manipulation_plan):
        print(f"  Acción {i+1:2d}: {act}")

    # ── 3. Nivel Geométrico/Cinemático: RRT-Connect con Kautham ───────────────
    print("\n[Nivel 3] Planificando trayectorias geométricas en Kautham...")
    generator = RRTTaskfileGenerator()
    success = generator.run(solution)
    
    generator.destroy_node()
    rclpy.shutdown()

    if success:
        print("\n✓ Proceso finalizado con éxito.")
    else:
        print("\n❌ Hubo un error durante la planificación RRT.")
        sys.exit(1)


if __name__ == "__main__":
    main()
