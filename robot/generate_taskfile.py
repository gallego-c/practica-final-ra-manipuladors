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
# CONVERSIÓN DE ÁNGULOS (grados) A ESPACIO DE CONTROL KAUTHAM [0, 1]
# ═══════════════════════════════════════════════════════════════════════════════

# Fórmulas de normalización (proporcionadas por el profesor):
#   Joints 1,2,4,5,6 : q en [-2π, 2π] rad  →  (q + 2π) / 4π
#   Joint 3 (codo)   : q en [-π,  π]  rad  →  (q +  π) / 2π
#
# Equivalente en grados:
#   Joints 1,2,4,5,6 : [-360°, 360°]  →  (q_deg + 360) / 720
#   Joint 3 (codo)   : [-180°, 180°]  →  (q_deg + 180) / 360
_JOINT_LIMITS_DEG = [
    (-360.0, 360.0),  # j1 shoulder_pan
    (-360.0, 360.0),  # j2 shoulder_lift
    (-180.0, 180.0),  # j3 elbow  ← rango distinto: [-π, π]
    (-360.0, 360.0),  # j4 wrist_1
    (-360.0, 360.0),  # j5 wrist_2
    (-360.0, 360.0),  # j6 wrist_3
]

import math as _math

def deg_to_kautham(joints_deg, gripper=None):
    """Convierte una configuración de 6 ángulos (en grados) al espacio [0,1] de Kautham.

    Args:
        joints_deg: lista de 6 ángulos en grados [j1, j2, j3, j4, j5, j6]
        gripper:    valor normalizado del gripper (0.5=abierto, 0.68=cerrado).
                    Si es None, no se añade el gripper al resultado.

    Returns:
        Lista de 6 o 7 valores normalizados en [0,1].
    """
    result = []
    for angle, (lo, hi) in zip(joints_deg, _JOINT_LIMITS_DEG):
        result.append((angle - lo) / (hi - lo))
    if gripper is not None:
        result.append(gripper)
    return result

def rad_to_kautham(joints_rad, gripper=None):
    """Convierte una configuración de 6 ángulos (en radianes) al espacio [0,1] de Kautham.

    Usa las mismas fórmulas que deg_to_kautham pero acepta radianes directamente
    (útil si los valores vienen del robot sin convertir).
    """
    joints_deg = [_math.degrees(r) for r in joints_rad]
    return deg_to_kautham(joints_deg, gripper)

# ═══════════════════════════════════════════════════════════════════════════════
# CONFIGURACIONES CLAVE DEL ROBOT (espacio de control Kautham, normalizado [0,1])
# ═══════════════════════════════════════════════════════════════════════════════

# Gripper normalizado: 0.5 = abierto, 0.680 = cerrado sobre el cubo de 5 cm
GRIPPER_OPEN   = 0.500
GRIPPER_CLOSED = 0.680

# ── IDLE unificado: j1-j5 son fijos; j6 define el eje de agarre ──────────────
# Eje X (j6 = 226.39°) — configuración medida con el robot agarrando por el eje X.
# Eje Y (j6 = 226.39° - 90° = 136.39°) — solo varía j6, el resto de joints no cambian.
# La pinza real (OnRobot RG2) se controla con open_close.py / pinza10/40UR3.py;
# los valores GRIPPER_OPEN/CLOSED aquí son solo para la simulación en Kautham.
_IDLE_BASE_DEG      = [48.42, -60.95,  47.99,  -77.8,  -90.05]  # j1-j5 (fijos)
_IDLE_LIFT_BASE_DEG = [48.41, -56.6,   30.75,  -64.91, -90.06]  # j1-j5 elevado 30 mm

_J6_X_DEG = 226.39   # j6 para eje X (config medida con el robot real)
_J6_Y_DEG = 136.39   # j6 para eje Y = _J6_X_DEG - 90°; ajustar si se mide directamente

def _idle(j6_deg, gripper):
    """Config IDLE con el j6 indicado y el estado de pinza dado."""
    return deg_to_kautham(_IDLE_BASE_DEG + [j6_deg], gripper)

def _idle_lift(j6_deg, gripper):
    """Config IDLE elevada 30 mm con el j6 indicado y el estado de pinza dado."""
    return deg_to_kautham(_IDLE_LIFT_BASE_DEG + [j6_deg], gripper)

IDLE_X_OPEN = _idle(_J6_X_DEG, GRIPPER_OPEN)
IDLE_X      = _idle(_J6_X_DEG, GRIPPER_CLOSED)
IDLE_X_LIFT = _idle_lift(_J6_X_DEG, GRIPPER_CLOSED)

IDLE_Y_OPEN = _idle(_J6_Y_DEG, GRIPPER_OPEN)
IDLE_Y      = _idle(_J6_Y_DEG, GRIPPER_CLOSED)
IDLE_Y_LIFT = _idle_lift(_J6_Y_DEG, GRIPPER_CLOSED)

# ── TILT X: volcado rotando alrededor del eje X ──────────────────────────────
TILT_X_LIFT = deg_to_kautham([92.12, -44.61,  90.25, -112.88, -177.36,  109.42], GRIPPER_CLOSED)
TILT_X      = deg_to_kautham([92.12, -30.61,  90.27, -117.76, -177.38,  109.87], GRIPPER_CLOSED)

# ── TILT Y: volcado rotando alrededor del eje Y ──────────────────────────────
TILT_Y_LIFT = deg_to_kautham([-20.19, -80.12, 143.27,  -59.15,  -18.08,  176.40], GRIPPER_CLOSED)
TILT_Y      = deg_to_kautham([-20.22, -68.11, 145.36,  -73.40,  -18.07,  176.57], GRIPPER_CLOSED)

# ── HOME ─────────────────────────────────────────────────────────────────────
# TODO: reemplazar con la config real medida del robot en reposo
HOME = [0.500, 0.375, 0.500, 0.375, 0.500, 0.500, GRIPPER_OPEN]

# ── Alias de compatibilidad ───────────────────────────────────────────────────
GRASP_OPEN = IDLE_X_OPEN
GRASP      = IDLE_X

# Pose inicial del cubo en el simulador (X Y Z WX WY WZ TH)
# XY: robot centro (0.294, 0.539) + offset FK (-0.187, -0.399) = (0.107, 0.140)
#   Centro robot = perimetro + radio UR3 (64 mm): 0.23+0.064=0.294, 0.475+0.064=0.539
# Z  = fixture_height - pocket_depth + cube_half = 0.075 - 0.013 + 0.025 = 0.087 m
CUBE_INITIAL_POSE = "0.107 0.140 0.087 0.0 0.0 0.0 1.0"

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

    def run(self, manipulation_plan):
        """Genera el taskfile RRT a partir del plan de manipulación de Fast Downward.

        Acciones PDDL esperadas (todas en minúsculas, sin parámetros de step):
          pick_x / pick_y     — coger el cubo desde el eje X o Y
          place               — soltar el cubo en el fixture
          change_pick         — cambiar eje de agarre (X↔Y) sin soltar el cubo
          tilt_x_pos/neg      — volcar el cubo alrededor del eje X (precondición: holding-y)
          tilt_y_pos/neg      — volcar el cubo alrededor del eje Y (precondición: holding-x)
          execute_u/r/f/...   — rotar capa superior ±90° (cambia eje X↔Y)
          execute_u2/r2/...   — rotar capa superior 180° (mantiene eje)
        """
        self.get_logger().info(f"Generando taskfile RRT para {len(manipulation_plan)} acciones PDDL...")

        root = ET.Element("Task", name="UR3_Rubik_RRT_Solver")
        init = ET.SubElement(root, "Initialstate")
        obj  = ET.SubElement(init, "Object", object="rubik_cube")
        obj.text = f" {CUBE_INITIAL_POSE} "

        # Estado de seguimiento
        current_grasp = None   # config actual del robot (lista de 7 valores Kautham)
        current_axis  = None   # 'x' o 'y'

        def _transfer_attrib():
            return {"object": "rubik_cube", "robot": "ur3_right", "link": "robotiq_85_base_link"}

        def _plan_or_fail(start, goal):
            p = self.plan_trajectory(start, goal)
            if not p:
                self.get_logger().error(f"RRT falló: {start} → {goal}")
            return p

        for idx, action in enumerate(manipulation_plan):
            self.get_logger().info(f"── PASO {idx+1}/{len(manipulation_plan)}: {action} ──")

            # ── pick_x: HOME → IDLE_X_OPEN (Transit), cerrar pinza ──────────────
            if action == 'pick_x':
                transit = ET.SubElement(root, "Transit")
                path = _plan_or_fail(HOME, IDLE_X_OPEN)
                if not path: return False
                self.write_path_to_xml(transit, path)
                kautham.kMoveRobot(self, IDLE_X_OPEN)
                kautham.kAttachObject(self, "ur3_right", "robotiq_85_base_link", "rubik_cube")
                current_grasp = list(IDLE_X)
                current_axis  = 'x'

            # ── pick_y: HOME → IDLE_Y_OPEN (Transit), cerrar pinza ──────────────
            elif action == 'pick_y':
                transit = ET.SubElement(root, "Transit")
                path = _plan_or_fail(HOME, IDLE_Y_OPEN)
                if not path: return False
                self.write_path_to_xml(transit, path)
                kautham.kMoveRobot(self, IDLE_Y_OPEN)
                kautham.kAttachObject(self, "ur3_right", "robotiq_85_base_link", "rubik_cube")
                current_grasp = list(IDLE_Y)
                current_axis  = 'y'

            # ── place: soltar cubo en fixture, volver a HOME ─────────────────────
            elif action == 'place':
                kautham.kDetachObject(self, "rubik_cube")
                current_open = list(current_grasp)
                current_open[-1] = GRIPPER_OPEN
                transit = ET.SubElement(root, "Transit")
                path = _plan_or_fail(current_open, HOME)
                if not path: return False
                self.write_path_to_xml(transit, path)
                current_grasp = None
                current_axis  = None

            # ── change_pick: cambiar eje de agarre X↔Y sin soltar el cubo ───────
            # El cubo permanece sujeto; solo cambia j6 (~90°) para pasar de
            # orientación X a Y o viceversa (Transfer sin detach).
            elif action == 'change_pick':
                if current_axis == 'x':
                    new_idle = IDLE_Y
                    new_axis = 'y'
                else:
                    new_idle = IDLE_X
                    new_axis = 'x'
                transfer = ET.SubElement(root, "Transfer", attrib=_transfer_attrib())
                path = _plan_or_fail(current_grasp, new_idle)
                if not path: return False
                self.write_path_to_xml(transfer, path)
                kautham.kMoveRobot(self, new_idle)
                current_grasp = list(new_idle)
                current_axis  = new_axis

            # ── tilt_x_pos/neg: volcar alrededor del eje X ───────────────────────
            # Precondición PDDL: holding-y → el brazo está en IDLE_Y (idle_lift = IDLE_Y_LIFT)
            elif action in ('tilt_x_pos', 'tilt_x_neg'):
                idle_lift  = IDLE_Y_LIFT
                tilt_lift  = TILT_X_LIFT
                tilt_place = TILT_X
                transfer = ET.SubElement(root, "Transfer", attrib=_transfer_attrib())
                for seg_start, seg_end in [
                    (current_grasp, idle_lift),
                    (idle_lift,     tilt_lift),
                    (tilt_lift,     tilt_place),
                ]:
                    path = _plan_or_fail(seg_start, seg_end)
                    if not path: return False
                    self.write_path_to_xml(transfer, path)
                kautham.kMoveRobot(self, tilt_place)
                kautham.kDetachObject(self, "rubik_cube")
                tilt_open = list(tilt_place); tilt_open[-1] = GRIPPER_OPEN
                transit = ET.SubElement(root, "Transit")
                path = _plan_or_fail(tilt_open, HOME)
                if not path: return False
                self.write_path_to_xml(transit, path)
                current_grasp = None
                current_axis  = None

            # ── tilt_y_pos/neg: volcar alrededor del eje Y ───────────────────────
            # Precondición PDDL: holding-x → el brazo está en IDLE_X (idle_lift = IDLE_X_LIFT)
            elif action in ('tilt_y_pos', 'tilt_y_neg'):
                idle_lift  = IDLE_X_LIFT
                tilt_lift  = TILT_Y_LIFT
                tilt_place = TILT_Y
                transfer = ET.SubElement(root, "Transfer", attrib=_transfer_attrib())
                for seg_start, seg_end in [
                    (current_grasp, idle_lift),
                    (idle_lift,     tilt_lift),
                    (tilt_lift,     tilt_place),
                ]:
                    path = _plan_or_fail(seg_start, seg_end)
                    if not path: return False
                    self.write_path_to_xml(transfer, path)
                kautham.kMoveRobot(self, tilt_place)
                kautham.kDetachObject(self, "rubik_cube")
                tilt_open = list(tilt_place); tilt_open[-1] = GRIPPER_OPEN
                transit = ET.SubElement(root, "Transit")
                path = _plan_or_fail(tilt_open, HOME)
                if not path: return False
                self.write_path_to_xml(transit, path)
                current_grasp = None
                current_axis  = None

            # ── execute_*: rotar capa superior mediante j6 ───────────────────────
            # execute_X      → +90°, cambia eje X↔Y
            # execute_X_prime→ -90°, cambia eje X↔Y
            # execute_X2     → +180°, mantiene eje
            #
            # Secuencia en simulación:
            #   IDLE → IDLE_LIFT   (Transfer: elevar cubo 30 mm fuera del fixture)
            #   IDLE_LIFT → PEAK_LIFT  (Transfer: girar j6 mientras el cubo está libre)
            #   abrir pinza, el cubo queda flotando en posición rotada
            #   Transit PEAK_LIFT_OPEN → next_IDLE_OPEN  (re-agarrar en nuevo eje)
            #
            # Por qué levantar: el cubo es un cuerpo rígido en Kautham. Si giramos j6
            # con el cubo dentro del pocket del fixture (~5 cm) las esquinas del cubo
            # (diagonal ≈ 7 cm) colisionan con las paredes. En el robot real la pinza
            # sólo agarra la mitad superior y no hay ese problema, pero en simulación
            # necesitamos liberar el cubo del pocket antes de rotar.
            elif action.startswith('execute_'):
                is_180   = action.endswith('2')
                is_prime = action.endswith('_prime')
                delta_deg = 180.0 if is_180 else (-90.0 if is_prime else +90.0)

                j6_deg      = current_grasp[5] * 720.0 - 360.0
                j6_peak_deg = j6_deg + delta_deg
                if j6_peak_deg >  360.0: j6_peak_deg -= 720.0
                if j6_peak_deg < -360.0: j6_peak_deg += 720.0

                # IDLE_LIFT para el eje actual (mismos j1-j5 que IDLE_LIFT, mismo j6 actual)
                idle_lift = IDLE_X_LIFT if current_axis == 'x' else IDLE_Y_LIFT

                # PEAK_LIFT = IDLE_LIFT con j6 rotado al ángulo del peak
                peak_lift       = list(idle_lift)
                peak_lift[5]    = (j6_peak_deg + 360.0) / 720.0
                peak_lift_open  = list(peak_lift); peak_lift_open[-1] = GRIPPER_OPEN

                # Transfer: IDLE → IDLE_LIFT → PEAK_LIFT  (cubo adjunto, libre de colisión)
                transfer = ET.SubElement(root, "Transfer", attrib=_transfer_attrib())
                for seg_start, seg_end in [
                    (current_grasp, idle_lift),
                    (idle_lift,     peak_lift),
                ]:
                    path = _plan_or_fail(seg_start, seg_end)
                    if not path: return False
                    self.write_path_to_xml(transfer, path)
                kautham.kMoveRobot(self, peak_lift)
                kautham.kDetachObject(self, "rubik_cube")

                # Eje tras la rotación — j6 canónico para evitar deriva acumulada
                if is_180:
                    next_axis = current_axis
                else:
                    next_axis = 'y' if current_axis == 'x' else 'x'

                next_j6        = _J6_X_DEG if next_axis == 'x' else _J6_Y_DEG
                next_idle_open = _idle(next_j6, GRIPPER_OPEN)
                next_idle      = _idle(next_j6, GRIPPER_CLOSED)

                # Transit: PEAK_LIFT_OPEN → next_IDLE_OPEN  (re-agarrar)
                transit = ET.SubElement(root, "Transit")
                path = _plan_or_fail(peak_lift_open, next_idle_open)
                if not path: return False
                self.write_path_to_xml(transit, path)
                kautham.kMoveRobot(self, next_idle_open)
                kautham.kAttachObject(self, "ur3_right", "robotiq_85_base_link", "rubik_cube")
                current_grasp = list(next_idle)
                current_axis  = next_axis

            else:
                self.get_logger().warn(f"Acción desconocida ignorada: '{action}'")

        # ── Tránsito final a HOME si el robot sigue sujetando el cubo ────────────
        if current_grasp is not None:
            kautham.kDetachObject(self, "rubik_cube")
            final_open = list(current_grasp); final_open[-1] = GRIPPER_OPEN
            transit = ET.SubElement(root, "Transit")
            path = self.plan_trajectory(final_open, HOME)
            if path:
                self.write_path_to_xml(transit, path)

        rough    = ET.tostring(root, encoding="unicode")
        reparsed = minidom.parseString(rough)
        xml_str  = reparsed.toprettyxml(indent="\t")
        os.makedirs(os.path.dirname(OUTPUT_FILE), exist_ok=True)
        with open(OUTPUT_FILE, "w", encoding="utf-8") as f:
            f.write(xml_str)
        self.get_logger().info(f"✓ Taskfile guardado en: {OUTPUT_FILE}")
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
    success = generator.run(manipulation_plan)
    
    generator.destroy_node()
    rclpy.shutdown()

    if success:
        print("\n✓ Proceso finalizado con éxito.")
    else:
        print("\n❌ Hubo un error durante la planificación RRT.")
        sys.exit(1)


if __name__ == "__main__":
    main()
