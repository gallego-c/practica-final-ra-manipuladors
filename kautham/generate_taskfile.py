#!/usr/bin/env python3
"""
generate_taskfile.py — Genera el taskfile XML de Kautham para el UR3 Rubik 2x2.

DIFERENCIA CLAVE respecto a la versión anterior:
  Ahora la geometría la dirige el PLAN DE FAST DOWNWARD (pick_x, pick_y, execute_*,
  tilt_*, place, change_pick), NO la solución BFS. Esto es la integración
  tarea<->movimiento que pide la práctica: cada acción simbólica del planificador
  de tareas se realiza con una trayectoria planificada por RRT-Connect en Kautham.

Flujo:
  scramble -> BFS (secuencia de cubo) -> problema PDDL -> Fast Downward (plan físico)
           -> por cada acción del plan: Kautham RRT-Connect -> taskfile XML

Uso:
  python3 robot/generate_taskfile.py
  python3 robot/generate_taskfile.py tilt_x_pos rotate_top_cw tilt_y_pos
"""

import sys
import os
import xml.etree.ElementTree as ET
from xml.dom import minidom

# ── Rutas (portables; se pueden sobreescribir por variables de entorno) ───────
_SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
_REPO_ROOT = os.path.dirname(_SCRIPT_DIR)
sys.path.insert(0, _SCRIPT_DIR)
from solver import scramble, bfs_solve, ROBOT_MOVES, SOLVED_STATE

import rclpy
from rclpy.node import Node
from kautham_ros_interfaces.srv import OpenProblem

# El módulo python de kautham_ros: por defecto el del entorno; si no, indícalo con
#   export KAUTHAM_ROS_PATH=/ruta/a/.../kautham_ros/kautham_ros/kautham_ros
_KAUTHAM_ROS_PATH = os.environ.get(
    "KAUTHAM_ROS_PATH",
    "/home/barrendeiro/robotica/ws_tamp/src/task_and_motion_planning2/"
    "kautham_ros/kautham_ros/kautham_ros",
)
if _KAUTHAM_ROS_PATH and _KAUTHAM_ROS_PATH not in sys.path:
    sys.path.append(_KAUTHAM_ROS_PATH)
import kautham_ros_interface_python as kautham

OUTPUT_FILE = os.path.join(_REPO_ROOT, "kautham", "taskfile_rubik_ur3.xml")
KAUTHAM_PROBLEM = os.path.join(_REPO_ROOT, "kautham", "ur3_rubik_kautham.xml")
DOMAIN_FILE = os.path.join(_SCRIPT_DIR, "manipulation_domain.pddl")
PROBLEM_FILE = os.path.join(_SCRIPT_DIR, "manipulation_problem.pddl")

# Carpetas donde Kautham busca los modelos. La del repo se añade automáticamente.
MODEL_FOLDERS = [
    "/usr/share/kautham/demos/models/",
    os.path.join(_REPO_ROOT, "kautham") + "/",
    "/usr/share/kautham/demos/OMPL_geometric_demos/chess_ur3_robotiq/",
]

# ═══════════════════════════════════════════════════════════════════════════════
# CONFIGURACIONES CLAVE (controles normalizados de Kautham [0,1])
#   orden: [pan, lift, elbow, w1, w2, w3, gripper]
# ═══════════════════════════════════════════════════════════════════════════════
GRIPPER_OPEN = 0.500
GRIPPER_CLOSED = 0.680

HOME = [0.500, 0.375, 0.500, 0.375, 0.500, 0.500, GRIPPER_OPEN]

# Grasp "x" (el que ya tenías afinado)
GRASP_X_OPEN = [0.545, 0.380, 0.835, 0.327, 0.375, 0.425, GRIPPER_OPEN]
GRASP_X      = [0.545, 0.380, 0.835, 0.327, 0.375, 0.425, GRIPPER_CLOSED]

# Delta normalizado para 90 grados de muñeca (rango total 4*pi => 1.0 = 720 deg)
DELTA_90 = 0.125

# Grasp "y": misma postura pero muñeca girada 90 deg (para coger el cubo por el otro eje).
# TODO: VERIFICA en la GUI que esta config es alcanzable y libre de colisiones.
def _wrist_shifted(cfg, delta):
    out = list(cfg)
    out[5] = cfg[5] + delta
    return out

GRASP_Y_OPEN = _wrist_shifted(GRASP_X_OPEN, DELTA_90)
GRASP_Y      = _wrist_shifted(GRASP_X, DELTA_90)

# Lift y picos de inclinación (los que ya tenías)
LIFT = [0.545, 0.300, 0.650, 0.280, 0.375, 0.425, GRIPPER_CLOSED]
TILT_PEAKS = {
    "tilt_x_pos": [0.545, 0.220, 0.750, 0.180, 0.375, 0.425, GRIPPER_CLOSED],
    "tilt_x_neg": [0.545, 0.440, 0.600, 0.430, 0.375, 0.425, GRIPPER_CLOSED],
    "tilt_y_pos": [0.620, 0.300, 0.700, 0.280, 0.400, 0.425, GRIPPER_CLOSED],
    "tilt_y_neg": [0.470, 0.300, 0.700, 0.280, 0.350, 0.425, GRIPPER_CLOSED],
}

CUBE_INITIAL_POSE = "0.0 0.4 0.046 0.0 0.0 0.0 1.0"


# ═══════════════════════════════════════════════════════════════════════════════
# GENERADOR TAMP
# ═══════════════════════════════════════════════════════════════════════════════
class RRTTaskfileGenerator(Node):
    def __init__(self):
        super().__init__("rrt_taskfile_generator")
        self.get_logger().info("Conectando con Kautham ROS 2...")
        if not self.open_kautham_problem(MODEL_FOLDERS, KAUTHAM_PROBLEM):
            self.get_logger().error("No se pudo abrir el problema en Kautham.")
            sys.exit(1)
        kautham.kSetPlannerByName(self, "omplRRTConnect")
        kautham.kSetPlannerParameter(self, "_Incremental (0/1)", "0")
        kautham.kSetPlannerParameter(self, "_Max Planning Time", "10")
        self.root = None

    # ---- servicios Kautham (sin cambios respecto a tu versión) ----
    def open_kautham_problem(self, model_folders, problem_file):
        cli = self.create_client(OpenProblem, '/kautham_node/OpenProblem')
        while not cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('OpenProblem no disponible, esperando...')
        req = OpenProblem.Request()
        req.problem = problem_file
        req.dir = model_folders
        fut = cli.call_async(req)
        rclpy.spin_until_future_complete(self, fut)
        res = fut.result()
        if res and res.response is True:
            self.get_logger().info("Problema de Kautham abierto correctamente")
            return True
        self.get_logger().error(f"ERROR abriendo problema. dirs={req.dir} file={req.problem}")
        return False

    def plan_trajectory(self, start, goal):
        """RRT-Connect entre dos configuraciones; devuelve el path o None."""
        self.get_logger().info(f"  RRT: {['%.3f' % v for v in start]} -> "
                               f"{['%.3f' % v for v in goal]}")
        kautham.kSetQuery(self, start, goal)
        if kautham.kSolve(self):
            path = kautham.kGetPath(self, False, 0)
            if path:
                return path
        self.get_logger().error("  RRT FALLÓ al encontrar trayectoria.")
        return None

    def write_path_to_xml(self, parent_elem, path):
        k = sorted(list(path.keys()))[-1][1] + 1
        p = sorted(list(path.keys()))[-1][0] + 1
        for i in range(p):
            c = ET.SubElement(parent_elem, "Conf")
            c.text = " " + " ".join(f"{path[i, j]:.6f}" for j in range(k)) + " "

    # ---- helpers de segmento ----
    def _segment(self, tag, start, goal, attrib=None, elem=None):
        """Crea (o reutiliza) un elemento Transit/Transfer y le planifica un tramo."""
        if elem is None:
            elem = ET.SubElement(self.root, tag, attrib=attrib or {})
        path = self.plan_trajectory(start, goal)
        if not path:
            return None
        self.write_path_to_xml(elem, path)
        return elem

    def _transit(self, start, goal):
        return self._segment("Transit", start, goal)

    def _transfer(self, start, goal, elem=None):
        attrib = {"object": "rubik_cube", "robot": "ur3_right",
                  "link": "robotiq_85_base_link"}
        return self._segment("Transfer", start, goal, attrib=attrib, elem=elem)

    def _attach(self):
        kautham.kAttachObject(self, "ur3_right", "robotiq_85_base_link", "rubik_cube")

    def _detach(self):
        kautham.kDetachObject(self, "rubik_cube")

    # ═══════════════ DISPATCH POR ACCIÓN DEL PLAN DE FAST DOWNWARD ═══════════════
    def run(self, plan):
        self.get_logger().info(f"Generando taskfile para {len(plan)} acciones del plan FD...")
        self.root = ET.Element("Task", name="UR3_Rubik_RRT_Solver")
        init = ET.SubElement(self.root, "Initialstate")
        obj = ET.SubElement(init, "Object", object="rubik_cube")
        obj.text = f" {CUBE_INITIAL_POSE} "

        # Estado del robot
        st = {"holding": False, "grasp": None, "grasp_open": None}

        for idx, action in enumerate(plan):
            self.get_logger().info(f"── PASO {idx+1}/{len(plan)}: {action} ──")
            a = action.lower()

            if a in ("pick_x", "pick_y"):
                if not self._do_pick(st, "y" if a == "pick_y" else "x"):
                    return False

            elif a == "place":
                if not self._do_place(st):
                    return False

            elif a == "change_pick":
                if not self._do_change_pick(st):
                    return False

            elif a in TILT_PEAKS:
                if not self._do_tilt(st, a):
                    return False

            elif a.startswith("execute_"):
                if not self._do_execute(st, a):
                    return False

            else:
                self.get_logger().warn(f"  Acción desconocida '{action}', se ignora.")

        # Si al final aún sostiene el cubo, lo deja en el fixture.
        if st["holding"]:
            self.get_logger().info("Acción final implícita: place (soltar el cubo).")
            if not self._do_place(st):
                return False

        return self._save()

    # ---- handlers ----
    def _do_pick(self, st, orient):
        if st["holding"]:
            self.get_logger().warn("  pick con el cubo ya sujeto; se omite el agarre.")
            return True
        grasp_open = GRASP_Y_OPEN if orient == "y" else GRASP_X_OPEN
        grasp = GRASP_Y if orient == "y" else GRASP_X
        if self._transit(HOME, grasp_open) is None:
            return False
        kautham.kMoveRobot(self, grasp_open)
        self._attach()
        st.update(holding=True, grasp=list(grasp), grasp_open=list(grasp_open))
        return True

    def _do_place(self, st):
        if not st["holding"]:
            self.get_logger().warn("  place sin cubo sujeto; se omite.")
            return True
        # bajar al fixture (Transfer) y soltar
        if self._transfer(st["grasp"], st["grasp"]) is None:  # asienta en grasp cerrado
            return False
        kautham.kMoveRobot(self, st["grasp"])
        self._detach()
        if self._transit(st["grasp_open"], HOME) is None:
            return False
        st.update(holding=False, grasp=None, grasp_open=None)
        return True

    def _do_change_pick(self, st):
        # Cambiar el eje de agarre = soltar y volver a coger girado 90 deg.
        if not st["holding"]:
            self.get_logger().warn("  change_pick sin cubo sujeto; se omite.")
            return True
        # decidir orientación destino (la contraria a la actual)
        is_y = abs(st["grasp"][5] - GRASP_Y[5]) < 1e-6
        target = "x" if is_y else "y"
        if not self._do_place(st):
            return False
        return self._do_pick(st, target)

    def _do_tilt(self, st, tilt_name):
        # En el dominio, tilt requiere robot-holding y termina con el cubo en el
        # fixture y la pinza libre. Por eso: lift -> peak -> lift -> place -> soltar.
        if not st["holding"]:
            # si el plan tiltea sin sujetar, primero coge (orientación x por defecto)
            if not self._do_pick(st, "x"):
                return False
        peak = TILT_PEAKS[tilt_name]
        grasp = st["grasp"]
        tr = self._transfer(grasp, LIFT)               # subir
        if tr is None:
            return False
        if self._transfer(LIFT, peak, elem=tr) is None:        # inclinar
            return False
        if self._transfer(peak, LIFT, elem=tr) is None:        # volver
            return False
        if self._transfer(LIFT, GRASP_X, elem=tr) is None:     # depositar
            return False
        kautham.kMoveRobot(self, GRASP_X)
        self._detach()
        if self._transit(GRASP_X_OPEN, HOME) is None:          # retirarse
            return False
        st.update(holding=False, grasp=None, grasp_open=None)
        return True

    def _do_execute(self, st, action):
        # Giro de cara superior = rotación de muñeca (w3) con el cubo sujeto.
        if not st["holding"]:
            self.get_logger().warn("  execute_* sin cubo sujeto; se coge primero (x).")
            if not self._do_pick(st, "x"):
                return False
        suffix = action[len("execute_"):]
        if suffix.endswith("_prime"):
            delta = -DELTA_90
        elif suffix.endswith("2"):
            delta = 2 * DELTA_90
        else:
            delta = DELTA_90
        grasp = st["grasp"]
        peak_closed = _wrist_shifted(grasp, delta)
        peak_open = _wrist_shifted(st["grasp_open"], delta)
        # 1. girar la muñeca sujetando el cubo (Transfer)
        if self._transfer(grasp, peak_closed) is None:
            return False
        # 2. soltar y resetear la muñeca con la pinza abierta (Transit)
        kautham.kMoveRobot(self, peak_closed)
        self._detach()
        if self._transit(peak_open, st["grasp_open"]) is None:
            return False
        # 3. volver a coger en la postura neutra
        kautham.kMoveRobot(self, st["grasp_open"])
        self._attach()
        st["grasp"] = list(grasp)  # muñeca neutra otra vez
        return True

    def _save(self):
        rough = ET.tostring(self.root, encoding="unicode")
        xml_str = minidom.parseString(rough).toprettyxml(indent="\t")
        os.makedirs(os.path.dirname(OUTPUT_FILE), exist_ok=True)
        with open(OUTPUT_FILE, "w", encoding="utf-8") as f:
            f.write(xml_str)
        self.get_logger().info(f"✓ Taskfile guardado en: {OUTPUT_FILE}")
        kautham.kCloseProblem(self)
        return True


# ═══════════════════════════════════════════════════════════════════════════════
# PDDL + FAST DOWNWARD
# ═══════════════════════════════════════════════════════════════════════════════
def generate_manipulation_problem(solution, filename=PROBLEM_FILE):
    lines = ["(define (problem solve-manipulation-sequence)",
             "  (:domain robot-manipulation)"]
    steps = [f"step{i+1}" for i in range(len(solution) + 1)]
    lines += ["  (:objects", "    " + " ".join(steps) + " - step", "  )", "  (:init",
              "    (cube-on-fixture)", "    (not (robot-holding))",
              "    (top-face face_u)", "    (bottom-face face_d)",
              "    (front-face face_f)", "    (back-face face_b)",
              "    (left-face face_l)", "    (right-face face_r)",
              "    (current-step step1)"]
    for i, action in enumerate(solution):
        lines.append(f"    (next-step step{i+1} step{i+2})")
        lines.append(f"    (step-type-{action.replace('_PRIME', '-prime')} step{i+1})")
    lines.append("  )")
    lines.append("  (:goal (and")
    for i in range(len(solution)):
        lines.append(f"    (step-completed step{i+1})")
    lines += ["  ))", ")"]
    with open(filename, "w") as f:
        f.write("\n".join(lines))
    print(f"✓ Problema PDDL escrito en {filename}")


def run_fast_downward(domain_path=DOMAIN_FILE, problem_path=PROBLEM_FILE):
    import subprocess
    print("Invocando a Fast Downward...")
    cmd = ["/usr/bin/python3", "/usr/bin/fast-downward",
           domain_path, problem_path, "--search", "astar(blind())"]
    result = subprocess.run(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
    if os.path.exists("sas_plan"):
        actions = []
        with open("sas_plan") as f:
            for line in f:
                line = line.strip()
                if line.startswith(";"):
                    break
                actions.append(line.replace("(", "").replace(")", "").split()[0])
        os.remove("sas_plan")
        return actions
    print("ERROR: Fast Downward no generó plan.")
    print("STDOUT:", result.stdout)
    print("STDERR:", result.stderr)
    return None


# ═══════════════════════════════════════════════════════════════════════════════
# MAIN
# ═══════════════════════════════════════════════════════════════════════════════
def main():
    rclpy.init()
    scramble_seq = sys.argv[1:] if len(sys.argv) > 1 else \
        ['tilt_x_pos', 'rotate_top_cw', 'tilt_y_pos', 'rotate_top_ccw']

    for m in scramble_seq:
        if m not in ROBOT_MOVES:
            print(f"ERROR: movimiento desconocido '{m}'. Válidos: {list(ROBOT_MOVES)}")
            sys.exit(1)

    print("=" * 60)
    print("  Generador TAMP de Taskfile — UR3 Rubik 2x2")
    print("=" * 60)

    init_state = scramble(scramble_seq)
    if init_state == SOLVED_STATE:
        print("El cubo ya está resuelto. No se genera taskfile.")
        sys.exit(0)

    print("\n[Nivel 1] Resolviendo el cubo con BFS...")
    solution = bfs_solve(init_state)
    if solution is None:
        print("ERROR: no se encontró solución.")
        sys.exit(1)
    print(f"✓ {len(solution)} movimientos de cubo.")

    print("\n[Nivel 2] Planificando acciones físicas con Fast Downward...")
    generate_manipulation_problem(solution)
    manipulation_plan = run_fast_downward()
    if manipulation_plan is None:
        sys.exit(1)
    print(f"✓ Plan FD ({len(manipulation_plan)} acciones):")
    for i, act in enumerate(manipulation_plan):
        print(f"  {i+1:2d}. {act}")

    print("\n[Nivel 3] Planificando trayectorias con Kautham RRT-Connect...")
    generator = RRTTaskfileGenerator()
    # ★ CLAVE: la geometría la dirige el PLAN DE FD, no la solución BFS.
    success = generator.run(manipulation_plan)
    generator.destroy_node()
    rclpy.shutdown()

    print("\n✓ Proceso finalizado." if success else "\n❌ Error en la planificación RRT.")
    sys.exit(0 if success else 1)


if __name__ == "__main__":
    main()
