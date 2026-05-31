#!/usr/bin/env python3
"""
try_robot_solve.py — Resuelve el cubo de Rubik 2x2 en tiempo real directamente en el simulador de Kautham.
Genera un scramble aleatorio, calcula la solución (BFS), planifica las tareas físicas (Fast Downward)
y ejecuta las trayectorias cinemáticas (RRT-Connect) online en Kautham en tiempo real, sin ficheros XML intermedios.

Uso:
    source /home/barrendeiro/robotica/ws_tamp/install/setup.bash
    /usr/bin/python3 robot/try_robot_solve.py
"""

import sys
import os
import time
import random

# ── Importar solver y utilidades de planificación ──────────────────────────────
_SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, _SCRIPT_DIR)

from solver import scramble, bfs_solve, ROBOT_MOVES, SOLVED_STATE
from generate_taskfile import generate_manipulation_problem, run_fast_downward

# ── Importar rclpy e interfaz de Kautham ──────────────────────────────────────
import rclpy
from rclpy.node import Node
from kautham_ros_interfaces.srv import OpenProblem

_KAUTHAM_ROS_PATH = "/home/barrendeiro/robotica/ws_tamp/src/task_and_motion_planning2/kautham_ros/kautham_ros/kautham_ros"
if _KAUTHAM_ROS_PATH not in sys.path:
    sys.path.append(_KAUTHAM_ROS_PATH)

import kautham_ros_interface_python as kautham

# ═══════════════════════════════════════════════════════════════════════════════
# CONFIGURACIONES DE CONTROL FÍSICAS [0,1]
# ═══════════════════════════════════════════════════════════════════════════════
GRIPPER_OPEN   = 0.500
GRIPPER_CLOSED = 0.680

HOME       = [0.500, 0.375, 0.500, 0.375, 0.500, 0.500, GRIPPER_OPEN]
GRASP_OPEN = [0.545, 0.380, 0.835, 0.327, 0.375, 0.425, GRIPPER_OPEN]
GRASP      = [0.545, 0.380, 0.835, 0.327, 0.375, 0.425, GRIPPER_CLOSED]
LIFT       = [0.545, 0.300, 0.650, 0.280, 0.375, 0.425, GRIPPER_CLOSED]

TILT_X_POS_PEAK = [0.545, 0.220, 0.750, 0.180, 0.375, 0.425, GRIPPER_CLOSED]
TILT_X_NEG_PEAK = [0.545, 0.440, 0.600, 0.430, 0.375, 0.425, GRIPPER_CLOSED]
TILT_Y_POS_PEAK = [0.620, 0.300, 0.700, 0.280, 0.400, 0.425, GRIPPER_CLOSED]
TILT_Y_NEG_PEAK = [0.470, 0.300, 0.700, 0.280, 0.350, 0.425, GRIPPER_CLOSED]

ROTATE_CW_PEAK       = [0.545, 0.380, 0.835, 0.327, 0.375, 0.675, GRIPPER_CLOSED]
ROTATE_CCW_PEAK      = [0.545, 0.380, 0.835, 0.327, 0.375, 0.175, GRIPPER_CLOSED]
ROTATE_CW_PEAK_OPEN  = [0.545, 0.380, 0.835, 0.327, 0.375, 0.675, GRIPPER_OPEN]
ROTATE_CCW_PEAK_OPEN = [0.545, 0.380, 0.835, 0.327, 0.375, 0.175, GRIPPER_OPEN]

# ═══════════════════════════════════════════════════════════════════════════════
# CLASE ANIMADOR ONLINE KAUTHAM
# ═══════════════════════════════════════════════════════════════════════════════

class RealtimeTAMPExecutor(Node):
    def __init__(self):
        super().__init__("realtime_tamp_executor")
        
        kautham_problem = os.path.join(os.path.dirname(_SCRIPT_DIR), "kautham", "ur3_rubik_kautham.xml")
        model_folders = [
            "/usr/share/kautham/demos/models/",
            os.path.join(os.path.dirname(_SCRIPT_DIR), "kautham/"),
            "/usr/share/kautham/demos/OMPL_geometric_demos/chess_ur3_robotiq/"
        ]
        
        self.get_logger().info("Conectando y abriendo problema en Kautham...")
        if not self.open_kautham_problem(model_folders, kautham_problem):
            self.get_logger().error("Fallo crítico abriendo el problema.")
            sys.exit(1)
            
        kautham.kSetPlannerByName(self, "omplRRTConnect")
        kautham.kSetPlannerParameter(self, "_Incremental (0/1)", "0")
        kautham.kSetPlannerParameter(self, "_Max Planning Time", "5")

    def open_kautham_problem(self, model_folders, problem_file):
        client = self.create_client(OpenProblem, '/kautham_node/OpenProblem')
        while not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Esperando al nodo Kautham...')
        
        req = OpenProblem.Request()
        req.problem = problem_file
        req.dir = model_folders
        future = client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        result = future.result()
        return result and result.response

    def animate_trajectory(self, start, goal):
        """Planifica la trayectoria y la ejecuta en tiempo real en el simulador."""
        self.get_logger().info(f"Planificando RRT de {start} a {goal}...")
        kautham.kSetQuery(self, start, goal)
        if kautham.kSolve(self):
            path = kautham.kGetPath(self, False, 0)
            if path:
                # Obtener dimensiones del path
                num_points = sorted(list(path.keys()))[-1][0] + 1
                num_dofs = sorted(list(path.keys()))[-1][1] + 1
                
                self.get_logger().info(f"¡Trayectoria calculada! Ejecutando {num_points} configuraciones online...")
                for i in range(num_points):
                    config = [path[i, j] for j in range(num_dofs)]
                    kautham.kMoveRobot(self, config)
                    time.sleep(0.04)  # Animación fluida a 25 Hz
                return True
        self.get_logger().error("RRT falló al resolver la trayectoria.")
        return False

    def execute_online(self, solution):
        self.get_logger().info(f"\n[+] Iniciando ejecución online de {len(solution)} acciones...")
        
        # Mover a Home inicial
        kautham.kMoveRobot(self, HOME)
        time.sleep(1.0)

        # 1. Tránsito inicial a GRASP_OPEN
        if not self.animate_trajectory(HOME, GRASP_OPEN):
            return False
            
        # Cerrar pinzas
        kautham.kMoveRobot(self, GRASP)
        kautham.kAttachObject(self, "ur3_right", "robotiq_85_base_link", "rubik_cube")
        current_grasp = list(GRASP)
        time.sleep(0.5)

        for idx, action in enumerate(solution):
            self.get_logger().info(f"\n[➡] EJECUTANDO EN SIMULADOR PASO {idx+1}/{len(solution)}: {action}")
            
            if action in ('rotate_top_cw', 'rotate_top_ccw'):
                peak_closed = list(current_grasp)
                peak_open = list(current_grasp)
                
                if action == 'rotate_top_cw':
                    peak_closed[5] = ROTATE_CW_PEAK[5]
                    peak_open[5] = ROTATE_CW_PEAK_OPEN[5]
                else:
                    peak_closed[5] = ROTATE_CCW_PEAK[5]
                    peak_open[5] = ROTATE_CCW_PEAK_OPEN[5]

                # Transfer (girar muñeca sujetando)
                if not self.animate_trajectory(current_grasp, peak_closed):
                    return False
                
                # Soltar
                kautham.kMoveRobot(self, peak_closed)
                kautham.kDetachObject(self, "rubik_cube")
                time.sleep(0.2)

                # Transit de retorno con pinza abierta
                if not self.animate_trajectory(peak_open, GRASP_OPEN):
                    return False
                
                # Volver a sujetar
                kautham.kMoveRobot(self, GRASP)
                kautham.kAttachObject(self, "ur3_right", "robotiq_85_base_link", "rubik_cube")
                current_grasp = list(GRASP)
                time.sleep(0.3)

            elif action in ('tilt_x_pos', 'tilt_x_neg', 'tilt_y_pos', 'tilt_y_neg'):
                # 1. Levantar cubo (Lift)
                if not self.animate_trajectory(current_grasp, LIFT):
                    return False
                
                # 2. Vuelco a la pose Peak
                if action == 'tilt_x_pos':
                    peak = TILT_X_POS_PEAK
                elif action == 'tilt_x_neg':
                    peak = TILT_X_NEG_PEAK
                elif action == 'tilt_y_pos':
                    peak = TILT_Y_POS_PEAK
                else:
                    peak = TILT_Y_NEG_PEAK
                    
                if not self.animate_trajectory(LIFT, peak):
                    return False
                
                # 3. Volver al nivel Lift
                if not self.animate_trajectory(peak, LIFT):
                    return False
                
                # 4. Dejar sobre la base
                if not self.animate_trajectory(LIFT, GRASP):
                    return False
                
                # Soltar cubo
                kautham.kMoveRobot(self, GRASP)
                kautham.kDetachObject(self, "rubik_cube")
                time.sleep(0.3)

                # Regresar a HOME de forma segura
                if not self.animate_trajectory(GRASP_OPEN, HOME):
                    return False
                
                # Si queda otra acción, volvemos a coger el cubo
                if idx + 1 < len(solution):
                    if not self.animate_trajectory(HOME, GRASP_OPEN):
                        return False
                    kautham.kMoveRobot(self, GRASP)
                    kautham.kAttachObject(self, "ur3_right", "robotiq_85_base_link", "rubik_cube")
                    current_grasp = list(GRASP)
                    time.sleep(0.3)

        # Regresar a HOME al final si sigue agarrado
        if solution[-1] in ('rotate_top_cw', 'rotate_top_ccw'):
            kautham.kMoveRobot(self, GRASP)
            kautham.kDetachObject(self, "rubik_cube")
            self.animate_trajectory(GRASP_OPEN, HOME)

        kautham.kCloseProblem(self)
        self.get_logger().info("\n[✓] ¡Resolución online del Cubo de Rubik finalizada con éxito!")
        return True


# ═══════════════════════════════════════════════════════════════════════════════
# MAIN
# ═══════════════════════════════════════════════════════════════════════════════

def main():
    rclpy.init()

    # Generar scramble aleatorio (4 movimientos)
    scramble_seq = ['tilt_x_pos', 'rotate_top_cw', 'tilt_y_pos', 'rotate_top_ccw']
    
    print("=" * 70)
    print("  TRY ROBOT SOLVE — EJECUCIÓN TAMP ONLINE EN TIEMPO REAL")
    print("=" * 70)
    print(f"\nMezcla Scramble: {' '.join(scramble_seq)}")

    # 1. Resolver con BFS
    init_state = scramble(scramble_seq)
    solution = bfs_solve(init_state)
    print(f"✓ Solución BFS: {solution}")

    # 2. Plan simbólico con Fast Downward
    generate_manipulation_problem(solution)
    manipulation_plan = run_fast_downward()
    print(f"✓ Plan simbólico de Fast Downward obtenido.")

    # 3. Planificar y animar online en Kautham
    executor = RealtimeTAMPExecutor()
    success = executor.execute_online(solution)

    executor.destroy_node()
    rclpy.shutdown()

    if success:
        print("\n✓ ¡Simulación completada con éxito en Kautham!")
    else:
        print("\n❌ Error durante la animación física.")

if __name__ == "__main__":
    main()
