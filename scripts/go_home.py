import socket
import time
import math
import os

# go_home.py -- Mueve el UR3 a la configuracion HOME antes del pick.

HOST = "10.10.73.239"   # IP del controlador del UR
PORT = 30002            # Secondary client interface

ACC   = 0.3     # aceleracion articular suave [rad/s^2]
VEL   = 0.3     # velocidad articular suave   [rad/s]

# HOME pose: Primera posicion del path tilt_x_ur3 (en radianes)
home_config = [0.844914, -0.987856, 0.536689, -1.13289, -1.57184, 0.892212]

def go_home(y_axis=False, open_gripper=False):
    config = list(home_config)
    if y_axis:
        config[5] -= math.pi / 2  # Girar 90 grados la muñeca para pick_y
        
    target_j6 = config[5]
    config_str = f"[{config[0]}, {config[1]}, {config[2]}, {config[3]}, {config[4]}, target_j6 + j6_offset]"

    if open_gripper:
        script_dir = os.path.dirname(os.path.abspath(__file__))
        script_pinza = os.path.join(script_dir, "gripper", "pinza40UR3.py")
        print(f"Conectando al robot en {HOST}:{PORT} para abrir pinza e ir a HOME (y_axis={y_axis})...")
        
        with open(script_pinza, "r", encoding="utf-8") as f:
            content = f.read()

        idx = content.rfind("while (True):")
        if idx == -1:
            raise ValueError("No se pudo encontrar 'while (True):' en el script de la pinza.")

        line_start = content.rfind("\n", 0, idx) + 1
        indent = content[line_start:idx]

        # Extraer comandos de la pinza
        grip_start = content.find("on_return = rg_grip", idx)
        grip_end = content.find("\n", grip_start)
        grip_line = content[grip_start:grip_end].strip()

        payload_start = content.find("rg_payload_set", grip_end)
        payload_end = content.find("\n", payload_start)
        payload_line = content[payload_start:payload_end].strip()

        # Construir programa URScript secuencial
        seq_code = f"""{indent}# --- Desactivar vigilante de RTDE ---
{indent}rtde_set_watchdog("input_int_register_24", 0, "ignore")
{indent}# --- Abrir Pinza Primero ---
{indent}{grip_line}
{indent}{payload_line}
{indent}# --- Movimiento Posterior a HOME ---
{indent}q_act = get_actual_joint_positions()
{indent}target_j6 = {target_j6}
{indent}j6_offset = floor((q_act[5] - target_j6) / 3.14159265 + 0.5) * 3.14159265
{indent}movej({config_str}, a={ACC}, v={VEL})
end
"""
        program = content[:line_start] + seq_code
    else:
        program = (
            "def home():\n"
            "  rtde_set_watchdog(\"input_int_register_24\", 0, \"ignore\")\n"
            "  set_tool_voltage(24)\n"
            "  set_tool_communication(True, 1000000, 2, 1, 1.5, 3.5)\n"
            "  q_act = get_actual_joint_positions()\n"
            f"  target_j6 = {target_j6}\n"
            "  j6_offset = floor((q_act[5] - target_j6) / 3.14159265 + 0.5) * 3.14159265\n"
            f"  movej({config_str}, a={ACC}, v={VEL})\n"
            "end\n"
            "home()\n"
        )
        print(f"Conectando al robot en {HOST}:{PORT} para ir a HOME (y_axis={y_axis})...")

    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.connect((HOST, PORT))
    print("Conexion establecida.")
    
    if open_gripper:
        print("Enviando script combinado (abrir pinza + ir a HOME)...")
    else:
        print("Moviendo a la pose HOME...")
        
    sock.sendall(program.encode("utf-8"))
    
    wait_time = 10.0 if open_gripper else 8.0
    time.sleep(wait_time)
    
    sock.close()
    print("Robot en pose HOME y conexion cerrada.")

if __name__ == "__main__":
    go_home()

