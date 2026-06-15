import socket
import time
import os

# turn_90_regrasp.py -- Abre la pinza, gira 90 grados (pi/2 rad) y la vuelve a cerrar.

HOST = "10.10.73.239"   # IP del controlador del UR
PORT = 30002            # Secondary client interface

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
script_pinza = os.path.join(SCRIPT_DIR, "gripper", "pinza40UR3.py")

with open(script_pinza, "r", encoding="utf-8") as f:
    content = f.read()

idx = content.rfind("while (True):")
if idx == -1:
    raise ValueError("No se pudo encontrar 'while (True):' en el script de la pinza.")

line_start = content.rfind("\n", 0, idx) + 1
indent = content[line_start:idx]

# Construir programa URScript secuencial:
# 1. Abrir pinza (75mm)
# 2. Girar joint 6 (wrist_3) por 90 grados (pi/2 rad = 1.570796)
# 3. Cerrar pinza (51mm)
seq_code = f"""{indent}# --- Desactivar vigilante de RTDE ---
{indent}rtde_set_watchdog("input_int_register_24", 0, "ignore")
{indent}# --- 1. Abrir Pinza ---
{indent}on_return = rg_grip(75.0, 40.0, tool_index = 0, blocking = True, depth_comp = False, popupmsg = True)
{indent}rg_payload_set(mass = 0.0, tool_index = 0, use_guard = True)
{indent}# --- 2. Girar 90 Grados (pi/2 rad) ---
{indent}q = get_actual_joint_positions()
{indent}q[5] = q[5] + 1.5707963  # 90 grados
{indent}movej(q, a=0.5, v=0.2)
{indent}# --- 3. Cerrar Pinza ---
{indent}on_return = rg_grip(51.0, 40.0, tool_index = 0, blocking = True, depth_comp = False, popupmsg = True)
{indent}rg_payload_set(mass = 0.0, tool_index = 0, use_guard = True)
end
"""

program = content[:line_start] + seq_code

print(f"Conectando al robot en {HOST}:{PORT}...")
sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
sock.connect((HOST, PORT))
print("Conexion establecida.")

print("Enviando script secuencial (abrir + girar 90 + cerrar)...")
sock.sendall(program.encode("utf-8"))

# Esperar a que terminen las acciones secuenciales
time.sleep(5.0)

sock.close()
print("Accion finalizada y conexion cerrada.")
