import socket
import time

# turn_counterclockwise.py -- Gira wrist_3 en sentido antihorario usando sockets.

HOST = "10.10.73.239"   # IP del controlador del UR
PORT = 30002            # Secondary client interface

print(f"Conectando al robot en {HOST}:{PORT}...")
sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
sock.connect((HOST, PORT))
print("Conexion establecida.")

program = (
    "def turn():\n"
    "  rtde_set_watchdog(\"input_int_register_24\", 0, \"ignore\")\n"
    "  set_tool_voltage(24)\n"
    "  set_tool_communication(True, 1000000, 2, 1, 1.5, 3.5)\n"
    "  q_start = get_actual_joint_positions()\n"
    "  # 1. Elevar un poco (joints 1-5 a la conf elevada, joint 6 sin rotar)\n"
    "  q_lift = [0.84736, -1.02992, 0.80599, -1.34670, -1.57080, q_start[5]]\n"
    "  movej(q_lift, a=0.5, v=0.2)\n"
    "  # 2. Rotar joint 6 (antihorario)\n"
    "  q_rot = [0.84736, -1.02992, 0.80599, -1.34670, -1.57080, q_start[5] - 1.620796]\n"
    "  movej(q_rot, a=0.5, v=0.2)\n"
    "  # 3. Volver a bajar a la pose original (pero con joint 6 rotado)\n"
    "  q_down = [q_start[0], q_start[1], q_start[2], q_start[3], q_start[4], q_start[5] - 1.620796]\n"
    "  movej(q_down, a=0.5, v=0.2)\n"
    "end\n"
    "turn()\n"
)

print("Enviando comando de giro...")
sock.sendall(program.encode())
time.sleep(6.4)  # Margen doble
sock.close()
print("Conexion cerrada.")

