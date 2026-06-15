import socket
import time

# turn_180.py -- Gira wrist_3 180 grados (pi rad) usando sockets.

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
    "  q = get_actual_joint_positions()\n"
    "  q[5] = q[5] + 3.14159265  # pi rad (180 grados)\n"
    "  movej(q, a=0.5, v=0.2)\n"
    "end\n"
    "turn()\n"
)

print("Enviando comando de giro de 180 grados...")
sock.sendall(program.encode())
time.sleep(5.0)  # Margen de tiempo para completar la rotacion
sock.close()
print("Conexion cerrada.")
