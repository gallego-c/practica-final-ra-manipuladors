import socket
import time

# turn_clockwise.py -- Gira wrist_3 en sentido horario usando sockets.

HOST = "10.10.73.239"   # IP del controlador del UR
PORT = 30002            # Secondary client interface

print(f"Conectando al robot en {HOST}:{PORT}...")
sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
sock.connect((HOST, PORT))
print("Conexion establecida.")

program = (
    "def turn():\n"
    "  q = get_actual_joint_positions()\n"
    "  q[5] = q[5] + 1.670796  # pi/2 + 0.1\n"
    "  movej(q, a=0.5, v=0.2)\n"
    "end\n"
    "turn()\n"
)

print("Enviando comando de giro...")
sock.sendall(program.encode())
time.sleep(4.0)  # Dar margen para que termine el giro
sock.close()
print("Conexion cerrada.")

