import math
import os
import socket
import sys
import time

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))

# pick_y.py -- Igual que pick_x, pero con wrist_3 (j6) rotado -90° para agarrar por el eje Y.

# ---------------------------------------------------------------------------
# Conexion
# ---------------------------------------------------------------------------
HOST = "10.10.73.239"   # IP del controlador del UR  (ajustala a tu robot)
PORT = 30002            # Secondary client interface

# ---------------------------------------------------------------------------
# Parametros de movimiento
# ---------------------------------------------------------------------------
ACC   = 0.8     # aceleracion articular [rad/s^2]
VEL   = 0.6     # velocidad articular   [rad/s]
BLEND = 0.0     # un solo punto -> sin blend (para en la meta)

# ---------------------------------------------------------------------------
# Scripts de la pinza (están en la subcarpeta scripts/gripper)
# ---------------------------------------------------------------------------
CERRAR_PINZA = os.path.join(SCRIPT_DIR, "gripper", "pinza10UR3.py")   # cierra a 10 mm  (agarrar)

# ---------------------------------------------------------------------------
# Esperas (s)
# ---------------------------------------------------------------------------
ESPERA_MOV   = 8.0   # tiempo de sobra para llegar a la pose de pick
ESPERA_PINZA = 1.0   # tiempo para que el agarre se complete

# Misma pose que pick_x, con j6 girado -90° (eje Y en lugar de eje X)
PICK_X_CONFIG = [0.88401, -0.98524, 0.73985, -1.31371, -1.61897, 0.8793]
pick_config = PICK_X_CONFIG[:5] + [PICK_X_CONFIG[5] - math.pi / 2]


def build_program(path, a, v, r):
    """Construye un programa URScript con los movej encadenados.

    Antes del movimiento general, orienta wrist_3 (j6) para evitar colisiones.
    """
    lines = ["def trayectoria():"]
    if len(path) > 0:
        target_j6 = path[0][5]
        lines.append("  q_act = get_actual_joint_positions()")
        lines.append(f"  movej([q_act[0], q_act[1], q_act[2], q_act[3], q_act[4], {target_j6}], a={a}, v={v})")
    n = len(path)
    for i, q in enumerate(path):
        blend = 0.0 if i == n - 1 else r
        q_str = "[" + ", ".join(str(x) for x in q) + "]"
        lines.append(f"  movej({q_str}, a={a}, v={v}, r={blend})")
    lines.append("end")
    lines.append("trayectoria()")
    return "\n".join(lines) + "\n"


def cerrar_pinza(sock, script_pinza):
    """Envia el contenido completo del script del URCap de la pinza."""
    with open(script_pinza, "rb") as f:
        sock.sendall(f.read())


# ---------------------------------------------------------------------------
# Programa principal
# ---------------------------------------------------------------------------
sys.path.insert(0, SCRIPT_DIR)
from go_home import go_home

go_home()
time.sleep(1.0)

print(f"Conectando al robot en {HOST}:{PORT} ...")
sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
sock.connect((HOST, PORT))
print("Conexion establecida.")

print("Moviendo a la pose de pick (eje Y)...")
program = build_program([pick_config], ACC, VEL, BLEND)
print(program)
sock.send(program.encode())
time.sleep(ESPERA_MOV)

print("Cerrando pinza...")
cerrar_pinza(sock, CERRAR_PINZA)
time.sleep(ESPERA_PINZA)

print("Pick Y finalizado.")

sock.close()
