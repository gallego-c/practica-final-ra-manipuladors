import socket
import time
import sys
import os

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))

# pick_x.py -- Mueve el UR3 a una configuracion de "pick" y, al llegar,
# cierra la pinza (10 mm) usando el script del URCap pinza10UR3.py.

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
# ABRIR_PINZA  = os.path.join(SCRIPT_DIR, "gripper", "pinza40UR3.py")   # abre  a 40 mm  (soltar)  <- por si lo necesitas

# ---------------------------------------------------------------------------
# Esperas (s)
#   No conocemos la pose inicial, asi que damos un margen generoso para que
#   el movej termine ANTES de mandar el script de la pinza: si lo mandasemos
#   antes, el nuevo programa interrumpiria el movimiento a medias.
# ---------------------------------------------------------------------------
ESPERA_MOV   = 1.0   # tiempo de sobra para llegar a la pose de pick
ESPERA_PINZA = 2.0   # tiempo para que el agarre se complete

pick_config = [0.88401, -0.98524, 0.73985, -1.31371, -1.61897, -0.6915]  # [50.65, -56.45, 42.39, -75.27, -92.76, -39.62] deg


def build_program(path, a, v, r):
    """Construye un programa URScript con los movej encadenados.

    El ultimo punto va con r=0 para terminar exactamente en su meta. Con un
    unico punto, simplemente se hace un movej que para en el objetivo.
    Antes de empezar el movimiento general, orientamos la pinza (wrist_3)
    a su posicion objetivo para evitar colisiones.
    """
    lines = ["def trayectoria():"]
    if len(path) > 1:
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
# Ir a HOME primero
sys.path.insert(0, SCRIPT_DIR)
from go_home import go_home
go_home(y_axis=True)
time.sleep(3.0)

print(f"Conectando al robot en {HOST}:{PORT} ...")
sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
sock.connect((HOST, PORT))
print("Conexion establecida.")

# 1. Mover a la configuracion de pick
print("Moviendo a la pose de pick...")
program = build_program([pick_config], ACC, VEL, BLEND)
print(program)
sock.send(program.encode())
time.sleep(ESPERA_MOV)

# 2. Cerrar la pinza (agarrar)
print("Cerrando pinza...")
cerrar_pinza(sock, CERRAR_PINZA)
time.sleep(ESPERA_PINZA)

print("Pick finalizado.")

# Cerrar la conexion
sock.close()
