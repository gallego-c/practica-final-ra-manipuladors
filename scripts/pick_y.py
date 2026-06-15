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
ESPERA_MOV   = 4.5   # tiempo de sobra para llegar a la pose de pick
ESPERA_PINZA = 2.0   # tiempo para que el agarre se complete

pick_config = [0.84736, -1.03219, 0.81856, -1.35700, -1.57080, 2.41379]  # [48.55, -59.14, 46.90, -77.75, -90.00, 138.30] deg


def build_program(path, a, v, r):
    """Construye un programa URScript con los movej encadenados.

    El ultimo punto va con r=0 para terminar exactamente en su meta. Con un
    unico punto, simplemente se hace un movej que para en el objetivo.
    Antes de empezar el movimiento general, orientamos la pinza (wrist_3)
    a su posicion objetivo para evitar colisiones.
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


def mover_y_cerrar_pinza(sock, script_pinza, pick_config, a, v):
    """Combina el movimiento y la activacion de la pinza en un unico URScript."""
    with open(script_pinza, "r", encoding="utf-8") as f:
        content = f.read()

    # Buscar el bucle principal 'while (True):' al final del script
    idx = content.rfind("while (True):")
    if idx == -1:
        raise ValueError("No se pudo encontrar 'while (True):' en el script de la pinza.")

    # Obtener el inicio de linea para mantener la indentacion
    line_start = content.rfind("\n", 0, idx) + 1
    indent = content[line_start:idx]

    target_j6 = pick_config[5]

    movement_lines = [
        "# --- Movimiento a pose de pick (evitando colision y giros inecesarios en j6) ---",
        "q_act = get_actual_joint_positions()",
        f"target_j6 = {target_j6}",
        "j6_offset = floor((q_act[5] - target_j6) / 3.14159265 + 0.5) * 3.14159265",
        f"movej([q_act[0], q_act[1], q_act[2], q_act[3], q_act[4], target_j6 + j6_offset], a={a}, v={v})",
        f"movej([{pick_config[0]}, {pick_config[1]}, {pick_config[2]}, {pick_config[3]}, {pick_config[4]}, target_j6 + j6_offset], a={a}, v={v})",
        "# --------------------------------------------------------------------------------",
        "while (True):"
    ]
    
    movement_code = "\n".join(indent + line for line in movement_lines)
    combined = content[:line_start] + movement_code + content[idx + len("while (True):"):]
    
    print("Enviando script combinado (movimiento + pinza)...")
    sock.sendall(combined.encode("utf-8"))


# ---------------------------------------------------------------------------
# Programa principal
# ---------------------------------------------------------------------------
# Ir a HOME abriendo la pinza primero
sys.path.insert(0, SCRIPT_DIR)
from go_home import go_home
go_home(open_gripper=True)
time.sleep(3.0)

print(f"Conectando al robot en {HOST}:{PORT} ...")
sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
sock.connect((HOST, PORT))
print("Conexion establecida.")

# Ejecutar movimiento y pinza en un solo programa
mover_y_cerrar_pinza(sock, CERRAR_PINZA, pick_config, ACC, VEL)

# Esperar a que terminen ambas acciones (movimiento + cerrado)
print("Esperando a que finalice el movimiento y agarre...")
time.sleep(ESPERA_MOV + ESPERA_PINZA)

print("Pick finalizado.")

# Cerrar la conexion
sock.close()

