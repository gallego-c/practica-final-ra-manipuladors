import socket
import time
import csv
import xml.etree.ElementTree as ET

# ⚠️ ¡ATENCIÓN! Cambia la X por el número real de tu robot (mirar en Teach Pendant -> Acerca de)
HOST = "10.10.73.239"
PORT = 30002

# Scripts para abrir y cerrar la pinza
Abrir_pinza  = 'robot/pinza40UR3.py'  # 40 mm (abierta)
Cerrar_pinza = 'robot/pinza10UR3.py'  # 10 mm (cerrada)

# ── Envío de trayectoria ──────────────────────────────────────────────────────

def send_joint_path(path, sock, acc=0.5, vel=0.5, blend=0.02, pause_after_first=0.0):
    """Envía la trayectoria como un único programa URScript con blending.

    - Primera posición: movej sin blend → para completo.
    - Pausa opcional en el robot (sleep en URScript).
    - Resto de waypoints: movej con radio de blend para movimiento fluido.
    """
    if not path:
        return

    lines = ["def traj():"]
    for i, joint_config in enumerate(path):
        if i == 0:
            lines.append(f"  movej({joint_config}, a={acc}, v={vel})")
            if pause_after_first > 0:
                lines.append(f"  sleep({pause_after_first})")
        elif i == len(path) - 1:
            lines.append(f"  movej({joint_config}, a={acc}, v={vel})")
        else:
            lines.append(f"  movej({joint_config}, a={acc}, v={vel}, r={blend})")
    lines += ["end", "traj()"]
    script = "\n".join(lines) + "\n"

    print(f"Enviando trayectoria fluida ({len(path)} waypoints, blend={blend} m)...")
    if pause_after_first > 0:
        print(f"  Pausa de {pause_after_first}s tras la primera posición.")
    sock.sendall(script.encode())

    # Tiempo estimado: llegada inicial + pausa + recorrido con blend
    wait_s = 5.0 + pause_after_first + len(path) * 0.4
    print(f"  Esperando ~{wait_s:.0f}s a que termine...")
    time.sleep(wait_s)

# ── Carga de trayectorias ─────────────────────────────────────────────────────

def cargar_trayectoria_csv(ruta_csv):
    """Lee un CSV exportado por Kautham y devuelve lista de [j1..j6] en radianes.

    El CSV tiene cabecera; las columnas 2-7 son:
      shoulder_pan, shoulder_lift, elbow, wrist_1, wrist_2, wrist_3  (en rad)
    """
    configuraciones = []
    with open(ruta_csv, newline='') as f:
        reader = csv.reader(f)
        next(reader)  # saltar cabecera
        for row in reader:
            joints = [float(row[2]), float(row[3]), float(row[4]),
                      float(row[5]), float(row[6]), float(row[7])]
            configuraciones.append(joints)
    return configuraciones

def cargar_trayectorias_xml(ruta_xml):
    """Lee un taskfile XML de Kautham y devuelve una lista de trayectorias.

    Cada trayectoria es una lista de configuraciones [j1..j6] en radianes.
    Los valores del UR3 están en los índices 2-7 del vector de estado de Kautham.
    """
    tree = ET.parse(ruta_xml)
    root = tree.getroot()
    trayectorias = []
    for bloque in root:
        if bloque.tag in ['Transit', 'Transfer']:
            trayectoria_actual = []
            for conf in bloque.findall('Conf'):
                valores = conf.text.split()
                ur3_joints = [float(x) for x in valores[2:8]]
                trayectoria_actual.append(ur3_joints)
            trayectorias.append(trayectoria_actual)
    return trayectorias

# ── Conexión al robot ─────────────────────────────────────────────────────────

def conectar():
    print(f"Conectando al robot en {HOST}:{PORT} ...")
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.connect((HOST, PORT))
    print("¡Conexión establecida!")
    return sock

def abrir_pinza(sock):
    with open(Abrir_pinza, 'rb') as f:
        sock.sendall(f.read())
    time.sleep(3)

def cerrar_pinza(sock):
    with open(Cerrar_pinza, 'rb') as f:
        sock.sendall(f.read())
    time.sleep(3)

# ═════════════════════════════════════════════════════════════════════════════
# EJECUCIÓN PRINCIPAL — TILT
# ═════════════════════════════════════════════════════════════════════════════

tilt_y = cargar_trayectoria_csv('confs/tilt_y.csv')

sock = conectar()

try:
    # El robot debe estar ya en la conf inicial del CSV (j6 ≈ 45.58°).
    # Llegar ahí con turn_counterclockwise.py (2× -90°), no con un movej directo
    # desde j6=226°, que implicaría ~180° y afectaría al cubo.
    print(f"\n--- Ejecutando TILT ({len(tilt_y)} waypoints) ---")
    send_joint_path(tilt_y, sock, acc=0.5, vel=0.5, blend=0.02, pause_after_first=3.0)
    print("\n¡Tilt finalizado con éxito!")
finally:
    sock.close()