import socket
import time
import csv
import xml.etree.ElementTree as ET

# ⚠️ ¡ATENCIÓN! Cambia la X por el número real de tu robot (mirar en Teach Pendant -> Acerca de)
HOST = "10.10.73.234"
PORT = 30002

# Scripts para abrir y cerrar la pinza
Abrir_pinza  = 'robot/pinza40UR3.py'  # 40 mm (abierta)
Cerrar_pinza = 'robot/pinza10UR3.py'  # 10 mm (cerrada)

# ── Envío de trayectoria ──────────────────────────────────────────────────────

def send_joint_path(path, sock, acc=0.3, vel=0.2, delay=1.0):
    """Envía una lista de configuraciones [j1..j6] en radianes al robot."""
    for joints in path:
        config_str = "[" + ", ".join(f"{j:.6f}" for j in joints) + "]"
        cmd = f"movej({config_str}, a={acc}, v={vel})\n"
        print(f"  → {config_str}")
        sock.sendall(cmd.encode())
        time.sleep(delay)

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
    print(f"\n--- Ejecutando TILT ({len(tilt_y)} waypoints) ---")
    send_joint_path(tilt_y, sock, acc=0.3, vel=0.2, delay=1.0)
    print("\n¡Tilt finalizado con éxito!")
finally:
    sock.close()