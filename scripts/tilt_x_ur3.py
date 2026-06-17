import socket
import time
import sys
import os
import math

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
ABRIR_PINZA = os.path.join(SCRIPT_DIR, "gripper", "pinza40UR3.py")

# Generado a partir de csv_to_ur3.py -- 97 waypoints, 6 articulaciones del UR3.

# ---------------------------------------------------------------------------
# Conexion
# ---------------------------------------------------------------------------
HOST = "10.10.73.239"   # IP del controlador del UR
PORT = 30002            # Secondary client interface

# ---------------------------------------------------------------------------
# Parametros de movimiento
# ---------------------------------------------------------------------------
ACC   = 0.8     # aceleracion articular [rad/s^2]
VEL   = 0.6     # velocidad articular   [rad/s]
BLEND = 0.002   # radio de blend [m] -- compromiso fluidez/fidelidad
PAUSA = 1.0     # pausa en el extremo antes de volver [s]

# ---------------------------------------------------------------------------
# Trayectoria -- configuraciones articulares (en radianes)
# ---------------------------------------------------------------------------
path = [
    [0.844914, -0.987856, 0.536689, -1.13289, -1.57184, 0.892212],
    [0.860598, -0.996334, 0.541489, -1.14649, -1.58542, 0.908396],
    [0.876269, -1.0048, 0.546296, -1.16007, -1.599, 0.92457],
    [0.891916, -1.01323, 0.551113, -1.17364, -1.61257, 0.940724],
    [0.907527, -1.02163, 0.555949, -1.18718, -1.62615, 0.956848],
    [0.923088, -1.02997, 0.560808, -1.20068, -1.63972, 0.972933],
    [0.938589, -1.03823, 0.565696, -1.21415, -1.65329, 0.988969],
    [0.954016, -1.04642, 0.570619, -1.22756, -1.66686, 1.00494],
    [0.969358, -1.0545, 0.575583, -1.24092, -1.68043, 1.02085],
    [0.984602, -1.06248, 0.580593, -1.2542, -1.694, 1.03668],
    [0.999736, -1.07033, 0.585656, -1.26741, -1.70756, 1.05242],
    [1.01475, -1.07803, 0.590778, -1.28054, -1.72112, 1.06806],
    [1.02963, -1.08559, 0.595963, -1.29357, -1.73468, 1.08359],
    [1.04436, -1.09298, 0.601219, -1.3065, -1.74823, 1.09901],
    [1.05893, -1.10018, 0.60655, -1.31932, -1.76178, 1.11429],
    [1.07333, -1.10719, 0.611963, -1.33202, -1.77532, 1.12944],
    [1.08755, -1.11399, 0.617464, -1.34459, -1.78886, 1.14444],
    [1.10157, -1.12056, 0.623058, -1.35703, -1.8024, 1.15929],
    [1.11539, -1.1269, 0.628751, -1.36932, -1.81593, 1.17396],
    [1.12898, -1.13298, 0.63455, -1.38147, -1.82945, 1.18846],
    [1.14235, -1.1388, 0.640459, -1.39344, -1.84297, 1.20277],
    [1.15546, -1.14434, 0.646485, -1.40525, -1.85648, 1.21688],
    [1.16833, -1.14959, 0.652633, -1.41688, -1.86999, 1.23078],
    [1.18092, -1.15452, 0.65891, -1.42833, -1.88349, 1.24447],
    [1.19323, -1.15914, 0.665321, -1.43958, -1.89698, 1.25793],
    [1.20525, -1.16342, 0.671872, -1.45062, -1.91047, 1.27116],
    [1.21697, -1.16735, 0.678569, -1.46146, -1.92394, 1.28414],
    [1.22837, -1.17091, 0.685417, -1.47207, -1.93741, 1.29686],
    [1.23943, -1.1741, 0.692423, -1.48245, -1.95088, 1.30932],
    [1.25016, -1.1769, 0.699593, -1.4926, -1.96433, 1.3215],
    [1.26053, -1.17929, 0.706931, -1.5025, -1.97777, 1.33339],
    [1.27054, -1.18126, 0.714445, -1.51214, -1.99121, 1.34499],
    [1.28016, -1.1828, 0.72214, -1.52152, -2.00463, 1.35628],
    [1.2894, -1.18389, 0.730021, -1.53063, -2.01805, 1.36725],
    [1.29826, -1.18454, 0.73809, -1.53948, -2.03146, 1.37793],
    [1.30675, -1.18476, 0.746345, -1.54808, -2.04488, 1.38831],
    [1.3149, -1.18457, 0.754787, -1.55644, -2.05831, 1.39842],
    [1.32271, -1.18398, 0.763417, -1.56458, -2.07177, 1.40827],
    [1.3302, -1.18299, 0.772233, -1.5725, -2.08525, 1.41787],
    [1.33739, -1.18162, 0.781238, -1.58021, -2.09876, 1.42723],
    [1.34428, -1.17987, 0.79043, -1.58773, -2.11233, 1.43636],
    [1.35089, -1.17777, 0.799809, -1.59507, -2.12594, 1.44529],
    [1.35724, -1.17531, 0.809377, -1.60224, -2.13961, 1.45402],
    [1.36334, -1.17251, 0.819132, -1.60925, -2.15335, 1.46257],
    [1.36919, -1.16939, 0.829075, -1.61611, -2.16716, 1.47095],
    [1.37483, -1.16594, 0.839207, -1.62284, -2.18105, 1.47917],
    [1.38026, -1.16219, 0.849527, -1.62943, -2.19504, 1.48725],
    [1.38549, -1.15814, 0.860035, -1.63591, -2.20912, 1.49519],
    [1.39054, -1.1538, 0.870732, -1.64229, -2.2233, 1.50302],
    [1.39542, -1.14919, 0.881618, -1.64858, -2.2376, 1.51075],
    [1.40015, -1.14431, 0.892692, -1.65478, -2.25202, 1.51838],
    [1.40474, -1.13918, 0.903956, -1.66092, -2.26657, 1.52594],
    [1.40921, -1.1338, 0.915408, -1.66699, -2.28125, 1.53343],
    [1.41356, -1.12819, 0.92705, -1.67302, -2.29607, 1.54088],
    [1.41782, -1.12236, 0.93888, -1.67901, -2.31105, 1.54828],
    [1.42199, -1.11632, 0.950901, -1.68498, -2.32618, 1.55566],
    [1.42609, -1.11008, 0.96311, -1.69093, -2.34148, 1.56303],
    [1.43014, -1.10365, 0.97551, -1.69688, -2.35695, 1.5704],
    [1.43415, -1.09704, 0.988099, -1.70284, -2.37261, 1.57778],
    [1.43812, -1.09026, 1.00088, -1.70882, -2.38845, 1.5852],
    [1.44209, -1.08333, 1.01385, -1.71482, -2.40449, 1.59265],
    [1.44605, -1.07625, 1.02701, -1.72087, -2.42073, 1.60016],
    [1.45003, -1.06903, 1.04036, -1.72698, -2.43719, 1.60774],
    [1.45404, -1.06168, 1.0539, -1.73314, -2.45386, 1.6154],
    [1.45809, -1.05423, 1.06763, -1.73939, -2.47077, 1.62315],
    [1.4622, -1.04667, 1.08155, -1.74571, -2.4879, 1.63101],
    [1.46636, -1.03901, 1.09565, -1.75213, -2.50527, 1.63898],
    [1.47057, -1.03125, 1.10994, -1.75862, -2.52285, 1.64704],
    [1.47484, -1.0234, 1.1244, -1.76519, -2.54065, 1.65521],
    [1.47915, -1.01545, 1.13902, -1.77184, -2.55865, 1.66346],
    [1.48351, -1.00743, 1.15381, -1.77857, -2.57685, 1.67181],
    [1.48792, -0.999312, 1.16875, -1.78536, -2.59524, 1.68025],
    [1.49237, -0.991117, 1.18383, -1.79222, -2.61381, 1.68877],
    [1.49686, -0.982845, 1.19907, -1.79914, -2.63256, 1.69737],
    [1.50139, -0.974498, 1.21444, -1.80613, -2.65148, 1.70605],
    [1.50597, -0.96608, 1.22994, -1.81318, -2.67056, 1.7148],
    [1.51058, -0.957594, 1.24556, -1.82028, -2.68979, 1.72362],
    [1.51522, -0.949044, 1.2613, -1.82744, -2.70917, 1.73251],
    [1.5199, -0.940432, 1.27716, -1.83465, -2.72869, 1.74147],
    [1.52461, -0.931761, 1.29313, -1.84191, -2.74835, 1.75048],
    [1.52935, -0.923036, 1.30919, -1.84921, -2.76812, 1.75956],
    [1.53411, -0.914259, 1.32535, -1.85656, -2.78802, 1.76868],
    [1.53891, -0.905433, 1.3416, -1.86395, -2.80802, 1.77786],
    [1.54372, -0.896563, 1.35794, -1.87137, -2.82813, 1.78708],
    [1.54856, -0.88765, 1.37435, -1.87883, -2.84833, 1.79635],
    [1.55343, -0.878698, 1.39083, -1.88633, -2.86862, 1.80566],
    [1.55831, -0.869711, 1.40738, -1.89385, -2.88899, 1.815],
    [1.56321, -0.860691, 1.42399, -1.9014, -2.90943, 1.82438],
    [1.56812, -0.851642, 1.44065, -1.90898, -2.92994, 1.83379],
    [1.57305, -0.842567, 1.45736, -1.91657, -2.95051, 1.84322],
    [1.57799, -0.83347, 1.47411, -1.92419, -2.97113, 1.85268],
    [1.58294, -0.824353, 1.4909, -1.93182, -2.99179, 1.86216],
    [1.5879, -0.81522, 1.50771, -1.93946, -3.01249, 1.87165],
    [1.59287, -0.806074, 1.52456, -1.94712, -3.03322, 1.88116],
    [1.59784, -0.796919, 1.54141, -1.95479, -3.05398, 1.89068],
    [1.60282, -0.787757, 1.55828, -1.96246, -3.07474, 1.90021],
    [1.6078, -0.778591, 1.57516, -1.97013, -3.09552, 1.90974],
]


def build_program(path, a, v, r):
    """Construye UN unico programa URScript con todos los movej encadenados.

    Todos los puntos se encolan y se mezclan (blend) sin parar en cada
    waypoint -> movimiento fluido, sin saltarse configuraciones. El primer y ultimo
    punto van con r=0 para terminar y empezar exactamente desde su meta de reposo.
    """
    lines = [
        "def trayectoria():",
        "  rtde_set_watchdog(\"input_int_register_24\", 0, \"ignore\")",
        "  set_tool_voltage(24)",
        "  set_tool_communication(True, 1000000, 2, 1, 1.5, 3.5)",
    ]
    
    # 1. Obtenemos la posicion actual del robot al iniciar
    lines.append("  q_act = get_actual_joint_positions()")
    
    # 2. Calculamos la diferencia entre la j6 actual y la j6 del primer punto
    target_j6 = path[0][5]
    lines.append(f"  target_j6 = {target_j6}")
    lines.append("  j6_offset = floor((q_act[5] - target_j6) / 3.14159265 + 0.5) * 3.14159265")
    
    # 3. Sumar j6_offset a joint 6
    n = len(path)
    for i, q in enumerate(path):
        blend = 0.0 if (i == 0 or i == n - 1) else r
        q_str = f"[{q[0]}, {q[1]}, {q[2]}, {q[3]}, {q[4]}, {q[5]} + j6_offset]"
        lines.append(f"  movej({q_str}, a={a}, v={v}, r={blend})")
        
    lines.append("end")
    lines.append("trayectoria()")
    return "\n".join(lines) + "\n"


def estimate_duration(path, v):
    """Estimacion del tiempo total, con margen para evitar solapamiento."""
    total = 0.0
    for i in range(1, len(path)):
        dq = max(abs(path[i][j] - path[i - 1][j]) for j in range(6))
        total += dq / v
    # Cada waypoint en un camino denso tiene una pequeña demora de procesamiento y blend
    return max(total + 2.0, len(path) * 0.18)


def send_trajectory(path, sock, a, v, r, label=""):
    """Envia una trayectoria completa y espera (estimado) a que termine."""
    program = build_program(path, a, v, r)
    print(f"--- {label} ---")
    print(program)
    sock.send(program.encode())
    time.sleep(estimate_duration(path, v))


# 1. Ir a HOME primero (primera posición de la trayectoria de tilt_x)
sys.path.insert(0, SCRIPT_DIR)
from go_home import go_home
go_home()
time.sleep(1.0)

# Asegurar continuidad en joint 6 (unwrap) partiendo del HOME para evitar giros de 360 grados
print("Asegurando continuidad en joint 6...")
current_j6 = 0.892212  # j6 de go_home()
for q in path:
    diff = q[5] - current_j6
    diff_normalized = (diff + math.pi) % (2 * math.pi) - math.pi
    q[5] = current_j6 + diff_normalized
    current_j6 = q[5]

# Conexion por socket al controlador
sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
sock.connect((HOST, PORT))

# Ida: de la pose inicial a la final
send_trajectory(path, sock, ACC, VEL, BLEND, "Ida")

# Mover a las dos configuraciones intermedias en el extremo (con joint 6 alineado dinámicamente)
print("Moviendo a Config 1...")
config_1 = [1.59436, -0.38397, 0.69935, -0.17733, -3.07143, 3.28436]
prog_1 = (
    "def step1():\n"
    "  rtde_set_watchdog(\"input_int_register_24\", 0, \"ignore\")\n"
    "  set_tool_voltage(24)\n"
    "  set_tool_communication(True, 1000000, 2, 1, 1.5, 3.5)\n"
    "  q_act = get_actual_joint_positions()\n"
    f"  target_j6 = {config_1[5]}\n"
    "  j6_offset = floor((q_act[5] - target_j6) / 3.14159265 + 0.5) * 3.14159265\n"
    f"  movej([{config_1[0]}, {config_1[1]}, {config_1[2]}, {config_1[3]}, {config_1[4]}, target_j6 + j6_offset], a={ACC}, v={VEL})\n"
    "end\n"
    "step1()\n"
)
sock.sendall(prog_1.encode())
time.sleep(1.0)

print("Moviendo a Config 2...")
config_2 = [1.59453, -0.46269, 0.91194, -0.52953, -3.07196, 3.06602]
prog_2 = (
    "def step2():\n"
    "  rtde_set_watchdog(\"input_int_register_24\", 0, \"ignore\")\n"
    "  set_tool_voltage(24)\n"
    "  set_tool_communication(True, 1000000, 2, 1, 1.5, 3.5)\n"
    "  q_act = get_actual_joint_positions()\n"
    f"  target_j6 = {config_2[5]}\n"
    "  j6_offset = floor((q_act[5] - target_j6) / 3.14159265 + 0.5) * 3.14159265\n"
    f"  movej([{config_2[0]}, {config_2[1]}, {config_2[2]}, {config_2[3]}, {config_2[4]}, target_j6 + j6_offset], a={ACC}, v={VEL})\n"
    "end\n"
    "step2()\n"
)
sock.sendall(prog_2.encode())
time.sleep(1.0)

# Abrir pinza
print("Abriendo pinza...")
with open(ABRIR_PINZA, "rb") as f:
    sock.sendall(f.read())
time.sleep(3.0)

# Vuelta: misma trayectoria invertida -> regresa exactamente a la pose inicial
send_trajectory(path[::-1], sock, ACC, VEL, BLEND, "Vuelta")

print("Trayectoria finalizada")

# Cerrar la conexion
sock.close()

# Al final: ir a HOME
print("Regresando a HOME al finalizar...")
go_home()
