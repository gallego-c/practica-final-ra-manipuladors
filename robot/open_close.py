import socket
import time

# ⚠️ ¡ATENCIÓN! Cambia la X por el número real de tu robot
HOST = "10.10.73.234" 
PORT = 30002

# Nombres de los scripts de la pinza
Abrir_pinza = 'scripts/gripper/pinza40UR3.py'  # 40mm (abierta)
Cerrar_pinza = 'scripts/gripper/pinza10UR3.py' # 10mm (cerrada)

print(f"Conectando al robot en la IP: {HOST} ...")
sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
sock.connect((HOST, PORT))
print("¡Conexión establecida!")

# 1. ABRIR PINZA
print("Abriendo pinza...")
with open(Abrir_pinza, 'rb') as f: 
    sock.sendall(f.read())
time.sleep(3)

# 2. CERRAR PINZA
print("Cerrando pinza...")
with open(Cerrar_pinza, 'rb') as f: 
    sock.sendall(f.read())
time.sleep(3)

# 3. ABRIR PINZA DE NUEVO (Para dejarla en el estado inicial)
print("Abriendo pinza otra vez...")
with open(Abrir_pinza, 'rb') as f: 
    sock.sendall(f.read())
time.sleep(3)

print("Test de pinza finalizado.")

# Cerrar conexión
sock.close()