import socket
import time
import math

# go_home.py -- Mueve el UR3 a la configuracion HOME antes del pick.

HOST = "10.10.73.239"   # IP del controlador del UR
PORT = 30002            # Secondary client interface

ACC   = 0.3     # aceleracion articular suave [rad/s^2]
VEL   = 0.3     # velocidad articular suave   [rad/s]

# HOME pose: Primera posicion del path tilt_x_ur3 (en radianes)
home_config = [0.844914, -0.987856, 0.536689, -1.13289, -1.57184, 0.892212]

def go_home():
    print(f"Conectando al robot en {HOST}:{PORT} para ir a HOME...")
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.connect((HOST, PORT))
    print("Conexion establecida.")

    # Construir programa URScript
    q_str = "[" + ", ".join(str(x) for x in home_config) + "]"
    program = f"def home():\n  movej({q_str}, a={ACC}, v={VEL})\nend\nhome()\n"
    
    print("Moviendo a la pose HOME de forma suave...")
    print(program)
    sock.send(program.encode())
    
    # Esperar lo suficiente para que el movimiento termine por completo
    time.sleep(8.0)
    
    # IMPORTANTE: Enviar el script de la pinza para inicializar y mantener el bus de campo activo
    import os
    SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
    PINZA_SCRIPT = os.path.join(SCRIPT_DIR, "gripper", "pinza40UR3.py")
    print("Inicializando la pinza para mantener el bus de campo activo...")
    with open(PINZA_SCRIPT, "rb") as f:
        sock.sendall(f.read())
    time.sleep(1.0)
    
    sock.close()
    print("Robot en pose HOME y conexion cerrada.")

if __name__ == "__main__":
    go_home()
