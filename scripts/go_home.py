import socket
import time
import math

# go_home.py -- Mueve el UR3 a la configuracion HOME antes del pick.

HOST = "10.10.73.239"   # IP del controlador del UR
PORT = 30002            # Secondary client interface

ACC   = 0.8     # aceleracion articular [rad/s^2]
VEL   = 0.6     # velocidad articular   [rad/s]

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
    
    print("Moviendo a la pose HOME...")
    print(program)
    sock.send(program.encode())
    
    # Esperar a que termine el movimiento
    time.sleep(5.0)
    
    sock.close()
    print("Robot en pose HOME y conexion cerrada.")

if __name__ == "__main__":
    go_home()
