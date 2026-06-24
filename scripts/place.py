#!/usr/bin/env python3
import sys
import os

# Asegurar que el directorio scripts está en el path de importaciones
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
if SCRIPT_DIR not in sys.path:
    sys.path.insert(0, SCRIPT_DIR)

from go_home import go_home

def main():
    print("[Action place] Iniciando: abrir pinza e ir a HOME...")
    go_home(open_gripper=True)
    print("[Action place] Completado con éxito.")

if __name__ == "__main__":
    main()
else:
    # Si se importa o recarga, se ejecuta automáticamente para mantener el
    # comportamiento síncrono del resto de scripts de la carpeta.
    main()
