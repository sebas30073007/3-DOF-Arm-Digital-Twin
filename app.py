#!/usr/bin/env python3
"""
Panel local del manipulador: cámara + visión + control por serial.

    python app.py                     # autodetecta la ESP32 y abre el navegador
    python app.py --puerto COM7       # puerto fijo
    python app.py --camara 1          # otra webcam
    python app.py --vision            # arranca con la visión encendida

Luego abre http://localhost:8000 (se abre solo salvo --no-abrir).
"""

import argparse
import threading
import webbrowser
from pathlib import Path

from manipulador.robot import Robot
from manipulador.seguimiento import Seguimiento
from manipulador.servidor import crear_servidor
from manipulador.vision import Camara, Vision

RAIZ = Path(__file__).resolve().parent


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--puerto", default=None, help="Puerto serial de la ESP32 (por defecto: autodetectar)")
    p.add_argument("--baud", type=int, default=115200)
    p.add_argument("--camara", type=int, default=0, help="Índice de la webcam")
    p.add_argument("--host", default="127.0.0.1", help="0.0.0.0 para abrirlo a la red local")
    p.add_argument("--http", type=int, default=8000, help="Puerto de la página")
    p.add_argument("--vision", action="store_true", help="Encender la visión al arrancar")
    p.add_argument("--no-abrir", action="store_true", help="No abrir el navegador")
    args = p.parse_args()

    robot = Robot(args.puerto, args.baud)
    camara = Camara(args.camara)
    vision = Vision(camara, RAIZ / "models")
    seguimiento = Seguimiento(robot, vision)
    if args.vision:
        vision.activar(True)

    srv = crear_servidor(args.host, args.http, robot, camara, vision, seguimiento)
    url = f"http://localhost:{args.http}"
    print(f"Panel en {url}   (Ctrl+C para salir)")
    if not args.no_abrir:
        threading.Timer(0.8, webbrowser.open, (url,)).start()

    try:
        srv.serve_forever(poll_interval=0.25)
    except KeyboardInterrupt:
        print("\nCerrando...")
    finally:
        seguimiento.activar(False)
        robot.cerrar()
        srv.server_close()


if __name__ == "__main__":
    main()
