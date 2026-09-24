"""
Servidor HTTP local (solo biblioteca estándar).

    GET  /                 página (web/index.html)
    GET  /static/<arch>    archivos de web/
    GET  /video            stream MJPEG de la cámara con el overlay
    GET  /api/estado?log=N estado de robot, visión y seguimiento + log serial desde N
    GET  /api/puertos      puertos seriales disponibles
    POST /api/cmd          {"linea": "GOTO B 10"}  -> manda una línea al robot
    POST /api/jog          {"eje": 0-2, "signo": -1|0|1}  (latido cada ~150 ms)
    POST /api/config       {"vision": {...}, "seguimiento": {...}, "camara": 0, "puerto": "COM7"}

Cualquier movimiento manual (o STOP) apaga el seguimiento automático.
"""

import json
import mimetypes
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import parse_qs, urlparse

from .robot import MOVIMIENTO, puertos

WEB = Path(__file__).resolve().parent.parent / "web"


def crear_servidor(host, puerto, robot, camara, vision, seguimiento):
    def apagar_seguimiento(motivo):
        if seguimiento.activo:
            seguimiento.activar(False)
            seguimiento.texto = f"apagado por {motivo}"

    def configurar(datos):
        if "puerto" in datos:
            robot.cambiar_puerto(datos["puerto"])
        if "camara" in datos:
            camara.cambiar(int(datos["camara"]))
        for destino, obj in (("vision", vision), ("seguimiento", seguimiento)):
            cambios = datos.get(destino) or {}
            for k, v in cambios.items():
                if k in obj.p:
                    tipo = type(obj.p[k])
                    obj.p[k] = v if tipo is str else tipo(v)
            if "activa" in cambios or "activo" in cambios:
                obj.activar(cambios.get("activa", cambios.get("activo")))
        if not vision.activa:
            apagar_seguimiento("visión apagada")

    class Manejador(BaseHTTPRequestHandler):
        protocol_version = "HTTP/1.1"

        def log_message(self, *a):
            pass

        def _json(self, obj, codigo=200):
            cuerpo = json.dumps(obj).encode()
            self.send_response(codigo)
            self.send_header("Content-Type", "application/json")
            self.send_header("Cache-Control", "no-store")
            self.send_header("Content-Length", str(len(cuerpo)))
            self.end_headers()
            self.wfile.write(cuerpo)

        def _archivo(self, ruta: Path):
            ruta = ruta.resolve()
            if WEB not in ruta.parents or not ruta.is_file():
                return self._json({"error": "no existe"}, 404)
            cuerpo = ruta.read_bytes()
            tipo = mimetypes.guess_type(ruta.name)[0] or "application/octet-stream"
            self.send_response(200)
            self.send_header("Content-Type", tipo + ("; charset=utf-8" if tipo.startswith("text") or tipo.endswith("javascript") else ""))
            self.send_header("Cache-Control", "no-store")
            self.send_header("Content-Length", str(len(cuerpo)))
            self.end_headers()
            self.wfile.write(cuerpo)

        def _mjpeg(self):
            self.send_response(200)
            self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
            self.send_header("Cache-Control", "no-store")
            self.send_header("Connection", "close")
            self.end_headers()
            seq = 0
            try:
                while True:
                    seq, jpg = vision.jpeg(seq)
                    if jpg is None:
                        continue
                    self.wfile.write(b"--frame\r\nContent-Type: image/jpeg\r\nContent-Length: "
                                     + str(len(jpg)).encode() + b"\r\n\r\n" + jpg + b"\r\n")
            except (ConnectionError, OSError):
                pass
            self.close_connection = True

        def do_GET(self):
            url = urlparse(self.path)
            if url.path == "/":
                return self._archivo(WEB / "index.html")
            if url.path.startswith("/static/"):
                return self._archivo(WEB / url.path[len("/static/"):])
            if url.path == "/video":
                return self._mjpeg()
            if url.path == "/api/puertos":
                return self._json(puertos())
            if url.path == "/api/estado":
                desde = int(parse_qs(url.query).get("log", ["0"])[0] or 0)
                seq, lineas = robot.log_desde(desde)
                return self._json({
                    "robot": robot.resumen(),
                    "vision": vision.resumen(),
                    "seguimiento": seguimiento.resumen(),
                    "log": {"seq": seq, "lineas": lineas},
                })
            self._json({"error": "no existe"}, 404)

        def do_POST(self):
            n = int(self.headers.get("Content-Length") or 0)
            try:
                datos = json.loads(self.rfile.read(n) or b"{}")
            except ValueError:
                return self._json({"error": "json inválido"}, 400)
            ruta = urlparse(self.path).path

            if ruta == "/api/cmd":
                linea = str(datos.get("linea", "")).strip()
                palabra = linea.split(" ", 1)[0].upper()
                if palabra == "STOP":
                    apagar_seguimiento("ALTO")
                elif palabra in MOVIMIENTO:
                    apagar_seguimiento("control manual")
                return self._json({"resp": robot.cmd(linea)})

            if ruta == "/api/jog":
                signo = int(datos.get("signo", 0))
                if signo:
                    apagar_seguimiento("control manual")
                return self._json({"resp": robot.jog(int(datos.get("eje", 0)), signo)})

            if ruta == "/api/config":
                try:
                    configurar(datos)
                except (TypeError, ValueError) as e:
                    return self._json({"error": str(e)}, 400)
                return self._json({"ok": True})

            self._json({"error": "no existe"}, 404)

    return ThreadingHTTPServer((host, puerto), Manejador)
