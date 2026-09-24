"""
Seguimiento: convierte el objetivo de la visión en objetivos para el robot.

Base (persona o mano), dos geometrías de cámara:
    "fija"  La webcam está quieta junto al robot. El ángulo es absoluto:
            base = centro + signo · (x - 0.5) · FOV · ganancia
            donde `centro` es el ángulo de la base que apunta al centro de la
            imagen (en v7 la base va de +20° a -200° y 0° es donde encendió).
    "base"  La cámara va montada en la base y gira con ella (lazo cerrado).
            Se corrige de forma incremental: base = actual + signo · error · ganancia.

Control con la mano (modo MANO de la visión):
    Los dedos funcionan como embrague. Con `mano_dedos` dedos extendidos el
    robot sigue la mano: la base con la x y, si está calibrado, el eslabón 1
    con la altura (arriba en la imagen = e1_max). Con menos dedos se pausa:
    STOP si se estaba moviendo, y no manda nada hasta que vuelvan los dedos.

El firmware acepta un GOTO nuevo mientras se mueve y corrige el destino al
vuelo, así que basta con mandar objetivos a ~10 Hz. Solo se manda cuando el
objetivo cambió más que la zona muerta. Ojo: el firmware mueve un eje a la
vez, así que con base y E1 activos los movimientos se alternan.
"""

import threading
import time


def _redondear(v, lo, hi):
    return round(min(max(v, lo + 0.5), hi - 0.5), 1) + 0.0   # + 0.0 evita "-0.0"


class Seguimiento:
    def __init__(self, robot, vision, hz=10.0):
        self.robot, self.vision = robot, vision
        self.periodo = 1.0 / hz
        self.activo = False
        self.p = {
            "camara": "fija",     # "fija" | "base"
            "centro": 0.0,        # fija: ángulo de base que mira al centro de la imagen (°)
            "fov": 60.0,          # campo de visión horizontal de la webcam (°)
            "ganancia": 1.0,      # fija: escala del ángulo; base: fracción del error por paso
            "invertir": False,    # invertir si la base gira al lado contrario
            "zona_muerta": 1.5,   # ° mínimos de cambio para mandar un GOTO
            "suavizado": 0.35,    # 0-1, filtro exponencial sobre el ángulo deseado
            "timeout": 0.5,       # s sin objetivo -> se queda quieto
            # --- control con la mano ---
            "mano_dedos": 4,      # dedos extendidos para que la mano mueva al robot
            "mano_e1": True,      # la altura de la mano mueve el eslabón 1
            "e1_min": 10.0,       # E1 con la mano abajo en la imagen (°)
            "e1_max": 60.0,       # E1 con la mano arriba en la imagen (°)
            "e1_invertir": False,
            "e1_zona_muerta": 3.0,
        }
        self.texto = "apagado"
        self.deseado = self.enviado = None
        self.deseado_e1 = self.enviado_e1 = None
        self.pausa = False
        self._f_base = self._f_e1 = None
        threading.Thread(target=self._bucle, daemon=True, name="seguimiento").start()

    def activar(self, si):
        self.activo = bool(si)
        self._reiniciar()
        if self.activo:
            self.vision.activar(True)
        else:
            self.texto = "apagado"

    def _reiniciar(self):
        self._f_base = self._f_e1 = None
        self.enviado = self.enviado_e1 = None
        self.pausa = self.vision.pausa = False

    def _bucle(self):
        while True:
            time.sleep(self.periodo)
            if self.activo:
                try:
                    self._paso()
                except Exception as e:
                    self.texto = f"error: {e}"

    def _paso(self):
        r, p = self.robot, self.p
        est = r.estado
        if not r.conectado or not est:
            self.texto = "sin robot"
            return
        if r.modo != "PC":
            self.texto = "el control lo tiene la web del ESP32"
            return
        if self.vision.modelos.estado != "listo":
            self.texto = f"visión {self.vision.modelos.estado}"
            return
        o = self.vision.objetivo
        if not o or time.time() - o["t"] > p["timeout"]:
            self.texto = "sin objetivo"
            self._f_base = self._f_e1 = None
            self.pausa = self.vision.pausa = False
            return

        mano = o["tipo"] == "mano"
        if mano and o.get("dedos", 0) < p["mano_dedos"]:
            if not self.pausa and est.get("ocu"):
                r.cmd("STOP")                     # congelar donde va
            self._reiniciar()
            self.pausa = self.vision.pausa = True
            self.texto = f"mano con {o.get('dedos', 0)} dedos: en pausa (muestra {p['mano_dedos']})"
            return
        self.pausa = self.vision.pausa = False

        avisos = []
        self._base(o, est, avisos)
        if mano and p["mano_e1"]:
            if est.get("cal"):
                self._e1(o, est, avisos)
            else:
                avisos.append("E1 sin calibrar")
        self.texto = f"siguiendo {o['tipo']}" + (f" ({', '.join(avisos)})" if avisos else "")

    def _base(self, o, est, avisos):
        p = self.p
        signo = 1.0 if p["invertir"] else -1.0   # objeto a la derecha -> giro negativo
        error = (o["x"] - 0.5) * p["fov"]        # ° respecto al centro de la imagen
        if p["camara"] == "base":
            if abs(error) < p["zona_muerta"]:
                avisos.append("centrado")
                return
            deseado = est["b"] + signo * error * p["ganancia"]
        else:
            deseado = p["centro"] + signo * error * p["ganancia"]

        a = min(max(p["suavizado"], 0.0), 0.95)
        self._f_base = deseado if self._f_base is None else a * self._f_base + (1 - a) * deseado
        self.deseado = _redondear(self._f_base, est["lim"][0], est["lim"][1])
        if self.enviado is None or abs(self.deseado - self.enviado) >= p["zona_muerta"]:
            resp = self.robot.cmd(f"GOTO B {self.deseado:.1f}")
            if resp == "OK GOTO":
                self.enviado = self.deseado
            else:
                avisos.append(f"base: {resp}")

    def _e1(self, o, est, avisos):
        p = self.p
        f = o["y"] if p["e1_invertir"] else 1.0 - o["y"]   # y crece hacia abajo en la imagen
        deseado = p["e1_min"] + f * (p["e1_max"] - p["e1_min"])
        a = min(max(p["suavizado"], 0.0), 0.95)
        self._f_e1 = deseado if self._f_e1 is None else a * self._f_e1 + (1 - a) * deseado
        self.deseado_e1 = _redondear(self._f_e1, est["lim"][2], est["lim"][3])
        if self.enviado_e1 is None or abs(self.deseado_e1 - self.enviado_e1) >= p["e1_zona_muerta"]:
            resp = self.robot.cmd(f"GOTO E1 {self.deseado_e1:.1f}")
            if resp == "OK GOTO":
                self.enviado_e1 = self.deseado_e1
            else:
                avisos.append(f"E1: {resp}")

    def resumen(self):
        return {"activo": self.activo, "texto": self.texto, "pausa": self.pausa,
                "deseado": self.deseado, "enviado": self.enviado,
                "deseado_e1": self.deseado_e1, "enviado_e1": self.enviado_e1, "p": self.p}
