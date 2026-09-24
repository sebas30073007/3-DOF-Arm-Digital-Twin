"""
Enlace serial con el firmware manipulador_v7 (ESP32-C3, USB CDC).

Un hilo mantiene la conexión (reintenta y autodetecta el puerto), otro lee
líneas. Cada comando se manda con `cmd()`, que espera la primera respuesta
que no sea aviso (MSG ...). El estado se refresca solo con STATE? cada
`periodo` segundos.

Protocolo (ver firmware/manipulador_v7/manipulador_v7.ino):
    PING  STATE?  MODE?  STOP  CAL  HOME  SALUDO
    GOTO <B|E1|E2> <grados>     POSE <b|NA> <e1|NA> <e2|NA>
    VEL <B|E> <rpm> [rpm/s]     PULSOS <B|E> <200|800|1000>

El firmware no tiene jog por serial: aquí se emula con GOTO al límite
mientras llegan latidos, y STOP al soltar o si se pierde el latido.
"""

import json
import queue
import threading
import time
from collections import deque

import serial
from serial.tools import list_ports

VID_ESPRESSIF = 0x303A
EJES = ("B", "E1", "E2")          # nombres en el protocolo
MOVIMIENTO = {"CAL", "HOME", "SALUDO", "GOTO", "POSE"}
LATIDO_JOG = 0.5                  # s sin latido -> STOP


def puertos():
    return [
        {"puerto": p.device, "desc": p.description, "esp32": p.vid == VID_ESPRESSIF}
        for p in list_ports.comports()
    ]


def buscar_esp32():
    for p in list_ports.comports():
        if p.vid == VID_ESPRESSIF:
            return p.device
    return None


class Robot:
    def __init__(self, puerto=None, baud=115200, periodo=0.15):
        self.puerto_pedido = puerto     # None = autodetectar
        self.puerto = None
        self.baud = baud
        self.periodo = periodo

        self.ser = None
        self.estado = None              # último JSON de STATE
        self.t_estado = 0.0
        self.modo = None                # "PC" | "WEB"
        self.error = ""

        self._lock_cmd = threading.Lock()
        self._resp = queue.Queue()
        self._silencio = False          # no registrar la respuesta del sondeo
        self._log = deque(maxlen=600)
        self._seq = 0
        self._lock_log = threading.Lock()
        self._jog = None                # [eje, signo, t_ultimo_latido]
        self._reconectar = threading.Event()

        threading.Thread(target=self._bucle, daemon=True, name="robot").start()

    # ------------------------------------------------------------ registro --

    def _registrar(self, direccion, texto):
        with self._lock_log:
            self._seq += 1
            self._log.append({"n": self._seq, "t": time.time(), "d": direccion, "x": texto})

    def log_desde(self, n):
        with self._lock_log:
            return self._seq, [l for l in self._log if l["n"] > n]

    # ------------------------------------------------------------ conexión --

    @property
    def conectado(self):
        return self.ser is not None

    def cambiar_puerto(self, puerto):
        self.puerto_pedido = puerto or None
        self._reconectar.set()

    def _abrir(self):
        p = self.puerto_pedido or buscar_esp32()
        if not p:
            self.error = "No se encontró la ESP32 (VID 303A)"
            return False
        try:
            s = serial.Serial(p, self.baud, timeout=0.1, write_timeout=0.5)
        except serial.SerialException as e:
            ocupado = "PermissionError" in str(e)
            self.error = f"{p} ocupado (¿monitor serial abierto?)" if ocupado else str(e)
            return False
        time.sleep(0.3)
        s.reset_input_buffer()
        self.ser, self.puerto, self.error = s, p, ""
        threading.Thread(target=self._lector, args=(s,), daemon=True, name="serial-rx").start()
        self._registrar("sys", f"Conectado a {p}")
        return True

    def _cerrar(self, motivo=""):
        s, self.ser = self.ser, None
        self.estado, self.modo, self._jog = None, None, None
        if s:
            try:
                s.close()
            except Exception:
                pass
            self._registrar("sys", f"Desconectado{': ' + motivo if motivo else ''}")

    def _lector(self, s):
        buf = b""
        while self.ser is s:
            try:
                buf += s.read(256)
            except Exception as e:
                if self.ser is s:
                    self.error = str(e)
                    self._cerrar("se perdió el puerto")
                return
            while b"\n" in buf:
                linea, buf = buf.split(b"\n", 1)
                self._recibir(linea.decode(errors="ignore").strip())

    def _recibir(self, linea):
        if not linea:
            return
        if linea.startswith("MSG "):
            self._registrar("msg", linea[4:])
            return
        if linea.startswith("STATE "):
            try:
                self.estado = json.loads(linea[6:])
                self.t_estado = time.time()
            except ValueError:
                pass
        elif linea.startswith("MODE "):
            self.modo = linea[5:].strip()
        if not self._silencio:
            self._registrar("rx", linea)
        self._resp.put(linea)

    def _bucle(self):
        t_modo = 0.0
        while True:
            if self._reconectar.is_set():
                self._reconectar.clear()
                self._cerrar("cambio de puerto")
            if not self.conectado:
                if not self._abrir():
                    self._reconectar.wait(1.5)
                    continue
                self.cmd("PING")
            ahora = time.time()
            self.cmd("STATE?", silencioso=True)
            if ahora - t_modo > 1.0:
                self.cmd("MODE?", silencioso=True)
                t_modo = ahora
            if self._jog and ahora - self._jog[2] > LATIDO_JOG:
                self._jog = None
                self.cmd("STOP")
            time.sleep(self.periodo)

    # ------------------------------------------------------------ comandos --

    def cmd(self, linea, silencioso=False, timeout=1.0):
        """Manda una línea y regresa la respuesta (o None si no hubo)."""
        linea = linea.strip()
        if not linea:
            return None
        with self._lock_cmd:
            s = self.ser
            if s is None:
                return "ERR SIN_CONEXION"
            while not self._resp.empty():
                self._resp.get_nowait()
            self._silencio = silencioso
            if not silencioso:
                self._registrar("tx", linea)
            try:
                s.write((linea + "\n").encode())
            except Exception as e:
                self._silencio = False
                self._cerrar(str(e))
                return "ERR SIN_CONEXION"
            try:
                r = self._resp.get(timeout=timeout)
            except queue.Empty:
                r = None
                if not silencioso:
                    self._registrar("sys", "(sin respuesta)")
            self._silencio = False
            return r

    def limites(self, eje):
        if not self.estado:
            return None
        lim = self.estado["lim"]
        return lim[2 * eje], lim[2 * eje + 1]

    def jog(self, eje, signo):
        """Llamar cada ~150 ms mientras se sostiene el botón; signo 0 = soltar."""
        if signo == 0:
            if self._jog:
                self._jog = None
                return self.cmd("STOP")
            return "OK"
        if self._jog and self._jog[0] == eje and self._jog[1] == signo:
            self._jog[2] = time.time()
            return "OK JOG"
        lim = self.limites(eje)
        if lim is None:
            return "ERR SIN_ESTADO"
        r = self.cmd(f"GOTO {EJES[eje]} {lim[1] if signo > 0 else lim[0]:.1f}")
        self._jog = [eje, signo, time.time()] if r == "OK GOTO" else None
        return r

    def resumen(self):
        return {
            "conectado": self.conectado,
            "puerto": self.puerto if self.conectado else self.puerto_pedido,
            "auto": self.puerto_pedido is None,
            "modo": self.modo,
            "error": self.error,
            "estado": self.estado,
            "edad": round(time.time() - self.t_estado, 2) if self.estado else None,
            "jog": self._jog[:2] if self._jog else None,
        }

    def cerrar(self):
        if self.conectado:
            self.cmd("STOP")
        self._cerrar()
