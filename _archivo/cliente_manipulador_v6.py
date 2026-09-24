"""
Cliente mínimo para manipulador_v6 por USB serial.
Requiere: pip install pyserial

Prueba de seguimiento: manda objetivos a la base a 10 Hz siguiendo una senoidal,
igual que lo haría el lazo de visión con la posición de la persona.
Ctrl+C manda STOP.
"""
import json
import math
import time

import serial

PUERTO = "COM7"          # ajusta al tuyo


class Manipulador:
    def __init__(self, puerto):
        self.s = serial.Serial(puerto, 115200, timeout=0.2)
        time.sleep(0.3)
        self.s.reset_input_buffer()

    def cmd(self, linea):
        """Envía un comando y regresa la primera respuesta (ignora avisos MSG)."""
        self.s.write((linea + "\n").encode())
        fin = time.time() + 1.0
        while time.time() < fin:
            r = self.s.readline().decode(errors="ignore").strip()
            if not r:
                continue
            if r.startswith("MSG "):
                print("  ·", r[4:])
                continue
            return r
        return None

    def estado(self):
        r = self.cmd("STATE?")
        return json.loads(r[6:]) if r and r.startswith("STATE ") else None


if __name__ == "__main__":
    m = Manipulador(PUERTO)
    print(m.cmd("PING"), m.cmd("MODE?"))

    try:
        t0 = time.time()
        while True:
            t = time.time() - t0
            # ±15° cada 20 s: pico ~4.7°/s, por debajo de los 6°/s de la base a 10 rpm
            objetivo = 15.0 * math.sin(2 * math.pi * t / 20.0)
            r = m.cmd(f"GOTO B {objetivo:.2f}")
            if r != "OK GOTO":
                print(r)
                break
            e = m.estado()
            print(f"objetivo {objetivo:6.1f}°  base {e['b']:6.1f}°")
            time.sleep(0.1)
    except KeyboardInterrupt:
        pass
    finally:
        print(m.cmd("STOP"))
