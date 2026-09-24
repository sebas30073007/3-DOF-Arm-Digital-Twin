"""
Cámara + visión.

- `Camara`: hilo que lee la webcam y guarda solo el último frame.
- `Vision`: hilo que toma ese frame, corre los modelos si la visión está
  activa, decide a quién seguir, dibuja el overlay y lo codifica a JPEG
  para el stream MJPEG.

Detección:
    Personas  -> YOLO (yolo11n, clase 0 de COCO) sobre el frame completo.
    Manos     -> MediaPipe Hand Landmarker (modo VIDEO, con tracking) sobre
                 el frame completo. Una palma "cerca" ocupa buena parte de la
                 imagen, así que no hace falta recortar.

Objetivo (qué sigue el robot):
    1. Palma abierta y cerca durante `frames_mano` frames -> modo MANO.
       En modo mano se sigue la mano más grande, aunque cierre los dedos,
       hasta que no se vea ninguna durante `soltar_mano_s`.
    2. Si no, la persona más grande, con preferencia por la que ya se seguía.

Todas las coordenadas del objetivo están normalizadas (0-1) y en el marco
de la cámara SIN espejo: el espejo es solo de visualización.
"""

import sys
import threading
import time
import urllib.request
from pathlib import Path

import cv2
import numpy as np

HAND_MODEL_URL = (
    "https://storage.googleapis.com/mediapipe-models/hand_landmarker/"
    "hand_landmarker/float16/latest/hand_landmarker.task"
)

# Colores BGR (paleta SEBS)
BLANCO = (255, 255, 255)
ALLOY = (155, 147, 142)
SENAL = (19, 6, 227)

TIPS, PIPS = (8, 12, 16, 20), (6, 10, 14, 18)
CONEXIONES_MANO = (
    (0, 1), (1, 2), (2, 3), (3, 4), (0, 5), (5, 6), (6, 7), (7, 8),
    (5, 9), (9, 10), (10, 11), (11, 12), (9, 13), (13, 14), (14, 15),
    (15, 16), (13, 17), (17, 18), (18, 19), (19, 20), (0, 17),
)


# ================================================================ cámara ===

class Camara:
    def __init__(self, indice=0, ancho=640, alto=480):
        self.indice, self.ancho, self.alto = indice, ancho, alto
        self.ok = False
        self._frame, self._seq = None, 0
        self._lock = threading.Lock()
        self._cambio = threading.Event()
        threading.Thread(target=self._bucle, daemon=True, name="camara").start()

    def cambiar(self, indice):
        if indice != self.indice:
            self.indice = indice
            self._cambio.set()

    def ultimo(self):
        with self._lock:
            return self._seq, self._frame

    def _abrir(self):
        api = cv2.CAP_DSHOW if sys.platform == "win32" else cv2.CAP_ANY
        cap = cv2.VideoCapture(self.indice, api)
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.ancho)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.alto)
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        return cap

    def _bucle(self):
        while True:
            self._cambio.clear()
            cap = self._abrir()
            fallas = 0
            while not self._cambio.is_set():
                ok, frame = cap.read()
                if not ok:
                    self.ok = False
                    fallas += 1
                    if fallas > 30:
                        break                   # reabrir
                    time.sleep(0.05)
                    continue
                fallas, self.ok = 0, True
                with self._lock:
                    self._frame, self._seq = frame, self._seq + 1
            cap.release()
            self.ok = False
            self._cambio.wait(1.0)


# ============================================================ detectores ===

def dedos_extendidos(lm):
    """Cuenta dedos extendidos (sin pulgar), independiente de la orientación:
    la punta está más lejos de la muñeca que la articulación media."""
    w = np.array([lm[0].x, lm[0].y])
    n = 0
    for tip, pip in zip(TIPS, PIPS):
        d_tip = np.linalg.norm(np.array([lm[tip].x, lm[tip].y]) - w)
        d_pip = np.linalg.norm(np.array([lm[pip].x, lm[pip].y]) - w)
        n += d_tip > d_pip * 1.15
    return int(n)


class Modelos:
    """Carga perezosa de YOLO y MediaPipe (tarda unos segundos)."""

    def __init__(self, dir_modelos: Path):
        self.dir = Path(dir_modelos)
        self.estado = "sin cargar"
        self.yolo = None
        self.manos = None
        self._t0 = time.monotonic()
        self._ts = -1

    def cargar(self):
        if self.estado in ("cargando", "listo"):
            return
        self.estado = "cargando"
        threading.Thread(target=self._cargar, daemon=True, name="modelos").start()

    def _cargar(self):
        try:
            from ultralytics import YOLO
            import mediapipe as mp
            from mediapipe.tasks import python as mp_python
            from mediapipe.tasks.python import vision as mp_vision

            self.yolo = YOLO(str(self.dir / "yolo11n.pt"))

            ruta = self.dir / "hand_landmarker.task"
            if not ruta.exists():
                ruta.parent.mkdir(parents=True, exist_ok=True)
                urllib.request.urlretrieve(HAND_MODEL_URL, ruta)
            opciones = mp_vision.HandLandmarkerOptions(
                base_options=mp_python.BaseOptions(model_asset_path=str(ruta)),
                running_mode=mp_vision.RunningMode.VIDEO,
                num_hands=2,
                min_hand_detection_confidence=0.5,
                min_hand_presence_confidence=0.5,
                min_tracking_confidence=0.5,
            )
            self.manos = mp_vision.HandLandmarker.create_from_options(opciones)
            self._mp = mp
            self.estado = "listo"
        except Exception as e:           # se muestra en la página
            self.estado = f"error: {e}"

    def personas(self, frame, conf):
        h, w = frame.shape[:2]
        r = self.yolo.predict(frame, classes=[0], conf=conf, verbose=False)[0]
        out = []
        for (x1, y1, x2, y2), c in zip(r.boxes.xyxy.cpu().numpy(), r.boxes.conf.cpu().numpy()):
            caja = tuple(float(v) for v in (x1 / w, y1 / h, x2 / w, y2 / h))
            out.append({"caja": caja, "conf": float(c)})
        out.sort(key=lambda p: p["caja"][3] - p["caja"][1], reverse=True)
        return out

    def detectar_manos(self, frame):
        img = self._mp.Image(image_format=self._mp.ImageFormat.SRGB,
                             data=cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))
        # MediaPipe exige timestamps estrictamente crecientes
        self._ts = max(int((time.monotonic() - self._t0) * 1000), self._ts + 1)
        res = self.manos.detect_for_video(img, self._ts)
        h, w = frame.shape[:2]
        out = []
        for lm in res.hand_landmarks or []:
            xs = [p.x for p in lm]
            ys = [p.y for p in lm]
            # tamaño en fracción de la ALTURA del frame, igual en ambos ejes
            tam = float(max((max(xs) - min(xs)) * w, (max(ys) - min(ys)) * h) / h)
            out.append({
                "puntos": [(p.x, p.y) for p in lm],
                "centro": ((lm[0].x + lm[9].x) / 2, (lm[0].y + lm[9].y) / 2),
                "tam": tam,
                "dedos": dedos_extendidos(lm),
            })
        out.sort(key=lambda m: m["tam"], reverse=True)
        return out


def iou(a, b):
    ix = max(0.0, min(a[2], b[2]) - max(a[0], b[0]))
    iy = max(0.0, min(a[3], b[3]) - max(a[1], b[1]))
    inter = ix * iy
    union = (a[2] - a[0]) * (a[3] - a[1]) + (b[2] - b[0]) * (b[3] - b[1]) - inter
    return inter / union if union > 0 else 0.0


# ================================================================ visión ===

class Vision:
    def __init__(self, camara: Camara, dir_modelos: Path):
        self.camara = camara
        self.modelos = Modelos(dir_modelos)
        self.activa = False
        self.p = {
            "conf": 0.5,          # confianza mínima YOLO
            "yolo_cada": 2,       # correr YOLO 1 de cada N frames
            "umbral_mano": 0.22,  # tamaño de mano (fracción de la altura) para "cerca"
            "dedos_min": 4,       # dedos extendidos para "palma abierta"
            "frames_mano": 4,     # frames seguidos de palma cerca para entrar a modo mano
            "soltar_mano_s": 0.8, # s sin ver mano para salir del modo mano
            "espejo": True,       # solo visualización
        }
        self.personas, self.manos = [], []
        self.objetivo = None      # {"tipo", "x", "y", "tam", "t"}
        self.modo_mano = False
        self.pausa = False        # lo pone el seguimiento: mano sin los dedos para mover
        self.fps = 0.0
        self.error = ""              # último error de detección (no detiene el bucle)

        self._racha_mano = 0
        self._t_mano = 0.0
        self._caja_prev = None
        self._n = 0
        self._jpeg, self._jpeg_seq = None, 0
        self._cond = threading.Condition()
        threading.Thread(target=self._bucle, daemon=True, name="vision").start()

    def activar(self, si):
        self.activa = bool(si)
        if self.activa:
            self.modelos.cargar()
        else:
            self.personas, self.manos, self.objetivo = [], [], None
            self.modo_mano, self._racha_mano = False, 0

    def jpeg(self, desde_seq, timeout=1.0):
        """Bloquea hasta que haya un frame más nuevo que `desde_seq`."""
        with self._cond:
            self._cond.wait_for(lambda: self._jpeg_seq > desde_seq, timeout)
            return self._jpeg_seq, self._jpeg

    # ------------------------------------------------------------------------

    def _bucle(self):
        visto, cuenta, t_fps = 0, 0, time.monotonic()
        while True:
            seq, frame = self.camara.ultimo()
            if frame is None or seq == visto:
                time.sleep(0.005)
                continue
            visto = seq
            frame = frame.copy()

            if self.activa and self.modelos.estado == "listo":
                try:
                    self._detectar(frame)
                except Exception as e:
                    self.error = f"{type(e).__name__}: {e}"

            vista = cv2.flip(frame, 1) if self.p["espejo"] else frame
            self._dibujar(vista)
            ok, buf = cv2.imencode(".jpg", vista, [cv2.IMWRITE_JPEG_QUALITY, 75])
            if ok:
                with self._cond:
                    self._jpeg, self._jpeg_seq = buf.tobytes(), self._jpeg_seq + 1
                    self._cond.notify_all()

            cuenta += 1
            ahora = time.monotonic()
            if ahora - t_fps >= 1.0:
                self.fps, cuenta, t_fps = cuenta / (ahora - t_fps), 0, ahora

    def _detectar(self, frame):
        p = self.p
        self._n += 1
        if self._n % max(1, int(p["yolo_cada"])) == 0:
            self.personas = self.modelos.personas(frame, p["conf"])
        self.manos = self.modelos.detectar_manos(frame)
        for m in self.manos:
            m["abierta"] = m["dedos"] >= p["dedos_min"]
            m["cerca"] = m["tam"] >= p["umbral_mano"]

        ahora = time.monotonic()
        palma = next((m for m in self.manos if m["abierta"] and m["cerca"]), None)
        self._racha_mano = self._racha_mano + 1 if palma else 0
        if self._racha_mano >= p["frames_mano"]:
            self.modo_mano = True
        if self.manos:
            self._t_mano = ahora
        elif self.modo_mano and ahora - self._t_mano > p["soltar_mano_s"]:
            self.modo_mano = False

        if self.modo_mano and self.manos:
            m = palma or self.manos[0]
            self.objetivo = {"tipo": "mano", "x": m["centro"][0], "y": m["centro"][1],
                             "tam": m["tam"], "dedos": m["dedos"], "t": time.time()}
            return
        if self.modo_mano:
            return                      # mano perdida hace poco: conservar objetivo

        if self.personas:
            elegida = self.personas[0]
            if self._caja_prev:
                mejor = max(self.personas, key=lambda q: iou(q["caja"], self._caja_prev))
                if iou(mejor["caja"], self._caja_prev) > 0.3:
                    elegida = mejor
            x1, y1, x2, y2 = elegida["caja"]
            self._caja_prev = elegida["caja"]
            self.objetivo = {"tipo": "persona", "x": (x1 + x2) / 2, "y": y1 + (y2 - y1) * 0.25,
                             "tam": y2 - y1, "t": time.time()}
        else:
            self._caja_prev = None
            self.objetivo = None

    def _dibujar(self, img):
        if not self.activa:
            return
        h, w = img.shape[:2]
        esp = self.p["espejo"]
        X = (lambda x: int((1 - x) * w)) if esp else (lambda x: int(x * w))
        Y = lambda y: int(y * h)

        for per in self.personas:
            x1, y1, x2, y2 = per["caja"]
            sel = self._caja_prev == per["caja"] and not self.modo_mano
            a, b = sorted((X(x1), X(x2)))
            cv2.rectangle(img, (a, Y(y1)), (b, Y(y2)), BLANCO if sel else ALLOY, 2 if sel else 1)

        for m in self.manos:
            pts = [(X(x), Y(y)) for x, y in m["puntos"]]
            color = BLANCO if m["abierta"] and m["cerca"] else ALLOY
            for i, j in CONEXIONES_MANO:
                cv2.line(img, pts[i], pts[j], color, 1, cv2.LINE_AA)
            for q in pts:
                cv2.circle(img, q, 2, color, -1, cv2.LINE_AA)

        o = self.objetivo
        if o and time.time() - o["t"] < 0.5:
            c = (X(o["x"]), Y(o["y"]))
            color = ALLOY if self.pausa else SENAL
            cv2.circle(img, c, 14, color, 2, cv2.LINE_AA)
            cv2.drawMarker(img, c, color, cv2.MARKER_CROSS, 36, 1, cv2.LINE_AA)
            if o["tipo"] == "mano":
                txt = f"{o['dedos']} dedos" + (" - pausa" if self.pausa else "")
                cv2.putText(img, txt, (c[0] + 20, c[1] - 16), cv2.FONT_HERSHEY_SIMPLEX,
                            0.6, (0, 0, 0), 3, cv2.LINE_AA)
                cv2.putText(img, txt, (c[0] + 20, c[1] - 16), cv2.FONT_HERSHEY_SIMPLEX,
                            0.6, BLANCO, 1, cv2.LINE_AA)
        # centro de la imagen como referencia del error horizontal
        cv2.line(img, (w // 2, h - 18), (w // 2, h - 4), ALLOY, 1)

    def resumen(self):
        o = self.objetivo
        return {
            "activa": self.activa,
            "modelos": self.modelos.estado,
            "error": self.error,
            "fps": round(self.fps, 1),
            "camara_ok": self.camara.ok,
            "camara": self.camara.indice,
            "personas": len(self.personas),
            "manos": [{"dedos": m["dedos"], "tam": round(m["tam"], 2),
                       "abierta": m.get("abierta"), "cerca": m.get("cerca")} for m in self.manos],
            "modo_mano": self.modo_mano,
            "objetivo": None if not o else {
                "tipo": o["tipo"], "x": round(o["x"], 3), "y": round(o["y"], 3), "dedos": o.get("dedos"),
                "edad": round(time.time() - o["t"], 2)},
            "p": self.p,
        }
