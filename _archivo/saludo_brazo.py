#!/usr/bin/env python3
"""
Detección de personas (YOLO) + manos (MediaPipe) para disparar el saludo
del manipulador de 3 GDL.

Pipeline:
    1. YOLO detecta personas en el frame completo (clase 0 de COCO).
    2. Si hay una persona lo bastante cerca (altura del bbox vs. altura del
       frame), se recorta esa región y solo ahí se corre MediaPipe Hands.
       Esto es mucho más rápido y más preciso que correr manos en todo el frame.
    3. Si la mano está abierta durante N frames seguidos, se manda el comando
       de saludo por serial a la ESP32 y arranca un cooldown.

Uso:
    python saludo_brazo.py --puerto COM5            # Windows
    python saludo_brazo.py --puerto /dev/ttyACM0    # Linux
    python saludo_brazo.py                          # sin brazo, solo prueba

Dependencias:
    pip install ultralytics mediapipe opencv-python pyserial
"""

import argparse
import time
import urllib.request
from pathlib import Path

import cv2
import numpy as np

# ---------------------------------------------------------------- modelos ---

HAND_MODEL_PATH = Path("models/hand_landmarker.task")
HAND_MODEL_URL = (
    "https://storage.googleapis.com/mediapipe-models/hand_landmarker/"
    "hand_landmarker/float16/latest/hand_landmarker.task"
)


def asegurar_modelo_manos() -> str:
    """Descarga el .task de MediaPipe la primera vez que se corre."""
    if not HAND_MODEL_PATH.exists():
        HAND_MODEL_PATH.parent.mkdir(parents=True, exist_ok=True)
        print(f"Descargando modelo de manos -> {HAND_MODEL_PATH}")
        urllib.request.urlretrieve(HAND_MODEL_URL, HAND_MODEL_PATH)
        print("Listo.")
    return str(HAND_MODEL_PATH)


# ------------------------------------------------------------ enlace serial --


class EnlaceBrazo:
    """Manda comandos de texto a la ESP32. Si no hay puerto, solo imprime."""

    def __init__(self, puerto=None, baud=115200, comando="SALUDO"):
        self.comando = comando
        self.ser = None
        if puerto:
            import serial  # pyserial

            self.ser = serial.Serial(puerto, baud, timeout=0.1)
            time.sleep(2.0)  # la ESP32 se resetea al abrir el puerto
            print(f"Serial abierto en {puerto} @ {baud}")
        else:
            print("Modo simulación: no se abrió ningún puerto serial.")

    def saludar(self):
        linea = (self.comando + "\r\n").encode()
        if self.ser:
            self.ser.write(linea)
            self.ser.flush()
        print(f"[BRAZO] -> {self.comando}")

    def cerrar(self):
        if self.ser:
            self.ser.close()


# ----------------------------------------------------------------- manos ----

TIPS = [8, 12, 16, 20]   # índice, medio, anular, meñique
PIPS = [6, 10, 14, 18]


def dedos_extendidos(landmarks) -> int:
    """Cuenta dedos extendidos (sin pulgar). y crece hacia abajo en la imagen."""
    n = 0
    for tip, pip in zip(TIPS, PIPS):
        if landmarks[tip].y < landmarks[pip].y:
            n += 1
    return n


class DetectorManos:
    def __init__(self, max_manos=2, conf=0.5):
        import mediapipe as mp
        from mediapipe.tasks import python as mp_python
        from mediapipe.tasks.python import vision

        self.mp = mp
        opciones = vision.HandLandmarkerOptions(
            base_options=mp_python.BaseOptions(
                model_asset_path=asegurar_modelo_manos()
            ),
            running_mode=vision.RunningMode.IMAGE,
            num_hands=max_manos,
            min_hand_detection_confidence=conf,
            min_hand_presence_confidence=conf,
        )
        self.detector = vision.HandLandmarker.create_from_options(opciones)

    def analizar(self, bgr):
        """Devuelve (hay_mano, mano_abierta, lista_de_landmarks)."""
        if bgr.size == 0:
            return False, False, []
        img = self.mp.Image(
            image_format=self.mp.ImageFormat.SRGB,
            data=cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB),
        )
        res = self.detector.detect(img)
        manos = res.hand_landmarks or []
        abierta = any(dedos_extendidos(m) >= 3 for m in manos)
        return len(manos) > 0, abierta, manos

    def cerrar(self):
        self.detector.close()


# --------------------------------------------------------------- personas ---


class DetectorPersonas:
    def __init__(self, modelo="yolo11n.pt", conf=0.5):
        from ultralytics import YOLO

        self.modelo = YOLO(modelo)
        self.conf = conf

    def detectar(self, frame):
        """Lista de bboxes (x1, y1, x2, y2) de personas, ordenadas por tamaño."""
        r = self.modelo.predict(
            frame, classes=[0], conf=self.conf, verbose=False
        )[0]
        cajas = [tuple(map(int, b)) for b in r.boxes.xyxy.cpu().numpy()]
        cajas.sort(key=lambda c: (c[3] - c[1]), reverse=True)
        return cajas


# ------------------------------------------------------------------ main ----


def recortar(frame, caja, margen=0.15):
    h, w = frame.shape[:2]
    x1, y1, x2, y2 = caja
    mx, my = int((x2 - x1) * margen), int((y2 - y1) * margen)
    x1, y1 = max(0, x1 - mx), max(0, y1 - my)
    x2, y2 = min(w, x2 + mx), min(h, y2 + my)
    return frame[y1:y2, x1:x2], (x1, y1)


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--camara", type=int, default=0)
    p.add_argument("--puerto", default=None, help="Puerto serial de la ESP32")
    p.add_argument("--baud", type=int, default=115200)
    p.add_argument("--cmd", default="SALUDO", help="Comando que espera el firmware")
    p.add_argument("--modelo", default="yolo11n.pt")
    p.add_argument("--conf", type=float, default=0.5)
    p.add_argument("--min-alto", type=float, default=0.40,
                   help="Altura mínima del bbox (0-1) para considerar 'cerca'")
    p.add_argument("--frames-confirma", type=int, default=5,
                   help="Frames seguidos con mano abierta antes de disparar")
    p.add_argument("--cooldown", type=float, default=6.0,
                   help="Segundos de espera entre saludos")
    p.add_argument("--yolo-cada", type=int, default=2,
                   help="Correr YOLO 1 de cada N frames")
    p.add_argument("--solo-persona", action="store_true",
                   help="Dispara con solo detectar persona cerca, sin mano")
    args = p.parse_args()

    personas = DetectorPersonas(args.modelo, args.conf)
    manos = None if args.solo_persona else DetectorManos(conf=args.conf)
    brazo = EnlaceBrazo(args.puerto, args.baud, args.cmd)

    cap = cv2.VideoCapture(args.camara)
    if not cap.isOpened():
        raise SystemExit("No se pudo abrir la cámara")

    cajas, n_frame, racha, ultimo_saludo = [], 0, 0, 0.0
    print("q para salir")

    try:
        while True:
            ok, frame = cap.read()
            if not ok:
                break
            h, w = frame.shape[:2]
            n_frame += 1

            if n_frame % args.yolo_cada == 0:
                cajas = personas.detectar(frame)

            objetivo = None
            for c in cajas:
                if (c[3] - c[1]) / h >= args.min_alto:
                    objetivo = c
                    break

            estado = "sin persona"
            disparar = False

            if objetivo is not None:
                x1, y1, x2, y2 = objetivo
                cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 200, 0), 2)

                if args.solo_persona:
                    estado = "persona cerca"
                    disparar = True
                else:
                    crop, (ox, oy) = recortar(frame, objetivo)
                    hay, abierta, lm = manos.analizar(crop)
                    ch, cw = crop.shape[:2]
                    for mano in lm:
                        for punto in mano:
                            cv2.circle(
                                frame,
                                (ox + int(punto.x * cw), oy + int(punto.y * ch)),
                                2, (0, 140, 255), -1,
                            )
                    estado = "mano abierta" if abierta else (
                        "mano cerrada" if hay else "persona cerca")
                    disparar = abierta
            elif cajas:
                estado = "persona lejos"

            racha = racha + 1 if disparar else 0
            ahora = time.time()
            if (racha >= args.frames_confirma
                    and ahora - ultimo_saludo >= args.cooldown):
                brazo.saludar()
                ultimo_saludo = ahora
                racha = 0

            espera = max(0.0, args.cooldown - (ahora - ultimo_saludo))
            cv2.putText(frame, f"{estado} | racha {racha} | cooldown {espera:4.1f}s",
                        (10, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.7,
                        (255, 255, 255), 2)
            cv2.imshow("saludo brazo", frame)
            if cv2.waitKey(1) & 0xFF == ord("q"):
                break
    finally:
        cap.release()
        cv2.destroyAllWindows()
        if manos:
            manos.cerrar()
        brazo.cerrar()


if __name__ == "__main__":
    main()