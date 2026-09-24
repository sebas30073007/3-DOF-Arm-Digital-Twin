# Detección de personas y manos para el manipulador de 3 GDL

Sistema de visión que corre en la computadora, detecta a una persona frente a
la cámara y, cuando levanta la mano, manda por puerto serial la orden de
saludo a la ESP32 que controla el brazo.

---

## Cómo funciona

El pipeline tiene tres etapas por cada frame de la cámara:

1. **Detección de persona (YOLO).** Se corre `yolo11n` filtrando únicamente la
   clase `person` (clase 0 de COCO). Devuelve las cajas delimitadoras de todas
   las personas visibles, ordenadas de mayor a menor altura.

2. **Filtro de cercanía.** Se toma la caja más alta y se compara su altura
   contra la altura del frame. Si ocupa menos del umbral (40% por defecto), se
   considera que la persona está demasiado lejos y no se procesa. Esto evita
   gastar cómputo en gente que solo pasa al fondo del laboratorio.

3. **Detección de mano (MediaPipe).** Sobre el **recorte** de esa persona
   —no sobre el frame completo— se corre el Hand Landmarker, que devuelve 21
   puntos por mano. Se cuentan los dedos extendidos comparando la altura de
   cada punta contra su articulación media; si hay 3 o más, se considera mano
   abierta.

Recortar antes de buscar manos es deliberado: la mano ocupa una fracción mucho
mayor del área analizada, así que el detector la encuentra mejor, y de paso no
se buscan manos en el fondo de la imagen.

### Anti-rebote

La detección corre a ~30 fps, así que un disparo directo haría que el brazo
intentara saludar 30 veces por segundo. Hay dos filtros en serie:

- **Racha:** la mano debe estar abierta durante N frames consecutivos
  (5 por defecto) antes de que cuente.
- **Cooldown:** tras un saludo, se ignoran nuevas detecciones durante N
  segundos (6 por defecto).

---

## Requisitos

### Software

| Componente | Versión | Para qué |
|---|---|---|
| Python | 3.12 | Intérprete. **No usar 3.9** (ver Problemas conocidos) |
| ultralytics | 8.4.x | YOLO y su modelo preentrenado |
| mediapipe | 1.0.x | Hand Landmarker |
| opencv-python | 5.0.x | Captura de cámara y dibujo |
| pyserial | 3.5 | Comunicación con la ESP32 |

`ultralytics` arrastra PyTorch (~2.5 GB instalado). Es lo que más tarda.

### Modelos

Ninguno se entrena: los dos vienen preentrenados y se descargan solos en la
primera ejecución.

| Modelo | Origen | Dataset | Se guarda en |
|---|---|---|---|
| `yolo11n.pt` | Ultralytics | COCO (80 clases, usamos solo `person`) | raíz del proyecto |
| `hand_landmarker.task` | MediaPipe | dataset propio de Google | `models/` |

### Hardware

- Cámara web (índice 0 por defecto, configurable)
- ESP32-C3 con el firmware del brazo, conectada por USB (opcional: sin ella el
  script corre en modo simulación e imprime los comandos en consola)

---

## Instalación

Desde PowerShell, **una línea a la vez** — si una falla, PowerShell sigue
ejecutando las siguientes y el resultado es confuso.

```powershell
# 1. Instalar Python 3.12 si no está
winget install Python.Python.3.12
# cerrar y reabrir la terminal para que tome el PATH

# 2. Verificar que aparezca -V:3.12
py -0

# 3. Crear el entorno virtual
cd E:\Proyectos\huber\Brazo_Sebas
py -3.12 -m venv .venv

# 4. Activarlo (el prompt debe quedar con "(.venv)" al inicio)
.\.venv\Scripts\Activate.ps1

# 5. Instalar dependencias
pip install ultralytics mediapipe opencv-python pyserial

# 6. Verificar
python -c "import mediapipe; print('ok')"
```

Si el paso 4 falla con un error de *execution policy*:

```powershell
Set-ExecutionPolicy -Scope Process -ExecutionPolicy Bypass
```

Solo afecta a esa ventana de PowerShell.

### VS Code

`Ctrl+Shift+P` → **Python: Select Interpreter** → elegir el que dice `.venv`.
Sin esto, el botón de Run sigue usando el Python global y reaparecen los
errores del entorno viejo.

---

## Uso

```powershell
python saludo_brazo.py                      # modo simulación, sin brazo
python saludo_brazo.py --puerto COM5        # conectado a la ESP32
```

`q` cierra la ventana.

En pantalla se ve el recuadro verde sobre la persona detectada, los puntos de
la mano en naranja, y una línea de estado con el estado actual, el conteo de
la racha y el cooldown restante.

### Parámetros

| Flag | Default | Qué hace |
|---|---|---|
| `--camara` | `0` | Índice de la cámara |
| `--puerto` | ninguno | Puerto serial de la ESP32 (`COM5`, `/dev/ttyACM0`) |
| `--baud` | `115200` | Velocidad del serial |
| `--cmd` | `SALUDO` | Texto que se manda al detectar el gesto |
| `--modelo` | `yolo11n.pt` | Modelo YOLO (`yolo11s.pt` es más preciso y lento) |
| `--conf` | `0.5` | Confianza mínima de detección |
| `--min-alto` | `0.40` | Fracción de altura del frame para considerar "cerca" |
| `--frames-confirma` | `5` | Frames seguidos con mano abierta antes de disparar |
| `--cooldown` | `6.0` | Segundos entre saludos |
| `--yolo-cada` | `2` | Corre YOLO 1 de cada N frames (sube para ganar fps) |
| `--solo-persona` | off | Dispara con solo detectar persona, sin revisar mano |

### Ajustes típicos

- **No detecta la mano:** acércate. El Hand Landmarker necesita ~1-2 m.
  Alternativamente baja `--min-alto` y `--conf`.
- **Dispara solo:** sube `--frames-confirma` a 8-10.
- **Va lento:** sube `--yolo-cada` a 3 o 4. YOLO es lo más caro del ciclo.
- **Solo quieres reacción a presencia:** usa `--solo-persona`, se brinca
  MediaPipe por completo.

---

## Protocolo serial

Pendiente de definir. Ahora mismo el script manda el texto de `--cmd` seguido
de `\r\n` y la ESP32 debe interpretarlo. Falta confirmar qué comandos espera
`main_manipulador_final.py` y si el saludo es un comando único o una secuencia
de posiciones.

La clase `EnlaceBrazo` espera 2 segundos tras abrir el puerto, porque la ESP32
se resetea al establecerse la conexión USB.

---

## Problemas conocidos

### MediaPipe truena al importarse en Python 3.9

**Síntoma:** `import mediapipe` falla con
`TypeError: unhashable type: 'list'`, con un traceback que pasa por
`tensorflow/python/framework/ops.py`. En PowerShell el traceback a veces ni
siquiera se alcanza a ver y el script parece morir en silencio.

**Causa:** MediaPipe importa un decorador de documentación desde TensorFlow en
su archivo `core/optional_dependencies.py`. Ese import está protegido con
`try/except ImportError`, pero TensorFlow —en versiones recientes, bajo Python
3.9— falla con `TypeError`, no con `ImportError`. El `except` no lo atrapa y se
cae el import completo de MediaPipe.

**Solución:** usar Python 3.12 en un venv, como describe la instalación. El
proyecto no usa TensorFlow para nada (YOLO va sobre PyTorch y MediaPipe trae su
propio runtime), así que en un entorno limpio el problema desaparece.

Alternativa si hay que quedarse en 3.9: `pip uninstall tensorflow tensorflow-intel`.

### Tracebacks invisibles en PowerShell

PowerShell trunca o envuelve la salida de `stderr` de programas nativos. Para
depurar cualquier cosa de este proyecto, redirigir a archivo:

```powershell
python -X faulthandler script.py 2> err.txt
$LASTEXITCODE
type err.txt
```

El código de salida distingue el tipo de fallo: `1` es excepción de Python,
`-1073741819` es violación de acceso (DLL faltante), `-1073741795` es
instrucción ilegal (CPU sin AVX).

### Error de pip al instalar MediaPipe 1.0.1 en Python 3.9

**Síntoma:** al terminar de copiar archivos, pip falla con
`SyntaxError: invalid character '∂'` en
`mediapipe/tasks/python/test/text/text_embedder_test.py`, seguido de
`TypeError: encode() argument 'encoding' must be str, not None`.

**Causa:** el wheel trae un archivo de tests con un carácter inválido. Pip
intenta byte-compilar todo lo que instala y ese archivo no compila.

**Solución:** `pip install --no-compile mediapipe`. El archivo es un test que
nunca se importa. En Python 3.12 el problema no aparece.

### Timeout descargando PyTorch

El wheel de torch pesa ~124 MB y la descarga se corta con conexiones
inestables:

```powershell
pip install --timeout 120 --retries 10 torch torchvision
```

Lo ya descargado queda en caché, así que no reinicia de cero. Si la red es
mala, la versión CPU es bastante más ligera y para este proyecto es suficiente
(no hay GPU NVIDIA en la máquina):

```powershell
pip install torch torchvision --index-url https://download.pytorch.org/whl/cpu
```

Hay que instalarla **antes** que ultralytics.

---

## Estructura

```
Brazo_Sebas/
├── .venv/                      # entorno virtual (no versionar)
├── models/
│   └── hand_landmarker.task    # se descarga solo
├── yolo11n.pt                  # se descarga solo
├── saludo_brazo.py             # script principal
└── README.md
```

Para `.gitignore`:

```
.venv/
models/
*.pt
```

Los modelos no se versionan: el script los vuelve a descargar donde haga falta.

---

## Pendientes

- Definir el protocolo serial real contra `main_manipulador_final.py`
- Probar el reconocimiento de señas específicas (`gestos_brazo.py`)
- Validar el rango útil de detección en el laboratorio con la cámara definitiva
