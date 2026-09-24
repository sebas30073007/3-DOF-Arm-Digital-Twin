# 3-DOF Arm Digital Twin

Brazo de 3 grados de libertad (base, eslabón 1 y eslabón 2, transmisión por
bandas) con tres CL57T y NEMA 17, controlado por una ESP32-C3. El repo tiene
tres partes:

| Parte | Carpeta | Qué es |
|---|---|---|
| **Pólux** | `gemelo/` | Gemelo digital en el navegador: modelo 3D del CAD, simulador del firmware, control del robot real por Web Serial y seguimiento con cámara. Página estática para GitHub Pages. |
| Firmware | `firmware/manipulador_v7/` | Sketch de la ESP32-C3: perfiles trapezoidales, calibración con switches, página web propia y protocolo serial. |
| Panel local | `app.py`, `manipulador/`, `web/` | Panel en Python: webcam, YOLO y MediaPipe, y control por serial. |

**Pólux en línea:** https://sebas30073007.github.io/3-DOF-Arm-Digital-Twin/
(se publica solo con GitHub Actions al hacer push a `main`; ver *Publicar* abajo).

---

## Panel local (Python)

Programa en la PC que abre la webcam, detecta personas y manos, y controla
el brazo (ESP32-C3 con `manipulador_v7`) por USB serial. Todo se maneja desde
una página local: la cámara a la izquierda y los controles a la derecha.

```powershell
.\.venv\Scripts\Activate.ps1
python app.py                  # autodetecta la ESP32 (VID 303A) y abre http://localhost:8000
python app.py --puerto COM7    # puerto fijo
python app.py --vision         # arranca con la visión encendida
python app.py --camara 1       # otra webcam
```

`Ctrl+C` para salir; al cerrar se manda `STOP`.

> Si el puerto aparece como *ocupado*, cierra el monitor serial del Arduino IDE.
> Solo un programa puede tener abierto el COM a la vez.

---

## Estructura

```
app.py                     punto de entrada
manipulador/
  robot.py                 enlace serial: reconexión, STATE? cada 150 ms, log, jog emulado
  vision.py                cámara + YOLO (personas) + MediaPipe (manos) + elección de objetivo
  seguimiento.py           objetivo de visión -> GOTO de la base
  servidor.py              HTTP local (stdlib): página, stream MJPEG y /api/*
web/                       index.html, style.css, app.js
models/                    yolo11n.pt, hand_landmarker.task (no van en git: se descargan solos)
gemelo/                    Pólux, el gemelo digital (ver abajo)
Ensamble_manipulador_solo_HOME.glb   CAD completo en la pose HOME (fuente del modelo 3D)
firmware/manipulador_v7/   firmware de la ESP32 (sketch de Arduino)
_archivo/                  versiones anteriores (firmware v6, saludo_brazo.py, cliente de prueba, README viejo)
```

## La página

**Encabezado del panel.** Estado de la conexión, selector de puerto y **ALTO**
(`Esc` desde cualquier parte). ALTO manda `STOP` y apaga el seguimiento.

**Control.** Hace lo mismo que la página del ESP32, pero por serial:
Calibrar, HOME y Saludo; indicadores de los switches; un slider por eje
(punto blanco = objetivo, rojo = posición real); jog con `‹ ›`; velocidad,
aceleración y pulsos/rev por grupo.

**Visión.** Enciende los modelos, el seguimiento y sus parámetros.

**Consola.** Log crudo del serial (`→` enviado, `←` respuesta, `·` avisos MSG)
y una línea para mandar cualquier comando del protocolo. `↑/↓` recorren el historial.

### Quién tiene el control

El firmware deja mover al robot a una sola fuente a la vez. Por serial no se
puede tomar el control: si la página del ESP32 (192.168.4.1) está en
*Control web*, el panel lo avisa y bloquea los movimientos hasta que ahí se pulse
*Control PC*. Al encender la ESP32 el control empieza en PC.

### Jog

El protocolo serial no tiene jog. Mientras sostienes `‹` o `›`, el navegador
manda un latido cada 150 ms y el servidor manda `GOTO` al límite del eje. Al soltar
se manda `STOP`. Si pasan 0.5 s sin latido (pestaña cerrada o red caída), el
servidor manda `STOP` por su cuenta. Por serial, E1 y E2 exigen calibración.

---

## Visión y seguimiento

1. **Personas.** YOLO `yolo11n` sobre el frame completo, solo la clase `person`.
   Se sigue la persona más grande y se prefiere la misma del frame anterior
   (IoU > 0.3), para que no salte entre personas.
2. **Palma cerca.** MediaPipe Hand Landmarker sobre el frame completo, en modo
   video con tracking. Una mano cuenta como **palma** si tiene al menos
   *Dedos para palma* dedos extendidos (se compara la distancia punta-muñeca
   contra articulación-muñeca, así no importa la orientación), y está **cerca**
   si su tamaño supera *Mano "cerca"* (fracción de la altura de la imagen).
3. **Prioridad.** Palma cerca durante *Frames para fijar mano* frames seguidos
   hace que se pase a **modo MANO**: la mano queda fijada aunque cierre los dedos,
   hasta que deja de verse 0.8 s. Luego regresa a seguir a la persona.

Con una persona se mueve solo la **base**, con `GOTO B` a unos 10 Hz. El firmware
corrige el destino al vuelo, así que no hace falta esperar a que termine.

### Control con la mano (embrague de 4 dedos)

En modo MANO los dedos funcionan como embrague:

- **4 dedos extendidos** (*Dedos para mover*): el robot sigue la mano. La base
  gira con la posición horizontal y, si está calibrado, el **eslabón 1** sigue
  la altura: mano arriba en la imagen = *E1 con la mano arriba* (60°), abajo =
  *E1 con la mano abajo* (10°). E1 tiene su propia zona muerta (3°).
- **Menos dedos**: pausa. Si el robot se estaba moviendo manda `STOP`, y no
  manda nada más mientras sigan faltando dedos. La mano sigue fijada; al volver
  a mostrar 4 dedos retoma desde ahí.

El conteo no incluye el pulgar (índice a meñique, máximo 4), así que una palma
abierta cuenta como 4. En el video aparece "N dedos" junto a la mano, y el
marcador pasa de rojo a gris con "pausa" cuando falta algún dedo.

Se ajusta en *Visión → Seguimiento → Control con la mano*: dedos, rango de E1,
zona muerta, invertir E1, o apagar E1 para mover solo la base.

**Limitación del firmware.** v7 mueve un eje a la vez (el `loop` atiende el
primer eje con objetivo pendiente). Con base y E1 activos, los movimientos se
turnan y se ven escalonados; además E1 es lento (20 rpm de motor ≈ 4.8°/s).

| Geometría | Cuándo | Cálculo |
|---|---|---|
| Cámara fija | La webcam está quieta junto al robot | `base = centro − (x − 0.5)·FOV·ganancia` |
| Cámara en la base | La cámara gira con la base | `base = actual − (x − 0.5)·FOV·ganancia` |

**Base en v7.** La base no tiene switch: 0° es la posición en la que se encendió
y el rango es de +20° a −200°. En cámara fija, *Base al centro de la imagen*
(`centro`) es el ángulo de la base que apunta a donde mira la cámara; mide
ese ángulo con jog y ajústalo. Los objetivos se recortan al rango del firmware.

Si la base gira hacia el lado contrario, activa **Invertir sentido de la base**.
Cualquier movimiento manual (slider, jog, HOME, Saludo, consola) apaga el seguimiento.

**Ajustes rápidos.** Si oscila, sube la zona muerta o el suavizado. Si va lento,
sube la velocidad de la base (VEL B, máx. 30 rpm de motor = 18°/s). Si no
entra a modo mano, baja *Mano "cerca"* o *Dedos para palma*.

---

## Pólux: gemelo digital (`gemelo/`)

Página estática, sin Python, pensada para GitHub Pages. Muestra el modelo 3D
del CAD en dos capas: el **sólido** es el robot (real o simulado) y el
**fantasma translúcido** es la referencia que tú mueves. El sólido la alcanza
con las rampas del firmware; cuando coinciden, el fantasma se esconde.

```powershell
cd gemelo
python -m http.server 8765      # http://localhost:8765 (no abre con file://)
```

- **Digital**: una ESP32 simulada en el navegador (`js/sim.js`) que habla el
  mismo protocolo que v7 y se mueve igual: un eje a la vez (B, E1, E2), perfil
  trapezoidal con la reducción de cada eje, corrección del destino al vuelo, CAL,
  HOME, Saludo y STOP con los mismos mensajes. En *Ajustes* se puede mover los tres
  ejes a la vez (como sería el firmware pendiente) y acelerar el tiempo.
- **Real**: Web Serial (Chrome o Edge de escritorio, sobre https o localhost).
  Es la misma lógica que `manipulador/robot.py`: STATE? cada 150 ms y una línea por
  comando. Solo un programa puede abrir el COM: cierra Thonny y el panel de Python.
- **Mover**: rieles copiados de la página del ESP32 (pulgar blanco = referencia,
  punto rojo = real, barra de HOME al real, casita en HOME; E2 va de 0 a −220
  de izquierda a derecha), arrastrar las piezas en el visor (al pasar el cursor
  aparece el arco del eje con su rango), o el modo **Punto**: una esfera en el
  punto de agarre con flechas XYZ y cinemática inversa.
- **Fantasma**: solo se dibuja desde la primera articulación que cambia (mover E2
  muestra solo el eslabón 2; E1, los eslabones 1 y 2; la base, todo el brazo).
- **Rutinas**: el fantasma muestra las poses clave. En CAL y HOME, la pose final
  desde el principio; en el Saludo, el final de cada movimiento, y avanza cuando
  el real lo alcanza.
- **Cámara** (también en Digital): cualquier webcam del navegador. En el mundo 3D
  aparece con su pirámide de visión y el video en un cuadro a 35 cm; la pose por
  defecto es X 0, Y 0, Z 450 mm mirando hacia +Z (se edita en *Pose de la
  cámara*). Visión con MediaPipe en el navegador (manos y personas; el wasm se baja
  de jsdelivr la primera vez). **Seguir**: con la palma abierta (4 dedos) el brazo
  lleva el punto de agarre hacia la mano, estimada en 3D por su tamaño; si no la
  alcanza, apunta hacia ella. Con menos dedos se detiene. Sin mano, la base sigue a
  la persona. Cualquier control manual apaga el seguimiento.
- Se manda `GOTO` (un eje) o `POSE` (varios) a lo más cada 120 ms mientras se
  arrastra y una vez más al soltar. `Esc` = ALTO. Clic izquierdo orbita, derecho o
  central desplaza, rueda acerca (el mouse abajo a la izquierda marca el botón en uso).

**Modelo.** `gemelo/tools/preparar_glb.mjs` toma `Ensamble_manipulador_solo_HOME.glb`
(17 MB, ~100 piezas sueltas), las agrupa en cuerpos rígidos (`fijo`, `base`,
`eslabon1`, `eslabon2`, `dedo1`, `dedo2`) con pivotes en los ejes reales y lo
comprime con meshopt a 2.8 MB. Guarda en `extras` los pivotes y el punto de agarre.
Usa las dependencias de Ítaca (`@gltf-transform/*`, `meshoptimizer`). El reparto
es por nombre de pieza: si se agregan piezas al CAD, revisa `grupoDe()`.

**Cinemática.** El CAD está en HOME (B 0°, E1 +1.73°, E2 −1.73°). E1+ inclina el
eslabón 1 hacia adelante; en HOME el eslabón 2 ya está casi plegado contra el 1,
así que E2− lo levanta. La base va invertida respecto al modelo (confirmado con el
robot). E2 es relativo al eslabón 1 (el firmware compensa la cinta 1:1). Si algo
gira al revés, se corrige en *Ajustes → Gemelo* (se guarda en el navegador).

**Publicar.** `.github/workflows/pages.yml` sube la carpeta `gemelo/` a GitHub
Pages en cada push a `main`. Solo hay que activarlo una vez: *Settings → Pages →
Source: GitHub Actions*.

**Pendiente:** probar el modo Real con la ESP32 (que Web Serial no reinicie la C3
al abrir), probar el seguimiento con una mano real y medir la pose y el FOV reales
de la cámara, y `GRIP` cuando exista en el firmware (los dedos ya son grupos aparte).

---

## Instalación

El `.venv` ya trae todo (Python 3.12.10). Si hay que recrearlo:

```powershell
py -3.12 -m venv .venv
.\.venv\Scripts\Activate.ps1
python -m pip install -r requirements.txt
```

Si `Activate.ps1` falla por *execution policy*:
`Set-ExecutionPolicy -Scope Process -ExecutionPolicy Bypass`.

Los problemas conocidos de MediaPipe en Python 3.9 y las descargas de PyTorch
están documentados en `_archivo/README_Deteccion_manos.md`.

## Estado al 23-sep-2026

**Probado con hardware:** conexión por COM7 con el firmware v7 (PING, STATE,
límites nuevos de la base), cámara a 30 fps, detección de personas y manos,
overlay y stream en la página.

**Probado solo con ESP32 simulada:** jog con latido y STOP de seguridad,
seguimiento de persona (cámara fija y en la base), embrague de 4 dedos
(4 dedos mueve B y E1, 3 dedos manda STOP, 4 otra vez retoma).

Al final de la sesión la ESP32 estaba desconectada del USB; el panel la toma
sola (modo Auto) al reconectarla.

## Pendientes

- Probar el control con la mano en el robot real: confirmar que un ángulo mayor
  de E1 sube el brazo (si no, *Invertir sentido de E1*) y ajustar el rango de E1.
- Medir el FOV real de la webcam, el ángulo `centro` de la base y el sentido de giro.
- Firmware: mover base y E1 a la vez para que el seguimiento de la mano sea fluido.
- Firmware: comando `MODE PC` por serial para tomar el control desde la PC.
- Si Thonny queda abierto con el intérprete de ESP32 en COM7, ocupa el puerto:
  cambiarle el intérprete o cerrarlo antes de correr el panel.
