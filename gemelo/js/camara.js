// camara.js - la webcam del navegador y los modelos de visión.
//
// Cámara: getUserMedia, cualquier cámara que el navegador vea (la integrada o
// una USB). La imagen se dibuja en un lienzo junto con lo que detectan los
// modelos; ese mismo lienzo es la miniatura y la textura del cuadro 3D.
//
// Visión (MediaPipe Tasks en el navegador, se descarga solo al encenderla):
//   manos     Hand Landmarker (el mismo .task que usa el panel de Python)
//   personas  Object Detector efficientdet_lite0, solo la clase "person"
//
// Dedos extendidos: como manipulador/vision.py, sin pulgar y sin depender de
// la orientación: la punta está más lejos de la muñeca que la articulación media.

const MP = "https://cdn.jsdelivr.net/npm/@mediapipe/tasks-vision@1.0.1";
const MODELO_MANOS = "modelos/hand_landmarker.task";
const MODELO_PERSONAS = "https://storage.googleapis.com/mediapipe-models/object_detector/efficientdet_lite0/float16/1/efficientdet_lite0.tflite";
const PUNTAS = [8, 12, 16, 20], MEDIAS = [6, 10, 14, 18];
const HUESOS = [[0, 1], [1, 2], [2, 3], [3, 4], [0, 5], [5, 6], [6, 7], [7, 8], [5, 9], [9, 10], [10, 11], [11, 12],
  [9, 13], [13, 14], [14, 15], [15, 16], [13, 17], [0, 17], [17, 18], [18, 19], [19, 20]];

export class Camara extends EventTarget {
  constructor() {
    super();
    this.video = Object.assign(document.createElement("video"), { muted: true, playsInline: true, autoplay: true });
    this.lienzo = Object.assign(document.createElement("canvas"), { width: 640, height: 360 });
    this.ctx = this.lienzo.getContext("2d");
    this.stream = null;
    this.deviceId = "";
    this.modelos = { manos: null, personas: null };
    this.usar = { manos: false, personas: false };
    this.cargando = "";
    this.resultado = { manos: [], personas: [] };
    this.tDetect = 0;
    this.ultimoFrame = -1;
    this.error = "";
  }

  get abierta() { return !!this.stream; }
  get ancho() { return this.video.videoWidth || 640; }
  get alto() { return this.video.videoHeight || 360; }

  async listar() {
    const ds = await navigator.mediaDevices.enumerateDevices();
    return ds.filter(d => d.kind === "videoinput").map((d, i) => ({ id: d.deviceId, nombre: d.label || `Cámara ${i + 1}` }));
  }

  async abrir(deviceId = this.deviceId) {
    this.cerrar();
    try {
      this.stream = await navigator.mediaDevices.getUserMedia({
        video: { deviceId: deviceId ? { exact: deviceId } : undefined, width: { ideal: 1280 }, height: { ideal: 720 } },
        audio: false,
      });
    } catch (e) {
      this.error = e.name === "NotAllowedError" ? "Sin permiso para usar la cámara" : e.name === "NotFoundError" ? "No hay cámara" : String(e.message || e);
      this.avisar();
      throw e;
    }
    this.deviceId = this.stream.getVideoTracks()[0]?.getSettings().deviceId || deviceId;
    this.video.srcObject = this.stream;
    await this.video.play().catch(() => {});
    await new Promise(r => (this.video.readyState >= 2 ? r() : this.video.addEventListener("loadeddata", r, { once: true })));
    this.lienzo.width = 640;
    this.lienzo.height = Math.round(640 * this.alto / this.ancho);
    this.error = "";
    this.avisar();
  }

  cerrar() {
    if (!this.stream) return;
    this.stream.getTracks().forEach(t => t.stop());
    this.stream = null;
    this.video.srcObject = null;
    this.resultado = { manos: [], personas: [] };
    this.avisar();
  }

  avisar() { this.dispatchEvent(new Event("cambio")); }

  // Carga perezosa: la primera vez baja ~12 MB de wasm y los modelos
  async activar(tipo, si) {
    this.usar[tipo] = si;
    if (!si || this.modelos[tipo]) { this.avisar(); return; }
    this.cargando = tipo;
    this.avisar();
    try {
      const vision = await import(`${MP}/vision_bundle.mjs`);
      this.fileset ||= await vision.FilesetResolver.forVisionTasks(`${MP}/wasm`);
      const crear = async delegate => tipo === "manos"
        ? vision.HandLandmarker.createFromOptions(this.fileset, {
          baseOptions: { modelAssetPath: MODELO_MANOS, delegate }, runningMode: "VIDEO", numHands: 2,
          minHandDetectionConfidence: 0.5, minHandPresenceConfidence: 0.5, minTrackingConfidence: 0.5,
        })
        : vision.ObjectDetector.createFromOptions(this.fileset, {
          baseOptions: { modelAssetPath: MODELO_PERSONAS, delegate }, runningMode: "VIDEO",
          scoreThreshold: 0.5, categoryAllowlist: ["person"], maxResults: 3,
        });
      try { this.modelos[tipo] = await crear("GPU"); } catch { this.modelos[tipo] = await crear("CPU"); }
    } catch (e) {
      this.usar[tipo] = false;
      this.error = "No se pudo cargar el modelo: " + (e.message || e);
    }
    this.cargando = "";
    this.avisar();
  }

  // Llamar cada cuadro. Detecta a lo más ~20 veces por segundo, solo con
  // cuadros nuevos del video, y redibuja el lienzo.
  cuadro(ahora) {
    if (!this.abierta || this.video.readyState < 2) return false;
    const nuevo = this.video.currentTime !== this.ultimoFrame;
    if (nuevo && ahora - this.tDetect > 50) {
      this.tDetect = ahora;
      this.ultimoFrame = this.video.currentTime;
      this.detectar(ahora);
    }
    this.dibujar();
    return true;
  }

  detectar(t) {
    const r = { manos: [], personas: [] };
    try {
      if (this.usar.manos && this.modelos.manos) {
        const m = this.modelos.manos.detectForVideo(this.video, t);
        r.manos = (m.landmarks || []).map(lm => this.describirMano(lm));
      }
      if (this.usar.personas && this.modelos.personas) {
        const d = this.modelos.personas.detectForVideo(this.video, t);
        r.personas = (d.detections || []).map(x => {
          const b = x.boundingBox;
          return { x: b.originX / this.ancho, y: b.originY / this.alto, w: b.width / this.ancho, h: b.height / this.alto, score: x.categories[0]?.score };
        });
      }
    } catch { /* un cuadro perdido no importa */ }
    this.resultado = r;
    this.dispatchEvent(new CustomEvent("deteccion", { detail: r }));
  }

  describirMano(lm) {
    const W = this.ancho, H = this.alto;
    const px = i => [lm[i].x * W, lm[i].y * H];
    const dist = (a, b) => Math.hypot(a[0] - b[0], a[1] - b[1]);
    const w = px(0);
    let dedos = 0;
    PUNTAS.forEach((p, k) => { if (dist(px(p), w) > dist(px(MEDIAS[k]), w) * 1.15) dedos++; });
    // Tamaño en px de dos medidas de la palma, para estimar la distancia
    return { lm, dedos, centro: [lm[9].x, lm[9].y], l09: dist(w, px(9)), l517: dist(px(5), px(17)) };
  }

  dibujar() {
    const { ctx, lienzo: c } = this;
    ctx.drawImage(this.video, 0, 0, c.width, c.height);
    const { manos, personas } = this.resultado;
    ctx.lineWidth = 2;
    for (const p of personas) {
      ctx.strokeStyle = "rgba(255,255,255,0.9)";
      ctx.setLineDash([6, 4]);
      ctx.strokeRect(p.x * c.width, p.y * c.height, p.w * c.width, p.h * c.height);
      ctx.setLineDash([]);
    }
    for (const m of manos) {
      const P = i => [m.lm[i].x * c.width, m.lm[i].y * c.height];
      ctx.strokeStyle = "rgba(17,19,23,0.85)";
      ctx.lineWidth = 3;
      ctx.beginPath();
      for (const [a, b] of HUESOS) { ctx.moveTo(...P(a)); ctx.lineTo(...P(b)); }
      ctx.stroke();
      ctx.strokeStyle = "rgba(255,255,255,0.95)";
      ctx.lineWidth = 1.5;
      ctx.stroke();
      for (const i of PUNTAS) {
        ctx.fillStyle = m.activa ? "#E30613" : "#FFFFFF";
        ctx.beginPath(); ctx.arc(...P(i), 4, 0, Math.PI * 2); ctx.fill();
      }
    }
  }
}
