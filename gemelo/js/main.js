// main.js - la página: une escena, enlace, cámara y paneles.
//
// La idea central es la REFERENCIA: la pose que pide quien controla (rieles,
// arrastre de piezas, punto de agarre, jog o el seguimiento de la mano). Se
// dibuja como el fantasma y se manda al firmware como GOTO (un eje) o POSE
// (varios), a lo más cada 120 ms mientras se arrastra y una vez más al soltar.
// El firmware corrige el destino al vuelo, así que el robot alcanza a la referencia.
//
// En las rutinas la referencia son sus poses clave: en CAL y HOME, la pose
// final desde el principio; en el Saludo, el final de cada movimiento, y pasa
// al siguiente solo cuando el real llegó. Después de ALTO se pega al real.

import { EJES, NOMBRES, FIRMWARE, HOME, grupo, rpmAGrados, limitar, cargarCalibracion, guardarCalibracion, GEMELO_DEFECTO, cargarCamara, guardarCamara } from "./config.js";
import { Cinematica } from "./cinematica.js";
import { Escena } from "./escena.js";
import { EnlaceSim, EnlaceSerial } from "./enlace.js";
import { Camara } from "./camara.js";

const $ = s => document.querySelector(s);
const $$ = s => [...document.querySelectorAll(s)];
const MS_ENVIO = 120;
const INVERTIDO = [false, false, true];   // como la página del ESP32: E2 va de 0 (izq.) a −220 (der.)
const LLEGO = 0.25;                       // grados para dar una pose clave por alcanzada

const cal = cargarCalibracion();
const poseCam = cargarCamara();
let cin = null;
let enlace = null;
const sim = new EnlaceSim();
let serial = null;
const camara = new Camara();

let ref = null;             // referencia [B, E1, E2]
let real = null;            // último estado del robot
let realVisto = null;       // lo que se dibuja (suavizado en modo real)
let refSigue = { hasta: 0, activo: true };
let rutina = null;          // { frames, i, t0, vioOcupado }
let enviado = [null, null, null];
let timerEnvio = null, tEnvio = 0;
let jog = null;
let puntoObjetivo = null;
const seg = { activo: false, filtro: null, ultimo: null, ancla: null, pausa: false };
const GANANCIA_MANO = 1;     // m del punto de agarre por m de la mano

// ------------------------------------------------------------- escena --

const escena = new Escena($("#visor"), {
  alCambiarRef: (q, { final, eje }) => cambiarRef(q, { final, eje, origen: "escena" }),
  alHover: info => mostrarEtiqueta(info),
  alPunto: (p, { final }) => irAPunto(p, final),
});

escena.cargar("modelos/manipulador.glb", extras => (cin = new Cinematica(extras, cal)))
  .then(() => {
    $("#cargando").hidden = true;
    escena.ponerCamara(poseCam);
    usarEnlace(sim);
  })
  .catch(err => {
    $("#cargando").textContent = "No se pudo cargar el modelo: " + err.message;
    console.error(err);
  });

// -------------------------------------------------------------- enlace --

function usarEnlace(nuevo) {
  if (enlace) {
    enlace.removeEventListener("estado", alEstado);
    enlace.removeEventListener("log", alLog);
    enlace.removeEventListener("conexion", alConexion);
  }
  enlace = nuevo;
  enlace.addEventListener("estado", alEstado);
  enlace.addEventListener("log", alLog);
  enlace.addEventListener("conexion", alConexion);
  real = null;
  realVisto = null;
  rutina = null;
  pegarRef();
  alConexion();
  if (enlace.estado) alEstado();
}

function alEstado() {
  const s = enlace.estado;
  if (!s) return;
  real = [s.b, s.c, s.m];
  if (!realVisto || enlace.tipo === "sim") realVisto = real.slice();
  const ahora = performance.now();

  if (rutina) {
    if (s.ocu) rutina.vioOcupado = true;
    while (rutina.i < rutina.frames.length - 1 && cerca(real, rutina.frames[rutina.i])) rutina.i++;
    ref = rutina.frames[rutina.i].slice();
    if (!s.ocu && (rutina.vioOcupado || ahora - rutina.t0 > 1500)) {
      rutina = null;
      pegarRef(0);
    }
    escena.setRef(ref);
  }
  if (refSigue.activo && !rutina) {
    ref = real.slice();
    escena.setRef(ref);
    if (!s.ocu && ahora > refSigue.hasta) refSigue.activo = false;
  }
  if (!ref) { ref = real.slice(); escena.setRef(ref); }
  pintarEstado(s);
}

const cerca = (a, b) => a.every((v, e) => Math.abs(v - b[e]) < LLEGO);

function pegarRef(ms = 300) {
  refSigue = { activo: true, hasta: performance.now() + ms };
  enviado = [null, null, null];
  clearTimeout(timerEnvio);
  timerEnvio = null;
  puntoObjetivo = null;
  escena.marcarPunto(null, true);
  if (real) { ref = real.slice(); escena.setRef(ref); }
}

function alConexion() {
  const esSim = enlace.tipo === "sim";
  const con = enlace.conectado;
  $("#conexion").hidden = esSim || con;
  $("#visorVacio").hidden = esSim || con;
  $("#ajustesSim").hidden = !esSim;
  if (!esSim) {
    $("#conexionTexto").textContent = !EnlaceSerial.disponible()
      ? "Este navegador no tiene Web Serial: usa Chrome o Edge"
      : serial?.error || "ESP32 desconectada";
    $("#bConectar").hidden = !EnlaceSerial.disponible();
  }
  if (con && !esSim) pegarRef(500);
  if (!con) {
    $("#ocupado").classList.remove("activo");
    $("#ocupado").textContent = "sin robot";
    $("#msgFw").textContent = "—";
    $("#avisoCal").hidden = $("#avisoWeb").hidden = true;
  }
  actualizarBloqueo();
}

// -------------------------------------------------------- referencia --

function cambiarRef(q, { final = false, eje = -1, origen = "" } = {}) {
  if (!puedeMover()) { if (real) { ref = real.slice(); escena.setRef(ref); } return; }
  if (origen !== "seguir" && seg.activo) ponerSeguimiento(false, "Seguimiento apagado por control manual");
  refSigue.activo = false;
  rutina = null;
  ref = q.map((v, e) => limitar(e, v));
  if (origen !== "escena") escena.setRef(ref);
  if (origen !== "punto" && escena.modo === "punto") { puntoObjetivo = null; escena.marcarPunto(null, true); }
  marcarActivo(eje);
  pintarRef();
  programarEnvio(final);
}

function puedeMover() {
  return enlace?.conectado && enlace.modo !== "WEB" && real;
}

function programarEnvio(final) {
  if (final) {
    clearTimeout(timerEnvio);
    timerEnvio = null;
    enviar();
    return;
  }
  if (timerEnvio) return;
  const espera = Math.max(0, MS_ENVIO - (performance.now() - tEnvio));
  timerEnvio = setTimeout(() => { timerEnvio = null; enviar(); }, espera);
}

async function enviar() {
  tEnvio = performance.now();
  const q = ref.map((v, e) => Math.round(limitar(e, v) * 10) / 10);
  const cambian = [0, 1, 2].filter(e => enviado[e] === null ? Math.abs(q[e] - real[e]) >= 0.05 : Math.abs(q[e] - enviado[e]) >= 0.05);
  if (!cambian.length) return;
  const linea = cambian.length === 1
    ? `GOTO ${EJES[cambian[0]]} ${q[cambian[0]].toFixed(1)}`
    : `POSE ${q.map((v, e) => (cambian.includes(e) ? v.toFixed(1) : "NA")).join(" ")}`;
  for (const e of cambian) enviado[e] = q[e];
  const r = await enlace.cmd(linea);
  if (r && r.startsWith("ERR")) rechazo(r, cambian);
}

function rechazo(r, ejes) {
  const textos = {
    "ERR NO_CALIBRADO": "E1 y E2 necesitan calibración",
    "ERR MODO_WEB": "El control lo tiene la página del ESP32",
    "ERR FUERA_DE_LIMITE": "Fuera de límite",
    "ERR SIN_CONEXION": "Sin conexión con la ESP32",
  };
  avisar(textos[r] || r);
  for (const e of ejes) { ref[e] = real[e]; enviado[e] = null; }
  escena.setRef(ref);
  pintarRef();
}

// ------------------------------------------------------------ punto --

function irAPunto(p, final) {
  if (!cin || !ref) return;
  puntoObjetivo = p;
  const r = cin.inversa(p, ref);
  escena.marcarPunto(null, r.alcanzable);
  cambiarRef(r.q, { final, origen: "punto" });
  pintarAlcance(r);
}

function pintarAlcance(r) {
  const el = $("#alcance");
  if (!r || r.alcanzable) { el.textContent = ""; el.classList.remove("fuera"); return; }
  el.textContent = `fuera de alcance · ${Math.round(r.error * 1000)} mm`;
  el.classList.add("fuera");
}

for (const id of ["#pX", "#pY", "#pZ"]) {
  $(id).addEventListener("change", () => {
    const p = ["#pX", "#pY", "#pZ"].map(s => Number($(s).value) / 1000);
    if (p.some(v => !Number.isFinite(v))) return;
    p[1] = Math.max(0.005, p[1]);
    escena.moverPunto(p);
    irAPunto(p, true);
  });
}

// ------------------------------------------------------------ rieles --

// Fracción del riel (0 izq., 1 der.) para un ángulo, y al revés
function fr(e, v) {
  const [a, b] = [FIRMWARE.limMin[e], FIRMWARE.limMax[e]];
  const f = Math.min(1, Math.max(0, (v - a) / (b - a)));
  return INVERTIDO[e] ? 1 - f : f;
}
function vr(e, f) {
  f = Math.min(1, Math.max(0, f));
  if (INVERTIDO[e]) f = 1 - f;
  const [a, b] = [FIRMWARE.limMin[e], FIRMWARE.limMax[e]];
  return Math.round((a + f * (b - a)) * 10) / 10;
}

const filas = [0, 1, 2].map(e => {
  const nodo = $("#tplEje").content.firstElementChild.cloneNode(true);
  nodo.querySelector(".eje-nombre").textContent = NOMBRES[e];
  const sl = nodo.querySelector(".sl");
  const [lo, hi] = [FIRMWARE.limMin[e], FIRMWARE.limMax[e]];
  sl.title = INVERTIDO[e] ? `${fmt(hi)} … ${fmt(lo)}` : `${fmt(lo)} … ${fmt(hi)}`;
  sl.setAttribute("aria-label", NOMBRES[e]);
  sl.setAttribute("aria-valuemin", lo);
  sl.setAttribute("aria-valuemax", hi);
  nodo.querySelector(".sl-casa").style.left = fr(e, HOME[e]) * 100 + "%";
  const num = nodo.querySelector(".eje-num");
  num.min = lo;
  num.max = hi;

  let arrastrando = false;
  const mover = ev => {
    const r = sl.getBoundingClientRect();
    cambiarEje(e, vr(e, (ev.clientX - r.left) / r.width), false);
  };
  sl.addEventListener("pointerdown", ev => {
    if (!puedeMover() || nodo.classList.contains("off")) return;
    arrastrando = true;
    sl.setPointerCapture(ev.pointerId);
    mover(ev);
  });
  sl.addEventListener("pointermove", ev => { if (arrastrando) mover(ev); });
  const fin = () => {
    if (!arrastrando) return;
    arrastrando = false;
    cambiarRef(ref, { final: true, eje: e });
    marcarActivo(-1);
  };
  sl.addEventListener("pointerup", fin);
  sl.addEventListener("pointercancel", fin);
  sl.addEventListener("keydown", ev => {
    const vis = { ArrowLeft: -1, ArrowRight: 1 }[ev.key];
    if (!vis || !ref) return;
    ev.preventDefault();
    cambiarEje(e, ref[e] + (INVERTIDO[e] ? -vis : vis) * (ev.shiftKey ? 10 : 1), true);
  });
  num.addEventListener("change", () => { if (num.value !== "") cambiarEje(e, Number(num.value), true); });

  const jogs = [...nodo.querySelectorAll(".jog")];
  for (const b of jogs) {
    const vis = Number(b.dataset.vis);
    b.addEventListener("pointerdown", ev => {
      b.setPointerCapture(ev.pointerId);
      empezarJog(e, INVERTIDO[e] ? -vis : vis, b);
    });
    for (const t of ["pointerup", "pointercancel", "lostpointercapture"]) b.addEventListener(t, () => terminarJog(b));
    b.addEventListener("contextmenu", ev => ev.preventDefault());
  }
  $("#ejes").appendChild(nodo);
  return {
    nodo, sl, num, jogs,
    vReal: nodo.querySelector(".v-real"),
    real: nodo.querySelector(".sl-real"),
    ref: nodo.querySelector(".sl-ref"),
    barra: nodo.querySelector(".sl-barra"),
  };
});

function cambiarEje(e, v, final) {
  if (!ref) return;
  const q = ref.slice();
  q[e] = v;
  cambiarRef(q, { final, eje: e });
}

function empezarJog(e, s, boton) {
  if (!ref || !puedeMover()) return;
  jog = { e, s, boton };
  boton.classList.add("pulsado");
}

function terminarJog(boton) {
  if (!jog || jog.boton !== boton) return;
  boton.classList.remove("pulsado");
  const e = jog.e;
  jog = null;
  cambiarRef(ref, { final: true, eje: e });
  marcarActivo(-1);
}

// El jog avanza la referencia a la velocidad configurada del eje: el
// fantasma va justo delante del robot y al soltar se detiene casi donde está.
function avanzarJog(dt) {
  if (!jog || !ref) return;
  const rpm = enlace.estado?.vel?.[grupo(jog.e)] ?? FIRMWARE.vel[grupo(jog.e)];
  const q = ref.slice();
  q[jog.e] = limitar(jog.e, q[jog.e] + jog.s * rpmAGrados(rpm, jog.e) * dt);
  cambiarRef(q, { eje: jog.e });
}

function marcarActivo(eje) {
  filas.forEach((f, e) => f.nodo.classList.toggle("activo", e === eje));
}

function pintarRef() {
  if (!ref) return;
  filas.forEach((f, e) => {
    f.ref.style.left = fr(e, ref[e]) * 100 + "%";
    f.sl.setAttribute("aria-valuenow", ref[e].toFixed(1));
    if (document.activeElement !== f.num) f.num.value = ref[e].toFixed(1);
  });
  const p = puntoObjetivo || (cin ? cin.directa(ref) : null);
  if (p) {
    ["#pX", "#pY", "#pZ"].forEach((s, i) => { if (document.activeElement !== $(s)) $(s).value = Math.round(p[i] * 1000); });
  }
  if (!puntoObjetivo) pintarAlcance(null);
}

// Cada cuadro, con el real suavizado: el punto rojo avanza con el robot
function pintarReal() {
  if (!realVisto) return;
  filas.forEach((f, e) => {
    const fa = fr(e, realVisto[e]), f0 = fr(e, HOME[e]);
    f.real.style.left = fa * 100 + "%";
    f.barra.style.left = Math.min(fa, f0) * 100 + "%";
    f.barra.style.width = Math.abs(fa - f0) * 100 + "%";
    f.vReal.textContent = fmt(realVisto[e], 1);
  });
}

function pintarEstado(s) {
  pintarRef();
  const oc = $("#ocupado");
  oc.classList.toggle("activo", !!s.ocu);
  oc.textContent = s.ocu ? "moviendo" : "quieto";
  $("#msgFw").textContent = s.msg || "—";
  $("#avisoCal").hidden = !!s.cal;
  $("#avisoWeb").hidden = enlace.modo !== "WEB";
  // Igual que la página del ESP32: sin calibrar, E1 y E2 no se mueven; con
  // un switch pisado, el jog hacia el switch se bloquea.
  filas.forEach((f, e) => {
    f.nodo.classList.toggle("off", e > 0 && !s.cal);
    f.jogs.forEach(b => {
      const vis = Number(b.dataset.vis), sgn = INVERTIDO[e] ? -vis : vis;
      b.classList.toggle("blq", s.bl?.[e] !== 0 && s.bl?.[e] === sgn);
    });
  });
  if (enlace.tipo === "sim" || !velEditada) pintarVelocidad(s);
  actualizarBloqueo();
}

function actualizarBloqueo() {
  const ok = !!puedeMover();
  for (const b of $$("[data-rutina]")) b.disabled = !ok;
  for (const f of filas) {
    f.num.disabled = !ok;
    f.jogs.forEach(b => { b.disabled = !ok; });
  }
}

// ----------------------------------------------------------- etiqueta --

let etiquetaEje = -1;
function mostrarEtiqueta(info) {
  etiquetaEje = info ? info.eje : -1;
  $("#etiquetaEje").hidden = etiquetaEje < 0;
}

function pintarEtiqueta() {
  const el = $("#etiquetaEje");
  const e = escena.arrastre ? escena.arrastre.eje : etiquetaEje;
  if (e < 0 || !ref || escena.modo !== "articulaciones") { el.hidden = true; return; }
  el.hidden = false;
  const { x, y } = escena.pantallaDeEje(e);
  el.style.left = x + "px";
  el.style.top = y + "px";
  el.innerHTML = `<b>${NOMBRES[e]}</b> <span class="mono">${fmt(ref[e], 1)}</span>`;
}

// ------------------------------------------------------------ ratón --

const raton = { botones: 0, rueda: 0 };
const lienzo3d = $("#visor canvas");
lienzo3d.addEventListener("pointerdown", e => { raton.botones = e.buttons; });
window.addEventListener("pointerup", e => { raton.botones = e.buttons; });
lienzo3d.addEventListener("pointermove", e => { raton.botones = e.buttons; });
lienzo3d.addEventListener("wheel", () => { raton.rueda = performance.now(); }, { passive: true });
lienzo3d.addEventListener("contextmenu", e => e.preventDefault());

function pintarRaton() {
  const on = raton.botones | (performance.now() - raton.rueda < 250 ? 4 : 0);
  for (const el of $$("#raton [data-b]")) el.classList.toggle("on", (on & Number(el.dataset.b)) !== 0);
  let izq = "Orbitar";
  const e = escena.arrastre ? escena.arrastre.eje : escena.hover;
  if (escena.modo === "articulaciones" && e >= 0) izq = `Girar ${EJES[e]}`;
  if (escena.modo === "punto" && escena.punto?.tc.axis) izq = "Mover punto";
  $("#ratonIzq").textContent = izq;
}

// ------------------------------------------------------------ cámara --

const toggles = Object.fromEntries($$(".toggle").map(b => [b.dataset.v, b]));

$("#bCamara").addEventListener("click", async () => {
  if (camara.abierta) { camara.cerrar(); return; }
  try { await abrirCamara($("#camSelect").value); } catch { /* el error sale en el panel */ }
});

$("#camSelect").addEventListener("change", e => abrirCamara(e.target.value).catch(() => {}));

async function abrirCamara(id) {
  await camara.abrir(id);
  // Los nombres de las cámaras solo aparecen después de dar permiso
  const lista = await camara.listar();
  const sel = $("#camSelect");
  sel.innerHTML = lista.map(c => `<option value="${c.id}">${c.nombre}</option>`).join("");
  sel.value = camara.deviceId;
  sel.hidden = lista.length < 2;
}

camara.addEventListener("cambio", () => {
  const abierta = camara.abierta;
  $("#bCamara").textContent = abierta ? "Cerrar" : "Abrir";
  const mini = $("#miniatura");
  mini.hidden = !abierta;
  if (abierta && !mini.contains(camara.lienzo)) mini.appendChild(camara.lienzo);
  escena.ponerCamara(poseCam, camara.ancho / camara.alto);
  const antes = escena.cam.rig.visible;
  escena.mostrarCamara(abierta, abierta ? camara.lienzo : null);
  if (antes !== abierta) escena.encuadrar();
  for (const [k, b] of Object.entries(toggles)) {
    const on = k === "seguir" ? seg.activo : camara.usar[k];
    b.classList.toggle("on", on);
    b.classList.toggle("cargando", camara.cargando === k);
    b.disabled = !abierta;
  }
  if (!abierta && seg.activo) ponerSeguimiento(false);
  pintarVision();
});

for (const [k, b] of Object.entries(toggles)) {
  b.addEventListener("click", async () => {
    if (k === "seguir") {
      const si = !seg.activo;
      if (si && !camara.usar.manos) await camara.activar("manos", true);
      ponerSeguimiento(si);
      return;
    }
    await camara.activar(k, !camara.usar[k]);
    if (k === "manos" && !camara.usar.manos && seg.activo) ponerSeguimiento(false);
  });
}

function ponerSeguimiento(si, motivo = "") {
  seg.activo = si && puedeMover() && camara.abierta;
  seg.filtro = seg.ultimo = seg.ancla = null;
  seg.pausa = false;
  if (!seg.activo) escena.marcarObjetivo(null);
  toggles.seguir.classList.toggle("on", seg.activo);
  if (motivo) avisar(motivo);
  if (si && !seg.activo) avisar(camara.abierta ? "Conecta el robot para seguir la mano" : "Abre la cámara primero");
}

camara.addEventListener("deteccion", e => procesarVision(e.detail));

// La mano en el mundo: dirección por el píxel de la palma y distancia por su
// tamaño (muñeca a nudillo medio ≈ 9 cm, nudillos índice a meñique ≈ 7.5 cm).
//
// Seguir es relativo, como un mouse: al abrir la palma (4 dedos) se anclan la
// mano y el punto de agarre, y después el punto de agarre se desplaza lo mismo
// que la mano. La mano suele estar a más de un metro del robot (fuera de su
// alcance), así que seguirla en absoluto solo lo haría apuntar hacia ella.
// Si el objetivo se sale del alcance, el ancla se corre con él (como el
// cursor en la orilla de la pantalla). Cerrar la mano detiene y suelta el
// ancla, como levantar el mouse.
function procesarVision(r) {
  if (!cin || !escena.cam?.pose) return;
  const mano = [...r.manos].sort((a, b) => b.l09 - a.l09)[0];
  let p = null, dist = 0;
  if (mano) {
    const ppm = Math.max(mano.l09 / 0.09, mano.l517 / 0.075);
    dist = Math.min(3, Math.max(0.12, escena.focal(camara.ancho) / ppm));
    p = escena.puntoDesdeImagen(mano.centro[0], mano.centro[1], dist);
    mano.activa = seg.activo && mano.dedos >= 4;
  }
  escena.marcarMano(p, !!mano?.activa);
  pintarVision(mano, dist, r.personas.length);

  if (!seg.activo || !puedeMover()) { escena.marcarObjetivo(null); return; }
  if (mano) {
    if (mano.dedos >= 4) {
      seg.pausa = false;
      seg.filtro = seg.filtro ? seg.filtro.map((v, i) => v + (p[i] - v) * 0.35) : p;
      if (!seg.ancla) seg.ancla = { h0: seg.filtro.slice(), p0: cin.directa(ref) };
      const { h0, p0 } = seg.ancla;
      let obj = p0.map((v, i) => v + GANANCIA_MANO * (seg.filtro[i] - h0[i]));
      obj[1] = Math.max(0.02, obj[1]);
      if (!seg.ultimo || Math.hypot(...obj.map((v, i) => v - seg.ultimo[i])) > 0.006) {
        const sol = cin.inversa(obj, ref);
        if (!sol.alcanzable) {
          const alc = cin.directa(sol.q);
          seg.ancla.p0 = p0.map((v, i) => v + alc[i] - obj[i]);
          obj = alc;
        }
        seg.ultimo = obj;
        cambiarRef(sol.q, { origen: "seguir" });
      }
      escena.marcarObjetivo(seg.ultimo);
    } else if (!seg.pausa) {
      seg.pausa = true;
      seg.filtro = seg.ultimo = seg.ancla = null;
      escena.marcarObjetivo(null);
      if (enlace.estado?.ocu) enlace.cmd("STOP");
      pegarRef(400);
    }
  } else if (r.personas.length) {
    // Sin mano: la base mira a la persona más grande (al pecho, a 1.5 m)
    const per = [...r.personas].sort((a, b) => b.h * b.w - a.h * a.w)[0];
    const pp = escena.puntoDesdeImagen(per.x + per.w / 2, per.y + per.h * 0.3, 1.5);
    const b = cin.baseHacia(pp);
    if (Math.abs(b - ref[0]) > 1.5) cambiarRef([b, ref[1], ref[2]], { origen: "seguir" });
  }
}

function pintarVision(mano, dist, nPersonas = 0) {
  const el = $("#estadoVision");
  if (camara.error) { el.textContent = camara.error; return; }
  if (camara.cargando) { el.textContent = `cargando ${camara.cargando}…`; return; }
  if (!camara.abierta) { el.textContent = ""; return; }
  const partes = [];
  if (mano) partes.push(`mano · ${mano.dedos} dedos · ${dist.toFixed(2)} m${seg.activo ? (mano.dedos >= 4 ? " · siguiendo" : " · pausa") : ""}`);
  if (nPersonas) partes.push(`${nPersonas} persona${nPersonas > 1 ? "s" : ""}`);
  el.textContent = partes.join(" · ");
}

function pintarPoseCam() {
  for (const i of $$(".pose-cam input")) i.value = poseCam[i.dataset.k];
}
for (const i of $$(".pose-cam input")) {
  i.addEventListener("change", () => {
    const v = Number(i.value);
    if (!Number.isFinite(v)) return;
    poseCam[i.dataset.k] = v;
    guardarCamara(poseCam);
    escena.ponerCamara(poseCam, camara.ancho / camara.alto);
  });
}
pintarPoseCam();

// ------------------------------------------------------------ cuadro --

let tPrev = performance.now();
function cuadro(t) {
  const dt = Math.min(0.1, (t - tPrev) / 1000);
  tPrev = t;
  avanzarJog(dt);
  if (real && realVisto) {
    // En real el STATE llega cada 150 ms: se suaviza para que no salte
    const k = enlace.tipo === "serial" ? 1 - Math.exp(-dt / 0.1) : 1;
    realVisto = realVisto.map((v, e) => v + (real[e] - v) * k);
    escena.setReal(realVisto);
    pintarReal();
  }
  camara.cuadro(t);
  pintarEtiqueta();
  pintarRaton();
  requestAnimationFrame(cuadro);
}
requestAnimationFrame(cuadro);

// ------------------------------------------------------------ acciones --

async function alto() {
  jog = null;
  rutina = null;
  $$(".jog.pulsado").forEach(b => b.classList.remove("pulsado"));
  if (seg.activo) ponerSeguimiento(false);
  pegarRef(400);
  if (enlace?.conectado) await enlace.cmd("STOP");
}

$("#bAlto").addEventListener("click", alto);
window.addEventListener("keydown", e => {
  if (e.key === "Escape") { e.preventDefault(); alto(); }
});

// Poses clave de cada rutina, a partir de la pose actual. El firmware mueve
// un eje por paso (HOME: E2, E1, base), así que cada paso es un cuadro.
function framesRutina(cmd, q) {
  const H = HOME;
  if (cmd === "CAL") return [[q[0], H[1], H[2]]];      // CAL no mueve la base
  if (cmd === "HOME") return [[0, H[1], H[2]]];
  const S = FIRMWARE.saludo, f = [];
  let p = q.slice();
  const paso = (e, v) => { if (Math.abs(p[e] - v) > 0.05) { p = p.slice(); p[e] = v; f.push(p); } };
  const irHome = () => { paso(2, H[2]); paso(1, H[1]); paso(0, 0); };
  irHome();
  paso(1, S.e1);
  for (let c = 0; c < S.ciclos; c++) { paso(2, S.e2a); paso(2, S.e2b); }
  irHome();
  return f.length ? f : [q.slice()];
}

for (const b of $$("[data-rutina]")) {
  b.addEventListener("click", async () => {
    const cmd = b.dataset.rutina;
    if (seg.activo) ponerSeguimiento(false);
    if (cmd !== "CAL" && !enlace.estado?.cal) { avisar("Calibra primero"); return; }
    pegarRef(0);
    rutina = { frames: framesRutina(cmd, real), i: 0, t0: performance.now(), vioOcupado: false };
    const r = await enlace.cmd(cmd);
    if (r && r.startsWith("ERR")) {
      rutina = null;
      pegarRef(0);
      avisar(r === "ERR MODO_WEB" ? "El control lo tiene la página del ESP32" : r);
    }
  });
}

$("#segModo").addEventListener("click", async e => {
  const b = e.target.closest("button");
  if (!b || b.classList.contains("on")) return;
  $$("#segModo button").forEach(x => x.classList.toggle("on", x === b));
  if (seg.activo) ponerSeguimiento(false);
  if (b.dataset.modo === "digital") {
    // El simulador sigue desde donde iba el real
    if (real && enlace.tipo === "serial") sim.ponerEn(real, !!enlace.estado?.cal);
    usarEnlace(sim);
  } else {
    if (!serial && EnlaceSerial.disponible()) serial = new EnlaceSerial();
    usarEnlace(serial || { tipo: "serial", conectado: false, estado: null, modo: null, addEventListener() {}, removeEventListener() {} });
    if (serial && !serial.conectado) {
      try { await serial.conectarGuardado(); } catch { /* se ve en el panel */ }
    }
  }
});

$("#bConectar").addEventListener("click", async () => {
  try { await serial.conectar(); } catch (e) { if (e.name !== "NotFoundError") avisar(serial.error || e.message); }
  alConexion();
});

$("#segInteraccion").addEventListener("click", e => {
  const b = e.target.closest("button");
  if (!b) return;
  $$("#segInteraccion button").forEach(x => x.classList.toggle("on", x === b));
  escena.setModo(b.dataset.i);
  puntoObjetivo = null;
  pintarAlcance(null);
});

$("#bEncuadrar").addEventListener("click", () => escena.encuadrar());

// ----------------------------------------------------------- ajustes --

let velEditada = false;
function pintarVelocidad(s) {
  for (const f of $$(".vel-fila")) {
    const g = Number(f.dataset.grupo);
    const v = f.querySelector(".vel"), a = f.querySelector(".acel");
    if (document.activeElement !== v) v.value = s.vel?.[g] ?? "";
    if (document.activeElement !== a) a.value = s.acel?.[g] ?? "";
  }
}
for (const f of $$(".vel-fila")) {
  const g = Number(f.dataset.grupo);
  const mandar = async () => {
    const v = f.querySelector(".vel").value, a = f.querySelector(".acel").value;
    if (!enlace?.conectado || v === "") return;
    velEditada = true;
    await enlace.cmd(`VEL ${g ? "E" : "B"} ${v}${a !== "" ? " " + a : ""}`);
    velEditada = false;
  };
  f.querySelectorAll("input").forEach(i => i.addEventListener("change", mandar));
}

$("#simSimultaneo").addEventListener("change", e => { sim.sim.simultaneo = e.target.checked; });
$("#segTiempo").addEventListener("click", e => {
  const b = e.target.closest("button");
  if (!b) return;
  $$("#segTiempo button").forEach(x => x.classList.toggle("on", x === b));
  sim.sim.escala = Number(b.dataset.x);
});

function pintarCalibracion() {
  for (const f of $$(".gemelo-fila")) {
    const e = Number(f.dataset.eje);
    f.querySelector(".invertir").checked = cal.sentido[e] !== GEMELO_DEFECTO.sentido[e];
    f.querySelector(".offset").value = cal.offset[e];
  }
}
for (const f of $$(".gemelo-fila")) {
  const e = Number(f.dataset.eje);
  f.querySelector(".invertir").addEventListener("change", ev => {
    cal.sentido[e] = GEMELO_DEFECTO.sentido[e] * (ev.target.checked ? -1 : 1);
    calibracionCambio();
  });
  f.querySelector(".offset").addEventListener("change", ev => {
    cal.offset[e] = Number(ev.target.value) || 0;
    calibracionCambio();
  });
}
$("#bCalDefecto").addEventListener("click", () => {
  Object.assign(cal, structuredClone(GEMELO_DEFECTO));
  pintarCalibracion();
  calibracionCambio();
});
function calibracionCambio() {
  guardarCalibracion(cal);
  escena.reconstruirArcos();
}
pintarCalibracion();

// ----------------------------------------------------------- consola --

const consola = $("#consola");
function alLog(ev) {
  const { d, x } = ev.detail;
  const div = document.createElement("div");
  div.className = d;
  div.textContent = ({ tx: "→ ", rx: "← ", msg: "· ", sys: "# " }[d] || "") + x;
  const abajo = consola.scrollTop + consola.clientHeight >= consola.scrollHeight - 4;
  consola.appendChild(div);
  while (consola.childElementCount > 400) consola.firstElementChild.remove();
  if (abajo) consola.scrollTop = consola.scrollHeight;
}

const historial = [];
let iHist = 0;
$("#consolaForm").addEventListener("submit", async e => {
  e.preventDefault();
  const input = $("#consolaInput");
  const linea = input.value.trim();
  if (!linea || !enlace?.conectado) return;
  historial.push(linea);
  iHist = historial.length;
  input.value = "";
  if (/^(CAL|HOME|SALUDO|GOTO|POSE|STOP)\b/i.test(linea)) { rutina = null; pegarRef(600); }
  await enlace.cmd(linea);
});
$("#consolaInput").addEventListener("keydown", e => {
  if (e.key !== "ArrowUp" && e.key !== "ArrowDown") return;
  e.preventDefault();
  iHist = Math.max(0, Math.min(historial.length, iHist + (e.key === "ArrowUp" ? -1 : 1)));
  e.target.value = historial[iHist] ?? "";
});

// ------------------------------------------------------------ varios --

let timerAviso = null;
function avisar(texto) {
  const el = $("#aviso");
  el.textContent = texto;
  el.hidden = false;
  clearTimeout(timerAviso);
  timerAviso = setTimeout(() => { el.hidden = true; }, 3200);
}

function fmt(v, dec = 0) {
  return (v < 0 ? "−" : "") + Math.abs(v).toFixed(dec) + "°";
}

// Al cerrar la pestaña con el robot real, que no se quede moviendo
window.addEventListener("pagehide", () => { if (serial?.conectado) serial.cmd("STOP"); });
