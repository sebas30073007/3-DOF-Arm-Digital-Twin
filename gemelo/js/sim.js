// sim.js - la ESP32 con manipulador_v7, simulada.
//
// Habla el mismo protocolo de texto que el firmware por USB (una línea, una
// respuesta; avisos con MSG) y se mueve como él:
//
//   - un eje a la vez: el loop atiende el primer eje con objetivo pendiente
//     (B, luego E1, luego E2), igual que v7;
//   - perfil trapezoidal en rpm del motor, con la reducción de cada eje;
//   - si llega otro GOTO al eje que se mueve, corrige el destino al vuelo si
//     todavía alcanza a frenar; si no, frena y lo ejecuta después;
//   - CAL, HOME y SALUDO con la misma secuencia y los mismos mensajes;
//   - STOP frena con rampa y vacía la cola.
//
// Lo que no simula: pasos perdidos, la página web del ESP32 (siempre manda la
// PC) y el botón BOOT. `simultaneo` mueve todos los ejes a la vez, para probar
// cómo se sentiría el firmware cuando lo haga.

import { FIRMWARE as F, HOME, grupo, rpmAGrados } from "./config.js";

const OK = 0, ALTO = 1;
const PASO_SIM = 0.002;          // s por subpaso de integración

const ppg = (e, pr) => (pr[grupo(e)] * F.reduccion[e]) / 360;       // pasos por grado
const pasosAGrados = (pasos1000, e, pr) => ((pasos1000 * pr[1]) / 1000) / ppg(e, pr);

export class SimESP32 {
  constructor(alAviso = () => {}) {
    this.alAviso = alAviso;            // recibe cada línea "MSG ..."
    this.pos = HOME.slice();
    this.calibrado = true;             // arranca en HOME ya calibrado: es para jugar
    this.pr = F.pulsos.slice();
    this.vel = F.vel.slice();
    this.acel = F.acel.slice();
    this.objetivo = [0, 0, 0];
    this.hay = [false, false, false];
    this.mov = [null, null, null];     // movimiento en curso por eje
    this.tareas = [];                  // corrutinas: irA, rutinas, calibración
    this.pedirAlto = false;
    this.pedirCalib = false;
    this.pedirRutina = 0;
    this.rechazos = 0;
    this.ultimo = [-1, 0, 0];          // eje, grados, segundos del último movimiento
    this.t = 0;
    this.simultaneo = false;
    this.escala = 1;                   // velocidad del tiempo simulado
    this.msg = "";
    this.setMsg("Simulador listo (v7)");
  }

  setMsg(t) {
    this.msg = t;
    this.alAviso("MSG " + t);
  }

  get ocupado() {
    return this.tareas.length > 0 || this.mov.some(Boolean) || this.hay.some(Boolean) ||
      this.pedirCalib || this.pedirRutina > 0;
  }

  // ---------------------------------------------------------- protocolo --

  linea(texto) {
    const [cmd0, a1, a2, a3] = texto.trim().split(/\s+/);
    if (!cmd0) return null;
    const cmd = cmd0.toUpperCase();
    if (cmd === "PING") return "PONG";
    if (cmd === "STATE?") return "STATE " + JSON.stringify(this.estado());
    if (cmd === "MODE?") return "MODE PC";
    if (cmd === "STOP") { this.accionAlto(); return "OK STOP"; }
    if (cmd === "HELP" || cmd === "?") return "OK PING STATE? MODE? STOP CAL HOME SALUDO GOTO POSE VEL PULSOS";
    if (cmd === "VEL") {
      const g = parseGrupo(a1), v = parseNum(a2), a = parseNum(a3);
      if (g < 0 || v === null) return "ERR ARGS";
      this.vel[g] = Math.min(F.velMax[g], Math.max(F.velMin, v));
      if (a !== null) this.acel[g] = Math.min(F.acelMax[g], Math.max(F.acelMin, a));
      return `OK VEL ${g ? "E" : "B"} ${Math.round(this.vel[g])} ${Math.round(this.acel[g])}`;
    }
    if (cmd === "PULSOS") {
      const g = parseGrupo(a1), v = parseInt(a2, 10);
      if (g < 0 || ![200, 800, 1000].includes(v)) return "ERR ARGS";
      this.pr[g] = v;
      this.setMsg(`${g === 0 ? "Base" : "Eslabones"}: ${v} pulsos/rev (ajusta el CL57T igual)`);
      return "OK PULSOS";
    }
    if (cmd === "GRIP") return "ERR NO_IMPLEMENTADO";
    if (!["CAL", "HOME", "SALUDO", "GOTO", "POSE"].includes(cmd)) return "ERR COMANDO";
    if (cmd === "CAL") { this.accionAlto(); this.pedirCalib = true; return "OK CAL"; }
    if (cmd === "HOME") { this.pedirRutina = 1; return "OK HOME"; }
    if (cmd === "SALUDO") { this.pedirRutina = 2; return "OK SALUDO"; }
    if (cmd === "GOTO") {
      const e = parseEje(a1), deg = parseNum(a2);
      if (e < 0 || deg === null) return "ERR ARGS";
      return this.encolar(e, deg) || "OK GOTO";
    }
    // POSE
    const t = [a1, a2, a3], v = [0, 0, 0], usar = [false, false, false];
    for (let e = 0; e < 3; e++) {
      if (t[e] === undefined) return "ERR ARGS";
      usar[e] = !["NA", "X", "-"].includes(t[e].toUpperCase());
      if (!usar[e]) continue;
      v[e] = parseNum(t[e]);
      if (v[e] === null) return "ERR ARGS";
      if (e !== 0 && !this.calibrado) return "ERR NO_CALIBRADO";
      if (v[e] < F.limMin[e] || v[e] > F.limMax[e]) return "ERR FUERA_DE_LIMITE";
    }
    for (let e = 0; e < 3; e++) if (usar[e]) this.encolar(e, v[e]);
    return "OK POSE";
  }

  encolar(e, deg) {
    if (e !== 0 && !this.calibrado) return "ERR NO_CALIBRADO";
    if (deg < F.limMin[e] || deg > F.limMax[e]) return "ERR FUERA_DE_LIMITE";
    this.objetivo[e] = deg;
    this.hay[e] = true;
    return null;
  }

  accionAlto() {
    this.hay = [false, false, false];
    this.pedirRutina = 0;
    this.pedirAlto = true;
  }

  estado() {
    const r2 = v => Math.round(v * 100) / 100;
    const sw = this.switches();
    return {
      b: r2(this.pos[0]), c: r2(this.pos[1]), m: r2(this.pos[2]),
      sw1: 0, sw2: sw[1] ? 1 : 0, sw3: sw[2] ? 1 : 0,
      cal: this.calibrado ? 1 : 0, ocu: this.ocupado ? 1 : 0, msg: this.msg,
      lim: [F.limMin[0], F.limMax[0], F.limMin[1], F.limMax[1], F.limMin[2], F.limMax[2]],
      pr: this.pr.slice(), vel: this.vel.map(Math.round), acel: this.acel.map(Math.round),
      vr: [F.velMin, F.velMax[0], F.velMax[1], F.acelMin, F.acelMax[0], F.acelMax[1]],
      bl: [0, sw[1] ? -1 : 0, sw[2] ? 1 : 0], rs: this.rechazos,
      ue: this.ultimo[0], ud: r2(this.ultimo[1]), us: r2(this.ultimo[2]), src: 1,
    };
  }

  // Los switches de home están en E1 = 0 y en E2 = 0 (el lado hacia el que calibran)
  switches() {
    return [false, this.pos[1] <= 0.01, this.pos[2] >= -0.01];
  }

  // ------------------------------------------------------------- tiempo --

  paso(dt) {
    let resto = Math.min(dt, 0.25) * this.escala;
    while (resto > 1e-9) {
      const h = Math.min(PASO_SIM, resto);
      resto -= h;
      this.t += h;
      this.loop();
      for (let e = 0; e < 3; e++) if (this.mov[e]) this.integrar(this.mov[e], h);
      this.reanudar();
    }
  }

  // Lo que hace loop() del firmware cuando no hay nada corriendo
  loop() {
    if (this.pedirCalib && !this.mov.some(Boolean)) {
      this.tareas = [];
      this.pedirCalib = false;
      this.pedirAlto = false;
      this.lanzar(this.calibrar());
      return;
    }
    if (this.tareas.length || this.mov.some(Boolean)) {
      if (!this.simultaneo || this.tareas.some(t => t.rutina)) return;
    } else {
      this.pedirAlto = false;
    }
    if (this.pedirRutina && !this.tareas.length) {
      const n = this.pedirRutina;
      this.pedirRutina = 0;
      this.lanzar(n === 1 ? this.rutinaHome() : this.rutinaSaludo(), true);
      return;
    }
    for (let e = 0; e < 3; e++) {
      if (!this.hay[e] || this.mov[e] || this.tareas.some(t => t.eje === e)) continue;
      this.hay[e] = false;
      this.lanzar(this.irA(e, this.objetivo[e]), false, e);
      if (!this.simultaneo) break;
    }
  }

  lanzar(gen, rutina = false, eje = -1) {
    const t = { gen, rutina, eje, espera: null, valor: undefined };
    this.tareas.push(t);
    this.avanzar(t);
  }

  // Empuja una corrutina hasta que pida esperar algo
  avanzar(t) {
    const r = t.gen.next(t.valor);
    if (r.done) { this.tareas = this.tareas.filter(x => x !== t); return; }
    t.espera = r.value;
    t.valor = undefined;
    if (t.espera.mover !== undefined) {
      t.espera.m = this.empezar(t.espera);
      if (!t.espera.m) t.valor = OK;       // ya estaba ahí
    } else if (t.espera.pausa !== undefined) {
      t.espera.hasta = this.t + t.espera.pausa / 1000;
    }
  }

  reanudar() {
    for (const t of this.tareas.slice()) {
      const w = t.espera;
      if (!w) continue;
      if (w.mover !== undefined) {
        if (w.m && !w.m.fin) continue;
        t.valor = w.m ? w.m.resultado : OK;
      } else if (w.pausa !== undefined) {
        if (this.pedirAlto) t.valor = ALTO;
        else if (this.t < w.hasta) continue;
        else t.valor = OK;
      }
      t.espera = null;
      this.avanzar(t);
    }
  }

  // ---------------------------------------------------------- movimiento --

  empezar({ mover: e, deg, rpm, acel, retarget = false, libre = false }) {
    const destino = libre ? deg : Math.min(F.limMax[e], Math.max(F.limMin[e], deg));
    const pasos = Math.round(destino * ppg(e, this.pr)) - Math.round(this.pos[e] * ppg(e, this.pr));
    if (pasos === 0) return null;
    const g = grupo(e);
    const vMax = rpmAGrados(rpm ?? this.vel[g], e);
    const m = {
      eje: e, signo: Math.sign(pasos), inicio: this.pos[e], total: Math.abs(pasos) / ppg(e, this.pr),
      hechos: 0, vMax, vMin: Math.min(vMax, rpmAGrados(F.rpmArranque, e)),
      a: rpmAGrados(acel ?? this.acel[g], e), retarget, frenando: false, fin: false, resultado: OK, t0: this.t,
    };
    m.v = m.vMin;
    this.mov[e] = m;
    return m;
  }

  integrar(m, h) {
    const e = m.eje;
    if (this.pedirAlto) m.frenando = true;
    if (m.retarget && !m.frenando && this.hay[e]) this.corregir(m);
    const freno = (m.v * m.v - m.vMin * m.vMin) / (2 * m.a);
    if (m.frenando || m.total - m.hechos <= freno) m.v = Math.max(m.vMin, m.v - m.a * h);
    else m.v = Math.min(m.vMax, m.v + m.a * h);
    m.hechos += m.v * h;
    let terminar = false;
    if (m.hechos >= m.total) { m.hechos = m.total; terminar = true; m.resultado = OK; }
    else if (m.frenando && m.v <= m.vMin + 1e-9) { terminar = true; m.resultado = ALTO; }
    this.pos[e] = m.inicio + m.signo * m.hechos;
    if (terminar) {
      const k = ppg(e, this.pr);
      this.pos[e] = Math.round(this.pos[e] * k) / k;
      this.ultimo = [e, Math.abs(this.pos[e] - m.inicio), this.t - m.t0];
      m.fin = true;
      this.mov[e] = null;
    }
  }

  // revisarRetarget() del firmware: alarga o acorta el movimiento si todavía
  // alcanza a frenar; si no, frena y el objetivo queda pendiente.
  corregir(m) {
    const e = m.eje;
    const deg = Math.min(F.limMax[e], Math.max(F.limMin[e], this.objetivo[e]));
    const rel = (deg - m.inicio) * m.signo;
    const freno = (m.v * m.v - m.vMin * m.vMin) / (2 * m.a);
    if (rel >= m.hechos + freno + 1 / ppg(e, this.pr)) {
      m.total = rel;
      this.hay[e] = false;
    } else {
      m.frenando = true;
    }
  }

  // ----------------------------------------------------------- rutinas --

  *irA(e, deg) {
    if (e !== 0 && !this.calibrado) { this.setMsg("Calibra primero"); this.rechazos++; return; }
    this.setMsg("Moviendo...");
    const r = yield { mover: e, deg, retarget: true };
    if (r === ALTO && !this.pedirAlto && this.hay[e]) return;   // frenó para cambiar de objetivo
    this.setMsg(r === ALTO ? "Detenido" : "Listo");
  }

  *irHome() {
    for (const e of [2, 1, 0]) {
      const r = yield { mover: e, deg: HOME[e] };
      if (r !== OK) return r;
    }
    return OK;
  }

  *rutinaHome() {
    if (!this.calibrado) { this.setMsg("Calibra primero"); this.rechazos++; return; }
    this.setMsg("Regresando a HOME...");
    const r = yield* this.irHome();
    this.setMsg(r === OK ? "En HOME" : "Detenido");
  }

  *rutinaSaludo() {
    if (!this.calibrado) { this.setMsg("Calibra primero"); this.rechazos++; return; }
    const S = F.saludo;
    this.setMsg("Saludo: yendo a HOME...");
    let r = yield* this.irHome();
    if (r === OK) { this.setMsg("Saludo: eslabón 1"); r = yield { mover: 1, deg: S.e1 }; }
    for (let c = 1; c <= S.ciclos && r === OK; c++) {
      this.setMsg(`Saludo ${c}/${S.ciclos}`);
      r = yield { mover: 2, deg: S.e2a };
      if (r === OK) r = yield { pausa: S.pausaMs };
      if (r === OK) r = yield { mover: 2, deg: S.e2b };
      if (r === OK) r = yield { pausa: S.pausaMs };
    }
    if (r === OK) { this.setMsg("Saludo: regresando a HOME..."); r = yield* this.irHome(); }
    this.setMsg(r === OK ? "Saludo terminado" : "Detenido");
  }

  // homeEje(): busca el switch rápido, se separa, lo vuelve a tocar lento y
  // se queda a la distancia de despeje. El switch de E1 está en 0 (hacia -),
  // el de E2 en 0 (hacia +).
  *homeEje(e) {
    const lejos = e === 1 ? 1 : -1;
    const lento = { rpm: F.homeLento, acel: F.homeAcel };
    const ir = (deg, v) => ({ mover: e, deg, libre: true, ...v });
    let r = yield ir(0, { rpm: F.homeRapido, acel: F.homeAcel });
    if (r === OK) r = yield ir(lejos * pasosAGrados(F.backoff, e, this.pr), lento);
    if (r === OK) r = yield ir(0, lento);
    if (r === OK) r = yield ir(lejos * pasosAGrados(F.clearance, e, this.pr), lento);
    return r;
  }

  *calibrar() {
    this.calibrado = false;
    this.setMsg("Calibrando eslabón 1...");
    let r = yield* this.homeEje(1);
    if (r === OK) r = yield { pausa: 300 };
    if (r === OK) { this.setMsg("Calibrando eslabón 2..."); r = yield* this.homeEje(2); }
    if (r !== OK) { this.setMsg("Detenido"); return; }
    this.pos[1] = HOME[1];
    this.pos[2] = HOME[2];
    this.calibrado = true;
    this.setMsg("Calibrado");
  }
}

function parseEje(t) {
  if (!t) return -1;
  const u = t.toUpperCase();
  if (u === "B" || u === "BASE" || u === "0") return 0;
  if (u === "E1" || u === "1") return 1;
  if (u === "E2" || u === "2") return 2;
  return -1;
}

function parseGrupo(t) {
  if (!t) return -1;
  const u = t.toUpperCase();
  if (u === "B" || u === "BASE" || u === "0") return 0;
  if (u === "E" || u === "ESL" || u === "1") return 1;
  return -1;
}

function parseNum(t) {
  if (t === undefined || t === "") return null;
  const v = Number(t);
  return Number.isFinite(v) ? v : null;
}
