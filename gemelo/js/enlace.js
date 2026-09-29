// enlace.js - con quién habla la página: la ESP32 simulada o la real.
//
// Los dos tienen la misma forma, para que el resto de la página no sepa cuál
// está usando:
//
//   await enlace.cmd("GOTO E1 45")  -> "OK GOTO" | "ERR ..." | null (sin respuesta)
//   enlace.estado                   último STATE (el JSON del firmware)
//   enlace.modo                     "PC" | "WEB" (quién tiene el control)
//   eventos: "estado", "log" {d: tx|rx|msg|sys, x}, "conexion",
//            "crudo" {d: tx|rx, x, sondeo}: cada línea tal cual pasa por el cable
//
// El real va por Web Serial (Chrome o Edge de escritorio, sobre https o
// localhost). Replica lo que hacía manipulador/robot.py: una línea, espera la
// primera respuesta que no sea MSG, y STATE? cada 150 ms.

import { SimESP32 } from "./sim.js";

class Base extends EventTarget {
  constructor() {
    super();
    this.estado = null;
    this.modo = null;
    this.conectado = false;
  }
  log(d, x) { this.dispatchEvent(new CustomEvent("log", { detail: { d, x, t: Date.now() } })); }
  crudo(d, x, sondeo = false) { this.dispatchEvent(new CustomEvent("crudo", { detail: { d, x, sondeo, t: Date.now() } })); }
  avisarEstado() { this.dispatchEvent(new Event("estado")); }
  avisarConexion() { this.dispatchEvent(new Event("conexion")); }
}

// ------------------------------------------------------------- simulado --

export class EnlaceSim extends Base {
  constructor() {
    super();
    this.tipo = "sim";
    this.sim = new SimESP32(l => { this.crudo("rx", l); this.log("msg", l.slice(4)); });
    this.modo = "PC";
    this.conectado = true;
    this.estado = this.sim.estado();
    let t0 = performance.now();
    // Intervalo y no requestAnimationFrame: el robot simulado sigue moviéndose
    // aunque la pestaña esté en segundo plano (el navegador lo baja a 1 Hz).
    this.timer = setInterval(() => {
      const t = performance.now();
      this.sim.paso((t - t0) / 1000);
      t0 = t;
      this.estado = this.sim.estado();
      this.avisarEstado();
    }, 16);
  }

  async cmd(linea, { silencioso = false } = {}) {
    if (!silencioso) this.log("tx", linea);
    this.crudo("tx", linea, silencioso);
    const r = this.sim.linea(linea);
    if (r) this.crudo("rx", r, silencioso);
    if (!silencioso && r) this.log("rx", r);
    return r;
  }

  // Arranca desde una pose (al pasar de Real a Digital, sigue donde iba el real)
  ponerEn(q, calibrado) {
    this.sim.pos = q.slice();
    this.sim.calibrado = calibrado;
  }

  cerrar() { clearInterval(this.timer); }
}

// ----------------------------------------------------------------- real --

const VID_ESPRESSIF = 0x303a;

// Web Serial no deja leer el nombre del puerto (COM7, ttyACM0): solo el VID y
// el PID del USB. Con eso se nombra lo que se conoce.
const CONOCIDOS = {
  "303a:1001": "ESP32-C3 · USB nativo",
  "10c4:ea60": "CP210x",
  "1a86:7523": "CH340",
  "1a86:55d4": "CH9102",
  "0403:6001": "FTDI",
};

export function nombrePuerto(p) {
  const { usbVendorId: v, usbProductId: d } = p.getInfo();
  if (v === undefined) return "Puerto serie";
  const hex = n => (n ?? 0).toString(16).padStart(4, "0");
  const id = `${hex(v)}:${hex(d)}`;
  return `${CONOCIDOS[id] || (v === VID_ESPRESSIF ? "Espressif" : "USB")} · ${id.toUpperCase()}`;
}

export const esEspressif = p => p.getInfo().usbVendorId === VID_ESPRESSIF;

export class EnlaceSerial extends Base {
  static disponible() { return "serial" in navigator; }

  constructor() {
    super();
    this.tipo = "serial";
    this.puerto = null;
    this.escritor = null;
    this.lector = null;
    this.cola = Promise.resolve();       // un comando a la vez
    this.esperando = null;               // resolve de la respuesta pendiente
    this.cerrando = false;
    this.sondeo = null;
    this.error = "";
    this.ultimo = null;                  // el último puerto que se abrió
    this.vuelta = 0;                     // sondeo vigente (al reconectar no se duplica)
  }

  // Puertos que el usuario ya autorizó antes: se conecta sin preguntar
  async conectarGuardado() {
    const ps = await navigator.serial.getPorts();
    const p = ps.find(x => x === this.ultimo) || ps.find(esEspressif) || ps[0];
    if (p) await this.abrir(p);
    return !!p;
  }

  // Sin filtro: la lista del navegador muestra todos los puertos con su COM
  async buscar() {
    await this.cambiarA(await navigator.serial.requestPort());
  }

  async cambiarA(p) {
    if (this.puerto === p && this.conectado) return;
    if (this.puerto) await this.cerrar();
    await this.abrir(p);
  }

  async abrir(p) {
    try {
      await p.open({ baudRate: 115200 });
    } catch (e) {
      this.error = /already open|InvalidState|NetworkError/i.test(String(e))
        ? "El puerto está ocupado: cierra Thonny, el monitor serial o el panel de Python"
        : String(e.message || e);
      this.log("sys", this.error);
      this.avisarConexion();
      throw new Error(this.error);
    }
    this.puerto = this.ultimo = p;
    this.cerrando = false;
    this.escritor = p.writable.getWriter();
    this.conectado = true;
    this.error = "";
    this.log("sys", "Conectado por USB");
    this.avisarConexion();
    this.leer();
    await this.cmd("PING");
    this.sondear();
  }

  async leer() {
    const decodificador = new TextDecoder();
    let buf = "";
    while (this.puerto?.readable && !this.cerrando) {
      this.lector = this.puerto.readable.getReader();
      try {
        for (;;) {
          const { value, done } = await this.lector.read();
          if (done) break;
          buf += decodificador.decode(value, { stream: true });
          let i;
          while ((i = buf.indexOf("\n")) >= 0) {
            const linea = buf.slice(0, i).trim();
            buf = buf.slice(i + 1);
            if (linea) this.recibir(linea);
          }
        }
      } catch (e) {
        if (!this.cerrando) this.log("sys", "Se perdió el puerto: " + (e.message || e));
        break;
      } finally {
        this.lector.releaseLock();
      }
    }
    if (!this.cerrando) this.cerrar("se desconectó");
  }

  recibir(linea) {
    this.crudo("rx", linea, !!this.esperando && this.esperandoSondeo && !linea.startsWith("MSG "));
    if (linea.startsWith("MSG ")) { this.log("msg", linea.slice(4)); return; }
    if (linea.startsWith("STATE ")) {
      try { this.estado = JSON.parse(linea.slice(6)); this.avisarEstado(); } catch { /* línea cortada */ }
    } else if (linea.startsWith("MODE ")) {
      this.modo = linea.slice(5).trim();
    }
    if (this.esperando) {
      const r = this.esperando;
      this.esperando = null;
      r(linea);
    } else {
      this.log("rx", linea);
    }
  }

  cmd(linea, { silencioso = false, timeout = 1000 } = {}) {
    const tarea = this.cola.then(async () => {
      if (!this.conectado) return "ERR SIN_CONEXION";
      if (!silencioso) this.log("tx", linea);
      const respuesta = new Promise(res => {
        this.esperando = res;
        this.esperandoSondeo = silencioso;
        setTimeout(() => { if (this.esperando === res) { this.esperando = null; res(null); } }, timeout);
      });
      try {
        this.crudo("tx", linea, silencioso);     // antes de escribir: la respuesta puede llegar primero
        await this.escritor.write(new TextEncoder().encode(linea + "\n"));
      } catch (e) {
        this.esperando = null;
        this.cerrar(String(e.message || e));
        return "ERR SIN_CONEXION";
      }
      const r = await respuesta;
      if (!silencioso) this.log(r === null ? "sys" : "rx", r ?? "(sin respuesta)");
      return r;
    });
    this.cola = tarea.catch(() => {});
    return tarea;
  }

  sondear() {
    let tModo = 0;
    const mia = ++this.vuelta;
    const vuelta = async () => {
      if (!this.conectado || mia !== this.vuelta) return;
      await this.cmd("STATE?", { silencioso: true });
      if (Date.now() - tModo > 1000) {
        tModo = Date.now();
        await this.cmd("MODE?", { silencioso: true });
      }
      this.sondeo = setTimeout(vuelta, 150);
    };
    vuelta();
  }

  async cerrar(motivo = "") {
    if (!this.puerto) return;
    this.cerrando = true;
    clearTimeout(this.sondeo);
    const p = this.puerto;
    this.puerto = null;
    this.conectado = false;
    this.estado = null;
    this.modo = null;
    try { await this.lector?.cancel(); } catch { /* ya cerrado */ }
    try { this.escritor?.releaseLock(); } catch { /* nada */ }
    try { await p.close(); } catch { /* nada */ }
    this.log("sys", "Desconectado" + (motivo ? ": " + motivo : ""));
    this.avisarConexion();
  }
}
