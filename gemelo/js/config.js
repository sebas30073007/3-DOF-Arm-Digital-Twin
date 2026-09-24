// config.js - constantes del firmware manipulador_v7 y del gemelo.
//
// Los números de FIRMWARE son copia de firmware/manipulador_v7/manipulador_v7.ino.
// Si cambian allá, cambian aquí: el simulador los usa para moverse igual que
// la ESP32, y la interfaz para saber hasta dónde deja llegar cada eje.

export const EJES = ["B", "E1", "E2"];                 // nombres en el protocolo
export const NOMBRES = ["Base", "Eslabón 1", "Eslabón 2"];

export const FIRMWARE = {
  reduccion: [10, 25, 25],
  limMin: [-200, 0, -220],
  limMax: [20, 136.5, 0],
  pulsos: [1000, 200],          // { base, eslabones } pulsos/rev de los CL57T
  vel: [10, 20],                // rpm del motor, por grupo
  acel: [70, 70],               // rpm/s
  velMin: 2, velMax: [30, 120],
  acelMin: 10, acelMax: [400, 400],
  rpmArranque: 5,
  homeRapido: 17, homeLento: 9, homeAcel: 200,
  // Distancias del homing, en pasos a 1000 pulsos/rev (se escalan)
  backoff: 220, clearance: 120,
  saludo: { e1: 30, e2a: -45, e2b: -30, ciclos: 2, pausaMs: 20 },
};

// HOME = donde termina la calibración: el despeje (clearance) tras tocar el
// switch. En grados no depende de los pulsos/rev: 0.12 rev de motor / 25.
const DESPEJE = (FIRMWARE.clearance / 1000) * 360 / FIRMWARE.reduccion[1];
export const HOME = [0, DESPEJE, -DESPEJE];

export const grupo = e => (e === 0 ? 0 : 1);

// Grados de articulación por segundo a partir de rpm del motor
export const rpmAGrados = (rpm, e) => (rpm * 6) / FIRMWARE.reduccion[e];

// ---- Gemelo: cómo se traduce un ángulo del firmware a una rotación del modelo ----
// El GLB está en la pose HOME. Rotación del modelo = sentido · (ángulo − HOME):
//   base  sentido −1  (confirmado contra el robot real)
//   E1    sentido +1  E1+ inclina el eslabón 1 hacia adelante (de 47° atrás a
//                     horizontal en 136.5°)
//   E2    sentido +1  en HOME el eslabón 2 ya está casi plegado contra el 1
//                     (a 36°), así que E2− lo levanta y lo lleva hacia atrás
// Se puede corregir en la página (Ajustes → Gemelo) y se guarda en el navegador.
export const GEMELO_DEFECTO = {
  sentido: [-1, 1, 1],
  offset: [0, 0, 0],
};

// Cámara en el mundo del gemelo: mm, con X lateral, Y altura, Z hacia adelante.
// giro (en Y) 0 = mira hacia +Z, lejos del robot; inclinación + = hacia arriba.
export const CAMARA_DEFECTO = { x: 0, y: 0, z: 450, giro: 0, inclinacion: 15, fov: 60 };

const CLAVE = "gemelo.calibracion.v2";

export function cargarCalibracion() {
  try {
    const g = JSON.parse(localStorage.getItem(CLAVE));
    if (g && g.sentido?.length === 3 && g.offset?.length === 3) return g;
  } catch { /* sin almacenamiento: valores por defecto */ }
  return structuredClone(GEMELO_DEFECTO);
}

export function guardarCalibracion(g) {
  try { localStorage.setItem(CLAVE, JSON.stringify(g)); } catch { /* nada */ }
}

export const limitar = (e, v) => Math.min(FIRMWARE.limMax[e], Math.max(FIRMWARE.limMin[e], v));

const CLAVE_CAMARA = "gemelo.camara.v1";

export function cargarCamara() {
  try {
    const c = JSON.parse(localStorage.getItem(CLAVE_CAMARA));
    if (c && Number.isFinite(c.z)) return { ...CAMARA_DEFECTO, ...c };
  } catch { /* nada */ }
  return { ...CAMARA_DEFECTO };
}

export function guardarCamara(c) {
  try { localStorage.setItem(CLAVE_CAMARA, JSON.stringify(c)); } catch { /* nada */ }
}
