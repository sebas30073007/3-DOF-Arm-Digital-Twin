// cinematica.js - directa e inversa del manipulador de 3 GDL.
//
// Las medidas salen del GLB (extras del nodo "robot", ver tools/preparar_glb.mjs),
// en metros, con Y arriba y el gripper hacia +Z en la pose HOME:
//
//   p1   eje del eslabón 1 respecto al eje de la base (a nivel del piso)
//   p2   eje del eslabón 2 respecto al eje del eslabón 1
//   tcp  punto de agarre respecto al eje del eslabón 2
//
// La base gira en Y y los dos eslabones en X. E2 es relativo al eslabón 1: el
// firmware compensa el acople 1:1 de la cinta, así que un cambio de E1 no
// mueve el ángulo E2 reportado.
//
// Para la inversa los eslabones se tratan como números complejos c = z + i·y,
// donde girar en X un ángulo a es multiplicar por e^(-ia).

import { FIRMWARE, HOME, limitar } from "./config.js";

const RAD = Math.PI / 180;

export class Cinematica {
  constructor(extras, calibracion) {
    this.p1 = extras.ejes.eslabon1.pivote;
    const p2abs = extras.ejes.eslabon2.pivote;
    this.p2 = [p2abs[0] - this.p1[0], p2abs[1] - this.p1[1], p2abs[2] - this.p1[2]];
    this.tcp = extras.tcp;
    this.cal = calibracion;
    this.l1 = Math.hypot(this.p2[1], this.p2[2]);
    this.l2 = Math.hypot(this.tcp[1], this.tcp[2]);
    this.alfa1 = Math.atan2(this.p2[1], this.p2[2]);
    this.alfa2 = Math.atan2(this.tcp[1], this.tcp[2]);
  }

  // Ángulo del firmware (grados) -> rotación del modelo (radianes)
  aModelo(e, q) {
    const { sentido, offset } = this.cal;
    return sentido[e] * (q - HOME[e] + offset[e]) * RAD;
  }

  aFirmware(e, rad) {
    const { sentido, offset } = this.cal;
    return rad / RAD / sentido[e] + HOME[e] - offset[e];
  }

  // Posición del TCP en el marco del robot, para q = [B, E1, E2] en grados
  directa(q) {
    const t = this.aModelo(0, q[0]), a = this.aModelo(1, q[1]), b = this.aModelo(2, q[2]);
    const [tx, ty, tz] = this.tcp;
    // tcp girado por el codo, más el eslabón 1
    let y = this.p2[1] + ty * Math.cos(b) - tz * Math.sin(b);
    let z = this.p2[2] + ty * Math.sin(b) + tz * Math.cos(b);
    const x = this.p2[0] + tx + this.p1[0];
    // girado por el hombro
    const y1 = y * Math.cos(a) - z * Math.sin(a);
    const z1 = y * Math.sin(a) + z * Math.cos(a);
    y = y1 + this.p1[1];
    z = z1 + this.p1[2];
    // girado por la base
    return [x * Math.cos(t) + z * Math.sin(t), y, -x * Math.sin(t) + z * Math.cos(t)];
  }

  dentro(q, tol = 1e-6) {
    return q.every((v, e) => v >= FIRMWARE.limMin[e] - tol && v <= FIRMWARE.limMax[e] + tol);
  }

  // Todas las soluciones exactas (hasta 4: base de frente o de espaldas, codo arriba o abajo)
  soluciones(p) {
    const [X, Y, Z] = p;
    const d = this.tcp[0] + this.p1[0] + this.p2[0];     // desfase lateral del plano del brazo
    const rho = Math.hypot(X, Z);
    if (rho < Math.abs(d) + 1e-6) return [];
    const phi = Math.atan2(X, Z);
    const delta = Math.asin(d / rho);
    const sols = [];
    for (const [theta, zb] of [[phi - delta, rho * Math.cos(delta)], [phi - Math.PI + delta, -rho * Math.cos(delta)]]) {
      const cz = zb - this.p1[2], cy = Y - this.p1[1];
      const D2 = cz * cz + cy * cy;
      const k = (D2 - this.l1 ** 2 - this.l2 ** 2) / (2 * this.l1 * this.l2);
      if (k < -1 || k > 1) continue;
      for (const s of [1, -1]) {
        const gamma = s * Math.acos(k);
        const psi2 = gamma - this.alfa2 + this.alfa1;
        const psi = Math.atan2(cy, cz) - this.alfa1 - Math.atan2(this.l2 * Math.sin(gamma), this.l1 + this.l2 * Math.cos(gamma));
        const q = [this.aFirmware(0, theta), this.aFirmware(1, -psi), this.aFirmware(2, -psi2)];
        // cada ángulo al representante (±360) que caiga en su rango, si hay
        for (let e = 0; e < 3; e++) q[e] = this.envolver(e, q[e]);
        sols.push(q);
      }
    }
    return sols;
  }

  // Ángulo de base (firmware, dentro de límites) que deja el punto en el plano del brazo, de frente
  baseHacia(p) {
    const d = this.tcp[0] + this.p1[0] + this.p2[0];
    const rho = Math.hypot(p[0], p[2]);
    const theta = Math.atan2(p[0], p[2]) - (rho > Math.abs(d) ? Math.asin(d / rho) : 0);
    return limitar(0, this.envolver(0, this.aFirmware(0, theta)));
  }

  envolver(e, v) {
    const mid = (FIRMWARE.limMin[e] + FIRMWARE.limMax[e]) / 2;
    return v - 360 * Math.round((v - mid) / 360);
  }

  // Inversa: la solución dentro de límites más cercana a `cerca`. Si el punto
  // no se alcanza, la pose dentro de límites que más se le acerca.
  inversa(p, cerca) {
    const dist = q => q.reduce((s, v, e) => s + (e === 0 ? 0.5 : 1) * (v - cerca[e]) ** 2, 0);
    const validas = this.soluciones(p).filter(q => this.dentro(q)).sort((a, b) => dist(a) - dist(b));
    if (validas.length) return { q: validas[0], alcanzable: true, error: 0 };

    // Mínimos cuadrados con límites (Levenberg-Marquardt, jacobiano numérico)
    const semillas = [cerca.map((v, e) => limitar(e, v)), ...this.soluciones(p).map(q => q.map((v, e) => limitar(e, v)))];
    let mejor = null;
    for (const s of semillas) {
      const r = this.ajustar(p, s);
      if (!mejor || r.error < mejor.error - 1e-5 || (Math.abs(r.error - mejor.error) < 1e-5 && dist(r.q) < dist(mejor.q))) mejor = r;
    }
    return { q: mejor.q, alcanzable: mejor.error < 0.002, error: mejor.error };
  }

  ajustar(p, q0) {
    let q = q0.slice(), lambda = 1e-3;
    const residuo = q => { const f = this.directa(q); return [f[0] - p[0], f[1] - p[1], f[2] - p[2]]; };
    let r = residuo(q), err = Math.hypot(...r);
    for (let it = 0; it < 60 && err > 1e-5; it++) {
      const J = [0, 1, 2].map(e => {
        const qh = q.slice(); qh[e] += 0.01;
        const rh = residuo(qh);
        return rh.map((v, i) => (v - r[i]) / 0.01);
      });                                      // J[e][i] = d r_i / d q_e
      // (JᵀJ + λ·diag) Δ = -Jᵀ r
      const A = [0, 1, 2].map(i => [0, 1, 2].map(j => J[i].reduce((s, v, k) => s + v * J[j][k], 0)));
      const g = [0, 1, 2].map(i => J[i].reduce((s, v, k) => s + v * r[k], 0));
      for (let i = 0; i < 3; i++) A[i][i] += lambda * (A[i][i] + 1e-9);
      const delta = resolver3(A, g.map(v => -v));
      if (!delta) break;
      const qn = q.map((v, e) => limitar(e, v + delta[e]));
      const rn = residuo(qn), en = Math.hypot(...rn);
      if (en < err) { q = qn; r = rn; err = en; lambda = Math.max(lambda / 3, 1e-7); }
      else { lambda *= 4; if (lambda > 1e6) break; }
    }
    return { q, error: err };
  }
}

function resolver3(A, b) {
  const [[a, b1, c], [d, e, f], [g, h, i]] = A;
  const det = a * (e * i - f * h) - b1 * (d * i - f * g) + c * (d * h - e * g);
  if (!Number.isFinite(det) || Math.abs(det) < 1e-30) return null;   // JᵀJ en (m/°)² es chico: ~1e-14
  const col = (k) => {
    const M = A.map((fila, r) => fila.map((v, s) => (s === k ? b[r] : v)));
    const [[a2, b2, c2], [d2, e2, f2], [g2, h2, i2]] = M;
    return (a2 * (e2 * i2 - f2 * h2) - b2 * (d2 * i2 - f2 * g2) + c2 * (d2 * h2 - e2 * g2)) / det;
  };
  return [col(0), col(1), col(2)];
}
