// preparar_glb.mjs - convierte el ensamble del CAD (pose HOME) en el modelo del gemelo.
//
//   node preparar_glb.mjs ../../Ensamble_manipulador_solo_HOME.glb ../modelos/manipulador.glb
//
// Dependencias (las mismas que Ítaca): @gltf-transform/core, /extensions y
// /functions 4.x y meshoptimizer. Las dos texturas de color que quedan (~200 KB)
// van tal cual: sharp no acepta su espacio de color.
//
// Fusion exporta ~100 piezas sueltas en la raíz, sin jerarquía. Aquí se
// reparten en cuerpos rígidos por nombre y se cuelgan de pivotes puestos en
// los ejes reales, para que en three.js cada articulación sea un solo
// rotation.x / rotation.y:
//
//   robot (origen = eje de la base a nivel del piso, Y arriba, metros)
//   ├─ fijo            sandwich, rodamiento inferior, polea 50T
//   └─ base            gira en Y
//      └─ eslabon1     gira en X, pivote en el eje del primer eslabón
//         └─ eslabon2  gira en X, pivote en la flecha de 8 mm
//            ├─ dedo1  cremallera + dedo (para cuando exista GRIP)
//            └─ dedo2
//
// En cada grupo las piezas se juntan en pocas mallas (join), se simplifican
// con error de 0.05% y se comprime con meshopt. En extras del nodo robot van
// los pivotes y el punto de agarre (TCP) en coordenadas de eslabon2.
import { NodeIO } from '@gltf-transform/core';
import { ALL_EXTENSIONS } from '@gltf-transform/extensions';
import { dedup, join, weld, simplify, prune, meshopt } from '@gltf-transform/functions';
import { getBounds } from '@gltf-transform/core';
import { MeshoptEncoder, MeshoptDecoder, MeshoptSimplifier } from 'meshoptimizer';
import fs from 'node:fs';

const [,, entrada, salida] = process.argv;
await MeshoptEncoder.ready; await MeshoptDecoder.ready; await MeshoptSimplifier.ready;
const io = new NodeIO().registerExtensions(ALL_EXTENSIONS)
  .registerDependencies({ 'meshopt.encoder': MeshoptEncoder, 'meshopt.decoder': MeshoptDecoder });
const doc = await io.read(entrada);
const root = doc.getRoot();
const escena = root.getDefaultScene() || root.listScenes()[0];
const piezas = escena.listChildren();

const nombre = n => n.getName().normalize('NFC');
const buscar = prefijo => {
  const n = piezas.find(p => nombre(p).startsWith(prefijo));
  if (!n) throw new Error(`No está la pieza ${prefijo}`);
  return n;
};
const centro = n => { const { min, max } = getBounds(n); return min.map((v, i) => (v + max[i]) / 2); };

// ---- Ejes, medidos sobre las piezas que los materializan ----
const [bx, , bz] = centro(buscar('rodamiento_150mm:1'));      // eje vertical de la base
const pisoY = getBounds(escena).min[1];
const [, e1y, e1z] = centro(buscar('eje_primer_eslabon'));     // eje del eslabón 1
const [, e2y, e2z] = centro(buscar('8mm_130mm'));              // eje del eslabón 2

// ---- Reparto por cuerpo rígido ----
const distE2 = n => { const [, y, z] = centro(n); return Math.hypot(y - e2y, z - e2z); };
function grupoDe(n) {
  const s = nombre(n);
  const y = centro(n)[1];
  if (/^(AA_sandwich_out|AA_sandwich_in|rodamiento_150mm:1|htd3m_50T_Base)/.test(s)) return 'fijo';
  if (s.startsWith('M6x8') && y < pisoY + 0.066) return 'fijo';   // tornillos de la parte baja del sandwich
  if (/^(AA_gripper:1|AA_cremallera_gripper:1)/.test(s)) return 'dedo1';
  if (/^(AA_gripper:2|AA_cremallera_gripper:2)/.test(s)) return 'dedo2';
  if (/^(eslabon_200mm|AA_20T_Eslabon2_3|8mm_130mm|AA_pi|AA_base_gripper|Soporte_Gripper|JGA25|Soporte_Carrera_3_complemento)/.test(s)) return 'eslabon2';
  if (/^(eslabon_300mm|Cinta_larga|IBERO 3D|MicroSwitchButton2:1|Soporte_Carrera_3:1)/.test(s)) return 'eslabon1';
  if (/^KFL08_chmacera:[4-7]$/.test(s)) return 'eslabon1';
  if (s.startsWith('M4x5') && distE2(n) < 0.05) return 'eslabon1';  // tornillos de las chumaceras del codo
  return 'base';
}

// El TCP: entre los dos dedos, a 3/4 del largo hacia la punta.
const d1 = getBounds(buscar('AA_gripper:1')), d2 = getBounds(buscar('AA_gripper:2'));
const tcpMundo = [
  (d1.min[0] + d1.max[0] + d2.min[0] + d2.max[0]) / 4,
  (d1.min[1] + d1.max[1] + d2.min[1] + d2.max[1]) / 4,
  Math.min(d1.min[2], d2.min[2]) + 0.75 * (Math.max(d1.max[2], d2.max[2]) - Math.min(d1.min[2], d2.min[2])),
];

// Posición de cada pivote en coordenadas del CAD
const pivote = {
  robot: [bx, pisoY, bz], fijo: [bx, pisoY, bz], base: [bx, pisoY, bz],
  eslabon1: [bx, e1y, e1z], eslabon2: [bx, e2y, e2z], dedo1: [bx, e2y, e2z], dedo2: [bx, e2y, e2z],
};
const padre = { fijo: 'robot', base: 'robot', eslabon1: 'base', eslabon2: 'eslabon1', dedo1: 'eslabon2', dedo2: 'eslabon2' };
const resta = (a, b) => a.map((v, i) => v - b[i]);

const grupos = {};
grupos.robot = doc.createNode('robot');
for (const g of Object.keys(padre)) {
  grupos[g] = doc.createNode(g).setTranslation(resta(pivote[g], pivote[padre[g]]));
}
for (const g of Object.keys(padre)) grupos[padre[g]].addChild(grupos[g]);

// Aplana cada pieza dentro de su grupo: un nodo por malla, con la matriz de
// mundo del CAD expresada respecto al pivote del grupo.
const conteo = {};
for (const pieza of piezas) {
  const g = grupoDe(pieza);
  conteo[g] = (conteo[g] || 0) + 1;
  pieza.traverse(n => {
    const malla = n.getMesh();
    if (!malla) return;
    const m = n.getWorldMatrix().slice();
    m[12] -= pivote[g][0]; m[13] -= pivote[g][1]; m[14] -= pivote[g][2];
    grupos[g].addChild(doc.createNode(nombre(pieza)).setMesh(malla).setMatrix(m));
  });
}
for (const p of piezas) { escena.removeChild(p); p.traverse(n => n.dispose()); }
escena.addChild(grupos.robot);

grupos.robot.setExtras({
  unidades: 'm',
  ejes: {
    base: { eje: [0, 1, 0] },
    eslabon1: { eje: [1, 0, 0], pivote: resta(pivote.eslabon1, pivote.robot) },
    eslabon2: { eje: [1, 0, 0], pivote: resta(pivote.eslabon2, pivote.robot) },
  },
  tcp: resta(tcpMundo, pivote.eslabon2),   // en coordenadas de eslabon2
  pose: 'HOME',
});

const resumen = d => {
  let tris = 0, prims = 0;
  d.getRoot().listMeshes().forEach(m => m.listPrimitives().forEach(p => {
    prims++; const i = p.getIndices(); tris += (i ? i.getCount() : p.getAttribute('POSITION').getCount()) / 3;
  }));
  return { primitivas: prims, materiales: d.getRoot().listMaterials().length, tris: Math.round(tris) };
};
// Los normal maps de Fusion (texturas de plástico, PNG de 16 bits) no se
// notan a esta escala, pesan 2.5 MB y sharp no los sabe convertir.
root.listMaterials().forEach(m => m.setNormalTexture(null));

console.log('piezas por grupo', conteo);
console.log('antes  ', resumen(doc));

await doc.transform(
  dedup(),

  join({ keepNamed: false }),
  weld(),
  simplify({ simplifier: MeshoptSimplifier, ratio: 0, error: 0.0005 }),
  prune(),
  meshopt({ encoder: MeshoptEncoder, level: 'medium' }),
);
console.log('despues', resumen(doc));
await io.write(salida, doc);
console.log('bytes', fs.statSync(entrada).size, '->', fs.statSync(salida).size);
console.log('extras', JSON.stringify(grupos.robot.getExtras()));
