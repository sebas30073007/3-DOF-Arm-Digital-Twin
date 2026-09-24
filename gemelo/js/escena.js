// escena.js - el gemelo en three.js.
//
// Dos copias del mismo GLB:
//   - el robot sólido, que es el real (o el simulado): lo mueve el STATE;
//   - el fantasma translúcido, que es la referencia: lo mueve quien controla
//     (sliders, arrastre de piezas o el punto). El sólido lo va alcanzando con
//     las rampas del firmware. Si coinciden, el fantasma se esconde.
//
// Interacción:
//   Articulaciones  pasar el cursor sobre una pieza muestra el arco de su eje
//                   (el rango completo y flechas hacia donde todavía puede
//                   girar); arrastrarla gira la referencia.
//   Punto           una esfera en el punto de agarre con flechas XYZ; la
//                   cinemática inversa pone la referencia.
// Clic izquierdo en el vacío orbita, derecho o central desplaza, rueda acerca.
//
// El fantasma solo dibuja lo que cambia: si la primera articulación distinta
// es E2, solo aparece el eslabón 2; si es E1, eslabones 1 y 2; si es la base,
// todo el brazo. Así no se encima con el sólido donde son iguales.
//
// La cámara (si está abierta) es un cuerpo con su pirámide de visión y el
// video en un cuadro a 35 cm; la mano detectada se marca en el mundo.
//
// Todo en metros, Y arriba, el gripper mira a +Z en HOME.

import * as THREE from "three";
import { OrbitControls } from "three/addons/controls/OrbitControls.js";
import { TransformControls } from "three/addons/controls/TransformControls.js";
import { GLTFLoader } from "three/addons/loaders/GLTFLoader.js";
import { MeshoptDecoder } from "three/addons/libs/meshopt_decoder.module.js";
import { RoomEnvironment } from "three/addons/environments/RoomEnvironment.js";
import { FIRMWARE, limitar } from "./config.js";

const COLOR = {
  fondo: 0xf2f1ed,
  grafito: 0x111317,
  alloy: 0x6b6f76,
  senal: 0xe30613,
  fantasma: 0x9aa0a8,
  reticula: 0xdcd9d1,
  reticulaFina: 0xe6e3dc,
};
const GRUPOS = ["fijo", "base", "eslabon1", "eslabon2", "dedo1", "dedo2"];
const EJE_DE_GRUPO = { base: 0, eslabon1: 1, eslabon2: 2, dedo1: 2, dedo2: 2 };
const VISTA_INICIAL = { pos: new THREE.Vector3(0.95, 0.7, 1.05), mira: new THREE.Vector3(0, 0.33, 0.03) };
const UMBRAL_FANTASMA = 0.4;     // grados: por debajo, referencia y real coinciden

export class Escena {
  constructor(contenedor, { alCambiarRef, alHover, alPunto }) {
    this.contenedor = contenedor;
    this.alCambiarRef = alCambiarRef;   // (q, {final}) al arrastrar
    this.alHover = alHover;             // (info | null) para la etiqueta
    this.alPunto = alPunto;             // (p) al mover la esfera en modo punto
    this.modo = "articulaciones";
    this.ref = null;
    this.real = null;
    this.hover = -1;
    this.arrastre = null;
    this.listo = false;

    const r = (this.renderer = new THREE.WebGLRenderer({ antialias: true }));
    r.setPixelRatio(Math.min(window.devicePixelRatio, 2));
    r.outputColorSpace = THREE.SRGBColorSpace;
    r.toneMapping = THREE.NeutralToneMapping;
    r.shadowMap.enabled = true;
    r.shadowMap.type = THREE.PCFSoftShadowMap;
    contenedor.appendChild(r.domElement);

    const s = (this.scene = new THREE.Scene());
    s.background = new THREE.Color(COLOR.fondo);
    const pmrem = new THREE.PMREMGenerator(r);
    s.environment = pmrem.fromScene(new RoomEnvironment(), 0.04).texture;
    s.environmentIntensity = 0.55;

    this.camera = new THREE.PerspectiveCamera(35, 1, 0.01, 50);
    this.camera.position.copy(VISTA_INICIAL.pos);

    // Nuestro pointerdown va antes que el de OrbitControls: si cae sobre una
    // pieza, se apagan los controles antes de que empiecen a orbitar.
    r.domElement.addEventListener("pointerdown", e => this.bajar(e));
    r.domElement.addEventListener("pointermove", e => this.mover(e));
    window.addEventListener("pointerup", e => this.soltar(e));
    r.domElement.addEventListener("pointerleave", () => { if (!this.arrastre) this.ponerHover(-1); });

    const c = (this.controles = new OrbitControls(this.camera, r.domElement));
    c.target.copy(VISTA_INICIAL.mira);
    c.enableDamping = true;
    c.dampingFactor = 0.12;
    c.minDistance = 0.35;
    c.maxDistance = 5;
    c.maxPolarAngle = Math.PI * 0.495;    // no pasar bajo el piso
    c.mouseButtons = { LEFT: THREE.MOUSE.ROTATE, MIDDLE: THREE.MOUSE.PAN, RIGHT: THREE.MOUSE.PAN };
    c.update();

    this.luces();
    this.piso();

    this.raycaster = new THREE.Raycaster();
    this.puntero = new THREE.Vector2();

    new ResizeObserver(() => this.ajustar()).observe(contenedor);
    this.ajustar();
    this.renderer.setAnimationLoop(() => this.cuadro());
  }

  luces() {
    const hemi = new THREE.HemisphereLight(0xffffff, 0xcbc6ba, 1.1);
    this.scene.add(hemi);
    const sol = (this.sol = new THREE.DirectionalLight(0xffffff, 2.2));
    sol.position.set(1.0, 2.2, 1.4);
    sol.castShadow = true;
    sol.shadow.mapSize.set(2048, 2048);
    Object.assign(sol.shadow.camera, { left: -0.9, right: 0.9, top: 0.9, bottom: -0.9, near: 0.5, far: 5 });
    sol.shadow.bias = -0.0004;
    sol.shadow.normalBias = 0.01;
    sol.shadow.radius = 4;
    this.scene.add(sol);
    const relleno = new THREE.DirectionalLight(0xffffff, 0.6);
    relleno.position.set(-1.5, 1, -1);
    this.scene.add(relleno);
  }

  piso() {
    const sombra = new THREE.Mesh(
      new THREE.CircleGeometry(4, 64).rotateX(-Math.PI / 2),
      new THREE.ShadowMaterial({ color: 0x3a342a, opacity: 0.16 }),
    );
    sombra.receiveShadow = true;
    this.scene.add(sombra);
    // Retícula de 10 cm con líneas de 1 m, que se desvanece hacia afuera
    const fina = new THREE.GridHelper(4, 40, COLOR.reticulaFina, COLOR.reticulaFina);
    const gruesa = new THREE.GridHelper(4, 4, COLOR.reticula, COLOR.reticula);
    for (const g of [fina, gruesa]) {
      g.position.y = 0.0005;
      g.material.transparent = true;
      g.material.depthWrite = false;
      g.material.onBeforeCompile = sh => {
        sh.vertexShader = sh.vertexShader.replace("#include <common>", "#include <common>\nvarying vec3 vMundo;")
          .replace("#include <fog_vertex>", "#include <fog_vertex>\nvMundo = (modelMatrix * vec4(position, 1.0)).xyz;");
        sh.fragmentShader = sh.fragmentShader.replace("#include <common>", "#include <common>\nvarying vec3 vMundo;")
          .replace("#include <opaque_fragment>", "#include <opaque_fragment>\ngl_FragColor.a *= 1.0 - smoothstep(0.6, 1.9, length(vMundo.xz));");
      };
      this.scene.add(g);
    }
  }

  ajustar() {
    const { clientWidth: w, clientHeight: h } = this.contenedor;
    if (!w || !h) return;
    this.renderer.setSize(w, h);
    this.camera.aspect = w / h;
    this.camera.updateProjectionMatrix();
  }

  // Con la cámara visible se aleja y se va de lado para que quepan el robot,
  // la cámara y su cuadro de video.
  encuadrar() {
    const conCam = this.cam?.rig.visible;
    if (conCam) {
      const c = this.cam.rig.position, mira = new THREE.Vector3(c.x / 2, 0.3, c.z / 2 + 0.1);
      this.controles.target.copy(mira);
      this.camera.position.copy(mira).add(new THREE.Vector3(1.9, 0.95, 0.35));
    } else {
      this.camera.position.copy(VISTA_INICIAL.pos);
      this.controles.target.copy(VISTA_INICIAL.mira);
    }
    this.controles.update();
  }

  // ------------------------------------------------------------- modelo --

  async cargar(url, cinematica) {
    const loader = new GLTFLoader().setMeshoptDecoder(MeshoptDecoder);
    const gltf = await loader.loadAsync(url);
    const raiz = gltf.scene.getObjectByName("robot");
    this.extras = raiz.userData;
    this.cin = cinematica(this.extras);

    this.solido = this.armar(raiz, false);
    this.fantasma = this.armar(raiz.clone(true), true);
    this.scene.add(this.solido.raiz, this.fantasma.raiz);

    this.arcos = [0, 1, 2].map(e => this.crearArco(e));
    this.crearPunto();
    this.crearCamara();
    this.listo = true;
    return this.extras;
  }

  // Encuentra los grupos, deja materiales propios por grupo (para resaltar uno
  // sin pintar los demás) y, en el fantasma, cambia todo por el material translúcido.
  armar(raiz, esFantasma) {
    const g = { raiz };
    for (const n of GRUPOS) g[n] = raiz.getObjectByName(n);
    const mallas = [];
    g.porGrupo = {};
    for (const n of GRUPOS) {
      const cache = new Map();
      const propias = [];
      g[n].traverse(o => { if (o.isMesh && grupoDe(o) === n) propias.push(o); });
      for (const o of propias) {
        o.userData.eje = EJE_DE_GRUPO[n] ?? -1;
        o.userData.fantasma = esFantasma;
        if (esFantasma) {
          o.material = MAT_FANTASMA;
          o.renderOrder = 2;
          o.castShadow = false;
          const profundidad = new THREE.Mesh(o.geometry, MAT_PROFUNDIDAD);
          profundidad.renderOrder = 1;
          profundidad.raycast = () => {};
          o.add(profundidad);
        } else {
          if (!cache.has(o.material)) {
            const m = o.material.clone();
            m.userData.emisivoBase = m.emissive?.clone();
            cache.set(o.material, m);
          }
          o.material = cache.get(o.material);
          o.castShadow = true;
          o.receiveShadow = true;
        }
        mallas.push(o);
      }
      g.porGrupo[n] = propias;
      g["mat_" + n] = [...cache.values()];
    }
    g.mallas = mallas;
    if (esFantasma) raiz.visible = false;
    return g;
  }

  poner(robot, q) {
    robot.base.rotation.y = this.cin.aModelo(0, q[0]);
    robot.eslabon1.rotation.x = this.cin.aModelo(1, q[1]);
    robot.eslabon2.rotation.x = this.cin.aModelo(2, q[2]);
  }

  setReal(q) {
    this.real = q.slice();
    if (!this.ref) this.setRef(q);
  }

  setRef(q) {
    this.ref = q.slice();
    if (this.punto && !this.punto.arrastrando && this.modo === "punto" && !this.punto.fuera) {
      this.punto.esfera.position.fromArray(this.cin.directa(this.ref));
    }
  }

  // ----------------------------------------------------------- arcos --

  // Tras cambiar el sentido u offset del gemelo, los rangos cambian de lugar
  reconstruirArcos() {
    for (const a of this.arcos) {
      a.grupo.removeFromParent();
      a.grupo.traverse(o => { o.geometry?.dispose(); o.material?.dispose(); });
    }
    this.arcos = [0, 1, 2].map(e => this.crearArco(e));
  }

  // El arco vive en el marco del padre de la articulación en el FANTASMA:
  // sigue a la referencia. Su plano es perpendicular al eje; el ángulo 0 del
  // arco es la dirección del eslabón con rotación 0 del modelo.
  crearArco(e) {
    const f = this.fantasma;
    const padre = [f.raiz, f.base, f.eslabon1][e];
    const pivote = e === 0 ? new THREE.Vector3(0, 0.003, 0) : [null, f.eslabon1, f.eslabon2][e].position.clone();
    const eje = e === 0 ? new THREE.Vector3(0, 1, 0) : new THREE.Vector3(1, 0, 0);
    const p2 = f.eslabon2.position, tcp = this.extras.tcp;
    const r0 = [new THREE.Vector3(0, 0, 1), new THREE.Vector3(0, p2.y, p2.z), new THREE.Vector3(0, tcp[1], tcp[2])][e].normalize();
    const w = new THREE.Vector3().crossVectors(eje, r0);
    // Cuelga de la escena y no del fantasma (que se esconde cuando coincide
    // con el real); cada cuadro copia la matriz del padre en el fantasma.
    const local = new THREE.Matrix4().makeBasis(r0, w, eje).setPosition(pivote);
    const grupo = new THREE.Group();
    grupo.matrixAutoUpdate = false;
    grupo.visible = false;
    this.scene.add(grupo);

    const radio = [0.2, 0.13, 0.11][e];
    const a0 = this.cin.aModelo(e, FIRMWARE.limMin[e]), a1 = this.cin.aModelo(e, FIRMWARE.limMax[e]);
    const rango = new THREE.Mesh(
      new THREE.RingGeometry(radio - 0.005, radio + 0.005, 128, 1, Math.min(a0, a1), Math.abs(a1 - a0)),
      nuevoMatGuia(COLOR.alloy, 0.55),
    );
    // marcas en los límites
    const marcas = [a0, a1].map(a => {
      const m = new THREE.Mesh(new THREE.PlaneGeometry(0.004, 0.03), nuevoMatGuia(COLOR.grafito, 0.8));
      m.position.set(Math.cos(a) * radio, Math.sin(a) * radio, 0);
      m.rotation.z = a - Math.PI / 2;
      return m;
    });
    const aguja = new THREE.Mesh(new THREE.PlaneGeometry(radio, 0.004).translate(radio / 2, 0, 0), nuevoMatGuia(COLOR.grafito, 0.85));
    const conoGeo = new THREE.ConeGeometry(0.013, 0.03, 20);
    const flechas = [1, -1].map(signo => {
      const m = new THREE.Mesh(conoGeo, nuevoMatGuia(COLOR.grafito, 0.9));
      m.userData.signo = signo;
      return m;
    });
    const real = new THREE.Mesh(new THREE.CircleGeometry(0.008, 24), nuevoMatGuia(COLOR.senal, 0.95));
    grupo.add(rango, ...marcas, aguja, ...flechas, real);
    for (const o of grupo.children) o.renderOrder = 10;
    return { e, grupo, padre, local, radio, aguja, flechas, real };
  }

  colocarArco(arco) {
    arco.padre.updateWorldMatrix(true, false);
    arco.grupo.matrix.multiplyMatrices(arco.padre.matrixWorld, arco.local);
    arco.grupo.matrixWorldNeedsUpdate = true;
  }

  actualizarArco(arco, activo) {
    const { e, radio } = arco;
    this.colocarArco(arco);
    const aRef = this.cin.aModelo(e, this.ref[e]);
    arco.aguja.rotation.z = aRef;
    // Flechas a ±18° de la referencia, hacia donde el ángulo del firmware
    // sube (+) o baja (-). Se esconden si ya no hay recorrido de ese lado.
    const sentidoModelo = Math.sign(this.cin.cal.sentido[e]);
    for (const f of arco.flechas) {
      const s = f.userData.signo;
      const hayRecorrido = s > 0 ? this.ref[e] < FIRMWARE.limMax[e] - 0.3 : this.ref[e] > FIRMWARE.limMin[e] + 0.3;
      f.visible = hayRecorrido;
      const a = aRef + s * sentidoModelo * 0.32;
      f.position.set(Math.cos(a) * radio, Math.sin(a) * radio, 0);
      // el cono apunta a +Y; lo giro hacia la tangente en el sentido de avance
      f.rotation.z = a + (s * sentidoModelo > 0 ? 0 : Math.PI);
      f.material.color.setHex(activo ? COLOR.senal : COLOR.grafito);
    }
    const aReal = this.cin.aModelo(e, this.real[e]);
    // El punto del real se dibuja en el marco del fantasma: para E1 y E2
    // es aproximado si el padre también se está moviendo, pero sirve de guía.
    arco.real.position.set(Math.cos(aReal) * radio, Math.sin(aReal) * radio, 0.001);
    arco.real.visible = Math.abs(this.real[e] - this.ref[e]) > UMBRAL_FANTASMA;
  }

  // ------------------------------------------------------------ punto --

  crearPunto() {
    const esfera = new THREE.Mesh(new THREE.SphereGeometry(0.011, 32, 16), new THREE.MeshStandardMaterial({ color: COLOR.senal, roughness: 0.4 }));
    esfera.visible = false;
    esfera.renderOrder = 11;
    this.scene.add(esfera);
    // Línea al piso y huella: sin ellas no se sabe a qué altura está la esfera
    const lineaGeo = new THREE.BufferGeometry().setFromPoints([new THREE.Vector3(), new THREE.Vector3()]);
    const linea = new THREE.Line(lineaGeo, new THREE.LineDashedMaterial({ color: COLOR.alloy, dashSize: 0.01, gapSize: 0.008, transparent: true, opacity: 0.8 }));
    const huella = new THREE.Mesh(new THREE.RingGeometry(0.008, 0.013, 32).rotateX(-Math.PI / 2), nuevoMatGuia(COLOR.alloy, 0.8));
    // Del TCP de la referencia a la esfera, cuando no la alcanza
    const faltaGeo = new THREE.BufferGeometry().setFromPoints([new THREE.Vector3(), new THREE.Vector3()]);
    const falta = new THREE.Line(faltaGeo, new THREE.LineDashedMaterial({ color: COLOR.senal, dashSize: 0.012, gapSize: 0.008 }));
    for (const o of [linea, huella, falta]) { o.visible = false; this.scene.add(o); }

    const tc = new TransformControls(this.camera, this.renderer.domElement);
    tc.setSize(0.75);
    tc.setSpace("world");
    tc.attach(esfera);
    tc.enabled = false;
    const ayuda = tc.getHelper();
    ayuda.visible = false;
    this.scene.add(ayuda);
    tc.addEventListener("dragging-changed", ev => {
      this.controles.enabled = !ev.value;
      this.punto.arrastrando = ev.value;
      if (!ev.value) this.alPunto?.(esfera.position.toArray(), { final: true });
    });
    tc.addEventListener("objectChange", () => {
      if (esfera.position.y < 0.005) esfera.position.y = 0.005;
      this.alPunto?.(esfera.position.toArray(), { final: false });
    });
    this.punto = { esfera, linea, huella, falta, tc, ayuda, arrastrando: false, fuera: false };
  }

  // Lo llama la página tras resolver la inversa: si no alcanza, la esfera
  // se queda donde la dejó el usuario y una línea marca lo que falta.
  marcarPunto(p, alcanzable) {
    this.punto.fuera = !alcanzable;
    if (p) this.punto.esfera.position.fromArray(p);
  }

  moverPunto(p) {
    this.punto.esfera.position.fromArray(p);
    this.punto.fuera = false;
  }

  setModo(modo) {
    this.modo = modo;
    const enPunto = modo === "punto";
    const pt = this.punto;
    pt.tc.enabled = enPunto;
    pt.ayuda.visible = enPunto;
    for (const o of [pt.esfera, pt.linea, pt.huella]) o.visible = enPunto;
    if (enPunto && this.ref) { pt.esfera.position.fromArray(this.cin.directa(this.ref)); pt.fuera = false; }
    this.ponerHover(-1);
  }

  // ------------------------------------------------------------ cámara --

  crearCamara() {
    const rig = new THREE.Group();            // mira hacia su -Z, como una cámara de three
    rig.rotation.order = "YXZ";
    const grafito = new THREE.MeshStandardMaterial({ color: COLOR.grafito, roughness: 0.5 });
    const cuerpo = new THREE.Mesh(new THREE.BoxGeometry(0.07, 0.03, 0.028), grafito);
    const lente = new THREE.Mesh(new THREE.CylinderGeometry(0.009, 0.01, 0.012, 24).rotateX(Math.PI / 2), grafito);
    lente.position.z = -0.018;
    cuerpo.castShadow = true;
    const lineas = new THREE.LineSegments(new THREE.BufferGeometry(), new THREE.LineBasicMaterial({ color: COLOR.alloy, transparent: true, opacity: 0.8 }));
    const plano = new THREE.Mesh(new THREE.PlaneGeometry(1, 1), new THREE.MeshBasicMaterial({
      color: 0xffffff, side: THREE.DoubleSide, transparent: true, opacity: 0.94, toneMapped: false,
    }));
    plano.visible = false;
    rig.add(cuerpo, lente, lineas, plano);
    rig.visible = false;
    this.scene.add(rig);

    const mano = new THREE.Mesh(new THREE.SphereGeometry(0.018, 24, 12), new THREE.MeshStandardMaterial({ color: COLOR.alloy, roughness: 0.5 }));
    const rayoGeo = new THREE.BufferGeometry().setFromPoints([new THREE.Vector3(), new THREE.Vector3()]);
    const rayo = new THREE.Line(rayoGeo, new THREE.LineDashedMaterial({ color: COLOR.alloy, dashSize: 0.015, gapSize: 0.01 }));
    // Objetivo del seguimiento: a dónde va el punto de agarre (la mano mueve
    // al objetivo en relativo, así que no coincide con la mano)
    const objetivo = new THREE.Mesh(new THREE.SphereGeometry(0.012, 24, 12), new THREE.MeshStandardMaterial({ color: COLOR.senal, roughness: 0.4 }));
    const ligaGeo = new THREE.BufferGeometry().setFromPoints([new THREE.Vector3(), new THREE.Vector3()]);
    const liga = new THREE.Line(ligaGeo, new THREE.LineDashedMaterial({ color: COLOR.senal, dashSize: 0.012, gapSize: 0.01, transparent: true, opacity: 0.6 }));
    mano.visible = rayo.visible = objetivo.visible = liga.visible = false;
    this.scene.add(mano, rayo, objetivo, liga);
    this.cam = { rig, lineas, plano, textura: null, mano, rayo, objetivo, liga, pose: null, aspecto: 16 / 9 };
  }

  // pose en mm y grados (ver CAMARA_DEFECTO en config.js)
  ponerCamara(pose, aspecto = this.cam.aspecto) {
    const c = this.cam;
    c.pose = pose;
    c.aspecto = aspecto;
    c.rig.position.set(pose.x / 1000, pose.y / 1000, pose.z / 1000);
    c.rig.rotation.set(pose.inclinacion * Math.PI / 180, pose.giro * Math.PI / 180 + Math.PI, 0);
    const D = 0.35, hw = D * Math.tan(pose.fov * Math.PI / 360), hh = hw / aspecto;
    const esq = [[-hw, -hh], [hw, -hh], [hw, hh], [-hw, hh]].map(([x, y]) => new THREE.Vector3(x, y, -D));
    const pts = [];
    for (let i = 0; i < 4; i++) pts.push(new THREE.Vector3(), esq[i], esq[i], esq[(i + 1) % 4]);
    c.lineas.geometry.setFromPoints(pts);
    c.plano.position.set(0, 0, -D);
    c.plano.scale.set(2 * hw, 2 * hh, 1);
  }

  mostrarCamara(si, lienzo) {
    const c = this.cam;
    c.rig.visible = si;
    if (lienzo && c.plano.material.map?.image !== lienzo) {
      c.textura?.dispose();
      c.textura = new THREE.CanvasTexture(lienzo);
      c.textura.colorSpace = THREE.SRGBColorSpace;
      c.plano.material.map = c.textura;
      c.plano.material.needsUpdate = true;
    }
    c.plano.visible = si && !!lienzo;
    if (!si) { this.marcarMano(null); this.marcarObjetivo(null); }
  }

  // Punto del mundo que está a `dist` m sobre el eje óptico, en el píxel (u, v) ∈ [0, 1]
  puntoDesdeImagen(u, v, dist) {
    const c = this.cam;
    const t = Math.tan(c.pose.fov * Math.PI / 360);
    const local = new THREE.Vector3((u - 0.5) * 2 * t * dist, -(v - 0.5) * 2 * (t / c.aspecto) * dist, -dist);
    c.rig.updateMatrixWorld(true);
    return local.applyMatrix4(c.rig.matrixWorld).toArray();
  }

  focal(anchoPx) { return (anchoPx / 2) / Math.tan(this.cam.pose.fov * Math.PI / 360); }

  marcarMano(p, activa = true) {
    const { mano, rayo, rig } = this.cam;
    mano.visible = rayo.visible = !!p;
    if (!p) return;
    mano.position.fromArray(p);
    mano.material.color.setHex(activa ? COLOR.grafito : COLOR.alloy);
    const a = rayo.geometry.attributes.position;
    a.setXYZ(0, rig.position.x, rig.position.y, rig.position.z);
    a.setXYZ(1, p[0], p[1], p[2]);
    a.needsUpdate = true;
    rayo.computeLineDistances();
  }

  marcarObjetivo(p) {
    const { objetivo, liga, mano } = this.cam;
    objetivo.visible = liga.visible = !!p;
    if (!p) return;
    objetivo.position.fromArray(p);
    const a = liga.geometry.attributes.position;
    a.setXYZ(0, mano.position.x, mano.position.y, mano.position.z);
    a.setXYZ(1, p[0], p[1], p[2]);
    a.needsUpdate = true;
    liga.computeLineDistances();
    liga.visible = mano.visible;
  }

  // ----------------------------------------------------------- puntero --

  rayo(e) {
    const r = this.renderer.domElement.getBoundingClientRect();
    this.puntero.set(((e.clientX - r.left) / r.width) * 2 - 1, -((e.clientY - r.top) / r.height) * 2 + 1);
    this.raycaster.setFromCamera(this.puntero, this.camera);
    return this.raycaster.ray;
  }

  pieza(e) {
    this.rayo(e);
    const objetos = [...this.solido.mallas, ...(this.fantasma.raiz.visible ? this.fantasma.mallas.filter(m => m.visible) : [])];
    const hit = this.raycaster.intersectObjects(objetos, false)[0];
    return hit ? hit.object.userData.eje : -1;
  }

  ponerHover(eje) {
    if (eje === this.hover) return;
    this.hover = eje;
    // Resalte suave: sube la emisión de las piezas del grupo, sin color
    for (const n of GRUPOS) {
      const on = eje >= 0 && EJE_DE_GRUPO[n] === eje;
      for (const m of this.solido["mat_" + n] || []) {
        if (!m.emissive) continue;
        m.emissive.copy(on ? new THREE.Color(0x3a3d44) : m.userData.emisivoBase);
      }
    }
    this.renderer.domElement.style.cursor = eje >= 0 ? "grab" : "";
    this.alHover?.(eje >= 0 ? { eje } : null);
  }

  bajar(e) {
    if (!this.listo || this.modo !== "articulaciones" || e.button !== 0) return;
    const eje = this.pieza(e);
    if (eje < 0) return;
    this.controles.enabled = false;
    this.renderer.domElement.setPointerCapture(e.pointerId);
    this.renderer.domElement.style.cursor = "grabbing";
    const arco = this.arcos[eje];
    this.colocarArco(arco);
    arco.grupo.updateMatrixWorld(true);
    const centro = new THREE.Vector3().setFromMatrixPosition(arco.grupo.matrixWorld);
    const ejeMundo = new THREE.Vector3().setFromMatrixColumn(arco.grupo.matrixWorld, 2).normalize();
    this.arrastre = {
      eje, centro, ejeMundo, q: this.ref[eje],
      plano: new THREE.Plane().setFromNormalAndCoplanarPoint(ejeMundo, centro),
      previo: this.enPlano(e, ejeMundo, centro), x: e.clientX, y: e.clientY,
    };
  }

  enPlano(e, ejeMundo, centro) {
    const ray = this.rayo(e);
    // Casi de canto el plano no sirve: se usa el desplazamiento en pantalla
    if (Math.abs(ray.direction.dot(ejeMundo)) < 0.18) return null;
    const p = new THREE.Vector3();
    return ray.intersectPlane(new THREE.Plane().setFromNormalAndCoplanarPoint(ejeMundo, centro), p) ? p.sub(centro) : null;
  }

  mover(e) {
    if (!this.listo) return;
    const a = this.arrastre;
    if (!a) {
      if (this.modo === "articulaciones" && e.buttons === 0) this.ponerHover(this.pieza(e));
      return;
    }
    const v = this.enPlano(e, a.ejeMundo, a.centro);
    let dModelo;
    if (v && a.previo) {
      const cruz = new THREE.Vector3().crossVectors(a.previo, v);
      dModelo = Math.atan2(cruz.dot(a.ejeMundo), a.previo.dot(v));
    } else {
      dModelo = -((e.clientX - a.x) + (e.clientY - a.y)) * 0.006;
    }
    a.previo = v;
    a.x = e.clientX; a.y = e.clientY;
    const dq = (dModelo * 180 / Math.PI) / this.cin.cal.sentido[a.eje];
    a.q = limitar(a.eje, a.q + dq);
    const q = this.ref.slice();
    q[a.eje] = a.q;
    this.setRef(q);
    this.alCambiarRef?.(q, { final: false, eje: a.eje });
  }

  soltar(e) {
    const a = this.arrastre;
    if (!a) return;
    this.arrastre = null;
    this.controles.enabled = true;
    try { this.renderer.domElement.releasePointerCapture(e.pointerId); } catch { /* nada */ }
    this.alCambiarRef?.(this.ref.slice(), { final: true, eje: a.eje });
    this.renderer.domElement.style.cursor = "";
    this.hover = -2;
    this.ponerHover(this.pieza(e));
  }

  // Posición en pantalla (px, relativa al visor) de un eje, para la etiqueta
  pantallaDeEje(eje) {
    const arco = this.arcos[eje];
    this.colocarArco(arco);
    arco.grupo.updateMatrixWorld(true);
    const p = new THREE.Vector3().setFromMatrixPosition(arco.grupo.matrixWorld);
    p.project(this.camera);
    const { clientWidth: w, clientHeight: h } = this.contenedor;
    return { x: (p.x + 1) / 2 * w, y: (1 - p.y) / 2 * h };
  }

  // ------------------------------------------------------------- cuadro --

  cuadro() {
    this.controles.update();
    if (this.listo && this.real && this.ref) {
      this.poner(this.solido, this.real);
      this.poner(this.fantasma, this.ref);
      const desde = [0, 1, 2].find(e => Math.abs(this.ref[e] - this.real[e]) > UMBRAL_FANTASMA) ?? 3;
      this.fantasma.raiz.visible = desde < 3;
      if (desde !== this.fantasmaDesde) {
        this.fantasmaDesde = desde;
        for (const n of GRUPOS) {
          const ver = n !== "fijo" && EJE_DE_GRUPO[n] >= desde;
          for (const m of this.fantasma.porGrupo[n]) m.visible = ver;
        }
      }

      const activo = this.arrastre ? this.arrastre.eje : this.hover;
      for (const arco of this.arcos) {
        arco.grupo.visible = this.modo === "articulaciones" && arco.e === activo;
        if (arco.grupo.visible) this.actualizarArco(arco, !!this.arrastre);
      }
      if (this.modo === "punto") this.actualizarPunto();
    }
    if (this.cam?.rig.visible && this.cam.textura) this.cam.textura.needsUpdate = true;
    this.renderer.render(this.scene, this.camera);
  }

  actualizarPunto() {
    const { esfera, linea, huella, falta } = this.punto;
    const p = esfera.position;
    linea.geometry.attributes.position.setXYZ(0, p.x, p.y, p.z);
    linea.geometry.attributes.position.setXYZ(1, p.x, 0, p.z);
    linea.geometry.attributes.position.needsUpdate = true;
    linea.computeLineDistances();
    huella.position.set(p.x, 0.001, p.z);
    falta.visible = this.punto.fuera;
    if (this.punto.fuera) {
      const t = this.cin.directa(this.ref);
      falta.geometry.attributes.position.setXYZ(0, t[0], t[1], t[2]);
      falta.geometry.attributes.position.setXYZ(1, p.x, p.y, p.z);
      falta.geometry.attributes.position.needsUpdate = true;
      falta.computeLineDistances();
    }
    esfera.material.color.setHex(this.punto.fuera ? COLOR.alloy : COLOR.senal);
  }
}

// El grupo "dueño" de una malla es el ancestro más cercano que sea grupo
function grupoDe(o) {
  for (let p = o; p; p = p.parent) if (GRUPOS.includes(p.name)) return p.name;
  return null;
}

const MAT_FANTASMA = new THREE.MeshStandardMaterial({
  color: COLOR.fantasma, roughness: 0.6, metalness: 0,
  transparent: true, opacity: 0.38, depthWrite: false,
  // Donde el fantasma y el sólido coinciden (el eje que no se movió), gana el sólido
  polygonOffset: true, polygonOffsetFactor: 2, polygonOffsetUnits: 4,
});
// Pasada de sola profundidad: el fantasma se ve como una sola capa, sin que
// sus piezas internas se encimen unas sobre otras.
const MAT_PROFUNDIDAD = new THREE.MeshBasicMaterial({
  colorWrite: false, transparent: true, depthWrite: true,
  polygonOffset: true, polygonOffsetFactor: 2, polygonOffsetUnits: 4,
});

function nuevoMatGuia(color, opacidad) {
  return new THREE.MeshBasicMaterial({ color, transparent: true, opacity: opacidad, depthTest: false, depthWrite: false, side: THREE.DoubleSide });
}
