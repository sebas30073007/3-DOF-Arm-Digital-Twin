// Panel del manipulador: habla con app.py (/api/*), que a su vez habla con la ESP32 por serial.
const $ = id => document.getElementById(id);
const post = (url, datos) => fetch(url, {
  method: 'POST', headers: {'Content-Type': 'application/json'}, body: JSON.stringify(datos),
}).then(r => r.json()).catch(() => ({}));
const cmd = linea => post('/api/cmd', {linea});
const config = datos => post('/api/config', datos);

const CASA = '<svg viewBox="0 0 24 24" width="16" height="16"><path fill="currentColor" d="M12 3 2 12h3v9h5v-6h4v6h5v-9h3z"/></svg>';
// k = clave en STATE, p = nombre en el protocolo, h = ángulo de HOME, inv = slider invertido
const C = [
  {n: 'Base', k: 'b', p: 'B', h: 0, inv: 0},        // v7: 0° = posición al encender
  {n: 'Eslabón 1', k: 'c', p: 'E1', h: 0, inv: 0},
  {n: 'Eslabón 2', k: 'm', p: 'E2', h: 0, inv: 1},
];
const G = [{n: 'Base', p: 'B', red: 10}, {n: 'Eslabones', p: 'E', red: 25}];

let L = null, cfgVel = false, cal = false, rsPrev = null, jog = null, puedeMover = false;
const ref = [0, 0, 0], act = [0, 0, 0], drag = [0, 0, 0], sync = [1, 1, 1], syncT = [0, 0, 0];

// =============================================================== tabs ===
document.querySelectorAll('.tabs button').forEach(b => b.onclick = () => {
  document.querySelectorAll('.tabs button').forEach(x => x.classList.toggle('sel', x === b));
  document.querySelectorAll('.tab').forEach(t => t.hidden = t.id !== 'tab-' + b.dataset.tab);
  if (b.dataset.tab === 'consola') bajarLog();
});

// ======================================================== acciones ===
function resync() { const t = Date.now() + 800; for (let i = 0; i < 3; i++) { sync[i] = 1; syncT[i] = t; } }
function alto() { if (jog) jogFin(jog.i); cmd('STOP'); resync(); }
$('bAlto').onclick = alto;
document.addEventListener('keydown', e => { if (e.key === 'Escape') { e.preventDefault(); alto(); } });
$('bCal').onclick = () => { resync(); cmd('CAL'); };
$('bHome').onclick = () => { resync(); cmd('HOME'); };
$('bSaludo').onclick = () => { resync(); cmd('SALUDO'); };
[$('bCal'), $('bHome'), $('bSaludo')].forEach(b => b.classList.add('mov'));

// ============================================================= ejes ===
function fr(i, v) { const a = L[2*i], b = L[2*i+1]; let f = (v - a) / (b - a); f = Math.min(1, Math.max(0, f)); return C[i].inv ? 1 - f : f; }
function vr(i, f) {
  f = Math.min(1, Math.max(0, f)); if (C[i].inv) f = 1 - f;
  const a = L[2*i], b = L[2*i+1];
  return Math.min(b, Math.max(a, Math.round((a + f * (b - a)) * 2) / 2));
}
function pinta(i) {
  if (!L) return;
  const fa = fr(i, act[i]), f0 = fr(i, C[i].h === null ? 0 : C[i].h);
  $('br'+i).style.left = Math.min(fa, f0) * 100 + '%';
  $('br'+i).style.width = Math.abs(fa - f0) * 100 + '%';
  $('pt'+i).style.left = fa * 100 + '%';
  $('rf'+i).style.left = fr(i, ref[i]) * 100 + '%';
  if (C[i].h !== null) $('ca'+i).style.left = f0 * 100 + '%';
  $('r'+i).textContent = ref[i].toFixed(1) + '°';
  $('a'+i).textContent = act[i].toFixed(1) + '°';
}

C.forEach((c, i) => {
  $('ejes').insertAdjacentHTML('beforeend', `
  <div class="eje mov" id="e${i}">
    <div class="eje-cab"><span>${c.n}</span><div class="v"><b id="r${i}">—</b><i id="a${i}">—</i></div></div>
    <div class="ctl">
      <button class="fl" id="fi${i}" aria-label="${c.n} menos">&lsaquo;</button>
      <div class="sl" id="sl${i}"><div class="pista"></div><div class="barra" id="br${i}"></div>
        ${c.h === null ? '' : `<div class="casa" id="ca${i}">${CASA}</div>`}
        <div class="real" id="pt${i}"></div><div class="ref" id="rf${i}"></div></div>
      <button class="fl" id="fd${i}" aria-label="${c.n} más">&rsaquo;</button>
    </div>
  </div>`);
  const sl = $('sl'+i);
  const mover = e => { const r = sl.getBoundingClientRect(); ref[i] = vr(i, (e.clientX - r.left) / r.width); sync[i] = 0; pinta(i); };
  sl.addEventListener('pointerdown', e => { if (!L) return; drag[i] = 1; sl.setPointerCapture(e.pointerId); mover(e); });
  sl.addEventListener('pointermove', e => { if (drag[i]) mover(e); });
  const fin = () => { if (!drag[i]) return; drag[i] = 0; syncT[i] = Date.now() + 800; cmd(`GOTO ${c.p} ${ref[i]}`); };
  sl.addEventListener('pointerup', fin); sl.addEventListener('pointercancel', fin);

  [['fi'+i, -1], ['fd'+i, 1]].forEach(([id, vis]) => {
    const b = $(id);
    b.addEventListener('pointerdown', e => { e.preventDefault(); b.setPointerCapture(e.pointerId); b.classList.add('on'); jogIni(i, vis); });
    const suelta = () => { if (!b.classList.contains('on')) return; b.classList.remove('on'); jogFin(i); };
    ['pointerup', 'pointercancel', 'lostpointercapture'].forEach(t => b.addEventListener(t, suelta));
    b.addEventListener('contextmenu', e => e.preventDefault());
  });
});

// El servidor manda GOTO al límite y STOP si deja de recibir latidos.
function jogIni(i, vis) {
  const signo = C[i].inv ? -vis : vis;
  if (jog) clearInterval(jog.t);
  const latido = () => post('/api/jog', {eje: i, signo});
  latido();
  jog = {i, t: setInterval(latido, 150)};
  sync[i] = 1;
}
function jogFin(i) {
  if (!jog) return;
  clearInterval(jog.t); jog = null;
  post('/api/jog', {eje: i, signo: 0});
  syncT[i] = Date.now() + 800;
  ['fi'+i, 'fd'+i].forEach(id => $(id).classList.remove('on'));
}
window.addEventListener('blur', () => { if (jog) jogFin(jog.i); });

// ======================================================== velocidad ===
G.forEach((g, i) => {
  $('grupos').insertAdjacentHTML('beforeend', `
  <div class="grp">
    <div class="gt"><span>${g.n}</span>
      <label><select id="pr${i}"><option>200</option><option>800</option><option>1000</option></select>p/rev</label></div>
    <div class="lin"><span>Motor</span><b id="vt${i}"></b></div>
    <input type="range" id="vel${i}" step="1">
    <div class="lin"><span>Aceleración</span><b id="at${i}"></b></div>
    <input type="range" id="ac${i}" step="5">
    <p class="info" id="vi${i}"></p>
  </div>`);
  let tv = null;
  [$('vel'+i), $('ac'+i)].forEach(x => x.addEventListener('input', () => {
    txtVel(i); clearTimeout(tv);
    tv = setTimeout(() => cmd(`VEL ${g.p} ${$('vel'+i).value} ${$('ac'+i).value}`), 150);
  }));
  $('pr'+i).onchange = e => cmd(`PULSOS ${g.p} ${e.target.value}`);
});
function txtVel(i) {
  const v = +$('vel'+i).value;
  $('vt'+i).textContent = v + ' rpm';
  $('at'+i).textContent = $('ac'+i).value + ' rpm/s';
  $('vi'+i).textContent = `${(v * 6 / G[i].red).toFixed(1)}°/s en la articulación`;
}

// ========================================================== visión ===
// [clave, etiqueta, min, max, paso, formato]
const P_SEG = [
  ['centro', 'Base al centro de la imagen', -200, 20, 1, v => (+v).toFixed(0) + '°'],
  ['fov', 'FOV horizontal', 30, 120, 1, v => v + '°'],
  ['ganancia', 'Ganancia', 0.1, 2, 0.05, v => (+v).toFixed(2)],
  ['zona_muerta', 'Zona muerta', 0.5, 10, 0.5, v => (+v).toFixed(1) + '°'],
  ['suavizado', 'Suavizado', 0, 0.9, 0.05, v => (+v).toFixed(2)],
];
const P_VIS = [
  ['conf', 'Confianza personas', 0.2, 0.9, 0.05, v => (+v).toFixed(2)],
  ['yolo_cada', 'YOLO cada N frames', 1, 6, 1, v => v],
  ['umbral_mano', 'Mano "cerca" (alto)', 0.08, 0.6, 0.01, v => Math.round(v * 100) + '%'],
  ['dedos_min', 'Dedos para palma', 2, 4, 1, v => v],
  ['frames_mano', 'Frames para fijar mano', 1, 15, 1, v => v],
];
const CHK_SEG = [['invertir', 'Invertir sentido de la base']];
const P_MANO = [
  ['mano_dedos', 'Dedos para mover', 1, 4, 1, v => v],
  ['e1_min', 'E1 con la mano abajo', 0, 136, 1, v => (+v).toFixed(0) + '°'],
  ['e1_max', 'E1 con la mano arriba', 0, 136, 1, v => (+v).toFixed(0) + '°'],
  ['e1_zona_muerta', 'Zona muerta E1', 1, 10, 0.5, v => (+v).toFixed(1) + '°'],
];
const CHK_MANO = [['mano_e1', 'Mover eslabón 1 con la altura'], ['e1_invertir', 'Invertir sentido de E1']];
const CHK_VIS = [['espejo', 'Ver en espejo']];

function construirParams(cont, grupo, rangos, checks) {
  rangos.forEach(([k, et, mn, mx, st, f]) => {
    cont.insertAdjacentHTML('beforeend', `
      <div class="param"><div class="lin"><span>${et}</span><b id="v-${grupo}-${k}"></b></div>
      <input type="range" id="p-${grupo}-${k}" min="${mn}" max="${mx}" step="${st}"></div>`);
    const x = $(`p-${grupo}-${k}`);
    x.addEventListener('input', () => {
      $(`v-${grupo}-${k}`).textContent = f(x.value);
      x.dataset.tocado = Date.now();
    });
    x.addEventListener('change', () => config({[grupo]: {[k]: +x.value}}));
  });
  checks.forEach(([k, et]) => {
    cont.insertAdjacentHTML('beforeend',
      `<label class="chk param"><input type="checkbox" id="p-${grupo}-${k}">${et}</label>`);
    const x = $(`p-${grupo}-${k}`);
    x.addEventListener('change', () => config({[grupo]: {[k]: x.checked}}));
  });
}
function pintarParams(grupo, rangos, checks, p) {
  rangos.forEach(([k, , , , , f]) => {
    const x = $(`p-${grupo}-${k}`);
    if (document.activeElement === x || Date.now() - (+x.dataset.tocado || 0) < 1500) return;
    x.value = p[k]; $(`v-${grupo}-${k}`).textContent = f(p[k]);
  });
  checks.forEach(([k]) => { $(`p-${grupo}-${k}`).checked = !!p[k]; });
}
construirParams($('pSeg'), 'seguimiento', P_SEG, CHK_SEG);
construirParams($('pMano'), 'seguimiento', P_MANO, CHK_MANO);
construirParams($('pVis'), 'vision', P_VIS, CHK_VIS);

$('tVision').onchange = e => config({vision: {activa: e.target.checked}});
$('tSeg').onchange = e => config({seguimiento: {activo: e.target.checked}});
document.querySelectorAll('#segCam button').forEach(b =>
  b.onclick = () => config({seguimiento: {camara: b.dataset.v}}));
$('selCam').onchange = e => config({camara: +e.target.value});

// ========================================================= consola ===
let logSeq = 0;
const hist = []; let hi = 0;
function bajarLog() { const l = $('log'); l.scrollTop = l.scrollHeight; }
function agregarLog(lineas) {
  const l = $('log');
  const abajo = l.scrollHeight - l.scrollTop - l.clientHeight < 24;
  const flecha = {tx: '→ ', rx: '← ', msg: '· ', sys: ''};
  for (const x of lineas) {
    const d = document.createElement('div');
    const t = new Date(x.t * 1000).toLocaleTimeString('es-MX', {hour12: false});
    const tipo = x.d === 'rx' && x.x.startsWith('ERR') ? 'err' : x.d;
    d.innerHTML = `<span class="t">${t}</span><span class="d-${tipo}"></span>`;
    d.lastChild.textContent = flecha[x.d] + x.x;
    l.appendChild(d);
  }
  while (l.childElementCount > 500) l.firstChild.remove();
  if (abajo) bajarLog();
}
$('fCmd').onsubmit = e => {
  e.preventDefault();
  const v = $('iCmd').value.trim();
  if (!v) return;
  if (hist[hist.length - 1] !== v) hist.push(v);
  hi = hist.length;
  $('iCmd').value = '';
  if (v.toUpperCase() === 'STOP') resync();
  cmd(v);
};
$('iCmd').addEventListener('keydown', e => {
  if (e.key === 'ArrowUp' && hi > 0) { hi--; $('iCmd').value = hist[hi]; e.preventDefault(); }
  if (e.key === 'ArrowDown') { hi = Math.min(hist.length, hi + 1); $('iCmd').value = hist[hi] || ''; e.preventDefault(); }
});
document.querySelectorAll('.rapidos button').forEach(b => b.onclick = () => cmd(b.dataset.c));

// ========================================================== puertos ===
async function cargarPuertos(actual, auto) {
  const lista = await fetch('/api/puertos').then(r => r.json()).catch(() => []);
  const s = $('selPuerto');
  s.innerHTML = '<option value="">Auto</option>' + lista.map(p =>
    `<option value="${p.puerto}">${p.puerto}${p.esp32 ? ' · ESP32' : ''}</option>`).join('');
  s.value = auto ? '' : (actual || '');
}
$('selPuerto').onchange = e => config({puerto: e.target.value || null});
$('selPuerto').addEventListener('focus', () => cargarPuertos($('selPuerto').value, $('selPuerto').value === ''));

// =========================================================== estado ===
function dl(el, pares) { el.innerHTML = pares.map(([k, v]) => `<dt>${k}</dt><dd>${v}</dd>`).join(''); }

function pintarRobot(r) {
  const j = r.estado;
  $('pConn').classList.toggle('on', r.conectado);
  $('tConn').textContent = r.conectado ? `${r.puerto} · Control ${r.modo || '…'}` : 'Sin conexión';
  $('tConn2').textContent = r.conectado ? (j ? (j.cal ? 'Calibrado' : 'Sin calibrar') + (j.ocu ? ' · moviendo' : '') : 'leyendo estado…')
                                        : (r.error || 'buscando ESP32…');
  puedeMover = r.conectado && r.modo === 'PC';
  document.body.classList.toggle('ro', !puedeMover);
  const av = $('aviso');
  av.hidden = !(r.conectado && r.modo === 'WEB');
  av.textContent = 'La página del ESP32 tiene el control. Pulsa "Control PC" en ella (192.168.4.1) para mover desde aquí.';
  if (!puedeMover && jog) jogFin(jog.i);
  if (!j) { $('msg').textContent = ''; return; }

  L = j.lim;   // se relee siempre: cambia entre versiones del firmware
  if (!cfgVel) {
    for (let g = 0; g < 2; g++) {
      const v = $('vel'+g), a = $('ac'+g);
      v.min = j.vr[0]; v.max = j.vr[1+g]; a.min = j.vr[3]; a.max = j.vr[4+g];
      v.value = j.vel[g]; a.value = j.acel[g]; txtVel(g);
    }
    cfgVel = true;
  }
  if (rsPrev !== null && j.rs !== rsPrev) resync();
  rsPrev = j.rs;
  if (j.cal && !cal) resync();
  cal = !!j.cal;
  ['bHome', 'bSaludo'].forEach(id => $(id).classList.toggle('off', !cal));

  const now = Date.now();
  C.forEach((c, i) => {
    act[i] = j[c.k];
    if (sync[i] && !drag[i]) {
      ref[i] = act[i];
      if (!j.ocu && !(jog && jog.i === i) && now > syncT[i]) sync[i] = 0;
    } else if (!drag[i] && !j.ocu && now > syncT[i]) {
      ref[i] = act[i];
    }
    $('e'+i).classList.toggle('off', i > 0 && !cal);   // E1/E2 por serial exigen calibración
    [['fi'+i, -1], ['fd'+i, 1]].forEach(([id, vis]) => {
      const s = c.inv ? -vis : vis, b = $(id);
      const blq = j.bl[i] !== 0 && s === j.bl[i];
      if (blq && b.classList.contains('on')) jogFin(i);
      b.classList.toggle('blq', blq);
    });
    pinta(i);
  });
  ['sw1', 'sw2', 'sw3'].forEach(k => $(k).classList.toggle('on', j[k] == 1));
  for (let g = 0; g < 2; g++) {
    if (document.activeElement !== $('pr'+g)) $('pr'+g).value = j.pr[g];
    if (document.activeElement !== $('vel'+g) && document.activeElement !== $('ac'+g)) {
      $('vel'+g).value = j.vel[g]; $('ac'+g).value = j.acel[g]; txtVel(g);
    }
  }
  $('msg').textContent = j.msg;
  if (j.ue >= 0) $('ult').textContent =
    `Último: ${C[j.ue].n} ${j.ud.toFixed(1)}° en ${j.us.toFixed(2)} s · ${(j.us > 0 ? j.ud / j.us : 0).toFixed(1)}°/s`;
}

function pintarVision(v, s) {
  $('tVision').checked = v.activa;
  $('tSeg').checked = s.activo;
  $('sinCam').hidden = v.camara_ok;
  $('chCam').textContent = v.camara_ok ? `Cámara ${v.camara} · ${v.fps.toFixed(0)} fps` : 'Sin cámara';
  $('chCam').classList.toggle('on', v.camara_ok);
  const estVis = !v.activa ? 'apagada' : (v.modelos === 'listo' ? 'activa' : v.modelos);
  $('chVis').textContent = 'Visión ' + estVis;
  $('chVis').classList.toggle('on', v.activa && v.modelos === 'listo');
  $('visEstado').textContent = !v.activa ? 'Apagada. Al encenderla se cargan YOLO y MediaPipe (unos segundos la primera vez).'
                                         : (v.modelos === 'listo' ? 'Detectando personas y manos.' + (v.error ? ' Último error: ' + v.error : '') : 'Modelos: ' + v.modelos);
  const o = v.objetivo;
  const txtObj = !o ? 'Sin objetivo' : o.tipo === 'mano' ? `Mano · ${o.dedos} dedos${s.pausa ? ' · pausa' : ''}` : `Persona · x ${o.x.toFixed(2)}`;
  $('chObj').textContent = s.activo ? 'Siguiendo · ' + txtObj : txtObj;
  $('chObj').classList.toggle('on', !!o);
  dl($('visDatos'), v.activa ? [
    ['personas', v.personas],
    ['manos', v.manos.map(m => `${m.dedos} dedos ${Math.round(m.tam * 100)}%${m.abierta && m.cerca ? ' palma' : ''}`).join(' · ') || '0'],
    ['modo', v.modo_mano ? 'MANO' : 'persona'],
    ['objetivo', o ? `${o.tipo} x=${o.x.toFixed(3)} y=${o.y.toFixed(3)}` : '—'],
  ] : []);
  $('segEstado').textContent = s.activo ? s.texto[0].toUpperCase() + s.texto.slice(1)
                                        : (s.texto === 'apagado' ? 'Apagado. Mueve la base hacia la persona o la palma.' : s.texto[0].toUpperCase() + s.texto.slice(1));
  dl($('segDatos'), s.activo ? [
    ['base deseada', s.deseado == null ? '—' : s.deseado.toFixed(1) + '°'],
    ['último GOTO B', s.enviado == null ? '—' : s.enviado.toFixed(1) + '°'],
    ['E1 deseado', s.deseado_e1 == null ? '—' : s.deseado_e1.toFixed(1) + '°'],
    ['último GOTO E1', s.enviado_e1 == null ? '—' : s.enviado_e1.toFixed(1) + '°'],
  ] : []);
  document.querySelectorAll('#segCam button').forEach(b => b.classList.toggle('sel', b.dataset.v === s.p.camara));
  pintarParams('seguimiento', P_SEG, CHK_SEG, s.p);
  pintarParams('seguimiento', P_MANO, CHK_MANO, s.p);
  pintarParams('vision', P_VIS, CHK_VIS, v.p);
  if (document.activeElement !== $('selCam')) $('selCam').value = v.camara;
}

let primera = true;
async function ciclo() {
  try {
    const e = await fetch('/api/estado?log=' + logSeq, {cache: 'no-store'}).then(r => r.json());
    if (primera) { cargarPuertos(e.robot.puerto, e.robot.auto); primera = false; }
    pintarRobot(e.robot);
    pintarVision(e.vision, e.seguimiento);
    if (e.log.lineas.length) agregarLog(e.log.lineas);
    logSeq = e.log.seq;
  } catch (err) {
    $('tConn').textContent = 'Servidor detenido';
    $('tConn2').textContent = 'reinicia app.py';
    $('pConn').classList.remove('on');
  }
  setTimeout(ciclo, 150);
}
ciclo();

// Si el stream se corta (servidor reiniciado), reintentar
$('video').onerror = () => setTimeout(() => { $('video').src = '/video?' + Date.now(); }, 1000);
