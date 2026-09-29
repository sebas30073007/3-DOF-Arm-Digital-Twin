#include <WiFi.h>
#include <WebServer.h>
#include <DNSServer.h>
#include "soc/soc.h"
#include "soc/gpio_reg.h"

/*
  Manipulador 3 GDL - v8
  ESP32-C3 SuperMini + 3x CL57T
  Core Arduino-ESP32 3.x  |  "USB CDC On Boot: Enabled"

  Pasos: timer a 40 kHz con acumulador de fase (DDA) y perfil trapezoidal
  (aceleración constante). Velocidad y aceleración en rpm del MOTOR.

  Novedad de v8: los tres ejes se mueven A LA VEZ. Hay un generador por
  articulación en el mismo timer; GOTO y POSE arrancan cada eje en cuanto
  llega su objetivo, sin esperar a los otros, y cada uno corrige su destino
  al vuelo como en v7 (seguimiento). El motor del eslabón 2 (driver 1) lo
  mueven dos articulaciones: el eslabón 1 (compensación 1:1) y el eslabón 2;
  sus pasos se suman con signo y un emisor aparte los saca con la dirección
  neta, así el eslabón 2 no se desalinea aunque E1 y E2 se muevan juntos.
  Calibración, HOME, saludo y jog siguen siendo un eje a la vez (en HOME el
  orden E2, E1, base evita choques). Las rutinas por pasos (RUTINA) mueven a
  la vez los ejes de cada paso.

  Grupo 0 = base (driver 3). Grupo 1 = eslabones (drivers 1 y 2: deben tener
  los mismos pulsos/rev porque el eslabón 1 arrastra al motor del eslabón 2 1:1).

  Control por dos fuentes:
    - PC por USB (comandos de texto, una línea por comando; ver "Protocolo serial")
    - Página web
  Solo una fuente puede mover el robot a la vez (selector en la página).
  ALTO desde la página siempre detiene y pasa el control a la web.

  Protocolo serial (respuestas: OK..., ERR..., STATE {json}; avisos: MSG ...)
    PING                      -> PONG
    STATE?                    -> STATE {json}
    MODE?                     -> MODE PC | MODE WEB
    STOP                      -> frena con rampa y vacía la cola
    CAL                       -> calibración
    HOME | SALUDO             -> rutinas
    RUTINA <nombre>           -> rutina por pasos: SALUDITO, REVERENCIA, LADO, BAILE, PICOTEO
    GOTO <B|E1|E2> <grados>   -> objetivo absoluto; si llega otro mientras se mueve,
                                 se corrige el destino al vuelo (seguimiento)
    POSE <b|NA> <e1|NA> <e2|NA>
    VEL <B|E> <rpm> [rpm/s]
    PULSOS <B|E> <200|800|1000>
    GRIP ...                  -> reservado para el gripper (aún no implementado)
*/

// Tipos antes de cualquier función (el IDE inserta prototipos arriba)
enum { RES_NADA, RES_OK, RES_SWITCH, RES_SEGURIDAD, RES_ALTO, RES_BLOQ };
enum { M_NADA, M_ACTIVAR, M_LIBERAR, M_SEGURIDAD };
enum { BASE = 0, CODO = 1, MUNECA = 2 };
enum { FUENTE_WEB = 0, FUENTE_PC = 1 };
// Paso de una rutina: pose a la que van a la vez los ejes que no son NAN, y
// espera al llegar
struct PasoRutina { float b, e1, e2; uint16_t ms; };
struct Rutina { const char* comando; const char* titulo; const PasoRutina* pasos; uint8_t n; };

// -------------------- Red --------------------
const char* NOMBRE_RED = "Manipulador";
const char* CLAVE_RED  = "12345678";

// -------------------- Pines --------------------
const uint8_t PUL1 = 20, DIR1 = 21;   // Driver 1 -> eslabón 2 (muñeca)
const uint8_t PUL2 = 7,  DIR2 = 10;   // Driver 2 -> eslabón 1 (codo)
const uint8_t PUL3 = 5,  DIR3 = 6;    // Driver 3 -> base

const uint8_t SW1_PIN = 1;   // base (no conectado)
const uint8_t SW2_PIN = 3;   // eslabón 1
const uint8_t SW3_PIN = 4;   // eslabón 2
const uint8_t BTN_PIN = 9;   // BOOT
const uint8_t LED_PIN = 8;   // LED integrado (activo en LOW)

// Nivel de DIR que produce giro articular positivo
const uint8_t DIR_POS_BASE   = 1;
const uint8_t DIR_POS_CODO   = 1;
const uint8_t DIR_POS_MUNECA = 0;
const bool COMPENSAR_MUNECA = true;   // 1:1

// -------------------- Mecánica --------------------
const float REDUCCION[3] = { 10.0f, 25.0f, 25.0f };
// Base: 0° = posición al encender (sin switch). Gira de +20° a -200°.
const float LIM_MIN[3]   = { -200.0f,   0.0f, -220.0f };
const float LIM_MAX[3]   = {   20.0f, 136.5f,    0.0f };

// Signo articular en el que está cada switch de home
const int HOME_SIGNO_CODO   = -1;   // SW2
const int HOME_SIGNO_MUNECA = +1;   // SW3

// Pulsos/rev por grupo: deben coincidir con el DIP de cada CL57T
const int PULSOS_OPCIONES[] = { 200, 800, 1000 };
int pulsosRev[2] = { 1000, 200 };   // { base, eslabones }

int grupo(int eje) { return eje == BASE ? 0 : 1; }

// -------------------- Velocidad (rpm del motor) --------------------
// Valores por defecto y topes, por grupo { base, eslabones }
float velRpm[2]   = { 10.0f, 20.0f };
float acelRpmS[2] = { 70.0f, 70.0f };
const float VEL_MIN = 2.0f;
const float VEL_MAX[2]  = { 50.0f, 120.0f };
const float ACEL_MIN = 10.0f;
const float ACEL_MAX[2] = { 400.0f, 400.0f };
const float RPM_ARRANQUE = 5.0f;          // velocidad de arranque y de paro

const float HOME_RAPIDO_RPM = 17.0f;
const float HOME_LENTO_RPM  = 9.0f;
const float HOME_ACEL       = 200.0f;

const float JOG_SIN_CAL_RPM = 20.0f;      // tope de velocidad sin calibrar
const float JOG_SIN_CAL_DEG = 360.0f;
const uint32_t LATIDO_MS = 600;           // si la página deja de latir, el jog frena

// -------------------- Rutinas --------------------
// HOME = punto donde termina la calibración (separado del switch); base a 0°.
// Orden de regreso a HOME: primero eslabón 2, luego eslabón 1, al final base.
const int ORDEN_HOME[3] = { MUNECA, CODO, BASE };

// Saludo: HOME -> E1 a 30° -> E2 alterna A/B n veces -> HOME
const float SALUDO_E1   = 30.0f;
const float SALUDO_E2_A = -45.0f;   // el eslabón 2 trabaja en negativo (0 a -220)
const float SALUDO_E2_B = -30.0f;
const int   SALUDO_CICLOS = 2;
const uint32_t SALUDO_PAUSA_MS = 20;

// Rutinas por pasos (RUTINA <comando>): salen de HOME y regresan a HOME.
// NAN = ese eje no se mueve en el paso. Copia de RUTINAS en gemelo/js/config.js.
const PasoRutina R_SALUDITO[] = {
  { NAN, 44, NAN, 0 }, { NAN, NAN, -90, 0 }, { NAN, NAN, -70, 0 }, { NAN, NAN, -90, 0 },
};
const PasoRutina R_REVERENCIA[] = {
  { NAN, 50, -40, 0 }, { NAN, NAN, -72, 0 }, { NAN, 100, NAN, 2000 },   // se queda 2 s abajo
  { NAN, 54, NAN, 0 }, { NAN, NAN, -62, 0 }, { NAN, NAN, -72, 0 },
};
const PasoRutina R_LADO[] = {
  { NAN, 50, -55, 0 }, { 15, NAN, NAN, 0 }, { -15, NAN, NAN, 0 }, { 15, NAN, NAN, 0 }, { -15, NAN, NAN, 0 },
};
const PasoRutina R_BAILE[] = {
  { NAN, 35, -35, 0 }, { NAN, 65, -70, 0 }, { NAN, 35, -45, 0 }, { NAN, 65, -70, 0 }, { NAN, 35, -35, 0 },
};
const PasoRutina R_PICOTEO[] = {
  { NAN, 70, -55, 0 }, { NAN, 85, -35, 0 }, { NAN, 65, -60, 0 }, { NAN, 85, -35, 0 }, { NAN, 70, -55, 0 },
};
#define PASOS(r) r, (uint8_t)(sizeof(r) / sizeof(r[0]))
const Rutina RUTINAS[] = {
  { "SALUDITO",   "Saludito",       PASOS(R_SALUDITO) },
  { "REVERENCIA", "Reverencia",     PASOS(R_REVERENCIA) },
  { "LADO",       "Saludo de lado", PASOS(R_LADO) },
  { "BAILE",      "Bailecito",      PASOS(R_BAILE) },
  { "PICOTEO",    "Picoteo",        PASOS(R_PICOTEO) },
};
const int N_RUTINAS = sizeof(RUTINAS) / sizeof(RUTINAS[0]);
const int RUTINA_PASOS = 10;        // pedirRutina = RUTINA_PASOS + índice en RUTINAS

// -------------------- Timer --------------------
const uint32_t TICK_HZ = 40000;           // 25 us
const int32_t TICKS_DIR = 8;              // 200 us
const int32_t TICKS_CONFIRMA = 200;       // 5 ms

// Distancias de homing (pasos referidos a 1000 p/rev)
const int32_t BACKOFF      = 220;
const int32_t CLEARANCE    = 120;
const int32_t MAX_BUSQUEDA = 12000;
const int32_t MAX_LIBERAR  = 1500;

// -------------------- Estado del generador (ISR) --------------------
// Un generador por articulación [BASE, CODO, MUNECA]; los tres corren a la vez.
// e_abierto: el movimiento arrancó y el loop todavía no lo cierra (sumar a pos).
volatile bool     e_activo[3], e_pedirAlto[3], e_confirmando[3], e_frenando[3], e_abierto[3];
volatile int8_t   e_signo[3];
volatile int32_t  e_total[3], e_hechos[3], e_espera[3];
volatile uint32_t e_acum[3], e_v[3], e_vMax2[3], e_vMin2[3], e_a2[3], e_mascaraSw[3];
volatile uint8_t  e_modo[3], e_resultado[3];

// Motor del eslabón 2 (driver 1): pasos pendientes con signo (+ = sentido
// positivo del eslabón 2), dirección puesta y espera tras cambiarla.
volatile int32_t  m1Pend = 0, m1Espera = 0;
volatile int8_t   m1Dir = 0;
volatile uint32_t pulBajo = 0;            // pines PUL que quedaron en LOW este tick

hw_timer_t* timer = nullptr;

// -------------------- Estado general --------------------
long pos[3] = { 0, 0, 0 };
volatile long posMunecaMotor = 0;
bool calibrado = false, ocupado = false;

volatile bool pedirCalib = false, pedirAlto = false;
volatile bool hayObjetivo[3] = { false, false, false };
volatile float objetivo[3] = { 0, 0, 0 };
volatile int pedirPulsos[2] = { 0, 0 };
volatile int pedirRutina = 0;      // 1 = HOME, 2 = saludo
volatile uint8_t fuente = FUENTE_PC;   // quién puede mover el robot al arrancar
volatile uint32_t rechazos = 0;   // la página resincroniza sus referencias cuando cambia

volatile bool pedirJog = false, jogParar = false, enJog = false;
volatile int jogEje = 0, jogSigno = 1;
volatile uint32_t ultimoLatido = 0;

int ultEje = -1;
float ultDeg = 0, ultSeg = 0;

char mensaje[96] = "Sin calibrar";

WebServer servidor(80);
DNSServer dns;
IPAddress IP_AP(192, 168, 4, 1);

// -------------------- Página --------------------
const char PAGINA[] = R"HTML(
<!DOCTYPE html>
<html lang="es"><head><meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1, user-scalable=no">
<title>Manipulador</title>
<style>
:root{--g0:#141416;--g1:#202023;--g2:#303034;--g3:#46464c;--gc:#c8c8cd;--w:#fff;--r:#8e0c26;--r2:#b3122f}
*{box-sizing:border-box}
body{margin:0;background:var(--g0);color:var(--w);font-family:system-ui,Arial,sans-serif;user-select:none;-webkit-user-select:none}
main{width:min(96vw,460px);margin:auto;padding:12px 0 24px}
.fila{display:flex;gap:8px}
button{border:0;border-radius:12px;background:var(--g2);color:var(--w);font-size:1rem;touch-action:none;-webkit-tap-highlight-color:transparent}
.top button{flex:1;height:52px;display:flex;align-items:center;justify-content:center;gap:8px;font-weight:600}
#alto{background:var(--r)}#alto:active{background:var(--r2)}
.rut{margin-top:8px}.rut button{height:46px;background:var(--g1)}
.rut button.off{opacity:.3;pointer-events:none}
.modo{margin-bottom:8px;background:var(--g1);border-radius:12px;padding:4px}
.modo button{flex:1;height:38px;background:transparent;color:var(--gc);font-weight:600}
.modo button.sel{background:var(--gc);color:var(--g0)}
.ro #cal,.ro .rut button,.ro .fl,.ro .sl{opacity:.3;pointer-events:none}
.sw{margin-top:8px}
.sw div{flex:1;height:34px;border-radius:10px;background:var(--g1);color:var(--gc);display:grid;place-items:center;font-weight:600;letter-spacing:.05em}
.sw div.on{background:var(--r);color:var(--w)}
#msg{color:var(--gc);text-align:center;font-size:.9rem;min-height:1.2em;margin:10px 0 0}
.caja{background:var(--g1);border-radius:16px;padding:14px;margin-top:10px}
.eje+.eje{margin-top:18px}
.cab{display:grid;grid-template-columns:1fr auto 1fr;align-items:center;margin-bottom:2px}
.cab span{color:var(--gc);font-size:.9rem}
.cab .v{display:flex;gap:16px;font-weight:600;font-variant-numeric:tabular-nums}
.cab .v i{font-style:normal;color:var(--r2)}
.cab .v b::before,.cab .v i::before{content:'';display:inline-block;width:9px;height:9px;border-radius:50%;margin-right:6px}
.cab .v b::before{background:var(--w)}.cab .v i::before{background:var(--r2)}
.ctl{display:flex;align-items:center;gap:8px}
.fl{width:44px;height:44px;flex:none;font-size:1.5rem;font-weight:700;line-height:1}
.fl.on{background:var(--r)}
.fl.blq{opacity:.2;pointer-events:none}
.sl{position:relative;flex:1;height:44px;touch-action:none;cursor:pointer}
.pista,.barra{position:absolute;top:24px;height:6px;border-radius:3px}
.pista{left:0;right:0;background:var(--g3)}
.barra{background:var(--r);transition:left .15s linear,width .15s linear}
.real,.ref{position:absolute;top:27px;border-radius:50%;transform:translate(-50%,-50%)}
.real{width:14px;height:14px;background:var(--r2);transition:left .15s linear;z-index:2}
.ref{width:22px;height:22px;background:var(--w);border:3px solid var(--g1);z-index:3}
.casa{position:absolute;top:0;transform:translateX(-50%);color:var(--gc);line-height:0}
.eje.off .sl{opacity:.3;pointer-events:none}
h2{font-size:.75rem;color:var(--gc);letter-spacing:.1em;text-transform:uppercase;margin:0;font-weight:600}
.lin{display:flex;justify-content:space-between;align-items:center;margin-top:10px;color:var(--gc)}
.lin b{color:var(--w)}
select{background:var(--g2);color:var(--w);border:0;border-radius:10px;height:38px;padding:0 10px;font-size:1rem}
input[type=range]{width:100%;accent-color:var(--r2);height:30px;margin:4px 0 0}
.info{color:var(--gc);font-size:.85rem;margin-top:8px}
.grp{margin-top:12px}.grp+.grp{border-top:1px solid var(--g3);padding-top:12px}
.gt{display:flex;justify-content:space-between;align-items:center;font-weight:600}
.gt label{color:var(--gc);font-weight:400;display:flex;align-items:center;gap:8px}
</style></head><body><main>
<div class="fila modo"><button id="mw" onclick="modo('web')">Control web</button><button id="mp" onclick="modo('pc')">Control PC</button></div>
<div class="fila top">
  <button id="cal" onclick="calibrar()">Calibrar</button>
  <button id="alto" onclick="alto()">ALTO</button>
</div>
<div class="fila top rut">
  <button id="bh" onclick="rut('home')">HOME</button>
  <button id="bs" onclick="rut('saludo')">Saludo</button>
</div>
<div class="fila sw"><div id="sw1">SW1</div><div id="sw2">SW2</div><div id="sw3">SW3</div></div>
<p id="msg"></p>
<div class="caja" id="ejes"></div>
<div class="caja">
  <h2>Velocidad</h2>
  <div id="grupos"></div>
  <div class="info" id="ult"></div>
</div>
</main>
<script>
const $=id=>document.getElementById(id);
const CASA='<svg viewBox="0 0 24 24" width="18" height="18"><path fill="currentColor" d="M12 3 2 12h3v9h5v-6h4v6h5v-9h3z"/></svg>';
const MIRA='<svg viewBox="0 0 24 24" width="18" height="18" fill="none" stroke="currentColor" stroke-width="2"><circle cx="12" cy="12" r="7"/><path d="M12 1v5M12 18v5M1 12h5M18 12h5"/></svg>';
$('cal').insertAdjacentHTML('afterbegin',MIRA);
$('bh').insertAdjacentHTML('afterbegin',CASA);
const C=[{n:'Base',k:'b',h:0,inv:0},{n:'Eslabón 1',k:'c',h:0,inv:0},{n:'Eslabón 2',k:'m',h:0,inv:1}];
const RED=[10,25,25];
let L=null,cfg=false,jog=null,cal=false,rsPrev=null;
const ref=[0,0,0],act=[0,0,0],drag=[0,0,0],sync=[1,1,1],syncT=[0,0,0];

function cmd(u){fetch(u,{cache:'no-store'}).catch(()=>{});}
function fr(i,v){const a=L[2*i],b=L[2*i+1];let f=(v-a)/(b-a);f=Math.min(1,Math.max(0,f));return C[i].inv?1-f:f;}
function vr(i,f){f=Math.min(1,Math.max(0,f));if(C[i].inv)f=1-f;const a=L[2*i],b=L[2*i+1];
  return Math.min(b,Math.max(a,Math.round((a+f*(b-a))*2)/2));}
function pinta(i){
  if(!L)return;
  const fa=fr(i,act[i]),f0=fr(i,C[i].h===null?0:C[i].h);
  $('br'+i).style.left=Math.min(fa,f0)*100+'%';
  $('br'+i).style.width=Math.abs(fa-f0)*100+'%';
  $('pt'+i).style.left=fa*100+'%';
  $('rf'+i).style.left=fr(i,ref[i])*100+'%';
  if(C[i].h!==null)$('ca'+i).style.left=f0*100+'%';
  $('r'+i).textContent=ref[i].toFixed(1)+'°';
  $('a'+i).textContent=act[i].toFixed(1)+'°';
}
function resync(){const t=Date.now()+800;for(let i=0;i<3;i++){sync[i]=1;syncT[i]=t;}}

C.forEach((c,i)=>{
  $('ejes').insertAdjacentHTML('beforeend',
  `<div class="eje" id="e${i}">
    <div class="cab"><span>${c.n}</span><div class="v"><b id="r${i}">-</b><i id="a${i}">-</i></div><span></span></div>
    <div class="ctl">
      <button class="fl" id="fi${i}">&lsaquo;</button>
      <div class="sl" id="sl${i}"><div class="pista"></div><div class="barra" id="br${i}"></div>
        ${c.h===null?'':`<div class="casa" id="ca${i}">${CASA}</div>`}
        <div class="real" id="pt${i}"></div><div class="ref" id="rf${i}"></div></div>
      <button class="fl" id="fd${i}">&rsaquo;</button>
    </div></div>`);
  const sl=$('sl'+i);
  const mover=e=>{const r=sl.getBoundingClientRect();ref[i]=vr(i,(e.clientX-r.left)/r.width);sync[i]=0;pinta(i);};
  sl.addEventListener('pointerdown',e=>{if(!L)return;drag[i]=1;sl.setPointerCapture(e.pointerId);mover(e);});
  sl.addEventListener('pointermove',e=>{if(drag[i])mover(e);});
  const fin=()=>{if(!drag[i])return;drag[i]=0;cmd(`/ir?e=${i}&v=${ref[i]}`);};
  sl.addEventListener('pointerup',fin);sl.addEventListener('pointercancel',fin);
  [['fi'+i,-1],['fd'+i,1]].forEach(([id,vis])=>{
    const b=$(id);
    b.addEventListener('pointerdown',e=>{e.preventDefault();b.classList.add('on');jogIni(i,vis);});
    const suelta=()=>{if(!b.classList.contains('on'))return;b.classList.remove('on');jogFin(i);};
    ['pointerup','pointerleave','pointercancel'].forEach(t=>b.addEventListener(t,suelta));
    b.addEventListener('contextmenu',e=>e.preventDefault());
  });
});

function jogIni(i,vis){
  const s=C[i].inv?-vis:vis,u=`/jog?e=${i}&d=${s}`;
  if(jog)clearInterval(jog.t);
  cmd(u);jog={i,t:setInterval(()=>cmd(u),200)};sync[i]=1;
}
function jogFin(i){
  if(!jog)return;clearInterval(jog.t);jog=null;
  cmd(`/jog?e=${i}&d=0`);syncT[i]=Date.now()+800;
}
function calibrar(){resync();cmd('/calibrar');}
function alto(){cmd('/alto');resync();}
function rut(n){resync();cmd('/rutina?n='+n);}
function modo(m){resync();cmd('/modo?m='+m);}

const G=[{n:'Base',red:10},{n:'Eslabones',red:25}];
G.forEach((g,i)=>{
  $('grupos').insertAdjacentHTML('beforeend',
  `<div class="grp"><div class="gt"><span>${g.n}</span>
     <label><select id="pr${i}"><option>200</option><option>800</option><option>1000</option></select>p/rev</label></div>
   <div class="lin"><span>Motor</span><b id="vt${i}"></b></div>
   <input type="range" id="vel${i}" step="1">
   <div class="lin"><span>Aceleración</span><b id="at${i}"></b></div>
   <input type="range" id="ac${i}" step="5">
   <div class="info" id="vi${i}"></div></div>`);
  let tv=null;
  [$('vel'+i),$('ac'+i)].forEach(x=>x.addEventListener('input',()=>{txtVel(i);clearTimeout(tv);
    tv=setTimeout(()=>cmd(`/vel?g=${i}&v=${$('vel'+i).value}&a=${$('ac'+i).value}`),150);}));
  $('pr'+i).onchange=e=>cmd(`/pulsos?g=${i}&v=${e.target.value}`);
});
function txtVel(i){
  const v=+$('vel'+i).value;
  $('vt'+i).textContent=v+' rpm';$('at'+i).textContent=$('ac'+i).value+' rpm/s';
  $('vi'+i).textContent=`${(v*6/G[i].red).toFixed(1)}°/s en la articulación`;
}

async function estado(){
  try{
    const j=await(await fetch('/estado',{cache:'no-store'})).json();
    if(!L)L=j.lim;
    if(!cfg){for(let g=0;g<2;g++){const v=$('vel'+g),a=$('ac'+g);
      v.min=j.vr[0];v.max=j.vr[1+g];a.min=j.vr[3];a.max=j.vr[4+g];
      v.value=j.vel[g];a.value=j.acel[g];txtVel(g);}cfg=true;}
    if(rsPrev!==null&&j.rs!==rsPrev)resync();
    rsPrev=j.rs;
    if(j.cal&&!cal)resync();
    cal=!!j.cal;
    const pc=j.src===1;
    document.querySelector('main').classList.toggle('ro',pc);
    $('mp').classList.toggle('sel',pc);$('mw').classList.toggle('sel',!pc);
    if(pc&&jog){const i=jog.i;['fi'+i,'fd'+i].forEach(id=>$(id).classList.remove('on'));jogFin(i);}
    ['bh','bs'].forEach(id=>$(id).classList.toggle('off',!cal));
    const now=Date.now();
    C.forEach((c,i)=>{
      act[i]=j[c.k];
      if(sync[i]&&!drag[i]){ref[i]=act[i];
        if(!j.ocu&&!(jog&&jog.i===i)&&now>syncT[i])sync[i]=0;}
      $('e'+i).classList.toggle('off',i>0&&!cal);
      [['fi'+i,-1],['fd'+i,1]].forEach(([id,vis])=>{
        const s=C[i].inv?-vis:vis;
        const blq=j.bl[i]!==0&&s===j.bl[i],b=$(id);
        if(blq&&b.classList.contains('on')){b.classList.remove('on');jogFin(i);}
        b.classList.toggle('blq',blq);
      });
      pinta(i);
    });
    ['sw1','sw2','sw3'].forEach(k=>$(k).classList.toggle('on',j[k]==1));
    for(let g=0;g<2;g++)if(document.activeElement!==$('pr'+g))$('pr'+g).value=j.pr[g];
    $('msg').textContent=j.msg;
    if(j.ue>=0)$('ult').textContent=`Último: ${C[j.ue].n} ${j.ud.toFixed(1)}° en ${j.us.toFixed(2)} s · ${(j.us>0?j.ud/j.us:0).toFixed(1)}°/s`;
  }catch(e){$('msg').textContent='Sin conexión';}
}
async function ciclo(){await estado();setTimeout(ciclo,150);}
ciclo();
</script>
</body></html>
)HTML";

// ==================== Utilidades de escala ====================
float ppg(int eje) { return pulsosRev[grupo(eje)] * REDUCCION[eje] / 360.0f; }   // pasos por grado

// Distancias de homing de los eslabones, escaladas a su resolución
int32_t escPasos(int32_t pasos1000) {
  int32_t p = (int32_t)lround((double)pasos1000 * pulsosRev[1] / 1000.0);
  return p < 1 ? 1 : p;
}

float grados(int eje) { return pos[eje] / ppg(eje); }

// ==================== Generador de pasos (ISR) ====================
static inline uint32_t IRAM_ATTR isqrt32(uint32_t n) {
  uint32_t r = 0, b = 1UL << 30;
  while (b > n) b >>= 2;
  while (b) {
    if (n >= r + b) { n -= r + b; r = (r >> 1) + b; }
    else r >>= 1;
    b >>= 2;
  }
  return r;
}

// Velocidad (pasos/s) del eje j para el paso número 'hechos': v² = v0² + 2·a·s
static inline uint32_t IRAM_ATTR velocidadPaso(int j, int32_t hechos) {
  int32_t rest = e_total[j] - hechos;
  if (rest < 0) rest = 0;
  uint64_t s1 = (uint64_t)e_vMin2[j] + (uint64_t)e_a2[j] * (uint32_t)hechos;
  uint64_t s2 = (uint64_t)e_vMin2[j] + (uint64_t)e_a2[j] * (uint32_t)rest;
  uint64_t m = s1 < s2 ? s1 : s2;
  if (m > e_vMax2[j]) m = e_vMax2[j];
  uint32_t v = isqrt32((uint32_t)m);
  return v < 1 ? 1 : v;
}

void IRAM_ATTR tick() {
  // Los pulsos del tick anterior terminan (duran un tick en LOW)
  uint32_t eraBajo = pulBajo;
  if (eraBajo) { REG_WRITE(GPIO_OUT_W1TS_REG, eraBajo); pulBajo = 0; }
  uint32_t bajar = 0;

  for (int j = 0; j < 3; j++) {
    if (!e_activo[j]) continue;
    if (e_espera[j] > 0) { e_espera[j]--; continue; }

    e_acum[j] += e_v[j];
    if (e_acum[j] < TICK_HZ) continue;
    e_acum[j] -= TICK_HZ;

    // Toca un paso nuevo
    if (e_pedirAlto[j] && !e_frenando[j]) {         // paro con rampa
      e_frenando[j] = true;
      uint64_t v2 = (uint64_t)e_v[j] * e_v[j];
      uint32_t d = (v2 > e_vMin2[j]) ? (uint32_t)((v2 - e_vMin2[j]) / e_a2[j]) : 0;
      if (e_hechos[j] + (int32_t)d < e_total[j]) e_total[j] = e_hechos[j] + (int32_t)d;
    }
    if (e_hechos[j] >= e_total[j]) {
      e_resultado[j] = e_frenando[j] ? RES_ALTO : RES_OK;
      e_activo[j] = false;
      continue;
    }

    if (e_modo[j] != M_NADA) {
      bool activo = (REG_READ(GPIO_IN_REG) & e_mascaraSw[j]) != e_mascaraSw[j];   // algún switch en LOW
      bool cond = (e_modo[j] == M_LIBERAR) ? !activo : activo;
      if (cond) {
        if (e_confirmando[j]) {
          e_resultado[j] = (e_modo[j] == M_SEGURIDAD) ? RES_SEGURIDAD : RES_SWITCH;
          e_activo[j] = false;
          continue;
        }
        e_confirmando[j] = true;
        e_espera[j] = TICKS_CONFIRMA;
        e_acum[j] = TICK_HZ;                        // reintenta justo al terminar la espera
        continue;
      }
      e_confirmando[j] = false;
    }

    e_hechos[j]++;
    if (j == BASE) bajar |= 1UL << PUL3;
    else {
      if (j == CODO) bajar |= 1UL << PUL2;
      if (j == MUNECA || COMPENSAR_MUNECA) m1Pend += e_signo[j];   // lo saca el emisor de abajo
    }
    e_v[j] = velocidadPaso(j, e_hechos[j]);
    if (e_hechos[j] >= e_total[j]) {
      e_resultado[j] = e_frenando[j] ? RES_ALTO : RES_OK;
      e_activo[j] = false;
    }
  }

  // Emisor del motor del eslabón 2: un paso por tick como máximo y nunca en
  // dos ticks seguidos (el pulso necesita su tick en HIGH). Al cambiar de
  // sentido espera TICKS_DIR antes del siguiente pulso.
  if (m1Espera > 0) {
    m1Espera--;
  } else if (m1Pend != 0 && !(eraBajo & (1UL << PUL1))) {
    int8_t s = m1Pend > 0 ? 1 : -1;
    if (s != m1Dir) {
      bool nivel = (s > 0) ? DIR_POS_MUNECA : !DIR_POS_MUNECA;
      REG_WRITE(nivel ? GPIO_OUT_W1TS_REG : GPIO_OUT_W1TC_REG, 1UL << DIR1);
      m1Dir = s;
      m1Espera = TICKS_DIR;
    } else {
      bajar |= 1UL << PUL1;
      m1Pend -= s;
      posMunecaMotor += s;
    }
  }

  if (bajar) { REG_WRITE(GPIO_OUT_W1TC_REG, bajar); pulBajo = bajar; }
}

// ==================== Utilidades ====================
void setMsg(const char* t) {
  strncpy(mensaje, t, sizeof(mensaje) - 1);
  mensaje[sizeof(mensaje) - 1] = 0;
  Serial.print("MSG ");
  Serial.println(mensaje);
}

bool switchActivo(uint8_t pin) { return digitalRead(pin) == LOW; }

bool switchConfirmado(uint8_t pin) {
  if (!switchActivo(pin)) return false;
  delay(5);
  return switchActivo(pin);
}

uint32_t vigilarSwitches() {        // solo los que NO están activos ahora
  uint32_t v = 0;
  if (!switchActivo(SW2_PIN)) v |= 1UL << SW2_PIN;
  if (!switchActivo(SW3_PIN)) v |= 1UL << SW3_PIN;
  return v;
}

// Sentido prohibido por un switch activo (0 = libre)
int signoBloqueado(int eje) {
  if (eje == CODO   && switchActivo(SW2_PIN)) return HOME_SIGNO_CODO;
  if (eje == MUNECA && switchActivo(SW3_PIN)) return HOME_SIGNO_MUNECA;
  return 0;
}

bool bloqueado(int eje, int signo) {
  if (signoBloqueado(eje) != signo) return false;
  setMsg(eje == CODO ? "Bloqueado: SW2 activo, solo puede alejarse"
                     : "Bloqueado: SW3 activo, solo puede alejarse");
  rechazos++;
  return true;
}

uint8_t nivelDir(uint8_t nivelPos, int signo) { return signo > 0 ? nivelPos : !nivelPos; }

void limpiarCola() { for (int i = 0; i < 3; i++) hayObjetivo[i] = false; }

void leerBoton() {
  static bool ultimo = HIGH, estable = HIGH;
  static uint32_t t = 0;
  bool raw = digitalRead(BTN_PIN);
  if (raw != ultimo) { ultimo = raw; t = millis(); }
  if (millis() - t > 35 && raw != estable) {
    estable = raw;
    if (estable == LOW) { limpiarCola(); pedirRutina = 0; pedirJog = false; pedirAlto = true; pedirCalib = true; }
  }
}

void leerSerial();

void atender() {
  dns.processNextRequest();
  servidor.handleClient();
  leerSerial();
  leerBoton();
  static uint32_t tLed = 0;
  if (millis() - tLed > 250) {
    tLed = millis();
    digitalWrite(LED_PIN, WiFi.softAPgetStationNum() > 0 ? LOW : HIGH);
  }
}

void esperar(uint32_t ms) {
  uint32_t t0 = millis();
  while (millis() - t0 < ms) { atender(); delay(1); }
}

bool algunoAbierto() { return e_abierto[0] || e_abierto[1] || e_abierto[2]; }

void frenarTodos() { for (int j = 0; j < 3; j++) if (e_activo[j]) e_pedirAlto[j] = true; }

// Arranca el movimiento de una articulación y regresa enseguida
void arrancar(int eje, int signo, int32_t pasos, float rpm, float rpmS, uint8_t modo, uint32_t mSw) {
  if (mSw == 0) modo = M_NADA;

  float k = pulsosRev[grupo(eje)] / 60.0f;          // rpm -> pasos/s
  float fMax = rpm * k;
  if (fMax > TICK_HZ / 2) fMax = TICK_HZ / 2;
  if (fMax < 1.0f) fMax = 1.0f;
  float fMin = RPM_ARRANQUE * k;
  if (fMin > fMax) fMin = fMax;
  if (fMin < 1.0f) fMin = 1.0f;
  float fA = rpmS * k;
  if (fA < 1.0f) fA = 1.0f;
  uint32_t vMax = (uint32_t)fMax, vMin = (uint32_t)fMin, a = (uint32_t)fA;

  // DIR de la base y del eslabón 1; la del motor del eslabón 2 la pone la ISR
  if (eje == BASE) digitalWrite(DIR3, nivelDir(DIR_POS_BASE, signo));
  else if (eje == CODO) digitalWrite(DIR2, nivelDir(DIR_POS_CODO, signo));

  e_signo[eje] = signo;
  e_total[eje] = pasos;
  e_hechos[eje] = 0;
  e_vMax2[eje] = vMax * vMax;
  e_vMin2[eje] = vMin * vMin;
  e_a2[eje] = 2 * a;
  e_v[eje] = vMin;
  e_acum[eje] = TICK_HZ - vMin;                     // primer paso al acabar la espera de DIR
  e_modo[eje] = modo;
  e_mascaraSw[eje] = mSw;
  e_confirmando[eje] = false; e_frenando[eje] = false; e_pedirAlto[eje] = false;
  e_resultado[eje] = RES_NADA;
  e_espera[eje] = TICKS_DIR;
  e_abierto[eje] = true;
  e_activo[eje] = true;                             // al final: la ISR ya ve todo listo
}

// Suma a pos lo que avanzó el movimiento terminado y devuelve su resultado
uint8_t cerrarEje(int eje) {
  pos[eje] += (long)e_signo[eje] * e_hechos[eje];
  e_abierto[eje] = false;
  return e_resultado[eje];
}

// Mueve una articulación y espera a que termine, manteniendo viva la página.
// Lo usan la calibración, las rutinas y el jog (un eje a la vez).
uint8_t moverEje(int eje, int signo, int32_t pasos, float rpm, float rpmS,
                 uint8_t modo, uint32_t mSw) {
  if (pasos <= 0) return RES_OK;
  arrancar(eje, signo, pasos, rpm, rpmS, modo, mSw);
  while (e_activo[eje]) {
    atender();
    if (pedirAlto || (enJog && (jogParar || millis() - ultimoLatido > LATIDO_MS)))
      e_pedirAlto[eje] = true;
    delay(1);
  }
  uint32_t t0 = millis();
  while (m1Pend != 0 && millis() - t0 < 50) delay(1);   // el motor del eslabón 2 termina sus pasos
  return cerrarEje(eje);
}

long posVivo(int eje) {
  long p = pos[eje];
  if (e_abierto[eje]) p += (long)e_signo[eje] * e_hechos[eje];
  return p;
}

void registrarUltimo(int eje, float d0, uint32_t t0) {
  ultEje = eje;
  ultDeg = fabsf(grados(eje) - d0);
  ultSeg = (millis() - t0) / 1000.0f;
}

// ==================== Calibración ====================
bool falla(const char* nombre, const char* motivo) {
  char t[96];
  snprintf(t, sizeof(t), "Error %s: %s", nombre, motivo);
  setMsg(t);
  return false;
}

bool homeEje(int eje, uint8_t swPin, int signoHome, const char* nombre) {
  uint32_t msk = 1UL << swPin;
  int lejos = -signoHome;
  uint8_t r;

  if (switchConfirmado(swPin)) {
    r = moverEje(eje, lejos, escPasos(MAX_LIBERAR), HOME_LENTO_RPM, HOME_ACEL, M_LIBERAR, msk);
    if (r == RES_ALTO) return falla(nombre, "cancelado");
    if (r != RES_SWITCH) return falla(nombre, "no se libera el switch");
  }

  r = moverEje(eje, signoHome, escPasos(MAX_BUSQUEDA), HOME_RAPIDO_RPM, HOME_ACEL, M_ACTIVAR, msk);
  if (r == RES_ALTO) return falla(nombre, "cancelado");
  if (r != RES_SWITCH) return falla(nombre, "no encuentra el switch");

  r = moverEje(eje, lejos, escPasos(BACKOFF), HOME_LENTO_RPM, HOME_ACEL, M_NADA, 0);
  if (r == RES_ALTO) return falla(nombre, "cancelado");

  r = moverEje(eje, signoHome, escPasos(BACKOFF + 300), HOME_LENTO_RPM, HOME_ACEL, M_ACTIVAR, msk);
  if (r == RES_ALTO) return falla(nombre, "cancelado");
  if (r != RES_SWITCH) return falla(nombre, "falla la busqueda fina");

  r = moverEje(eje, lejos, escPasos(CLEARANCE), HOME_LENTO_RPM, HOME_ACEL, M_NADA, 0);
  if (r == RES_ALTO) return falla(nombre, "cancelado");
  return true;
}

void calibrar() {
  pedirCalib = false;
  pedirAlto = false;
  pedirJog = false;
  limpiarCola();
  bool previo = calibrado;
  calibrado = false;
  ocupado = true;

  setMsg("Calibrando eslabón 1...");
  if (!homeEje(CODO, SW2_PIN, HOME_SIGNO_CODO, "eslabón 1")) { ocupado = false; return; }
  float deriva1 = (pos[CODO] - escPasos(CLEARANCE)) / ppg(CODO);
  pos[CODO] = escPasos(CLEARANCE);

  esperar(300);

  setMsg("Calibrando eslabón 2...");
  if (!homeEje(MUNECA, SW3_PIN, HOME_SIGNO_MUNECA, "eslabón 2")) { ocupado = false; return; }
  float deriva2 = (pos[MUNECA] + escPasos(CLEARANCE)) / ppg(MUNECA);
  pos[MUNECA] = -escPasos(CLEARANCE);
  posMunecaMotor = -escPasos(CLEARANCE);

  calibrado = true;
  ocupado = false;
  char t[96];
  // La deriva solo tiene sentido si ya estaba calibrado: mide pasos perdidos
  if (previo) snprintf(t, sizeof(t), "Calibrado · deriva E1 %+.2f° E2 %+.2f°", deriva1, deriva2);
  else snprintf(t, sizeof(t), "Calibrado");
  setMsg(t);
}

// ==================== Movimientos ====================
// Mueve una articulación a un ángulo absoluto. Sin mensajes: lo usan irA y las rutinas.
uint8_t moverA(int eje, float deg) {
  if (deg < LIM_MIN[eje]) deg = LIM_MIN[eje];
  if (deg > LIM_MAX[eje]) deg = LIM_MAX[eje];
  long delta = lroundf(deg * ppg(eje)) - pos[eje];
  if (delta == 0) return RES_OK;
  int signo = delta > 0 ? 1 : -1;
  if (signoBloqueado(eje) == signo) return RES_BLOQ;
  int g = grupo(eje);
  uint32_t t0 = millis();
  float d0 = grados(eje);
  uint8_t r = moverEje(eje, signo, labs(delta), velRpm[g], acelRpmS[g], M_SEGURIDAD, vigilarSwitches());
  registrarUltimo(eje, d0, t0);
  return r;
}

void mensajeResultado(int eje, uint8_t r, const char* ok) {
  if (r == RES_BLOQ) {
    setMsg(eje == CODO ? "Bloqueado: SW2 activo, solo puede alejarse"
                       : "Bloqueado: SW3 activo, solo puede alejarse");
    rechazos++;
  } else if (r == RES_SEGURIDAD) { setMsg("Detenido por final de carrera"); rechazos++; }
  else if (r == RES_ALTO) setMsg("Detenido");
  else setMsg(ok);
}

// ==================== Seguimiento (GOTO / POSE) ====================
// Cada articulación con objetivo arranca en cuanto está libre, sin esperar a
// las demás. Si llega otro objetivo para un eje que ya se mueve, se alarga o
// acorta el movimiento al vuelo; si ya no alcanza a frenar o el objetivo quedó
// atrás, frena con rampa y el objetivo se ejecuta después (como v7).
float d0Eje[3];
uint32_t t0Eje[3];

void corregirDestino(int e) {
  if (e_frenando[e]) return;
  float deg = objetivo[e];
  if (deg < LIM_MIN[e]) deg = LIM_MIN[e];
  if (deg > LIM_MAX[e]) deg = LIM_MAX[e];
  long rel = (lroundf(deg * ppg(e)) - pos[e]) * e_signo[e];   // pasos desde el inicio, mismo sentido
  uint64_t v2 = (uint64_t)e_v[e] * e_v[e];
  int32_t freno = (v2 > e_vMin2[e]) ? (int32_t)((v2 - e_vMin2[e]) / e_a2[e]) : 0;
  if (rel >= (long)e_hechos[e] + freno + 1) {
    e_total[e] = (int32_t)rel;
    if (e_activo[e]) hayObjetivo[e] = false;        // si ya había terminado, queda pendiente
  } else {
    e_pedirAlto[e] = true;
  }
}

// puedeArrancar = false mientras hay un ALTO o algo exclusivo esperando
void servirObjetivos(bool puedeArrancar) {
  bool termino = false;
  for (int e = 0; e < 3; e++) {
    if (e_abierto[e] && !e_activo[e]) {
      uint8_t r = cerrarEje(e);
      registrarUltimo(e, d0Eje[e], t0Eje[e]);
      termino = true;
      if (r == RES_SEGURIDAD) { setMsg("Detenido por final de carrera"); rechazos++; }
      else if (r == RES_ALTO && pedirAlto) setMsg("Detenido");
      // RES_ALTO sin ALTO = frenó por cambio de objetivo: el siguiente sigue en cola
    }
    if (e_activo[e]) {
      if (hayObjetivo[e] && puedeArrancar) corregirDestino(e);
      continue;
    }
    if (!hayObjetivo[e] || !puedeArrancar) continue;
    hayObjetivo[e] = false;
    if (e != BASE && !calibrado) { setMsg("Calibra primero"); rechazos++; continue; }
    float deg = objetivo[e];
    if (deg < LIM_MIN[e]) deg = LIM_MIN[e];
    if (deg > LIM_MAX[e]) deg = LIM_MAX[e];
    long delta = lroundf(deg * ppg(e)) - pos[e];
    if (delta == 0) continue;
    int signo = delta > 0 ? 1 : -1;
    if (bloqueado(e, signo)) continue;
    int g = grupo(e);
    d0Eje[e] = grados(e);
    t0Eje[e] = millis();
    arrancar(e, signo, labs(delta), velRpm[g], acelRpmS[g], M_SEGURIDAD, vigilarSwitches());
    if (strcmp(mensaje, "Moviendo...")) setMsg("Moviendo...");
  }
  if (termino && !algunoAbierto() && !hayObjetivo[0] && !hayObjetivo[1] && !hayObjetivo[2] &&
      !strcmp(mensaje, "Moviendo..."))
    setMsg("Listo");
}

// ==================== Rutinas ====================
float homeDeg(int eje) {
  if (eje == CODO)   return  escPasos(CLEARANCE) / ppg(CODO);
  if (eje == MUNECA) return -escPasos(CLEARANCE) / ppg(MUNECA);
  return 0.0f;
}

// Devuelve RES_OK o el primer resultado distinto; ejeFalla indica dónde falló
uint8_t irHome(int& ejeFalla) {
  for (int k = 0; k < 3; k++) {
    int eje = ORDEN_HOME[k];
    uint8_t r = moverA(eje, homeDeg(eje));
    if (r != RES_OK) { ejeFalla = eje; return r; }
  }
  return RES_OK;
}

uint8_t pausa(uint32_t ms) {
  esperar(ms);
  return pedirAlto ? RES_ALTO : RES_OK;
}

bool algunoActivo() { return e_activo[0] || e_activo[1] || e_activo[2]; }

// Lleva a la vez los ejes que no son NAN a su ángulo y espera a que lleguen
// todos. Devuelve RES_OK o el primer resultado distinto (ejeFalla = dónde).
uint8_t moverPose(const float q[3], int& ejeFalla) {
  long delta[3] = { 0, 0, 0 };
  for (int e = 0; e < 3; e++) {
    if (isnan(q[e])) continue;
    float deg = q[e];
    if (deg < LIM_MIN[e]) deg = LIM_MIN[e];
    if (deg > LIM_MAX[e]) deg = LIM_MAX[e];
    delta[e] = lroundf(deg * ppg(e)) - pos[e];
    if (delta[e] && signoBloqueado(e) == (delta[e] > 0 ? 1 : -1)) { ejeFalla = e; return RES_BLOQ; }
  }
  uint32_t mSw = vigilarSwitches();
  for (int e = 0; e < 3; e++) {
    if (!delta[e]) continue;
    int g = grupo(e);
    arrancar(e, delta[e] > 0 ? 1 : -1, labs(delta[e]), velRpm[g], acelRpmS[g], M_SEGURIDAD, mSw);
  }
  while (algunoActivo()) {
    atender();
    if (pedirAlto) frenarTodos();
    delay(1);
  }
  uint32_t t0 = millis();
  while (m1Pend != 0 && millis() - t0 < 50) delay(1);   // el motor del eslabón 2 termina sus pasos
  uint8_t peor = RES_OK;
  for (int e = 0; e < 3; e++) {
    if (!e_abierto[e]) continue;
    uint8_t r = cerrarEje(e);
    if (r != RES_OK && peor == RES_OK) { peor = r; ejeFalla = e; }
  }
  return peor;
}

int buscarRutina(const char* comando) {
  if (!comando) return -1;
  for (int i = 0; i < N_RUTINAS; i++) if (!strcasecmp(comando, RUTINAS[i].comando)) return i;
  return -1;
}

// HOME -> pasos de la rutina (los ejes de cada paso a la vez) -> HOME
void rutinaPasos(int i) {
  const Rutina& ru = RUTINAS[i];
  int eje = CODO;
  char t[96];
  snprintf(t, sizeof(t), "%s: yendo a HOME...", ru.titulo);
  setMsg(t);
  uint8_t r = irHome(eje);
  for (uint8_t k = 0; k < ru.n && r == RES_OK; k++) {
    snprintf(t, sizeof(t), "%s %d/%d", ru.titulo, k + 1, ru.n);
    setMsg(t);
    const PasoRutina& p = ru.pasos[k];
    const float q[3] = { p.b, p.e1, p.e2 };
    r = moverPose(q, eje);
    if (r == RES_OK && p.ms) r = pausa(p.ms);
  }
  if (r == RES_OK) {
    snprintf(t, sizeof(t), "%s: regresando a HOME...", ru.titulo);
    setMsg(t);
    r = irHome(eje);
  }
  ocupado = false;
  snprintf(t, sizeof(t), "%s terminado", ru.titulo);
  mensajeResultado(eje, r, t);
}

void rutina(int n) {
  limpiarCola();
  if (!calibrado) { setMsg("Calibra primero"); rechazos++; return; }
  ocupado = true;
  int eje = CODO;
  uint8_t r;
  char t[96];

  if (n >= RUTINA_PASOS && n < RUTINA_PASOS + N_RUTINAS) { rutinaPasos(n - RUTINA_PASOS); return; }

  if (n == 1) {
    setMsg("Regresando a HOME...");
    r = irHome(eje);
    ocupado = false;
    mensajeResultado(eje, r, "En HOME");
    return;
  }

  setMsg("Saludo: yendo a HOME...");
  r = irHome(eje);
  if (r == RES_OK) {
    setMsg("Saludo: eslabón 1");
    eje = CODO;
    r = moverA(CODO, SALUDO_E1);
  }
  for (int c = 1; c <= SALUDO_CICLOS && r == RES_OK; c++) {
    snprintf(t, sizeof(t), "Saludo %d/%d", c, SALUDO_CICLOS);
    setMsg(t);
    eje = MUNECA;
    r = moverA(MUNECA, SALUDO_E2_A);
    if (r == RES_OK) r = pausa(SALUDO_PAUSA_MS);
    if (r == RES_OK) r = moverA(MUNECA, SALUDO_E2_B);
    if (r == RES_OK) r = pausa(SALUDO_PAUSA_MS);
  }
  if (r == RES_OK) {
    setMsg("Saludo: regresando a HOME...");
    r = irHome(eje);
  }
  ocupado = false;
  mensajeResultado(eje, r, "Saludo terminado");
}

void jog(int eje, int signo) {
  if (millis() - ultimoLatido > LATIDO_MS) return;
  if (bloqueado(eje, signo)) return;
  int g = grupo(eje);
  bool conLimite = (eje == BASE) || calibrado;
  float rpm = velRpm[g];
  long destino;
  if (conLimite) {
    destino = lroundf((signo > 0 ? LIM_MAX[eje] : LIM_MIN[eje]) * ppg(eje));
  } else {
    destino = pos[eje] + signo * lroundf(JOG_SIN_CAL_DEG * ppg(eje));
    if (rpm > JOG_SIN_CAL_RPM) rpm = JOG_SIN_CAL_RPM;
  }
  long delta = destino - pos[eje];
  if (delta * signo <= 0) { setMsg("En el límite"); return; }

  enJog = true;
  jogParar = false;
  ocupado = true;
  setMsg("Jog");
  uint32_t t0 = millis();
  float d0 = grados(eje);
  uint8_t r = moverEje(eje, signo, labs(delta), rpm, acelRpmS[g], M_SEGURIDAD, vigilarSwitches());
  enJog = false;
  ocupado = false;
  registrarUltimo(eje, d0, t0);

  if (r == RES_SEGURIDAD) { setMsg("Detenido por final de carrera"); rechazos++; }
  else if (r == RES_OK && conLimite) setMsg("En el límite");
  else setMsg("Listo");
}

void cambiarPulsos(int g, int nuevo) {
  if (nuevo == pulsosRev[g]) return;
  double k = (double)nuevo / pulsosRev[g];       // conserva los ángulos
  if (g == 0) {
    pos[BASE] = lround(pos[BASE] * k);
  } else {
    pos[CODO] = lround(pos[CODO] * k);
    pos[MUNECA] = lround(pos[MUNECA] * k);
    posMunecaMotor = lround(posMunecaMotor * k);
  }
  pulsosRev[g] = nuevo;
  char t[96];
  snprintf(t, sizeof(t), "%s: %d pulsos/rev (ajusta el CL57T igual)",
           g == 0 ? "Base" : "Eslabones", nuevo);
  setMsg(t);
}

// ==================== Servidor ====================
void hPagina() {
  servidor.sendHeader("Cache-Control", "no-store");
  servidor.send(200, "text/html; charset=utf-8", PAGINA);
}

void estadoJSON(char* buf, size_t n) {
  bool ocu = ocupado || algunoAbierto() || pedirJog || pedirCalib ||
             hayObjetivo[0] || hayObjetivo[1] || hayObjetivo[2] || pedirRutina;
  snprintf(buf, n,
    "{\"b\":%.2f,\"c\":%.2f,\"m\":%.2f,"
    "\"sw1\":%d,\"sw2\":%d,\"sw3\":%d,\"cal\":%d,\"ocu\":%d,\"msg\":\"%s\","
    "\"lim\":[%.1f,%.1f,%.1f,%.1f,%.1f,%.1f],\"pr\":[%d,%d],"
    "\"vel\":[%.0f,%.0f],\"acel\":[%.0f,%.0f],\"vr\":[%.0f,%.0f,%.0f,%.0f,%.0f,%.0f],"
    "\"bl\":[0,%d,%d],\"rs\":%lu,"
    "\"ue\":%d,\"ud\":%.2f,\"us\":%.2f,\"src\":%d}",
    posVivo(BASE) / ppg(BASE), posVivo(CODO) / ppg(CODO), posVivo(MUNECA) / ppg(MUNECA),
    switchActivo(SW1_PIN), switchActivo(SW2_PIN), switchActivo(SW3_PIN),
    calibrado, ocu, mensaje,
    LIM_MIN[0], LIM_MAX[0], LIM_MIN[1], LIM_MAX[1], LIM_MIN[2], LIM_MAX[2],
    pulsosRev[0], pulsosRev[1],
    velRpm[0], velRpm[1], acelRpmS[0], acelRpmS[1],
    VEL_MIN, VEL_MAX[0], VEL_MAX[1], ACEL_MIN, ACEL_MAX[0], ACEL_MAX[1],
    signoBloqueado(CODO), signoBloqueado(MUNECA), (unsigned long)rechazos,
    ultEje, ultDeg, ultSeg, (int)fuente);
}

void hEstado() {
  char buf[720];
  estadoJSON(buf, sizeof(buf));
  servidor.sendHeader("Cache-Control", "no-store");
  servidor.send(200, "application/json", buf);
}

// ==================== Acciones (web y serial) ====================
void accionAlto() {
  limpiarCola();
  pedirRutina = 0;
  pedirJog = false;
  pedirAlto = true;
}

void accionCalibrar() {
  limpiarCola();
  pedirRutina = 0;
  pedirJog = false;
  pedirAlto = true;
  pedirCalib = true;
}

void accionVel(int g, bool hayV, float v, bool hayA, float a) {
  if (hayV) velRpm[g] = v < VEL_MIN ? VEL_MIN : (v > VEL_MAX[g] ? VEL_MAX[g] : v);
  if (hayA) acelRpmS[g] = a < ACEL_MIN ? ACEL_MIN : (a > ACEL_MAX[g] ? ACEL_MAX[g] : a);
}

bool accionPulsos(int g, int v) {
  for (int op : PULSOS_OPCIONES) if (op == v) { pedirPulsos[g] = v; return true; }
  return false;
}

// Rechaza en la web los comandos de movimiento cuando manda la PC
bool webPuedeMover() {
  if (fuente == FUENTE_WEB) return true;
  servidor.send(403, "text/plain", "modo PC");
  return false;
}

void hIr() {
  if (!webPuedeMover()) return;
  int e = servidor.arg("e").toInt();
  if (e >= 0 && e <= 2 && servidor.hasArg("v")) {
    objetivo[e] = servidor.arg("v").toFloat();
    hayObjetivo[e] = true;
  }
  servidor.send(200, "text/plain", "ok");
}

void hJog() {
  if (!webPuedeMover()) return;
  int e = servidor.arg("e").toInt();
  int d = servidor.arg("d").toInt();
  if (e >= 0 && e <= 2) {
    if (d == 0) {
      jogParar = true;
      pedirJog = false;
    } else {
      int s = d > 0 ? 1 : -1;
      ultimoLatido = millis();
      if (!(enJog && jogEje == e && jogSigno == s)) {
        if (enJog) jogParar = true;                  // cambio de eje o sentido: frena el actual
        jogEje = e;
        jogSigno = s;
        pedirJog = true;
      }
    }
  }
  servidor.send(200, "text/plain", "ok");
}

void hVel() {
  int g = servidor.arg("g").toInt();
  if (g < 0 || g > 1) { servidor.send(400, "text/plain", "g"); return; }
  accionVel(g, servidor.hasArg("v"), servidor.arg("v").toFloat(),
               servidor.hasArg("a"), servidor.arg("a").toFloat());
  servidor.send(200, "text/plain", "ok");
}

void hCalibrar() {
  if (!webPuedeMover()) return;
  accionCalibrar();
  servidor.send(200, "text/plain", "ok");
}

// ALTO desde la página siempre funciona y además quita el control a la PC
void hAlto() {
  accionAlto();
  if (fuente != FUENTE_WEB) { fuente = FUENTE_WEB; setMsg("ALTO: control pasa a la web"); }
  servidor.send(200, "text/plain", "ok");
}

void hModo() {
  String m = servidor.arg("m");
  if (m == "pc" && fuente != FUENTE_PC) {
    accionAlto();
    fuente = FUENTE_PC;
    setMsg("Control: PC");
  } else if (m == "web" && fuente != FUENTE_WEB) {
    accionAlto();
    fuente = FUENTE_WEB;
    setMsg("Control: web");
  }
  servidor.send(200, "text/plain", "ok");
}

void hPulsos() {
  int g = servidor.arg("g").toInt();
  int v = servidor.arg("v").toInt();
  if (g >= 0 && g <= 1) accionPulsos(g, v);
  servidor.send(200, "text/plain", "ok");
}

void hRutina() {
  if (!webPuedeMover()) return;
  String n = servidor.arg("n");
  if (n == "home") pedirRutina = 1;
  else if (n == "saludo") pedirRutina = 2;
  servidor.send(200, "text/plain", "ok");
}

void hRedirigir() {
  servidor.sendHeader("Location", "http://192.168.4.1/", true);
  servidor.send(302, "text/plain", "");
}

// ==================== Protocolo serial (USB) ====================
int parseEje(const char* t) {
  if (!t) return -1;
  if (!strcasecmp(t, "B") || !strcasecmp(t, "BASE") || !strcmp(t, "0")) return BASE;
  if (!strcasecmp(t, "E1") || !strcmp(t, "1")) return CODO;
  if (!strcasecmp(t, "E2") || !strcmp(t, "2")) return MUNECA;
  return -1;
}

int parseGrupo(const char* t) {
  if (!t) return -1;
  if (!strcasecmp(t, "B") || !strcasecmp(t, "BASE") || !strcmp(t, "0")) return 0;
  if (!strcasecmp(t, "E") || !strcasecmp(t, "ESL") || !strcmp(t, "1")) return 1;
  return -1;
}

bool esNA(const char* t) {
  return !strcasecmp(t, "NA") || !strcasecmp(t, "X") || !strcmp(t, "-");
}

bool parseNum(const char* t, float& v) {
  if (!t) return false;
  char* fin;
  v = strtof(t, &fin);
  return fin != t && *fin == 0;
}

// Valida y encola un objetivo; devuelve texto de error o nullptr
const char* encolarObjetivo(int e, float deg) {
  if (e != BASE && !calibrado) return "ERR NO_CALIBRADO";
  if (deg < LIM_MIN[e] || deg > LIM_MAX[e]) return "ERR FUERA_DE_LIMITE";
  objetivo[e] = deg;
  hayObjetivo[e] = true;
  return nullptr;
}

void procesarLinea(char* l) {
  char* cmd = strtok(l, " \t");
  if (!cmd) return;
  for (char* p = cmd; *p; p++) *p = toupper(*p);
  char* a1 = strtok(nullptr, " \t");
  char* a2 = strtok(nullptr, " \t");
  char* a3 = strtok(nullptr, " \t");

  // ---- siempre permitidos ----
  if (!strcmp(cmd, "PING")) { Serial.println("PONG"); return; }
  if (!strcmp(cmd, "STATE?")) {
    char buf[720];
    estadoJSON(buf, sizeof(buf));
    Serial.print("STATE ");
    Serial.println(buf);
    return;
  }
  if (!strcmp(cmd, "MODE?")) { Serial.println(fuente == FUENTE_PC ? "MODE PC" : "MODE WEB"); return; }
  if (!strcmp(cmd, "STOP")) { accionAlto(); Serial.println("OK STOP"); return; }
  if (!strcmp(cmd, "HELP") || !strcmp(cmd, "?")) {
    Serial.println("OK PING STATE? MODE? STOP CAL HOME SALUDO RUTINA GOTO POSE VEL PULSOS");
    return;
  }
  if (!strcmp(cmd, "VEL")) {
    int g = parseGrupo(a1);
    float v, a;
    bool hv = parseNum(a2, v), ha = parseNum(a3, a);
    if (g < 0 || !hv) { Serial.println("ERR ARGS"); return; }
    accionVel(g, true, v, ha, a);
    Serial.printf("OK VEL %s %.0f %.0f\n", g ? "E" : "B", velRpm[g], acelRpmS[g]);
    return;
  }
  if (!strcmp(cmd, "PULSOS")) {
    int g = parseGrupo(a1);
    if (g < 0 || !a2 || !accionPulsos(g, atoi(a2))) { Serial.println("ERR ARGS"); return; }
    Serial.println("OK PULSOS");
    return;
  }
  if (!strcmp(cmd, "GRIP")) { Serial.println("ERR NO_IMPLEMENTADO"); return; }

  // ---- movimiento: solo con control PC ----
  bool mov = !strcmp(cmd, "CAL") || !strcmp(cmd, "HOME") || !strcmp(cmd, "SALUDO") || !strcmp(cmd, "RUTINA") ||
             !strcmp(cmd, "GOTO") || !strcmp(cmd, "POSE");
  if (!mov) { Serial.println("ERR COMANDO"); return; }
  if (fuente != FUENTE_PC) { Serial.println("ERR MODO_WEB"); return; }

  if (!strcmp(cmd, "CAL"))    { accionCalibrar(); Serial.println("OK CAL"); return; }
  if (!strcmp(cmd, "HOME"))   { pedirRutina = 1; Serial.println("OK HOME"); return; }
  if (!strcmp(cmd, "SALUDO")) { pedirRutina = 2; Serial.println("OK SALUDO"); return; }
  if (!strcmp(cmd, "RUTINA")) {
    int i = buscarRutina(a1);
    if (i < 0) { Serial.println("ERR ARGS"); return; }
    pedirRutina = RUTINA_PASOS + i;
    Serial.println("OK RUTINA");
    return;
  }

  if (!strcmp(cmd, "GOTO")) {
    int e = parseEje(a1);
    float deg;
    if (e < 0 || !parseNum(a2, deg)) { Serial.println("ERR ARGS"); return; }
    const char* err = encolarObjetivo(e, deg);
    Serial.println(err ? err : "OK GOTO");
    return;
  }

  if (!strcmp(cmd, "POSE")) {
    char* t[3] = { a1, a2, a3 };
    float v[3];
    bool usar[3];
    for (int e = 0; e < 3; e++) {
      if (!t[e]) { Serial.println("ERR ARGS"); return; }
      usar[e] = !esNA(t[e]);
      if (usar[e] && !parseNum(t[e], v[e])) { Serial.println("ERR ARGS"); return; }
      if (usar[e] && e != BASE && !calibrado) { Serial.println("ERR NO_CALIBRADO"); return; }
      if (usar[e] && (v[e] < LIM_MIN[e] || v[e] > LIM_MAX[e])) { Serial.println("ERR FUERA_DE_LIMITE"); return; }
    }
    for (int e = 0; e < 3; e++) if (usar[e]) encolarObjetivo(e, v[e]);
    Serial.println("OK POSE");
    return;
  }
}

void leerSerial() {
  static char buf[128];
  static uint8_t n = 0;
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\r') continue;
    if (c == '\n') {
      buf[n] = 0;
      if (n) procesarLinea(buf);
      n = 0;
    } else if (n < sizeof(buf) - 1) {
      buf[n++] = c;
    }
  }
}

// ==================== Setup / Loop ====================
void setup() {
  Serial.begin(115200);
  Serial.setTxTimeoutMs(0);   // si no hay PC conectada, no se bloquea al imprimir

  const uint8_t puls[] = { PUL1, PUL2, PUL3 };
  const uint8_t dirs[] = { DIR1, DIR2, DIR3 };
  for (int i = 0; i < 3; i++) {
    pinMode(puls[i], OUTPUT); digitalWrite(puls[i], HIGH);   // reposo en HIGH
    pinMode(dirs[i], OUTPUT); digitalWrite(dirs[i], HIGH);
  }
  pinMode(SW1_PIN, INPUT_PULLUP);
  pinMode(SW2_PIN, INPUT_PULLUP);
  pinMode(SW3_PIN, INPUT_PULLUP);
  pinMode(BTN_PIN, INPUT_PULLUP);
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);

  WiFi.mode(WIFI_AP);
  WiFi.softAPConfig(IP_AP, IP_AP, IPAddress(255, 255, 255, 0));
  WiFi.softAP(NOMBRE_RED, CLAVE_RED, 6);
  WiFi.setTxPower(WIFI_POWER_8_5dBm);
  WiFi.setSleep(false);

  dns.start(53, "*", IP_AP);
  servidor.on("/", hPagina);
  servidor.on("/estado", hEstado);
  servidor.on("/ir", hIr);
  servidor.on("/jog", hJog);
  servidor.on("/vel", hVel);
  servidor.on("/calibrar", hCalibrar);
  servidor.on("/alto", hAlto);
  servidor.on("/pulsos", hPulsos);
  servidor.on("/rutina", hRutina);
  servidor.on("/modo", hModo);
  servidor.onNotFound(hRedirigir);
  servidor.begin();

  timer = timerBegin(1000000);                  // 1 MHz
  timerAttachInterrupt(timer, &tick);
  timerAlarm(timer, 1000000 / TICK_HZ, true, 0);

  setMsg("Sin calibrar");
}

void loop() {
  atender();

  // GOTO / POSE: los tres ejes a la vez
  if (pedirAlto) frenarTodos();
  bool exclusivo = pedirCalib || pedirRutina || pedirJog || pedirPulsos[0] || pedirPulsos[1];
  servirObjetivos(!pedirAlto && !exclusivo);
  bool enMovimiento = algunoAbierto();
  if (pedirAlto && !enMovimiento) pedirAlto = false;

  // Calibración, rutinas, jog y cambio de pulsos van solos: esperan a que
  // paren los ejes
  if (!exclusivo) { delay(1); return; }
  if (enMovimiento) { frenarTodos(); delay(1); return; }

  if (pedirCalib) { calibrar(); return; }

  for (int g = 0; g < 2; g++) {
    if (pedirPulsos[g]) {
      int v = pedirPulsos[g];
      pedirPulsos[g] = 0;
      cambiarPulsos(g, v);
      return;
    }
  }

  if (pedirRutina) {
    int n = pedirRutina;
    pedirRutina = 0;
    rutina(n);
    return;
  }

  if (pedirJog) {
    pedirJog = false;
    jog(jogEje, jogSigno);
  }
}
