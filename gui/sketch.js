/* MoRo 2025-2 @ Hongik Univ.
p5.js 2-DOF Robot Arm with PID / Impedance / Admittance + IK + Web Serial
===
Kyung Yun Choi
===
Hobby servos have no position feedback (encoder) and no torque control, so PID and impedance
loops cannot really be closed on the hardware. Instead, the GUI simulates the arm dynamics
(inertia, Coriolis, friction, gravity, motor torque-speed limit) at 1 kHz, and the controllers
compute joint *torques* that drive this model.
With "Follow Sim" ON, the simulated joint angles are sent to the servos, so the real arm
reproduces the simulated response.

Mouse (canvas):
  Right-drag         move the red target (Joint mode: sets joint targets through IK)
  Shift + left-drag  pull the end effector with a virtual spring (external force -> feel stiffness)
  Left-drag          rotate view (pan when PAN is checked),  wheel: zoom,  double-click: reset view
*/

let arm;
let ui = {};
let serial;
let hud;

// === Simulation ===
const PHYS_DT = 0.001;            // physics + control period (1 kHz)
const GRAV = 9.81;
const D2R = Math.PI / 180, R2D = 180 / Math.PI;
const PX = 1000;                  // render scale [px/m]
let simTime = 0, simAcc = 0;

const clamp = (v, lo, hi) => Math.min(hi, Math.max(lo, v));
const $ = id => document.getElementById(id);
function gauss(){
  let u = 0; while(u === 0) u = Math.random();
  return Math.sqrt(-2 * Math.log(u)) * Math.cos(2 * Math.PI * Math.random());
}

// === User settings (updated by the UI) ===
let cfg = {
  mode: 'ik',            // 'joint' | 'ik'
  ctrl: 'pid',           // controller in IK mode: 'pid' | 'imp' | 'impF' | 'adm'
  elbowUp: true,
  gravity: false,        // vertical-plane mounting (gravity along -y)
  payload: 0,            // tip payload [kg] (unknown to the controller model)
  noiseDeg: 0,           // angle sensor noise std [deg]
  dCutHz: 50,            // low-pass cutoff of the velocity estimate (D term) [Hz]
  imp: { Kx: 300, Ky: 300, B: 10, M: 0.1 },   // [N/m], [N/m], [N s/m], [kg]
  showEllipse: true, showTrail: true, showGhost: true, shading: true,
};

const CTRL_INFO = {
  pid:  ['IK + Joint PID',
         'IK turns the target into joint angles, and each joint PID outputs a torque. Motion happens in ' +
         'joint space, so the tip path curves and the tip stiffness (green ellipse) changes with the pose.'],
  imp:  ['Impedance (no F/T)',
         'τ = Jᵀ[K(x_d − x) − B ẋ] + g(q). A virtual spring-damper attached to the tip. Works without a ' +
         'force sensor, but the apparent mass is the arm\'s own inertia, so M is not used.'],
  impF: ['Impedance + F/T (M shaping)',
         'A (simulated) F/T sensor measures the external force, so the inertia can be reshaped to M as well ' +
         '(computed torque). Small M feels light, large M feels heavy. Hits the torque limit easily.'],
  adm:  ['Admittance (position servo + F/T)',
         'The measured force drives a virtual M-B-K reference point x_c (cyan) → IK → joint PID. This is the ' +
         'scheme usable on real hobby servos without torque control (with Follow Sim OFF, x_c goes to the servos).'],
};

// === Mouse interaction ===
let view = { rx: 0, rz: 0, zoom: 1, panX: 0, panY: 0 };
let isPanning = false;
let drag = null;                  // 'target' | 'poke' | 'orbit' | 'pan'
let interact = {
  poke: null,                     // point grabbed by the mouse {x,y} [m]
  kPoke: 80, fPokeMax: 3,         // virtual spring [N/m], max force [N]
  kick: null,                     // {t, F:[Fx,Fy]} short impulse force
};

// === Hardware mapping (servo 2 at 90° is kinematic 0°) ===
const SERVO_MIN = 0;
const SERVO_MAX = 180;
const J1_OFFSET = 0;    // servo 1:  cmd =  θ1
const J2_OFFSET = 90;   // servo 2:  cmd = 90 + θ2

// Joint limits [deg] = servo range 0..180 expressed in kinematic angles
const J_LIM = [[SERVO_MIN - J1_OFFSET, SERVO_MAX - J1_OFFSET],
               [SERVO_MIN - J2_OFFSET, SERVO_MAX - J2_OFFSET]];
const limitDeg = (deg) => deg.map((d, i) => clamp(d, J_LIM[i][0], J_LIM[i][1]));

// Servo pulse width [us] (must match PULSE_MIN / PULSE_MAX in src/main.cpp). 1 us ≈ 0.095°
const PULSE_MIN = 500;
const PULSE_MAX = 2400;

function servoUs(angleDeg, offset) {
  const deg = clamp(offset + angleDeg, SERVO_MIN, SERVO_MAX);
  return Math.round(PULSE_MIN + deg / (SERVO_MAX - SERVO_MIN) * (PULSE_MAX - PULSE_MIN));
}

// === Scope / trail / step test ===
const SCOPE_LEN = 360;            // ~6 s @ 60 fps
const TRAIL_LEN = 180;
let scope = { e1: [], e2: [], t1: [], t2: [], ee: [], f: [] };
let trail = [];
const STEP_PTS = [[0.071, 0.071], [-0.017, 0.098]];   // reachable in both elbow branches within the limits
let stepIdx = 1, stepRec = null;

function setup(){
  const holder = $('sketch-holder');
  const cnv = createCanvas(holder.clientWidth, holder.clientHeight, WEBGL);
  cnv.parent('sketch-holder');
  pixelDensity(1);
  setupPointer(cnv.elt);
  hud = new HUD(holder);

  arm = new TwoLinkArm();
  serial = new SerialLink();

  ui.mode = $('mode');
  ui.elbow = $('elbow');
  ui.ctrl = $('ctrl');
  ui.xt = $('xt');
  ui.yt = $('yt');
  ui.j1t = $('j1t');
  ui.j2t = $('j2t');
  ui.infoAngles = $('infoAngles');
  ui.infoEE = $('infoEE');
  ui.followSim = $('followSim');
  ui.btnSerial = $('btnSerial');
  ui.btnSerialClose = $('btnSerialClose');
  ui.serialStatus = $('serialStatus');

  if(ui.mode) cfg.mode = ui.mode.value;
  if(ui.ctrl) cfg.ctrl = ui.ctrl.value;
  if(ui.elbow) cfg.elbowUp = ui.elbow.value === 'up';

  on('mode', 'change', e => {
    // carry the current target over so the arm does not jump when switching modes
    const t = eeTarget();
    if(e.target.value === 'joint') arm.jointTargetDeg = arm.ghostDeg.map(d => Math.round(d));
    else { arm.xTarget = t.x; arm.yTarget = t.y; }
    cfg.mode = e.target.value; arm.xc = null;
    syncTargetInputs(); refreshPanels();
  });
  on('elbow', 'change', e => cfg.elbowUp = e.target.value === 'up');
  on('ctrl', 'change', e => { cfg.ctrl = e.target.value; arm.xc = null; arm.resetControllers(); refreshPanels(); });
  on('btnReset', 'click', () => arm.resetControllers());
  on('btnCenter', 'click', () => { arm.centerEE(); syncTargetInputs(); });
  on('btnStep', 'click', runStepTest);
  on('btnKick', 'click', kick);
  on('panToggle', 'change', e => isPanning = e.target.checked);

  arm.pid.forEach((pid, i) => {
    bindRange(`j${i+1}kp`, v => pid.Kp = v, 1);
    bindRange(`j${i+1}ki`, v => pid.Ki = v, 0);
    bindRange(`j${i+1}kd`, v => pid.Kd = v, 2);
  });
  bindRange('j1t', v => arm.jointTargetDeg[0] = clamp(v, J_LIM[0][0], J_LIM[0][1]), 0);
  bindRange('j2t', v => arm.jointTargetDeg[1] = clamp(v, J_LIM[1][0], J_LIM[1][1]), 0);

  bindRange('Kx', v => cfg.imp.Kx = v, 0);
  bindRange('Ky', v => cfg.imp.Ky = v, 0);
  bindRange('B',  v => cfg.imp.B = v, 1);
  bindRange('M',  v => cfg.imp.M = v, 2);

  bindRange('payload', v => cfg.payload = v / 1000, 0);
  bindRange('noise', v => cfg.noiseDeg = v, 2);
  bindRange('dcut', v => cfg.dCutHz = v, 0);
  bindCheck('gravity', v => cfg.gravity = v);
  bindCheck('showEllipse', v => cfg.showEllipse = v);
  bindCheck('showTrail', v => cfg.showTrail = v);
  bindCheck('showGhost', v => cfg.showGhost = v);
  bindCheck('shading', v => cfg.shading = v);

  on('xt', 'change', () => setTarget(parseFloat(ui.xt.value), arm.yTarget));
  on('yt', 'change', () => setTarget(arm.xTarget, parseFloat(ui.yt.value)));

  on('btnSerial', 'click', async ()=>{
    try{
      await serial.connect();
      ui.serialStatus.textContent = "Serial connected";
      ui.btnSerial.disabled = true;
      ui.btnSerialClose.disabled = false;
    }catch(e){
      ui.serialStatus.textContent = "Connect failed: " + e;
    }
  });
  on('btnSerialClose', 'click', ()=>{
    serial.disconnect();
    ui.serialStatus.textContent = "Not connected";
    ui.btnSerial.disabled = false;
    ui.btnSerialClose.disabled = true;
  });

  arm.centerEE();
  syncTargetInputs();
  refreshPanels();
}

// --- UI helpers (elements missing from the HTML are silently ignored) ---
function on(id, evt, fn){ const el = $(id); if(el) el.addEventListener(evt, fn); }
function bindRange(id, apply, digits){
  const el = $(id); if(!el) return;
  const out = $(id + '_v');
  const f = () => { const v = parseFloat(el.value); apply(v); if(out) out.textContent = v.toFixed(digits); };
  el.addEventListener('input', f); f();
}
function bindCheck(id, apply){
  const el = $(id); if(!el) return;
  el.addEventListener('change', () => apply(el.checked)); apply(el.checked);
}
function setRange(id, v, digits){
  const el = $(id); if(!el) return;
  el.value = v;
  const out = $(id + '_v'); if(out) out.textContent = v.toFixed(digits);
}

function refreshPanels(){
  const ik = cfg.mode === 'ik';
  const show = (id, vis) => { const el = $(id); if(el) el.style.display = vis ? '' : 'none'; };
  show('jointTargets', !ik);
  show('ikTargets', ik);
  show('impParams', ik && cfg.ctrl !== 'pid');
  if(ui.ctrl) ui.ctrl.disabled = !ik;
  const m = $('M'); if(m) m.disabled = cfg.ctrl === 'imp';
  const desc = $('ctrlDesc');
  if(desc) desc.textContent = ik ? CTRL_INFO[cfg.ctrl][1]
                                 : 'Joint mode: set joint targets with the sliders (or right-drag); each joint PID tracks them.';
}

function syncTargetInputs(){
  if(ui.xt && document.activeElement !== ui.xt) ui.xt.value = arm.xTarget.toFixed(3);
  if(ui.yt && document.activeElement !== ui.yt) ui.yt.value = arm.yTarget.toFixed(3);
  setRange('j1t', arm.jointTargetDeg[0], 0);
  setRange('j2t', arm.jointTargetDeg[1], 0);
}

// Set the target (kept inside the workspace). In Joint mode it is converted to joint targets via IK
function setTarget(x, y){
  if(!isFinite(x) || !isFinite(y)) return;
  const r = Math.hypot(x, y), rMax = arm.L1 + arm.L2, rMin = 0.01;
  if(r > rMax){ x *= rMax / r; y *= rMax / r; }
  else if(r < rMin && r > 0){ x *= rMin / r; y *= rMin / r; }
  arm.xTarget = x; arm.yTarget = y;
  if(cfg.mode === 'joint') arm.jointTargetDeg = arm.ik(x, y, cfg.elbowUp);
  syncTargetInputs();
}

// End-effector target of the current mode [m]
function eeTarget(){
  if(cfg.mode === 'joint') return arm.forward(arm.jointTargetDeg[0], arm.jointTargetDeg[1]);
  return { x: arm.xTarget, y: arm.yTarget };
}

function kick(){
  // 2 N for 50 ms, tangential to the tip
  const p = arm.fkRad(arm.q), r = Math.hypot(p.x, p.y) || 1;
  interact.kick = { t: 0.05, F: [-p.y / r * 2, p.x / r * 2] };
}

function runStepTest(){
  stepIdx ^= 1;
  const p = arm.fkRad(arm.q);
  setTarget(STEP_PTS[stepIdx][0], STEP_PTS[stepIdx][1]);
  const t = eeTarget();
  const d0 = Math.hypot(t.x - p.x, t.y - p.y);
  if(d0 < 1e-3){ stepRec = null; return; }
  stepRec = { t0: simTime, x0: p.x, y0: p.y, xt: t.x, yt: t.y, d0,
              ux: (t.x - p.x) / d0, uy: (t.y - p.y) / d0,
              t10: null, t90: null, os: 0, dev: 0, ts: 0, err: d0 };
  for(const k in scope) scope[k].length = 0;
  trail.length = 0;
}

function updateStepMetrics(){
  const s = stepRec; if(!s) return;
  const t = simTime - s.t0; if(t > 6) return;
  const p = arm.fkRad(arm.q);
  const rx = p.x - s.x0, ry = p.y - s.y0;
  const prog = (rx * s.ux + ry * s.uy) / s.d0;           // progress 0 → 1
  if(s.t10 === null && prog >= 0.1) s.t10 = t;
  if(s.t90 === null && prog >= 0.9) s.t90 = t;
  s.os = Math.max(s.os, (prog - 1) * s.d0);              // distance past the target
  s.dev = Math.max(s.dev, Math.abs(rx * s.uy - ry * s.ux)); // deviation from the straight path
  s.err = Math.hypot(p.x - s.xt, p.y - s.yt);
  if(s.err > Math.max(0.02 * s.d0, 0.0005)) s.ts = t;    // last time outside the 2% band
}

// --- Screen coords -> robot plane coords (inverts the transforms in draw()) ---
function screenToPlane(mx, my, zPx){
  const D = (height / 2) / Math.tan(Math.PI / 6);
  const inv = v => {
    let x = (v[0] - view.panX) / view.zoom, y = (v[1] - view.panY) / view.zoom, z = v[2] / view.zoom;
    const cx = Math.cos(view.rx), sx = Math.sin(view.rx);
    [y, z] = [cx * y + sx * z, -sx * y + cx * z];          // Rx^T
    const cz = Math.cos(view.rz), sz = Math.sin(view.rz);
    [x, y] = [cz * x + sz * y, -sz * x + cz * y];          // Rz^T
    return [-x, -y, z];                                    // scale(-1,-1,1)
  };
  const e = inv([0, 0, D]), s = inv([mx - width / 2, my - height / 2, 0]);
  const dz = s[2] - e[2];
  if(Math.abs(dz) < 1e-9) return null;
  const t = (zPx - e[2]) / dz;
  if(t < 0) return null;
  return { x: (e[0] + t * (s[0] - e[0])) / PX, y: (e[1] + t * (s[1] - e[1])) / PX };
}

function setupPointer(el){
  let last = [0, 0];
  const local = e => { const r = el.getBoundingClientRect(); return [e.clientX - r.left, e.clientY - r.top]; };
  const pointerAt = (mx, my) => {
    const p = screenToPlane(mx, my, drag === 'target' ? 8 : 10);
    if(!p) return;
    if(drag === 'target') setTarget(p.x, p.y);
    else if(drag === 'poke') interact.poke = p;
  };
  el.addEventListener('contextmenu', e => e.preventDefault());
  el.addEventListener('pointerdown', e => {
    if(e.button === 2) drag = 'target';
    else if(e.button === 0 && e.shiftKey) drag = 'poke';
    else if(e.button === 1 || (e.button === 0 && isPanning)) drag = 'pan';
    else if(e.button === 0) drag = 'orbit';
    else return;
    e.preventDefault();
    el.setPointerCapture(e.pointerId);
    last = local(e);
    pointerAt(last[0], last[1]);
  });
  el.addEventListener('pointermove', e => {
    if(!drag) return;
    const [mx, my] = local(e);
    const dx = mx - last[0], dy = my - last[1];
    last = [mx, my];
    if(drag === 'orbit'){ view.rz += dx * 0.01; view.rx = clamp(view.rx - dy * 0.01, -1.4, 1.4); }
    else if(drag === 'pan'){ view.panX += dx; view.panY += dy; }
    else pointerAt(mx, my);
  });
  const end = () => { drag = null; interact.poke = null; };
  el.addEventListener('pointerup', end);
  el.addEventListener('pointercancel', end);
  el.addEventListener('wheel', e => {
    e.preventDefault();
    view.zoom = clamp(view.zoom * Math.exp(-e.deltaY * 0.001), 0.3, 6);
  }, { passive: false });
  el.addEventListener('dblclick', () => { view = { rx: 0, rz: 0, zoom: 1, panX: 0, panY: 0 }; });
}

function windowResized(){
  const holder = $('sketch-holder');
  resizeCanvas(holder.clientWidth, holder.clientHeight);
  hud.resize();
}

function draw(){
  const D = (height / 2) / Math.tan(Math.PI / 6);
  camera(0, 0, D, 0, 0, 0, 0, 1, 0);
  perspective(Math.PI / 3, width / height, D / 50, D * 20);
  background(20);

  // Lighting (p5's built-in Phong shader). Set before the model transforms, so the lights stay fixed
  // relative to the camera: key light from the upper left, a dim cool fill light from the lower right
  if(cfg.shading){
    ambientLight(70);
    specularColor(255);
    directionalLight(255, 250, 240, -0.3, 0.4, -1);
    specularColor(0);
    directionalLight(50, 55, 70, 0.6, -0.4, -1);
  }

  // === Physics + control (fixed 1 ms steps) ===
  simAcc += Math.min(deltaTime / 1000, 0.05);
  while(simAcc >= PHYS_DT){
    arm.step(PHYS_DT);
    simTime += PHYS_DT;
    simAcc -= PHYS_DT;
  }
  arm.updateGhost();

  const ee = arm.fkRad(arm.q);
  const tgt = eeTarget();
  trail.push(ee); if(trail.length > TRAIL_LEN) trail.shift();
  updateStepMetrics();

  // === 3D scene ===
  push();
  translate(view.panX, view.panY, 0);
  scale(view.zoom);
  rotateX(view.rx);
  rotateZ(view.rz);
  scale(-1, -1, 1);   // p5 coords -> robot coords (+y up)

  push(); stroke(60);
  for(let i=-10;i<=10;i++){
    line(i*50, -10*50, i*50, 10*50);
    line(-10*50, i*50, 10*50, i*50);
  }
  pop();
  drawScene(ee, tgt);
  pop();

  // === Info ===
  const a = arm.angleDeg, g = arm.ghostDeg;
  if(ui.infoAngles) ui.infoAngles.textContent =
    `J1 ${a[0].toFixed(1)}° → ${g[0].toFixed(1)}°, J2 ${a[1].toFixed(1)}° → ${g[1].toFixed(1)}°`;
  if(ui.infoEE) ui.infoEE.textContent = `(${ee.x.toFixed(3)}, ${ee.y.toFixed(3)}) m`;

  // === Send to hardware (every frame, in microseconds; the firmware interpolates between frames) ===
  if (serial.isConnected()) {
    // Follow Sim ON: simulated joint angles, OFF: target pose (ghost)
    const useSim = ui.followSim && ui.followSim.checked;
    const cmd = useSim ? a : g;
    serial.send(`U:${servoUs(cmd[0], J1_OFFSET)},${servoUs(cmd[1], J2_OFFSET)}\n`);
  }

  // === Scope ===
  const push2 = (buf, v) => { buf.push(v); if(buf.length > SCOPE_LEN) buf.shift(); };
  push2(scope.e1, g[0] - a[0]);
  push2(scope.e2, g[1] - a[1]);
  push2(scope.t1, arm.tau[0] * 1000);
  push2(scope.t2, arm.tau[1] * 1000);
  push2(scope.ee, Math.hypot(tgt.x - ee.x, tgt.y - ee.y) * 1000);
  push2(scope.f, Math.hypot(arm.Fext[0], arm.Fext[1]));
  hud.draw();
}

function drawScene(ee, tgt){
  // reachable workspace: the four joint-limit edges mapped through FK
  push(); noFill(); stroke(70, 70, 90); strokeWeight(1);
  const [[a0, a1], [b0, b1]] = J_LIM;
  const edges = [[[a0, b0], [a1, b0]], [[a0, b1], [a1, b1]], [[a0, b0], [a0, b1]], [[a1, b0], [a1, b1]]];
  for(const [s, e] of edges){
    beginShape();
    for(let i=0;i<=48;i++){
      const p = arm.forward(s[0] + (e[0] - s[0]) * i / 48, s[1] + (e[1] - s[1]) * i / 48);
      vertex(p.x*PX, p.y*PX, 1);
    }
    endShape();
  }
  pop();

  // straight reference line of the step test
  if(stepRec){
    push(); stroke(120, 120, 140); strokeWeight(1);
    line(stepRec.x0*PX, stepRec.y0*PX, 2, stepRec.xt*PX, stepRec.yt*PX, 2);
    pop();
  }

  // end-effector trail
  if(cfg.showTrail && trail.length > 1){
    push(); strokeWeight(2);
    for(let i=1;i<trail.length;i++){
      stroke(80, 200, 255, 255 * i / trail.length);
      line(trail[i-1].x*PX, trail[i-1].y*PX, 3, trail[i].x*PX, trail[i].y*PX, 3);
    }
    pop();
  }

  if(cfg.showGhost) arm.renderGhost(arm.ghostDeg);
  arm.render();

  // target
  push(); translate(tgt.x*PX, tgt.y*PX, 8); noStroke(); fill(255, 120, 120);
  sphere(drag === 'target' ? 8 : 5); pop();

  // admittance reference point x_c
  if(cfg.mode === 'ik' && cfg.ctrl === 'adm' && arm.xc){
    push(); stroke(0, 220, 220); line(tgt.x*PX, tgt.y*PX, 8, arm.xc[0]*PX, arm.xc[1]*PX, 8);
    translate(arm.xc[0]*PX, arm.xc[1]*PX, 8); noStroke(); fill(0, 220, 220); sphere(4); pop();
  }

  // compliance ellipse: static deflection per 1 N (×10)
  if(cfg.showEllipse){
    const [a, b, c] = arm.complianceAtEE();
    const m = (a + c) / 2, d = Math.sqrt(((a - c) / 2) ** 2 + b * b);
    const s = PX * 10;
    const A = Math.min((m + d) * s, 250), B = Math.min(Math.max(0, m - d) * s, 250);
    push(); translate(ee.x*PX, ee.y*PX, 4); rotateZ(0.5 * Math.atan2(2 * b, a - c));
    noFill(); stroke(80, 220, 140); strokeWeight(1.5);
    beginShape(); for(let i=0;i<=48;i++){ const t = i / 48 * TWO_PI; vertex(A*Math.cos(t), B*Math.sin(t), 0); } endShape();
    pop();
  }

  // external forces
  if(interact.poke){
    push(); stroke(255, 80, 220); strokeWeight(2);
    line(ee.x*PX, ee.y*PX, 12, interact.poke.x*PX, interact.poke.y*PX, 12);
    translate(interact.poke.x*PX, interact.poke.y*PX, 12); noStroke(); fill(255, 80, 220); sphere(4); pop();
  }
  if(interact.kick){
    const F = interact.kick.F;
    push(); stroke(255, 80, 220); strokeWeight(3);
    line(ee.x*PX, ee.y*PX, 12, ee.x*PX + F[0]*20, ee.y*PX + F[1]*20, 12); pop();
  }
}

// === HUD: 2D overlay (scopes, metrics, help) ===
class HUD {
  constructor(holder){
    this.holder = holder;
    this.cv = $('hud') || holder.appendChild(document.createElement('canvas'));
    Object.assign(this.cv.style, { position: 'absolute', left: '0', top: '0', pointerEvents: 'none' });
    this.ctx = this.cv.getContext('2d');
    this.resize();
  }
  resize(){
    const dpr = window.devicePixelRatio || 1;
    this.w = this.holder.clientWidth; this.h = this.holder.clientHeight;
    this.cv.width = this.w * dpr; this.cv.height = this.h * dpr;
    this.cv.style.width = this.w + 'px'; this.cv.style.height = this.h + 'px';
    this.ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
  }
  strip(x, y, w, h, title, series, opt){
    const c = this.ctx;
    let range = opt.fixed || opt.min;
    if(!opt.fixed) for(const [buf] of series) for(const v of buf) range = Math.max(range, Math.abs(v) * 1.1);
    c.fillStyle = 'rgba(0,0,0,0.5)'; c.fillRect(x, y, w, h);
    c.strokeStyle = '#555'; c.lineWidth = 1; c.strokeRect(x + 0.5, y + 0.5, w - 1, h - 1);
    const y0 = opt.positive ? y + h - 2 : y + h / 2;
    const k = (opt.positive ? h - 4 : h / 2 - 2) / range;
    c.strokeStyle = '#333'; c.beginPath(); c.moveTo(x, y0); c.lineTo(x + w, y0); c.stroke();
    if(opt.limit){
      c.strokeStyle = 'rgba(255,90,90,0.6)'; c.setLineDash([3, 3]); c.beginPath();
      c.moveTo(x, y0 - opt.limit * k); c.lineTo(x + w, y0 - opt.limit * k);
      c.moveTo(x, y0 + opt.limit * k); c.lineTo(x + w, y0 + opt.limit * k);
      c.stroke(); c.setLineDash([]);
    }
    for(const [buf, col] of series){
      c.strokeStyle = col; c.lineWidth = 1.3; c.beginPath();
      for(let i=0;i<buf.length;i++){
        const px = x + i / (SCOPE_LEN - 1) * w, py = y0 - clamp(buf[i] * k, -h, h);
        i ? c.lineTo(px, py) : c.moveTo(px, py);
      }
      c.stroke();
    }
    c.fillStyle = '#aaa'; c.fillText(`${title}  ±${range.toFixed(range < 10 ? 1 : 0)}`, x + 4, y + 11);
    c.textAlign = 'right';
    series.forEach(([buf, col], i) => {
      c.fillStyle = col; c.fillText(buf.length ? buf[buf.length - 1].toFixed(1) : '-', x + w - 4 - i * 44, y + 11);
    });
    c.textAlign = 'left';
  }
  draw(){
    const c = this.ctx, W = this.w, H = this.h;
    c.clearRect(0, 0, W, H);
    c.font = '11px system-ui, sans-serif';
    const w = Math.min(300, W - 20), h = 58, x = W - w - 10;
    let y = 10;
    const lim = arm.tauStall * 1000;
    this.strip(x, y, w, h, 'Joint error [deg]  J1 / J2', [[scope.e1, '#fff'], [scope.e2, '#fc0']], { min: 2 }); y += h + 6;
    this.strip(x, y, w, h, 'Motor torque [mNm]', [[scope.t1, '#fff'], [scope.t2, '#fc0']], { fixed: lim * 1.15, limit: lim }); y += h + 6;
    this.strip(x, y, w, h, 'EE error [mm]', [[scope.ee, '#4cf']], { min: 2, positive: true }); y += h + 6;
    this.strip(x, y, w, h, 'External force [N]', [[scope.f, '#f6c']], { min: 0.5, positive: true }); y += h + 10;

    // step test metrics
    const s = stepRec;
    if(s){
      const band = Math.max(0.02 * s.d0, 0.0005);
      const lines = [
        `Step test  (${(s.d0 * 1000).toFixed(0)} mm)`,
        `rise 10-90%   ${s.t10 !== null && s.t90 !== null ? (s.t90 - s.t10).toFixed(3) + ' s' : '—'}`,
        `overshoot     ${(s.os * 1000).toFixed(1)} mm  (${(s.os / s.d0 * 100).toFixed(1)}%)`,
        `settle (2%)   ${s.err > band ? '— (err ' + (s.err * 1000).toFixed(1) + ' mm)' : s.ts.toFixed(2) + ' s'}`,
        `path dev max  ${(s.dev * 1000).toFixed(1)} mm`,
      ];
      c.fillStyle = 'rgba(0,0,0,0.5)'; c.fillRect(x, y, w, lines.length * 15 + 8);
      lines.forEach((l, i) => { c.fillStyle = i ? '#ddd' : '#8cf'; c.fillText(l, x + 6, y + 15 + i * 15); });
    }

    // bottom-left: status + help
    const name = cfg.mode === 'joint' ? 'Joint PID' : CTRL_INFO[cfg.ctrl][0];
    const F = arm.Fext;
    const st = [
      `${name}   |   gravity ${cfg.gravity ? 'ON (−y)' : 'off'}   payload ${(cfg.payload * 1000).toFixed(0)} g   noise ${cfg.noiseDeg.toFixed(2)}°`,
      `τ = (${(arm.tau[0] * 1000).toFixed(0)}, ${(arm.tau[1] * 1000).toFixed(0)}) mNm   F_ext = (${F[0].toFixed(2)}, ${F[1].toFixed(2)}) N`,
      'green ellipse: deflection per 1 N (×10)   blue: EE trail   faint red: target pose',
      'Right-drag: move target   Shift+drag: pull arm   Drag: rotate   Wheel: zoom   Dbl-click: reset view',
    ];
    c.fillStyle = 'rgba(0,0,0,0.5)'; c.fillRect(6, H - st.length * 15 - 10, Math.min(W - 12, 600), st.length * 15 + 6);
    st.forEach((l, i) => { c.fillStyle = i < 2 ? '#ddd' : '#888'; c.fillText(l, 12, H - (st.length - i) * 15 + 1); });
  }
}

// === Classes ===
class PID {
  // Units: Kp [mNm/deg], Ki [mNm/(deg s)], Kd [mNm s/deg]  ->  output torque [Nm]
  constructor(Kp=10, Ki=0, Kd=0.5){
    this.Kp=Kp; this.Ki=Ki; this.Kd=Kd;
    this.integral=0;
  }
  reset(){ this.integral=0; }
  update(errDeg, rateDeg, dt, tauMax){
    const lim = tauMax * 1000;
    const p = this.Kp * errDeg;
    const d = -this.Kd * rateDeg;          // derivative on measurement: no kick when the target jumps
    if(this.Ki === 0){ this.integral = 0; }
    else {
      // anti-windup: do not integrate further into saturation (conditional integration)
      const iNext = this.integral + errDeg * dt;
      const uNext = p + this.Ki * iNext + d;
      if(Math.abs(uNext) <= lim || Math.sign(uNext) !== Math.sign(errDeg)) this.integral = iNext;
    }
    return clamp(p + this.Ki * this.integral + d, -lim, lim) / 1000;
  }
}

// Googly eye on the end effector: the pupil is a small free mass held loosely to the eye center by a
// weak spring-damper and bounced off the eye's elliptical rim, so it lags and rolls when the arm moves
class GooglyEye {
  constructor(side){
    this.side = side;                   // +1 / -1: which side of link 2
    this.cx = 0.006; this.cy = 0.0085;  // eye center in the link-2 tip frame [m]
    this.ex = 0.011; this.ey = 0.009;   // eye semi-axes (along / across the link) [m]
    this.pr = 0.0042;                   // pupil radius [m]
    this.tilt = 0.25;                   // eye tilt [rad] (~14°): the top leans to the face's right
    // centering spring [1/s^2], damping [1/s]: ~3 Hz, damping ratio ~0.7.
    // Lower k (and b) for wilder eyes; the static lag is acceleration / k
    this.k = 400; this.b = 28;
    this.e = 0.4;                       // restitution at the rim
    this.reset();
  }
  reset(){ this.s = null; this.v = [0, 0]; this.cPrev = null; this.r = [0, 0]; }
  update(arm, dt){
    const phi = arm.q[0] + arm.q[1], cs = Math.cos(phi), sn = Math.sin(phi);
    const tip = arm.fkRad(arm.q), ly = this.side * this.cy;
    const c = [tip.x + cs * this.cx - sn * ly, tip.y + sn * this.cx + cs * ly];
    if(!this.s){ this.s = c.slice(); this.cPrev = c.slice(); }
    const vc = [(c[0] - this.cPrev[0]) / dt, (c[1] - this.cPrev[1]) / dt];
    this.cPrev = c;
    const g = cfg.gravity ? -GRAV : 0;
    for(let i=0;i<2;i++){
      const a = -this.k * (this.s[i] - c[i]) - this.b * (this.v[i] - vc[i]) + (i === 1 ? g : 0);
      this.v[i] += a * dt;
      this.s[i] += this.v[i] * dt;
    }
    // keep the pupil inside the rim (tilted ellipse in the eye frame), bounce the relative velocity
    const pe = phi + this.tilt, ce = Math.cos(pe), se = Math.sin(pe);
    const dx = this.s[0] - c[0], dy = this.s[1] - c[1];
    let rx = ce * dx + se * dy, ry = -se * dx + ce * dy;
    const ax = this.ex - this.pr, ay = this.ey - this.pr;
    const n = Math.hypot(rx / ax, ry / ay);
    if(n > 1){
      rx /= n; ry /= n;
      this.s = [c[0] + ce * rx - se * ry, c[1] + se * rx + ce * ry];
      let nx = rx / (ax * ax), ny = ry / (ay * ay);
      const nl = Math.hypot(nx, ny); nx /= nl; ny /= nl;
      const Nx = ce * nx - se * ny, Ny = se * nx + ce * ny;   // rim normal in world
      const ux = this.v[0] - vc[0], uy = this.v[1] - vc[1];
      const un = ux * Nx + uy * Ny;
      if(un > 0){ this.v[0] -= (1 + this.e) * un * Nx; this.v[1] -= (1 + this.e) * un * Ny; }
    }
    this.r = [rx, ry];
  }
  render(){
    push();
    translate(this.cx * PX, this.side * this.cy * PX, 17);
    rotateZ(this.tilt);
    fill(255); stroke(40, 70, 200); strokeWeight(2);
    ellipse(0, 0, 2 * this.ex * PX, 2 * this.ey * PX);
    noStroke(); fill(30, 60, 190);
    translate(this.r[0] * PX, this.r[1] * PX, 0.6);
    ellipse(0, 0, 2.1 * this.pr * PX, 1.9 * this.pr * PX);
    pop();
  }
}

class TwoLinkArm {
  constructor(){
    // kinematics [m]
    this.L1 = 0.06; this.L2 = 0.06;
    // dynamics (rough estimates for a hobby-servo-sized arm)
    this.m1 = 0.05; this.lc1 = 0.035;  // link 1 + elbow servo [kg], center of mass [m]
    this.m2 = 0.03; this.lc2 = 0.03;   // link 2
    this.Jr = 4e-4;                    // rotor inertia reflected through the gearbox [kg m^2]
    this.bv = 0.002;                   // viscous friction [Nm s/rad]
    this.tc = 0.004;                   // Coulomb friction [Nm]
    this.tauStall = 0.25;              // stall torque [Nm]
    this.wNoLoad = 10;                 // no-load speed [rad/s] (~0.1 s/60°)

    this.q = [0, 0]; this.qd = [0, 0];     // true state [rad], [rad/s]
    this.tau = [0, 0];                     // motor output torque [Nm]
    this.pid = [new PID(), new PID()];
    this.jointTargetDeg = [0, 0];          // Joint mode target
    this.qTargetDeg = [0, 0];              // target tracked by the PIDs (IK result etc.)
    this.ghostDeg = [0, 0];                // target pose for display / servo output
    this.xTarget = 0.12; this.yTarget = 0.00;
    this.scale = PX;
    this.elbowSign = +1;                   // +1: elbow-up, -1: elbow-down
    this.qmPrev = null; this.qdEst = [0, 0];
    this.xc = null; this.vc = [0, 0];      // admittance reference point
    this.Fext = [0, 0];                    // true external force [N]
    this.Fmeas = [0, 0];                   // F/T sensor reading [N]
    this.eyes = [new GooglyEye(+1), new GooglyEye(-1)];
  }

  get angleDeg(){ return [this.q[0] * R2D, this.q[1] * R2D]; }

  resetControllers(){ this.pid.forEach(p => p.reset()); }
  centerEE(){
    this.q = [0, 0]; this.qd = [0, 0]; this.qmPrev = null; this.qdEst = [0, 0];
    this.jointTargetDeg = [0, 0]; this.xc = null;
    this.eyes.forEach(e => e.reset());
    const p = this.forward(0, 0);
    this.xTarget = p.x; this.yTarget = p.y;
    this.resetControllers();
  }

  forward(th1Deg, th2Deg){ return this.fkRad([th1Deg * D2R, th2Deg * D2R]); }
  fkRad(q){
    return { x: this.L1 * Math.cos(q[0]) + this.L2 * Math.cos(q[0] + q[1]),
             y: this.L1 * Math.sin(q[0]) + this.L2 * Math.sin(q[0] + q[1]) };
  }
  // J = [j11, j12, j21, j22],  [ẋ; ẏ] = J q̇
  jac(q){
    const s1 = Math.sin(q[0]), c1 = Math.cos(q[0]);
    const s12 = Math.sin(q[0] + q[1]), c12 = Math.cos(q[0] + q[1]);
    return [-this.L1*s1 - this.L2*s12, -this.L2*s12, this.L1*c1 + this.L2*c12, this.L2*c12];
  }
  jdotqd(q, qd){
    const w1 = qd[0], w12 = qd[0] + qd[1];
    return [-this.L1*Math.cos(q[0])*w1*w1 - this.L2*Math.cos(q[0]+q[1])*w12*w12,
            -this.L1*Math.sin(q[0])*w1*w1 - this.L2*Math.sin(q[0]+q[1])*w12*w12];
  }

  // M(q), Coriolis/centrifugal h(q,q̇), gravity g(q).  mp: tip payload [kg] (point mass merged into link 2)
  dyn(q, qd, mp){
    const m2 = this.m2 + mp;
    const lc2 = (this.m2 * this.lc2 + mp * this.L2) / m2;
    const I2 = this.m2 * this.L2 ** 2 / 12 + this.m2 * (this.lc2 - lc2) ** 2 + mp * (this.L2 - lc2) ** 2;
    const I1 = this.m1 * this.L1 ** 2 / 12;
    const c2 = Math.cos(q[1]), hc = m2 * this.L1 * lc2 * Math.sin(q[1]);
    const M11 = I1 + I2 + this.m1 * this.lc1 ** 2 + m2 * (this.L1 ** 2 + lc2 ** 2 + 2 * this.L1 * lc2 * c2) + this.Jr;
    const M12 = I2 + m2 * (lc2 ** 2 + this.L1 * lc2 * c2);
    const M22 = I2 + m2 * lc2 ** 2 + this.Jr;
    const h = [-hc * (2 * qd[0] * qd[1] + qd[1] ** 2), hc * qd[0] ** 2];
    let g = [0, 0];
    if(cfg.gravity){
      const c1 = Math.cos(q[0]), c12 = Math.cos(q[0] + q[1]);
      g = [(this.m1 * this.lc1 + m2 * this.L1) * GRAV * c1 + m2 * lc2 * GRAV * c12, m2 * lc2 * GRAV * c12];
    }
    return { M11, M12, M22, h, g };
  }

  // IK (deg): elbow-branch continuity + softened singularities, clamped to the joint limits
  ik(x, y, elbowUp=true){
    let r = Math.hypot(x, y);
    const rMin = Math.abs(this.L1 - this.L2) + 1e-4;
    const rMax = (this.L1 + this.L2) - 1e-4;
    const nx = r > 1e-9 ? x / r : 1, ny = r > 1e-9 ? y / r : 0;
    r = clamp(r, rMin, rMax);

    const desiredSign = elbowUp ? +1 : -1;
    if (Math.abs(r - rMax) > 0.005 && Math.abs(r - rMin) > 0.005) {
      this.elbowSign = desiredSign;
    }

    const c2 = clamp((r*r - this.L1*this.L1 - this.L2*this.L2) / (2*this.L1*this.L2), -1, 1);
    const s2 = this.elbowSign * Math.sqrt(Math.max(0, 1 - c2*c2));
    const th2 = Math.atan2(s2, c2);
    let th1 = Math.atan2(ny, nx) - Math.atan2(this.L2*s2, this.L1 + this.L2*c2);
    th1 = Math.atan2(Math.sin(th1), Math.cos(th1));  // wrap to (-180°, 180°]
    if(th1 < -Math.PI / 2) th1 += 2 * Math.PI;       // then to [-90°, 270°) so the clamp picks the nearer limit
    return limitDeg([th1 * R2D, th2 * R2D]);
  }

  // Motor torque-speed curve: driving torque drops with speed, braking is allowed up to stall torque
  motorSat(tau, w){
    const lim = tau * w > 0 ? this.tauStall * Math.max(0, 1 - Math.abs(w) / this.wNoLoad) : this.tauStall;
    return clamp(tau, -lim, lim);
  }

  step(dt){
    // 1) sensor: noisy angle + low-pass filtered finite-difference velocity
    const n = cfg.noiseDeg * D2R;
    const qm = [this.q[0] + n * gauss(), this.q[1] + n * gauss()];
    if(this.qmPrev){
      const a = Math.exp(-2 * Math.PI * cfg.dCutHz * dt);
      for(let i=0;i<2;i++) this.qdEst[i] = a * this.qdEst[i] + (1 - a) * (qm[i] - this.qmPrev[i]) / dt;
    }
    this.qmPrev = qm;

    // 2) external force: mouse spring + kick
    const p = this.fkRad(this.q);
    let F = [0, 0];
    if(interact.poke){
      F = [interact.kPoke * (interact.poke.x - p.x), interact.kPoke * (interact.poke.y - p.y)];
      const f = Math.hypot(F[0], F[1]);
      if(f > interact.fPokeMax){ F[0] *= interact.fPokeMax / f; F[1] *= interact.fPokeMax / f; }
    }
    if(interact.kick){
      F[0] += interact.kick.F[0]; F[1] += interact.kick.F[1];
      interact.kick.t -= dt;
      if(interact.kick.t <= 0) interact.kick = null;
    }
    this.Fext = F;
    // wrist F/T sensor: external force + payload weight
    this.Fmeas = [F[0], F[1] - (cfg.gravity ? cfg.payload * GRAV : 0)];

    // 3) control -> 4) motor saturation
    const u = this.control(qm, this.qdEst, dt);
    for(let i=0;i<2;i++) this.tau[i] = this.motorSat(u[i], this.qd[i]);

    // 5) plant:  M q̈ + h + g + friction = τ + Jᵀ F
    const d = this.dyn(this.q, this.qd, cfg.payload);
    const J = this.jac(this.q);
    const tExt = [J[0] * F[0] + J[2] * F[1], J[1] * F[0] + J[3] * F[1]];
    const r = [0, 1].map(i => this.tau[i] + tExt[i] - d.h[i] - d.g[i]
                              - this.bv * this.qd[i] - this.tc * Math.tanh(this.qd[i] / 0.02));
    const det = d.M11 * d.M22 - d.M12 * d.M12;
    this.qd[0] += ( d.M22 * r[0] - d.M12 * r[1]) / det * dt;
    this.qd[1] += (-d.M12 * r[0] + d.M11 * r[1]) / det * dt;
    for(let i=0;i<2;i++){
      this.q[i] += this.qd[i] * dt;
      // hard stops at the servo limits (inelastic)
      const lo = J_LIM[i][0] * D2R, hi = J_LIM[i][1] * D2R;
      if(this.q[i] < lo){ this.q[i] = lo; this.qd[i] = Math.max(this.qd[i], 0); }
      if(this.q[i] > hi){ this.q[i] = hi; this.qd[i] = Math.min(this.qd[i], 0); }
    }
    this.eyes.forEach(e => e.update(this, dt));
  }

  // Torque command [Nm] from measurements (qm, qdm) only. Model-based terms use the nominal model (payload unknown)
  control(qm, qdm, dt){
    const ctrl = cfg.mode === 'ik' ? cfg.ctrl : 'pid';
    if(ctrl === 'pid' || ctrl === 'adm'){
      if(cfg.mode === 'joint') this.qTargetDeg = this.jointTargetDeg.slice();
      else if(ctrl === 'pid') this.qTargetDeg = this.ik(this.xTarget, this.yTarget, cfg.elbowUp);
      else { this.admittanceStep(dt); this.qTargetDeg = this.ik(this.xc[0], this.xc[1], cfg.elbowUp); }
      return [0, 1].map(i => this.pid[i].update(this.qTargetDeg[i] - qm[i] * R2D, qdm[i] * R2D, dt, this.tauStall));
    }

    const K = cfg.imp;
    const J = this.jac(qm), p = this.fkRad(qm);
    const v = [J[0] * qdm[0] + J[1] * qdm[1], J[2] * qdm[0] + J[3] * qdm[1]];
    const ex = p.x - this.xTarget, ey = p.y - this.yTarget;
    const dn = this.dyn(qm, qdm, 0);

    if(ctrl === 'imp'){
      // τ = Jᵀ (−K e − B ẋ) + ĝ(q)
      const Fx = -K.Kx * ex - K.B * v[0], Fy = -K.Ky * ey - K.B * v[1];
      return [J[0] * Fx + J[2] * Fy + dn.g[0], J[1] * Fx + J[3] * Fy + dn.g[1]];
    }

    // impF: enforce the target dynamics  M ẍ + B ẋ + K e = F_ext  with computed torque
    const F = this.Fmeas;
    const bx = (F[0] - K.B * v[0] - K.Kx * ex) / K.M;
    const by = (F[1] - K.B * v[1] - K.Ky * ey) / K.M;
    const jd = this.jdotqd(qm, qdm);
    const qdd = this.dls(J, bx - jd[0], by - jd[1]);
    return [dn.M11 * qdd[0] + dn.M12 * qdd[1] + dn.h[0] + dn.g[0] - (J[0] * F[0] + J[2] * F[1]),
            dn.M12 * qdd[0] + dn.M22 * qdd[1] + dn.h[1] + dn.g[1] - (J[1] * F[0] + J[3] * F[1])];
  }

  // Damped least-squares inverse: Jᵀ (J Jᵀ + λ² I)⁻¹ b  (keeps it bounded near singularities)
  dls(J, bx, by){
    const lam2 = 1.6e-5;
    const A11 = J[0]*J[0] + J[1]*J[1] + lam2, A12 = J[0]*J[2] + J[1]*J[3], A22 = J[2]*J[2] + J[3]*J[3] + lam2;
    const det = A11 * A22 - A12 * A12;
    const yx = (A22 * bx - A12 * by) / det, yy = (-A12 * bx + A11 * by) / det;
    return [J[0] * yx + J[2] * yy, J[1] * yx + J[3] * yy];
  }

  // Admittance: M v̇_c + B v_c + K (x_c − x_d) = F_meas  →  x_c
  admittanceStep(dt){
    if(!this.xc){ const p = this.fkRad(this.q); this.xc = [p.x, p.y]; this.vc = [0, 0]; }
    // implicit Euler: numerically stable even for light and stiff settings (small M, large K, B)
    const K = cfg.imp, F = this.Fmeas, xd = [this.xTarget, this.yTarget], Ks = [K.Kx, K.Ky];
    for(let i=0;i<2;i++){
      this.vc[i] = (K.M * this.vc[i] + dt * (F[i] - Ks[i] * (this.xc[i] - xd[i]))) / (K.M + dt * K.B + dt * dt * Ks[i]);
      this.xc[i] += this.vc[i] * dt;
    }
    const r = Math.hypot(this.xc[0], this.xc[1]);
    const rc = clamp(r, 0.01, this.L1 + this.L2 - 1e-3);
    if(r !== rc && r > 0){ this.xc[0] *= rc / r; this.xc[1] *= rc / r; this.vc = [0, 0]; }
  }

  // target pose for display / servo output (deg)
  updateGhost(){
    const ctrl = cfg.mode === 'ik' ? cfg.ctrl : 'pid';
    this.ghostDeg = (ctrl === 'pid' || ctrl === 'adm') ? this.qTargetDeg.slice()
                                                       : this.ik(this.xTarget, this.yTarget, cfg.elbowUp);
  }

  // Static tip compliance C [m/N] (symmetric: [c11, c12, c22]).
  // PID: joint P gains act as springs → C = J Kq⁻¹ Jᵀ,  impedance: diag(1/Kx, 1/Ky),  admittance: both in series
  complianceAtEE(){
    const ctrl = cfg.mode === 'ik' ? cfg.ctrl : 'pid';
    const Kd = cfg.imp;
    let c = [0, 0, 0];
    if(ctrl === 'pid' || ctrl === 'adm'){
      const J = this.jac(this.q);
      const k1 = Math.max(this.pid[0].Kp * 1e-3 * R2D, 1e-4), k2 = Math.max(this.pid[1].Kp * 1e-3 * R2D, 1e-4);
      c = [J[0]*J[0]/k1 + J[1]*J[1]/k2, J[0]*J[2]/k1 + J[1]*J[3]/k2, J[2]*J[2]/k1 + J[3]*J[3]/k2];
    }
    if(ctrl !== 'pid'){
      c[0] += 1 / Math.max(Kd.Kx, 1e-3);
      c[2] += 1 / Math.max(Kd.Ky, 1e-3);
    }
    return c;
  }

  renderGhost(deg){
    const a = deg[0] * D2R, b = (deg[0] + deg[1]) * D2R;
    const ex = this.L1 * Math.cos(a) * PX, ey = this.L1 * Math.sin(a) * PX;
    const px = ex + this.L2 * Math.cos(b) * PX, py = ey + this.L2 * Math.sin(b) * PX;
    push(); stroke(255, 120, 120, 110); strokeWeight(3);
    line(0, 0, 5, ex, ey, 5); line(ex, ey, 5, px, py, 5);
    pop();
  }

  render(){
    const [a1, a2] = this.angleDeg;
    push();
    noStroke();
    if(cfg.shading){ specularMaterial(90); shininess(20); }   // glossy highlights from the key light
    // base
    push(); fill(90); cylinder(40,10,40); pop();

    // link1
    push();
    rotateZ(radians(a1));
    translate((this.L1*this.scale)/2, 0, 10);
    fill(250,250,250); box(this.L1*this.scale, 12, 12);
    translate((this.L1*this.scale)/2, 0, 0);
    push(); fill(0,160,255); sphere(6); pop();

    // link2
    rotateZ(radians(a2));
    translate((this.L2*this.scale)/2, 0, 10);
    fill(255,200,0); box(this.L2*this.scale, 12, 12);
    translate((this.L2*this.scale)/2, 0, -10);

    // face on the end effector (link 2 top surface is at local z = 16)
    this.eyes.forEach(e => e.render());
    push();
    translate(-2, 0, 16);
    rotateY(-0.6);                     // tilt it down the face so its oval shape shows from above
    translate(0, 0, 3);
    if(cfg.shading){ specularMaterial(120); shininess(25); }
    fill(235, 115, 25);
    ellipsoid(5, 5, 7, 24, 16);        // short orange oval nose
    pop();
    pop();
    pop();
  }
}
