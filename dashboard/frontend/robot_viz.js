/**
 * robot_viz.js — Three.js 3D visualisation of the SO-101 arm.
 *
 * Draws a simplified stick-figure robot arm based on the DH geometry,
 * updated in real time from WebSocket joint tick data.
 */

// ── DH geometry (matches kinematics C++ table) ────────────────────────────────
const DH = [
  { a: 0,       alpha: 0,          d: 0.08,  offset: 0 },
  { a: 0,       alpha: Math.PI/2,  d: 0.48,  offset: 0 },
  { a: 0.32062, alpha: 0,          d: 0,     offset: 0 },
  { a: 0.26268, alpha: 0,          d: 0,     offset: 0 },
  { a: 0.20,    alpha: Math.PI/2,  d: 0,     offset: 0 },
  { a: 0.09,    alpha: -Math.PI/2, d: 0,     offset: 0 },
  { a: 0.17,    alpha: 0,          d: 0,     offset: 0 },  // EE
];
const CALIB_OFFSET = [2365, 1740, 1410, 3000, 3200, 3000];
const MOTOR_NAMES  = ["base","shoulder","elbow","palm","wrist","gripper"];

function ticksToRad(ticks) {
  return MOTOR_NAMES.map((n, i) => (ticks[n] || 2048 - CALIB_OFFSET[i]) * (2*Math.PI/4096));
}

function dhMatrix4(row, q) {
  const ct = Math.cos(q + row.offset), st = Math.sin(q + row.offset);
  const ca = Math.cos(row.alpha),       sa = Math.sin(row.alpha);
  const m = new THREE.Matrix4();
  m.set(
    ct,      -st,       0,    row.a,
    st*ca,    ct*ca,   -sa,  -sa*row.d,
    st*sa,    ct*sa,    ca,   ca*row.d,
    0,        0,        0,    1
  );
  return m;
}

function forwardKinematics(ticks) {
  const q = ticksToRad(ticks);
  const positions = [new THREE.Vector3(0,0,0)];
  let T = new THREE.Matrix4();
  for (let i = 0; i < 6; i++) {
    T.multiply(dhMatrix4(DH[i], q[i]));
    positions.push(new THREE.Vector3().setFromMatrixPosition(T));
  }
  // EE
  const T_ee = T.clone().multiply(dhMatrix4(DH[6], 0));
  positions.push(new THREE.Vector3().setFromMatrixPosition(T_ee));
  return positions;
}

// ── Scene setup ───────────────────────────────────────────────────────────────
const canvas   = document.getElementById("three-canvas");
const renderer = new THREE.WebGLRenderer({ canvas, antialias: true });
renderer.setPixelRatio(window.devicePixelRatio);
renderer.setClearColor(0x0f1117);

const scene  = new THREE.Scene();
const camera = new THREE.PerspectiveCamera(45, 1, 0.01, 10);
camera.position.set(1.2, 0.8, 1.2);
camera.lookAt(0, 0.3, 0);

scene.add(new THREE.AmbientLight(0xffffff, 0.4));
const dLight = new THREE.DirectionalLight(0xffffff, 0.8);
dLight.position.set(2, 3, 2);
scene.add(dLight);

// Grid.
const grid = new THREE.GridHelper(2, 20, 0x2a2d3a, 0x2a2d3a);
scene.add(grid);

// Arm link lines.
const linkMat  = new THREE.LineBasicMaterial({ color: 0x7dd3fc, linewidth: 2 });
const jointMat = new THREE.MeshStandardMaterial({ color: 0xfbbf24 });
const eeMat    = new THREE.MeshStandardMaterial({ color: 0x22c55e });

const linkGeom = new THREE.BufferGeometry();
const linkPositions = new Float32Array(3 * 2 * 7);
linkGeom.setAttribute("position", new THREE.BufferAttribute(linkPositions, 3));
const linkLine = new THREE.LineSegments(linkGeom, linkMat);
scene.add(linkLine);

// Joint spheres.
const jointSpheres = [];
for (let i = 0; i <= 7; i++) {
  const r = i === 7 ? 0.015 : 0.012;
  const mat = i === 7 ? eeMat : jointMat;
  const mesh = new THREE.Mesh(new THREE.SphereGeometry(r, 8, 8), mat);
  scene.add(mesh);
  jointSpheres.push(mesh);
}

function updateArm(ticks) {
  const pts = forwardKinematics(ticks);
  // Update line segments.
  for (let i = 0; i < pts.length - 1; i++) {
    linkPositions[i * 6 + 0] = pts[i].x;
    linkPositions[i * 6 + 1] = pts[i].z;      // Y-up in Three.js
    linkPositions[i * 6 + 2] = -pts[i].y;
    linkPositions[i * 6 + 3] = pts[i+1].x;
    linkPositions[i * 6 + 4] = pts[i+1].z;
    linkPositions[i * 6 + 5] = -pts[i+1].y;
  }
  linkGeom.attributes.position.needsUpdate = true;
  // Update spheres.
  pts.forEach((p, i) => {
    if (jointSpheres[i]) {
      jointSpheres[i].position.set(p.x, p.z, -p.y);
    }
  });
}

// ── Resize handler ────────────────────────────────────────────────────────────
function onResize() {
  const w = canvas.parentElement.clientWidth;
  const h = canvas.parentElement.clientHeight;
  renderer.setSize(w, h);
  camera.aspect = w / h;
  camera.updateProjectionMatrix();
}
window.addEventListener("resize", onResize);
onResize();

// ── Render loop ───────────────────────────────────────────────────────────────
function animate() {
  requestAnimationFrame(animate);
  renderer.render(scene, camera);
}
animate();

// Export for controls.js
window._updateArm = updateArm;
