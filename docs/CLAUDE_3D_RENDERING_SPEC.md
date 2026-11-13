```javascript
// ───────────── Renderer setup ─────────────
function configureRenderer(renderer) {
  renderer.outputEncoding = THREE.sRGBEncoding;
  renderer.toneMapping = THREE.ACESFilmicToneMapping;
  renderer.toneMappingExposure = 1.1;
  renderer.shadowMap.enabled = true;
  renderer.shadowMap.type = THREE.PCFSoftShadowMap;
  renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
}

// ───────────── 1. Studio environment (IBL) ─────────────
function createStudioEnvironment(renderer, scene) {
  const W = 512, H = 256;
  const c = document.createElement('canvas');
  c.width = W; c.height = H;
  const g = c.getContext('2d');

  // Base: dim neutral studio, vertical gradient (top = ceiling, bottom = floor)
  const base = g.createLinearGradient(0, 0, 0, H);
  base.addColorStop(0.00, '#2a2e36');
  base.addColorStop(0.45, '#14161b');
  base.addColorStop(0.50, '#101216');
  base.addColorStop(0.55, '#1a1b1e');
  base.addColorStop(1.00, '#3a3834'); // floor bounce
  g.fillStyle = base;
  g.fillRect(0, 0, W, H);

  // Soft-edged elliptical softbox (wraps horizontally across the seam)
  const softbox = (cx, cy, rx, ry, inner, outer) => {
    for (const ox of [-W, 0, W]) {
      g.save();
      g.translate(cx + ox, cy);
      g.scale(rx, ry);
      const rg = g.createRadialGradient(0, 0, 0, 0, 0, 1);
      rg.addColorStop(0.00, inner);
      rg.addColorStop(0.45, inner);
      rg.addColorStop(1.00, outer);
      g.fillStyle = rg;
      g.beginPath(); g.arc(0, 0, 1, 0, Math.PI * 2); g.fill();
      g.restore();
    }
  };

  g.globalCompositeOperation = 'lighter';
  softbox(W * 0.15, H * 0.30, 90, 48, 'rgba(255,214,170,1.0)', 'rgba(255,214,170,0)');  // warm key
  softbox(W * 0.62, H * 0.34, 70, 36, 'rgba(150,190,255,0.85)', 'rgba(150,190,255,0)'); // cool rim
  softbox(W * 0.88, H * 0.22, 60, 28, 'rgba(210,220,255,0.45)', 'rgba(210,220,255,0)'); // soft fill
  softbox(W * 0.40, H * 0.08, 120, 18, 'rgba(255,255,255,0.55)', 'rgba(255,255,255,0)'); // overhead strip

  // Floor gradient glow (warm bounce below horizon)
  const fl = g.createLinearGradient(0, H * 0.5, 0, H);
  fl.addColorStop(0, 'rgba(90,84,76,0)');
  fl.addColorStop(1, 'rgba(120,112,100,0.55)');
  g.fillStyle = fl;
  g.fillRect(0, H * 0.5, W, H * 0.5);
  g.globalCompositeOperation = 'source-over';

  const tex = new THREE.CanvasTexture(c);
  tex.encoding = THREE.sRGBEncoding;
  tex.mapping = THREE.EquirectangularReflectionMapping;

  const pmrem = new THREE.PMREMGenerator(renderer);
  pmrem.compileEquirectangularShader();
  const envMap = pmrem.fromEquirectangular(tex).texture;

  scene.environment = envMap;   // background left untouched
  tex.dispose();
  pmrem.dispose();
  return envMap;
}

// ───────────── 2. Contact shadow ─────────────
function createContactShadow() {
  const S = 128;
  const c = document.createElement('canvas');
  c.width = c.height = S;
  const g = c.getContext('2d');
  const rg = g.createRadialGradient(S / 2, S / 2, 0, S / 2, S / 2, S / 2);
  rg.addColorStop(0.00, 'rgba(0,0,0,0.7)');
  rg.addColorStop(0.35, 'rgba(0,0,0,0.45)');
  rg.addColorStop(0.70, 'rgba(0,0,0,0.12)');
  rg.addColorStop(1.00, 'rgba(0,0,0,0)');
  g.fillStyle = rg;
  g.fillRect(0, 0, S, S);

  const tex = new THREE.CanvasTexture(c);
  tex.encoding = THREE.sRGBEncoding;

  const mesh = new THREE.Mesh(
    new THREE.PlaneGeometry(0.5, 0.5),
    new THREE.MeshBasicMaterial({
      map: tex, transparent: true, depthWrite: false, toneMapped: false,
      polygonOffset: true, polygonOffsetFactor: -1, polygonOffsetUnits: -1
    })
  );
  mesh.rotation.x = -Math.PI / 2;
  mesh.position.y = 0.002;
  mesh.renderOrder = 1;
  return mesh;
}

// Shadow follows the root on the floor; fades and widens as the foot lifts.
function updateContactShadow(mesh, rootPos, footAltitude) {
  const a = Math.max(0, footAltitude || 0);
  const k = Math.exp(-a / 0.12);                 // 1 on ground → 0 airborne
  mesh.position.set(rootPos.x, 0.002, rootPos.z);
  mesh.scale.setScalar(1 + a * 3.0);
  mesh.material.opacity = THREE.MathUtils.clamp(k, 0, 1);
  mesh.visible = mesh.material.opacity > 0.01;
}

// ───────────── 3. PBR materials ─────────────
const _matCache = new Map();

function getPartMaterial(role, rgb, bodyIdx = 0) {
  const norm = rgb.map(v => (v > 1 ? v / 255 : v));
  const key = `${role}|${norm.join(',')}|${bodyIdx}`;
  if (_matCache.has(key)) return _matCache.get(key);

  const P = {
    shell:    { roughness: 0.32, metalness: 0.05, envMapIntensity: 1.2 },
    trim:     { roughness: 0.48, metalness: 0.02, envMapIntensity: 0.8 },
    beak:     { roughness: 0.48, metalness: 0.02, envMapIntensity: 0.8 },
    sole:     { roughness: 0.60, metalness: 0.02, envMapIntensity: 0.5 },
    internal: { roughness: 0.28, metalness: 0.75, envMapIntensity: 1.4 },
    metal:    { roughness: 0.28, metalness: 0.75, envMapIntensity: 1.4 },
    wheel:    { roughness: 0.22, metalness: 0.15, envMapIntensity: 1.3 },
    roller:   { roughness: 0.22, metalness: 0.15, envMapIntensity: 1.3 }
  };
  const p = P[role] || P.shell;

  const mat = new THREE.MeshStandardMaterial({
    color: new THREE.Color(norm[0], norm[1], norm[2]),
    roughness: p.roughness,
    metalness: p.metalness,
    envMapIntensity: p.envMapIntensity
  });
  mat.name = `${role}_${bodyIdx}`;
  _matCache.set(key, mat);
  return mat;
}

function applyShadowFlags(root) {
  root.traverse(o => { if (o.isMesh) { o.castShadow = true; o.receiveShadow = true; } });
}

// ───────────── 4. Studio lighting ─────────────
function setupStudioLighting(scene, { keyLight, rimLight, hemisphereLight, fillLight } = {}) {
  keyLight = keyLight || new THREE.DirectionalLight();
  rimLight = rimLight || new THREE.DirectionalLight();
  fillLight = fillLight || new THREE.DirectionalLight();
  hemisphereLight = hemisphereLight || new THREE.HemisphereLight();

  // Key: warm, front-right, high, soft shadows
  keyLight.color.set(0xfff0dc);
  keyLight.intensity = 2.2;
  keyLight.position.set(2.5, 4.0, 2.5);
  keyLight.castShadow = true;
  keyLight.shadow.mapSize.set(2048, 2048);
  keyLight.shadow.radius = 4;
  keyLight.shadow.bias = -0.0004;
  keyLight.shadow.normalBias = 0.02;
  const sc = keyLight.shadow.camera;
  sc.near = 0.5; sc.far = 12;
  sc.left = sc.bottom = -2.5; sc.right = sc.top = 2.5;
  sc.updateProjectionMatrix();

  // Fill: cool, front-left, low, no shadows
  fillLight.color.set(0xb4ccff);
  fillLight.intensity = 0.6;
  fillLight.position.set(-3.0, 1.8, 2.0);
  fillLight.castShadow = false;

  // Rim: cool-white, behind/above, edge separation
  rimLight.color.set(0xa8c8ff);
  rimLight.intensity = 1.6;
  rimLight.position.set(-1.5, 3.0, -3.5);
  rimLight.castShadow = false;

  // Hemisphere: very subtle since IBL provides ambient
  hemisphereLight.color.set(0xdfe8ff);
  hemisphereLight.groundColor.set(0x2a2622);
  hemisphereLight.intensity = 0.25;

  [keyLight, fillLight, rimLight, hemisphereLight].forEach(l => {
    if (!l.parent) scene.add(l);
  });
  return { keyLight, fillLight, rimLight, hemisphereLight };
}

// ───────────── Usage ─────────────
// configureRenderer(renderer);
// createStudioEnvironment(renderer, scene);
// setupStudioLighting(scene, { keyLight, rimLight, hemisphereLight });
// const shadow = createContactShadow(); scene.add(shadow);
// floor.receiveShadow = true;
// floor.material.roughness = 0.9; floor.material.envMapIntensity = 0.3;
// applyShadowFlags(robotRoot);
// per frame: updateContactShadow(shadow, rootPos, footAltitude);
```