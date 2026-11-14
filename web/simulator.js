(() => {
  "use strict";

  // ---------------------------------------------------------------------------
  // 1. Ground-Truth Constants (matching duck-control/src/model.rs & obs.rs)
  // ---------------------------------------------------------------------------
  const JOINT_NAMES = [
    "left_hip_yaw", "left_hip_roll", "left_hip_pitch", "left_knee", "left_ankle",
    "neck_pitch", "head_pitch", "head_yaw", "head_roll", "mouth",
    "right_hip_yaw", "right_hip_roll", "right_hip_pitch", "right_knee", "right_ankle",
  ];
  const JOINT_IDS = [11, 12, 13, 14, 15, 21, 22, 23, 24, 25, 31, 32, 33, 34, 35];
  const MOUTH_INDEX = 9;

  const DEFAULT_POSITION = [
    0.0, -0.0873, -0.4579, -0.0049, 0.4530,
    0.3491, 0.3491, 0.0, 0.0, 0.0,
    0.0, 0.0873, 0.4579, 0.0049, -0.4530,
  ];

  const REST_POSITION = [
    0.3129, 0.4050, -0.2338, 1.5493, 0.5522,
    0.9572, 1.4787, -0.0129, -0.0522, -0.0121,
    0.0353, -0.5660, 0.2853, -1.4696, -0.4096,
  ];

  const PAUSE_POSE_OFFSETS = [
    0.0, -0.100, 0.038, -0.145, 0.044,
    -0.096, 0.008, 0.0, 0.0,
    0.0, 0.100, -0.038, 0.145, -0.044,
  ];

  const MOUTH_CLOSED = (-5.0 * Math.PI) / 180.0;
  const MOUTH_OPEN = (30.0 * Math.PI) / 180.0;

  function mouthTarget(openFrac) {
    const f = Number.isFinite(openFrac) ? Math.max(0, Math.min(1, openFrac)) : 0;
    return MOUTH_CLOSED + f * (MOUTH_OPEN - MOUTH_CLOSED);
  }

  // ---------------------------------------------------------------------------
  // 2. DOM Elements
  // ---------------------------------------------------------------------------
  const canvas3d = document.getElementById("sim-3d-canvas");
  const obsCanvas = document.getElementById("obs-canvas");
  const obsCtx = obsCanvas.getContext("2d");

  const pillHealth = document.getElementById("pill-health");
  const pillMode = document.getElementById("pill-mode");
  const skillStatusTag = document.getElementById("skill-status-tag");
  const mOdom = document.getElementById("m-odom");
  const mBrain = document.getElementById("m-brain");
  const mSafety = document.getElementById("m-safety");
  const mTrips = document.getElementById("m-trips");
  const hudPoseReadout = document.getElementById("hud-pose-readout");
  const logFeed = document.getElementById("log-feed");
  const jointsTbody = document.getElementById("joints-tbody");
  const tofGrid = document.getElementById("tof-grid");

  const sliderVx = document.getElementById("slider-vx");
  const sliderVy = document.getElementById("slider-vy");
  const sliderVyaw = document.getElementById("slider-vyaw");
  const sliderHeadYaw = document.getElementById("slider-head-yaw");
  const sliderHeadPitch = document.getElementById("slider-head-pitch");
  const sliderMouth = document.getElementById("slider-mouth");
  const sliderTheremin = document.getElementById("slider-theremin");

  const valVx = document.getElementById("val-vx");
  const valVy = document.getElementById("val-vy");
  const valVyaw = document.getElementById("val-vyaw");
  const valHeadYaw = document.getElementById("val-head-yaw");
  const valHeadPitch = document.getElementById("val-head-pitch");
  const valMouth = document.getElementById("val-mouth");
  const valTheremin = document.getElementById("val-theremin");

  const btnToggleMesh = document.getElementById("btn-toggle-mesh");
  const btnToggleTofRays = document.getElementById("btn-toggle-tof-rays");
  const btnModeAutod = document.getElementById("btn-mode-autod");
  const btnModeLaser = document.getElementById("btn-mode-laser");
  const btnModeSoccer = document.getElementById("btn-mode-soccer");
  const btnCamFollow = document.getElementById("btn-cam-follow");
  const btnTheremin = document.getElementById("btn-theremin");
  const thereminHz = document.getElementById("theremin-hz");
  const choraleBeat = document.getElementById("chorale-beat");
  const selectGuard = document.getElementById("select-guard");

  // ---------------------------------------------------------------------------
  // 3. Simulation State
  // ---------------------------------------------------------------------------
  const state = {
    time: 0,
    dt: 0.02, // 50 Hz control tick
    dpadMode: "move",
    locomotionMode: "walk", // walk | roller
    rollerSkatesForced: true, // Visible by default on feet
    brainMode: "manual", // manual | autod | laser | soccer
    activeMesh: "cad", // cad | flux_ai
    showTofRays: true,
    followCam: true,

    cmd: {
      vx: 0,
      vy: 0,
      vyaw: 0,
      neckPitch: 0,
      headPitch: 0,
      headYaw: 0,
      headRoll: 0,
      bodyZ: 0,
      bodyRoll: 0,
      bodyPitch: 0,
      mouthOpen: 0,
    },

    joints: [...DEFAULT_POSITION],
    velocities: new Array(15).fill(0),
    prevAction14: new Array(14).fill(0),
    obs61: new Array(61).fill(0),
    impactGuardStates: new Array(15).fill(0), // 0 armed, 1 braking, 2 soft hold

    pose: { x: 0, z: 0, yaw: 0, liftY: 0, pitchFall: 0, rollFall: 0 },
    ball: { x: 0.55, z: -0.25, vx: 0, vz: 0 },
    laser: { x: 0.45, z: 0.30, active: false },

    isSeated: false,
    isResting: false,
    isPickedUp: false,
    pickupElapsed: 0,
    pickupStartPose: [...DEFAULT_POSITION],

    isFallen: false,
    fallTrips: 0,
    fallCount: 0,
    guardLevel: "R",

    customJointOverrides: {},
    customScript: {
      active: false,
      fn: null,
      lastError: null,
    },

    activeSkill: null, // { name, elapsed, duration }
    activeEmotion: null, // { name, keyframes, elapsed, duration }

    thereminOn: false,
    choraleOn: false,
    choraleTick: 0,
    autodState: "Wander",
    autodTimer: 0,
    autodGoal: { x: 0.5, z: 0.2 },

    vetbook: {
      name: "duck-alpha-sim",
      boots: 1,
      metersWalked: 0,
      kicks: 0,
      quacks: 0,
      emotionsPlayed: 0,
      songsSung: 0,
      falls: 0,
      pickups: 0,
    },
  };

  function pushLog(tag, msg) {
    const div = document.createElement("div");
    const ts = state.time.toFixed(1).padStart(5, " ");
    div.textContent = `[${ts}s] [${tag}] ${msg}`;
    logFeed.prepend(div);
    while (logFeed.children.length > 40) {
      logFeed.removeChild(logFeed.lastChild);
    }
  }

  // ---------------------------------------------------------------------------
  // 4. Web Audio Engine (Duck Quack, 48kHz-style Formant Theremin & 4-Part Chorale)
  // ---------------------------------------------------------------------------
  let audioCtx = null;
  let thereminOsc = null;
  let thereminOsc2 = null;
  let thereminGain = null;

  function ensureAudio() {
    if (!audioCtx) {
      const Ctx = window.AudioContext || window.webkitAudioContext;
      if (Ctx) audioCtx = new Ctx();
    }
    if (audioCtx && audioCtx.state === "suspended") {
      audioCtx.resume();
    }
    return audioCtx;
  }

  function playProceduralQuack(pitchMult = 1.0) {
    const ctx = ensureAudio();
    if (!ctx) return;
    const now = ctx.currentTime;
    const osc = ctx.createOscillator();
    const filter = ctx.createBiquadFilter();
    const gain = ctx.createGain();
    osc.type = "sawtooth";
    osc.frequency.setValueAtTime(340 * pitchMult, now);
    osc.frequency.exponentialRampToValueAtTime(210 * pitchMult, now + 0.14);

    filter.type = "bandpass";
    filter.frequency.setValueAtTime(1150, now);
    filter.frequency.exponentialRampToValueAtTime(780, now + 0.14);
    filter.Q.value = 3.2;

    gain.gain.setValueAtTime(0.001, now);
    gain.gain.linearRampToValueAtTime(0.18, now + 0.02);
    gain.gain.exponentialRampToValueAtTime(0.001, now + 0.18);

    osc.connect(filter);
    filter.connect(gain);
    gain.connect(ctx.destination);
    osc.start(now);
    osc.stop(now + 0.19);
    state.vetbook.quacks += 1;
    state.cmd.mouthOpen = 0.85;
    setTimeout(() => {
      if (!state.thereminOn && !state.activeEmotion) state.cmd.mouthOpen = Number(sliderMouth.value);
    }, 180);
  }

  function updateThereminAudio(active, freqHz) {
    if (!active || !state.thereminOn) {
      if (thereminGain && audioCtx) {
        thereminGain.gain.setTargetAtTime(0.0001, audioCtx.currentTime, 0.04);
      }
      return;
    }
    const ctx = ensureAudio();
    if (!ctx) return;
    if (!thereminOsc) {
      thereminOsc = ctx.createOscillator();
      thereminOsc2 = ctx.createOscillator();
      const filt = ctx.createBiquadFilter();
      thereminGain = ctx.createGain();

      thereminOsc.type = "sawtooth";
      thereminOsc2.type = "triangle";
      filt.type = "bandpass";
      filt.frequency.value = 1100;
      filt.Q.value = 2.5;
      thereminGain.gain.value = 0.0001;

      thereminOsc.connect(filt);
      thereminOsc2.connect(filt);
      filt.connect(thereminGain);
      thereminGain.connect(ctx.destination);
      thereminOsc.start();
      thereminOsc2.start();
    }
    thereminOsc.frequency.setTargetAtTime(freqHz, ctx.currentTime, 0.03);
    thereminOsc2.frequency.setTargetAtTime(freqHz * 2.0, ctx.currentTime, 0.03);
    thereminGain.gain.setTargetAtTime(0.08, ctx.currentTime, 0.04);
  }

  // 4-part Chorale chord sequence inspired by wistful.duckscore
  const WISTFUL_CHORDS_HZ = [
    [130.81, 196.0, 261.63, 329.63], // C3 G3 C4 E4
    [123.47, 196.0, 246.94, 349.23], // B2 G3 B3 F4
    [110.0, 174.61, 220.0, 329.63],  // A2 F3 A3 E4
    [103.83, 155.56, 207.65, 311.13],// Ab2 Eb3 Ab3 Eb4 (borrowed minor)
    [130.81, 196.0, 261.63, 392.0],  // C3 G3 C4 G4
  ];

  let choraleInterval = null;
  function playChoraleChord(freqs) {
    const ctx = ensureAudio();
    if (!ctx) return;
    const now = ctx.currentTime;
    freqs.forEach((f, idx) => {
      const osc = ctx.createOscillator();
      const filt = ctx.createBiquadFilter();
      const gain = ctx.createGain();
      osc.type = idx === 0 ? "triangle" : "sawtooth";
      osc.frequency.setValueAtTime(f, now);
      filt.type = "lowpass";
      filt.frequency.setValueAtTime(950 + idx * 180, now);
      gain.gain.setValueAtTime(0.001, now);
      gain.gain.linearRampToValueAtTime(0.045, now + 0.14);
      gain.gain.exponentialRampToValueAtTime(0.001, now + 1.05);
      osc.connect(filt);
      filt.connect(gain);
      gain.connect(ctx.destination);
      osc.start(now);
      osc.stop(now + 1.1);
    });
  }

  // ---------------------------------------------------------------------------
  // 5. Three.js 3D Scene, Authentic CAD Rig (`duck.bin`) & FLUX AI GLB Loader
  // ---------------------------------------------------------------------------
  const renderer = new THREE.WebGLRenderer({ canvas: canvas3d, antialias: true, alpha: true });
  renderer.setPixelRatio(Math.min(window.devicePixelRatio || 1, 2));
  renderer.shadowMap.enabled = true;
  renderer.shadowMap.type = THREE.PCFSoftShadowMap;
  renderer.outputEncoding = THREE.sRGBEncoding;
  renderer.toneMapping = THREE.ACESFilmicToneMapping;
  renderer.toneMappingExposure = 1.05;

  const scene = new THREE.Scene();
  scene.background = new THREE.Color(0x0b101b);
  scene.fog = new THREE.FogExp2(0x0b101b, 0.14);

  const camera = new THREE.PerspectiveCamera(42, 16 / 9, 0.02, 40);
  camera.position.set(0.55, 0.36, 0.58);

  const controls = new THREE.OrbitControls(camera, canvas3d);
  controls.enableDamping = true;
  controls.dampingFactor = 0.08;
  controls.rotateSpeed = 0.75;
  controls.zoomSpeed = 0.85;
  controls.target.set(0, 0.14, 0);
  controls.minDistance = 0.20;
  controls.maxDistance = 3.5;
  controls.maxPolarAngle = Math.PI * 0.495; // Never dip below the floor

  // Lighting
  scene.add(new THREE.HemisphereLight(0xdbeafe, 0x1e293b, 0.65));
  const keyLight = new THREE.DirectionalLight(0xfffbeb, 1.35);
  keyLight.position.set(1.8, 3.0, 2.2);
  keyLight.castShadow = true;
  keyLight.shadow.mapSize.set(2048, 2048);
  keyLight.shadow.camera.near = 0.1;
  keyLight.shadow.camera.far = 10;
  keyLight.shadow.camera.left = -2;
  keyLight.shadow.camera.right = 2;
  keyLight.shadow.camera.top = 2;
  keyLight.shadow.camera.bottom = -2;
  scene.add(keyLight);

  const rimLight = new THREE.DirectionalLight(0x38bdf8, 0.65);
  rimLight.position.set(-2.0, 1.4, -1.8);
  scene.add(rimLight);

  // Arena Floor + Grid
  const floorGeo = new THREE.PlaneGeometry(14, 14);
  const floorMat = new THREE.MeshStandardMaterial({
    color: 0x0f172a,
    roughness: 0.78,
    metalness: 0.08,
  });
  const floorMesh = new THREE.Mesh(floorGeo, floorMat);
  floorMesh.rotation.x = -Math.PI / 2;
  floorMesh.receiveShadow = true;
  scene.add(floorMesh);

  const gridHelper = new THREE.GridHelper(6, 60, 0x334155, 0x1e293b);
  gridHelper.position.y = 0.001;
  scene.add(gridHelper);

  // Contact shadow disc under robot
  const shadowDisc = new THREE.Mesh(
    new THREE.CircleGeometry(0.11, 32),
    new THREE.MeshBasicMaterial({ color: 0x000000, transparent: true, opacity: 0.45 })
  );
  shadowDisc.rotation.x = -Math.PI / 2;
  shadowDisc.position.y = 0.002;
  scene.add(shadowDisc);

  // Purple Soccer Ball
  const ballMesh = new THREE.Mesh(
    new THREE.SphereGeometry(0.042, 24, 24),
    new THREE.MeshStandardMaterial({ color: 0x9333ea, roughness: 0.35, metalness: 0.1 })
  );
  ballMesh.castShadow = true;
  ballMesh.position.set(state.ball.x, 0.042, state.ball.z);
  scene.add(ballMesh);

  // Red Laser Dot
  const laserDotMesh = new THREE.Mesh(
    new THREE.CircleGeometry(0.018, 24),
    new THREE.MeshBasicMaterial({ color: 0xff2244 })
  );
  laserDotMesh.rotation.x = -Math.PI / 2;
  laserDotMesh.position.set(state.laser.x, 0.004, state.laser.z);
  laserDotMesh.visible = false;
  scene.add(laserDotMesh);

  // Duck World Root (Y-up world placement)
  const duckWorldRoot = new THREE.Group();
  scene.add(duckWorldRoot);

  // Z-up -> Y-up adapter group
  const duckCadContainer = new THREE.Group();
  duckCadContainer.rotation.x = -Math.PI / 2;
  duckWorldRoot.add(duckCadContainer);

  // FLUX AI Mesh Container
  const fluxAiContainer = new THREE.Group();
  fluxAiContainer.visible = false;
  duckWorldRoot.add(fluxAiContainer);

  // Companion Choir Ducks Group (for 4-Duck Chorale)
  const choirGroup = new THREE.Group();
  choirGroup.visible = false;
  scene.add(choirGroup);

  // 8x8 ToF 3D Ray Dots
  const tofMarkersGroup = new THREE.Group();
  scene.add(tofMarkersGroup);
  const tofMarkerMeshes = [];
  const tofDotGeo = new THREE.SphereGeometry(0.006, 8, 8);
  for (let i = 0; i < 64; i++) {
    const m = new THREE.Mesh(
      tofDotGeo,
      new THREE.MeshBasicMaterial({ color: 0x22d3ee })
    );
    tofMarkersGroup.add(m);
    tofMarkerMeshes.push(m);
  }

  let cadBodyGroups = [];
  let cadRigData = null;
  const rollerWheels = [];
  let tireAssetData = null;

  function tryAttachRollerWheels() {
    if (!tireAssetData || !cadBodyGroups || cadBodyGroups.length < 15 || rollerWheels.length > 0) return;
    const g = new THREE.BufferGeometry();
    const verts = new Float32Array(tireAssetData.verts.flat());
    const indices = new Uint32Array(tireAssetData.faces.flat());
    g.setAttribute("position", new THREE.BufferAttribute(verts, 3));
    g.setIndex(new THREE.BufferAttribute(indices, 1));
    g.computeVertexNormals();

    const mat = new THREE.MeshStandardMaterial({
      color: 0x00d8f6, // Vibrant cyan polyurethane skate tire
      roughness: 0.35,
      metalness: 0.15,
      flatShading: true,
    });
    const radius = tireAssetData.radius_m || 0.015;
    // Ensure world matrices reflect Home Pose before sampling foot bounds
    applyJointsToCadGroups(cadBodyGroups, DEFAULT_POSITION);
    duckWorldRoot.updateMatrixWorld(true);

    // Target world orientation: wheel axle along world Z axis (rolls forward along X)
    const targetWorldQuat = new THREE.Quaternion().setFromAxisAngle(new THREE.Vector3(0, 1, 0), Math.PI / 2);

    // Mount 2 wheels on outer edge of left foot (body 5) and right foot (body 14)
    [
      { bodyIdx: 5, sideSign: -1 }, // Left foot (Z < 0)
      { bodyIdx: 14, sideSign: 1 },  // Right foot (Z > 0)
    ].forEach(({ bodyIdx, sideSign }) => {
      const ankleGrp = cadBodyGroups[bodyIdx];
      const box = new THREE.Box3().setFromObject(ankleGrp);
      const center = new THREE.Vector3();
      const size = new THREE.Vector3();
      box.getCenter(center);
      box.getSize(size);

      const ankleWorldQuat = new THREE.Quaternion();
      ankleGrp.getWorldQuaternion(ankleWorldQuat);
      const localWheelQuat = ankleWorldQuat.clone().invert().multiply(targetWorldQuat);

      [-0.018, 0.018].forEach((xOff) => {
        const w = new THREE.Mesh(g, mat);
        w.castShadow = true;
        w.receiveShadow = true;
        const worldPos = new THREE.Vector3(
          center.x + xOff,
          radius,
          center.z + sideSign * (size.z * 0.5 + 0.008)
        );
        ankleGrp.worldToLocal(worldPos);
        w.position.copy(worldPos);
        w.quaternion.copy(localWheelQuat);
        w.userData.radius = radius;
        w.visible = state.locomotionMode === "roller" || state.rollerSkatesForced;
        ankleGrp.add(w);
        rollerWheels.push(w);
      });
    });

    const btnToggleRollerSkates = document.getElementById("btn-toggle-roller-skates");
    if (btnToggleRollerSkates && state.rollerSkatesForced) {
      btnToggleRollerSkates.classList.add("active");
    }
    pushLog("STL", `Mounted 4 real CAD wheels from V5_AUDIO_VIDEO_TIRE_01.stl (${tireAssetData.faces.length} tris, r=${(radius * 1000).toFixed(1)}mm).`);
  }

  function buildCadRigInThree(rig, targetContainer) {
    // Build Three.js BufferGeometries for the 29 unique CAD meshes
    const geometries = rig.meshes.map((m) => {
      const g = new THREE.BufferGeometry();
      const verts = new Float32Array(m.verts.flat());
      const indices = new Uint16Array(m.tris.flat());
      g.setAttribute("position", new THREE.BufferAttribute(verts, 3));
      g.setIndex(new THREE.BufferAttribute(indices, 1));
      g.computeVertexNormals();
      return g;
    });

    const groups = rig.bodies.map((b) => {
      const grp = new THREE.Group();
      grp.position.set(b.pos[0], b.pos[1], b.pos[2]);
      // MJCF quat is (w, x, y, z); Three.js is (x, y, z, w)
      grp.userData.baseQuat = new THREE.Quaternion(b.quat[1], b.quat[2], b.quat[3], b.quat[0]).normalize();
      grp.userData.axis = new THREE.Vector3(b.axis[0], b.axis[1], b.axis[2]).normalize();
      grp.userData.jointIdx = b.joint;
      grp.quaternion.copy(grp.userData.baseQuat);
      return grp;
    });

    rig.bodies.forEach((b, idx) => {
      if (b.parent >= 0) {
        groups[b.parent].add(groups[idx]);
      } else {
        targetContainer.add(groups[idx]);
      }
    });

    rig.parts.forEach((p) => {
      const role = classifyCadPartRole(p.rgb, p.body);
      const isPlastic = role !== null;
      const mat = new THREE.MeshStandardMaterial({
        color: new THREE.Color(p.rgb[0] / 255, p.rgb[1] / 255, p.rgb[2] / 255),
        roughness: isPlastic ? 0.65 : 0.48,
        metalness: isPlastic ? 0.05 : 0.18,
        flatShading: true,
        side: THREE.DoubleSide,
      });
      const mesh = new THREE.Mesh(geometries[p.mesh], mat);
      mesh.castShadow = true;
      mesh.receiveShadow = true;
      mesh.position.set(p.pos[0], p.pos[1], p.pos[2]);
      mesh.quaternion.set(p.quat[1], p.quat[2], p.quat[3], p.quat[0]).normalize();
      mesh.userData.baseRgb = p.rgb;
      mesh.userData.bodyIdx = p.body;
      groups[p.body].add(mesh);
    });

    return groups;
  }

  const HARDWARE_COLORWAYS = {
    white_orange: { label: "Classic White & Orange", shell: 0xf4f1ea, trim: 0xf97316, eyeRing: 0xfacc15, sole: 0xfacc15 },
    sky_blue:     { label: "Sky Blue & Orange",      shell: 0xb5e8f7, trim: 0xf97316, eyeRing: 0xfacc15, sole: 0xfacc15 },
    lavender_yellow: { label: "Lavender & Cyan",     shell: 0xcbb7e8, trim: 0xfacc15, eyeRing: 0x99e2e8, sole: 0xfacc15 },
    slate_purple: { label: "Stealth Slate & Purple", shell: 0x525866, trim: 0xfacc15, eyeRing: 0x9d6be8, sole: 0x9d6be8 },
    cad_raw:      null,
  };

  function classifyCadPartRole(rgb, bodyIdx) {
    const key = `${rgb[0]},${rgb[1]},${rgb[2]}`;
    if (key === "234,234,234" || key === "137,218,211" || key === "217,193,221" || key === "196,226,243") {
      return "shell";
    }
    if (key === "248,135,1" || key === "232,146,42" || key === "138,174,214") {
      return "trim";
    }
    if (key === "250,182,1") {
      return bodyIdx === 9 ? "eyeRing" : "sole";
    }
    return null;
  }

  function applyColorway(targetRoot, colorwayKey) {
    const palette = HARDWARE_COLORWAYS[colorwayKey] || null;
    targetRoot.traverse((child) => {
      if (!child.isMesh || !child.userData.baseRgb) return;
      const base = child.userData.baseRgb;
      if (!palette) {
        child.material.color.setRGB(base[0] / 255, base[1] / 255, base[2] / 255);
        return;
      }
      const role = classifyCadPartRole(base, child.userData.bodyIdx);
      if (role && palette[role] !== undefined) {
        child.material.color.setHex(palette[role]);
      } else {
        child.material.color.setRGB(base[0] / 255, base[1] / 255, base[2] / 255);
      }
    });
  }

  function applyJointsToCadGroups(groups, joints15) {
    if (!groups || !groups.length) return;
    const tmpQ = new THREE.Quaternion();
    for (let i = 0; i < groups.length; i++) {
      const g = groups[i];
      g.quaternion.copy(g.userData.baseQuat);
      const j = g.userData.jointIdx;
      if (j >= 0 && j < joints15.length) {
        tmpQ.setFromAxisAngle(g.userData.axis, joints15[j]);
        g.quaternion.multiply(tmpQ);
      }
    }
  }

  // Load ground-truth CAD rig & FLUX AI GLB
  fetch("assets/models/microduck_cad_rig.json")
    .then((r) => r.json())
    .then((rig) => {
      cadRigData = rig;
      cadBodyGroups = buildCadRigInThree(rig, duckCadContainer);
      // Shift Z-up container slightly so Home Pose feet rest flush on Y=0
      const footOffset = rig.home_pose_stats ? rig.home_pose_stats.foot_offset_y : 0;
      duckCadContainer.position.y = footOffset;
      applyJointsToCadGroups(cadBodyGroups, state.joints);
      const selectColorway = document.getElementById("select-colorway");
      applyColorway(duckCadContainer, selectColorway ? selectColorway.value : "white_orange");
      tryAttachRollerWheels();
      pushLog("CAD", `Loaded duck.bin rig (${rig.bodies.length} bodies, ${rig.parts.length} parts).`);

      // Build 3 companion ducks for the 4-Duck Chorale with distinct authentic hardware colorways
      const choirOffsets = [
        { x: -0.35, z: -0.18, yaw: 0.45, colorway: "sky_blue" },
        { x: 0.35, z: -0.18, yaw: -0.45, colorway: "lavender_yellow" },
        { x: 0.0, z: -0.38, yaw: 0.0, colorway: "slate_purple" },
      ];
      choirOffsets.forEach((cfg) => {
        const cRoot = new THREE.Group();
        cRoot.position.set(cfg.x, 0, cfg.z);
        cRoot.rotation.y = cfg.yaw;
        const cCad = new THREE.Group();
        cCad.rotation.x = -Math.PI / 2;
        cCad.position.y = footOffset;
        cRoot.add(cCad);
        const cGroups = buildCadRigInThree(rig, cCad);
        applyJointsToCadGroups(cGroups, DEFAULT_POSITION);
        applyColorway(cCad, cfg.colorway);
        choirGroup.add(cRoot);
      });
    })
    .catch((err) => pushLog("ERR", `CAD rig load failed: ${err}`));

  fetch("assets/models/xgoduck_v5_tire.json")
    .then((r) => r.json())
    .then((tire) => {
      tireAssetData = tire;
      tryAttachRollerWheels();
    })
    .catch((err) => pushLog("STL", `Roller wheel STL pending: ${err}`));

  const gltfLoader = new THREE.GLTFLoader();
  gltfLoader.load(
    "assets/models/xgoduck_v5_stl_kit.glb",
    (gltf) => {
      const model = gltf.scene;
      model.traverse((child) => {
        if (child.isMesh) {
          child.castShadow = true;
          child.receiveShadow = true;
        }
      });
      fluxAiContainer.add(model);
      pushLog("STL-KIT", "Loaded XGO-Duck V5 50-STL Kit Platter (xgoduck_v5_stl_kit.glb).");
    },
    undefined,
    () => {
      pushLog("STL-KIT", "XGO-Duck V5 STL Kit GLB not loaded; articulated CAD rig active.");
    }
  );

  // Raycast click on arena floor to move Laser Dot
  const raycaster = new THREE.Raycaster();
  const pointer = new THREE.Vector2();
  let pointerDownPos = { x: 0, y: 0 };
  canvas3d.addEventListener("pointerdown", (e) => {
    pointerDownPos = { x: e.clientX, y: e.clientY };
  });
  canvas3d.addEventListener("pointerup", (e) => {
    const dist = Math.hypot(e.clientX - pointerDownPos.x, e.clientY - pointerDownPos.y);
    if (dist > 5) return; // Was an orbit drag
    const rect = canvas3d.getBoundingClientRect();
    pointer.x = ((e.clientX - rect.left) / rect.width) * 2 - 1;
    pointer.y = -((e.clientY - rect.top) / rect.height) * 2 + 1;
    raycaster.setFromCamera(pointer, camera);
    const hits = raycaster.intersectObject(floorMesh);
    if (hits.length > 0) {
      const pt = hits[0].point;
      state.laser.x = Math.max(-2.5, Math.min(2.5, pt.x));
      state.laser.z = Math.max(-2.5, Math.min(2.5, pt.z));
      state.laser.active = true;
      laserDotMesh.position.set(state.laser.x, 0.004, state.laser.z);
      laserDotMesh.visible = true;
      pushLog("mediad", `Laser dot placed at (${state.laser.x.toFixed(2)}, ${state.laser.z.toFixed(2)}) m`);
    }
  });

  // ---------------------------------------------------------------------------
  // 6. Real Emotions Loader (JSON keyframes + WAV audio)
  // ---------------------------------------------------------------------------
  async function playRealEmotion(name) {
    try {
      const resp = await fetch(`assets/emotions/${name}.json`);
      const data = await resp.json();
      const kfs = data.keyframes || [];
      if (!kfs.length) return;
      state.activeEmotion = {
        name,
        keyframes: kfs,
        elapsed: 0,
        duration: kfs[kfs.length - 1].t || 2.0,
      };
      state.vetbook.emotionsPlayed += 1;
      const audio = new Audio(`assets/emotions/${name}.wav`);
      audio.play().catch(() => playProceduralQuack(1.1));
      pushLog("robotd", `Emotion "${name}" (${state.activeEmotion.duration.toFixed(1)}s, ${kfs.length} keyframes)`);
    } catch (err) {
      pushLog("ERR", `Failed to load emotion ${name}: ${err}`);
    }
  }

  function sampleEmotion(em, t) {
    const kfs = em.keyframes;
    if (t <= kfs[0].t) return kfs[0];
    if (t >= kfs[kfs.length - 1].t) return kfs[kfs.length - 1];
    for (let i = 0; i < kfs.length - 1; i++) {
      const a = kfs[i];
      const b = kfs[i + 1];
      if (t >= a.t && t <= b.t) {
        const u = (t - a.t) / Math.max(1e-5, b.t - a.t);
        const lerp = (k) => (a[k] || 0) + ((b[k] || 0) - (a[k] || 0)) * u;
        return {
          neck: lerp("neck"),
          head_pitch: lerp("head_pitch"),
          head_yaw: lerp("head_yaw"),
          head_roll: lerp("head_roll"),
          body_pitch: lerp("body_pitch"),
          body_z: lerp("body_z"),
          mouth: lerp("mouth"),
          skill: b.skill || a.skill,
        };
      }
    }
    return kfs[kfs.length - 1];
  }

  // ---------------------------------------------------------------------------
  // 7. 50 Hz Closed-Loop Physics, Observation & UI Telemetry Step
  // ---------------------------------------------------------------------------
  const tofCells = [];
  for (let i = 0; i < 64; i++) {
    const c = document.createElement("div");
    c.className = "tof-cell";
    c.textContent = "·";
    tofGrid.appendChild(c);
    tofCells.push(c);
  }

  const jointRows = [];
  JOINT_NAMES.forEach((name, idx) => {
    const tr = document.createElement("tr");
    tr.innerHTML = `<td>${JOINT_IDS[idx]}</td><td>${name}</td><td class="ang">0.0°</td><td class="grd">0</td>`;
    jointsTbody.appendChild(tr);
    jointRows.push({
      ang: tr.querySelector(".ang"),
      grd: tr.querySelector(".grd"),
    });
  });

  function updateAutonomousBrain(dt) {
    if (state.brainMode === "laser" && state.laser.active) {
      const dx = state.laser.x - state.pose.x;
      const dz = state.laser.z - state.pose.z;
      const dist = Math.hypot(dx, dz);
      // In Three.js Y-up, forward is +X when yaw=0, +Z is right (-Y in Z-up)
      const desiredYaw = Math.atan2(-dz, dx);
      let errYaw = desiredYaw - state.pose.yaw;
      while (errYaw > Math.PI) errYaw -= 2 * Math.PI;
      while (errYaw < -Math.PI) errYaw += 2 * Math.PI;

      state.cmd.headYaw = Math.max(-1.0, Math.min(1.0, errYaw));
      state.cmd.headPitch = dist < 0.8 ? 0.35 : 0.15;
      if (dist > 0.16) {
        state.cmd.vx = Math.min(0.28, dist * 0.45);
        state.cmd.vyaw = Math.max(-1.0, Math.min(1.0, errYaw * 1.6));
      } else {
        state.cmd.vx = 0;
        state.cmd.vyaw = 0;
      }
    } else if (state.brainMode === "soccer") {
      const dx = state.ball.x - state.pose.x;
      const dz = state.ball.z - state.pose.z;
      const dist = Math.hypot(dx, dz);
      const desiredYaw = Math.atan2(-dz, dx);
      let errYaw = desiredYaw - state.pose.yaw;
      while (errYaw > Math.PI) errYaw -= 2 * Math.PI;
      while (errYaw < -Math.PI) errYaw += 2 * Math.PI;

      state.cmd.headYaw = Math.max(-1.0, Math.min(1.0, errYaw));
      state.cmd.headPitch = 0.42;
      if (dist > 0.17) {
        state.cmd.vx = Math.abs(errYaw) < 0.4 ? 0.25 : 0.06;
        state.cmd.vyaw = Math.max(-1.0, Math.min(1.0, errYaw * 1.8));
      } else {
        state.cmd.vx = 0;
        state.cmd.vyaw = 0;
        if (!state.activeSkill) {
          triggerSkill("kick_left");
          // Kick the ball forward along duck heading
          const fwdX = Math.cos(state.pose.yaw);
          const fwdZ = -Math.sin(state.pose.yaw);
          state.ball.vx = fwdX * 1.35;
          state.ball.vz = fwdZ * 1.35;
          state.vetbook.kicks += 1;
          pushLog("soccer", `Kick executed! Ball launched (${state.vetbook.kicks} total kicks).`);
        }
      }
    } else if (state.brainMode === "autod") {
      state.autodTimer += dt;
      if (state.autodTimer > 3.5) {
        state.autodTimer = 0;
        const moods = ["Wander", "LookAround", "Groom", "Wander"];
        state.autodState = moods[Math.floor(Math.random() * moods.length)];
        if (state.autodState === "Wander") {
          state.autodGoal = {
            x: (Math.random() - 0.5) * 2.2,
            z: (Math.random() - 0.5) * 2.2,
          };
        }
        pushLog("autod", `State -> ${state.autodState}`);
      }
      if (state.autodState === "Wander") {
        const dx = state.autodGoal.x - state.pose.x;
        const dz = state.autodGoal.z - state.pose.z;
        const d = Math.hypot(dx, dz);
        const yawTarget = Math.atan2(-dz, dx);
        let err = yawTarget - state.pose.yaw;
        while (err > Math.PI) err -= 2 * Math.PI;
        while (err < -Math.PI) err += 2 * Math.PI;
        state.cmd.vx = d > 0.15 ? 0.20 : 0;
        state.cmd.vyaw = Math.max(-0.8, Math.min(0.8, err * 1.3));
        state.cmd.headYaw = Math.sin(state.time * 1.4) * 0.4;
      } else if (state.autodState === "LookAround") {
        state.cmd.vx = 0;
        state.cmd.vyaw = 0;
        state.cmd.headYaw = Math.sin(state.time * 1.8) * 0.65;
        state.cmd.headPitch = Math.cos(state.time * 1.3) * 0.25;
      } else {
        state.cmd.vx = 0;
        state.cmd.vyaw = 0;
        state.cmd.headRoll = Math.sin(state.time * 4.0) * 0.25;
        state.cmd.neckPitch = 0.2;
      }
    }
  }

  const duckApi = {
    setJoint(nameOrIdx, rad) {
      let idx = typeof nameOrIdx === "string" ? JOINT_NAMES.indexOf(nameOrIdx) : nameOrIdx;
      if (idx >= 0 && idx < 15 && Number.isFinite(rad)) {
        state.customJointOverrides[idx] = Math.max(ACTUATOR_MIN_RAD, Math.min(ACTUATOR_MAX_RAD, rad));
      }
    },
    set_joint(nameOrIdx, rad) { this.setJoint(nameOrIdx, rad); },

    setMouth(ratio) {
      if (Number.isFinite(ratio)) {
        state.cmd.mouthOpen = Math.max(0, Math.min(1, ratio));
        state.joints[MOUTH_INDEX] = mouthTarget(state.cmd.mouthOpen);
      }
    },
    set_mouth(ratio) { this.setMouth(ratio); },

    setHead(yaw, pitch, roll) {
      if (Number.isFinite(yaw)) state.cmd.headYaw = Math.max(-1.0, Math.min(1.0, yaw));
      if (Number.isFinite(pitch)) state.cmd.headPitch = Math.max(-0.6, Math.min(0.6, pitch));
      if (Number.isFinite(roll)) state.cmd.headRoll = Math.max(-0.6, Math.min(0.6, roll));
    },
    set_head(yaw, pitch, roll) { this.setHead(yaw, pitch, roll); },

    setTwist(vx, vyaw) {
      if (Number.isFinite(vx)) state.cmd.vx = Math.max(-0.35, Math.min(0.45, vx));
      if (Number.isFinite(vyaw)) state.cmd.vyaw = Math.max(-1.2, Math.min(1.2, vyaw));
    },
    set_twist(vx, vyaw) { this.setTwist(vx, vyaw); },

    readTof() {
      const d = Number(sliderTheremin.value) || 0.34;
      return {
        distance_m: d,
        hand_present: d < 0.60,
        raw_mm: Math.round(d * 1000),
      };
    },
    read_tof() { return this.readTof(); },

    playQuack(pitchMultiplier = 1.0) {
      playProceduralQuack(pitchMultiplier);
    },
    play_quack(pitchMultiplier = 1.0) { this.playQuack(pitchMultiplier); },

    applyImpulse(rad_s = 7.5) {
      injectPushImpulse(rad_s);
    },
    apply_impulse(rad_s = 7.5) { this.applyImpulse(rad_s); },

    reset() {
      state.customJointOverrides = {};
      state.cmd.vx = 0;
      state.cmd.vyaw = 0;
      state.cmd.mouthOpen = 0;
    },
  };

  // ---------------------------------------------------------------------------
  // 6.5. Physically-Grounded Robot Skills Kinematics (Claude Endgegner Engine)
  // ---------------------------------------------------------------------------
  function applySkillAnimation(name, p, action14, state) {
    p = p < 0 ? 0 : (p > 1 ? 1 : p);
    const a = action14;
    for (let i = 0; i < 14; i++) a[i] = 0;

    const pose = state.pose;
    const cmd = state.cmd;
    pose.liftY = 0;
    pose.pitchFall = 0;
    pose.rollFall = 0;

    function clamp01(t) { return t < 0 ? 0 : (t > 1 ? 1 : t); }
    function ss(e0, e1, x) {
      const t = clamp01((x - e0) / (e1 - e0));
      return t * t * (3 - 2 * t);
    }
    function eo(e0, e1, x) {
      const t = 1 - clamp01((x - e0) / (e1 - e0));
      return 1 - t * t * t;
    }
    function pulse(r0, r1, f0, f1, x) {
      return ss(r0, r1, x) * (1 - ss(f0, f1, x));
    }
    function bothLegs(hip, knee, ankle) {
      // Right leg pitch joints are mirrored in CAD (DEFAULT_POSITION left=-0.4579, right=+0.4579)
      a[2] = hip;   a[3] = knee;   a[4] = ankle;
      a[11] = -hip; a[12] = -knee; a[13] = -ankle;
    }

    function kick(s) {
      const sw = s > 0 ? 0 : 9;
      const sp = s > 0 ? 9 : 0;
      const swDir = sw === 0 ? 1 : -1;
      const spDir = sp === 0 ? 1 : -1;

      const lean = pulse(0.0, 0.25, 0.78, 1.0, p);
      const wind = ss(0.25, 0.45, p);
      const whip = eo(0.45, 0.56, p);
      const back = ss(0.68, 1.0, p);

      const hip = 0.45 * wind - 1.45 * whip + 1.0 * back;
      const knee = 0.9 * wind - 0.8 * whip - 0.1 * back;
      const toe = -0.3 * whip * (1 - ss(0.68, 0.9, p));

      a[sw + 1] = s * 0.08 * lean;
      a[sw + 2] = swDir * hip;
      a[sw + 3] = swDir * knee;
      a[sw + 4] = swDir * toe;

      const cushion = lean * (0.6 + 0.4 * pulse(0.4, 0.5, 0.6, 0.8, p));
      a[sp + 1] = s * 0.15 * lean;
      a[sp + 2] = spDir * (-0.07 * cushion);
      a[sp + 3] = spDir * (0.15 * cushion);
      a[sp + 4] = spDir * (-0.07 * cushion);

      pose.rollFall = s * 0.12 * lean;
      pose.pitchFall = -0.06 * pulse(0.4, 0.52, 0.62, 0.8, p);
      a[8] = -s * 0.08 * lean;
      a[6] = 0.1 * pulse(0.4, 0.52, 0.62, 0.8, p);
    }

    switch (name) {
      case "kick_left":
        kick(1);
        break;
      case "kick_right":
        kick(-1);
        break;
      case "flamingo": {
        const shift = pulse(0.0, 0.2, 0.82, 0.97, p);
        const lift = pulse(0.15, 0.38, 0.72, 0.92, p);
        const sway = Math.sin(p * Math.PI * 6) * 0.012 * lift;

        pose.rollFall = 0.1 * shift + sway;
        a[10] = 0.15 * shift;
        a[8] = -0.06 * shift - sway * 2;

        a[2] = -0.7 * lift;
        a[3] = 1.1 * lift;
        a[4] = 0.2 * lift;
        a[1] = -0.04 * lift;

        const touch = Math.sin(Math.PI * clamp01((p - 0.82) / 0.16)) * (p > 0.82 ? 1 : 0);
        const sup = 0.5 * shift;
        a[11] = 0.05 * sup + 0.04 * touch;
        a[12] = -0.1 * sup - 0.1 * touch;
        a[13] = 0.05 * sup + 0.04 * touch;
        a[6] = 0.05 * lift;
        break;
      }
      case "polite_bow": {
        const cush = pulse(0.0, 0.2, 0.8, 1.0, p);
        const bow = pulse(0.15, 0.45, 0.62, 0.9, p);
        bothLegs(-0.1 * cush, 0.2 * cush, -0.1 * cush);
        a[5] = 0.7 * bow;
        a[6] = 0.45 * bow;
        pose.pitchFall = 0.04 * bow;
        cmd.mouthOpen = 0.25 * pulse(0.4, 0.5, 0.6, 0.75, p);
        break;
      }
      case "ground_pick": {
        const squat = pulse(0.0, 0.3, 0.7, 1.0, p);
        const neck = pulse(0.2, 0.45, 0.7, 0.92, p);
        pose.liftY = -0.04 * squat;
        bothLegs(-0.35 * squat, 0.7 * squat, -0.35 * squat);
        pose.pitchFall = 0.06 * squat;
        a[5] = 0.9 * neck;
        a[6] = 0.5 * neck;
        cmd.mouthOpen = ss(0.46, 0.5, p) * (1 - ss(0.5, 0.56, p));
        break;
      }
      case "roulade": {
        pose.liftY = 0.22 * Math.sin(Math.PI * p);
        pose.pitchFall = p * 2 * Math.PI;

        const crouch = ss(0.0, 0.15, p) * (1 - ss(0.15, 0.24, p));
        const tuck = ss(0.2, 0.36, p) * (1 - ss(0.72, 0.85, p));
        const reach = ss(0.72, 0.85, p) * (1 - ss(0.92, 1.0, p));

        const hip = -0.5 * crouch - 1.4 * tuck - 0.25 * reach;
        const knee = 1.0 * crouch + 2.0 * tuck + 0.08 * reach;
        const ankle = -0.5 * crouch + 0.5 * tuck - 0.1 * reach;
        bothLegs(hip, knee, ankle);

        a[5] = 0.3 * crouch + 0.6 * tuck;
        a[6] = 0.2 * crouch + 0.4 * tuck - 0.15 * reach;
        break;
      }
    }
  }

  function stepSimulation(dt) {
    state.time += dt;
    updateAutonomousBrain(dt);

    // Ball physics
    if (Math.hypot(state.ball.vx, state.ball.vz) > 0.005) {
      state.ball.x += state.ball.vx * dt;
      state.ball.z += state.ball.vz * dt;
      state.ball.vx *= 0.96;
      state.ball.vz *= 0.96;
      if (Math.abs(state.ball.x) > 2.6) state.ball.vx *= -0.7;
      if (Math.abs(state.ball.z) > 2.6) state.ball.vz *= -0.7;
      ballMesh.position.set(state.ball.x, 0.042, state.ball.z);
    }

    // Theremin computation
    let mouthOpen = state.cmd.mouthOpen;
    if (state.thereminOn) {
      const d = Number(sliderTheremin.value);
      if (d >= 0.08 && d <= 0.60) {
        const norm = (0.60 - d) / (0.60 - 0.08);
        const freq = 220 * Math.pow(2, norm * 1.5);
        thereminHz.textContent = `${freq.toFixed(1)} Hz`;
        mouthOpen = norm;
        updateThereminAudio(true, freq);
      } else {
        thereminHz.textContent = "SILENT (>60cm)";
        mouthOpen = 0;
        updateThereminAudio(false, 0);
      }
    } else {
      thereminHz.textContent = "OFF";
    }

    // Compute 14-D action offsets
    const action14 = new Array(14).fill(0);
    const speed = Math.hypot(state.cmd.vx, state.cmd.vy, state.cmd.vyaw);

    if (state.isResting) {
      for (let i = 0; i < 15; i++) {
        state.joints[i] += (REST_POSITION[i] - state.joints[i]) * 0.12;
      }
      state.pose.liftY += (-0.055 - state.pose.liftY) * 0.12;
    } else if (state.isPickedUp) {
      state.pickupElapsed += dt;
      const t = Math.min(1, state.pickupElapsed / 0.30);
      let sIdx = 0;
      for (let i = 0; i < 15; i++) {
        if (i === MOUTH_INDEX) {
          state.joints[i] = mouthTarget(mouthOpen);
        } else {
          const target = DEFAULT_POSITION[i] + PAUSE_POSE_OFFSETS[sIdx];
          state.joints[i] = state.pickupStartPose[i] + (target - state.pickupStartPose[i]) * t;
          action14[sIdx] = state.joints[i] - DEFAULT_POSITION[i];
          sIdx++;
        }
      }
      state.pose.liftY += (0.14 - state.pose.liftY) * 0.12;
    } else if (state.isFallen) {
      state.pose.pitchFall += (1.38 - state.pose.pitchFall) * 0.14;
      state.pose.liftY += (-0.04 - state.pose.liftY) * 0.14;
    } else {
      state.pose.pitchFall *= 0.85;
      state.pose.rollFall *= 0.85;
      state.pose.liftY += ((state.isSeated ? -0.05 : 0.0) - state.pose.liftY) * 0.15;

      // Walking / Roller locomotion cycle
      if (!state.isSeated && speed > 0.01) {
        if (state.locomotionMode === "roller") {
          // Dynamic Roller Skating Leg Stride (matching authentic XGO-Duck roller policy)
          // The duck performs rhythmic alternating skating thrusts, pushing laterally & backward
          // while leaning gracefully into each gliding stride on its 4 polyurethane wheels.
          const skateFreq = 5.2; // Natural dynamic skating cadence
          const phase = state.time * skateFreq;
          const s = Math.sin(phase);
          const strideAmp = Math.min(1.0, speed * 2.8);

          // Athletic low aerodynamic crouch
          const crouch = 0.20;
          action14[2] = -crouch;
          action14[3] = crouch * 1.8;
          action14[4] = -crouch * 0.8;
          action14[11] = crouch;
          action14[12] = -crouch * 1.8;
          action14[13] = crouch * 0.8;

          // Alternating left/right leg skating push:
          if (s > 0) {
            // Left leg pushes outward & back (thrust phase)
            const thrust = s * strideAmp;
            action14[1] = -0.22 * thrust;        // Left hip roll outward
            action14[2] += -0.16 * thrust;       // Left hip pitch extension
            action14[3] += -0.22 * thrust;       // Left knee push
            action14[4] += 0.12 * thrust;        // Left ankle flexion
            // Right leg carries weight & glides smoothly
            action14[10] = 0.06 * thrust;
            action14[12] += -0.10 * thrust;      // Right knee compliant flex
            // Body rolls into the gliding right side
            state.pose.rollFall = -0.07 * thrust;
            action14[0] = -0.08 * thrust;        // Counter yaw
            action14[9] = 0.08 * thrust;
          } else {
            // Right leg pushes outward & back (thrust phase)
            const thrust = -s * strideAmp;
            action14[10] = 0.22 * thrust;       // Right hip roll outward
            action14[11] += 0.16 * thrust;      // Right hip pitch extension
            action14[12] += 0.22 * thrust;      // Right knee push
            action14[13] += -0.12 * thrust;     // Right ankle flexion
            // Left leg carries weight & glides smoothly
            action14[1] = -0.06 * thrust;
            action14[3] += 0.10 * thrust;       // Left knee compliant flex
            // Body rolls into the gliding left side
            state.pose.rollFall = 0.07 * thrust;
            action14[0] = 0.08 * thrust;        // Counter yaw
            action14[9] = -0.08 * thrust;
          }

          // Aerodynamic neck & head tilt during skate glide
          action14[5] += 0.14 * strideAmp;      // Neck reaches forward
          action14[8] += -state.pose.rollFall * 0.75; // Head stabilizes horizontal gaze
        } else {
          const phase = state.time * 8.0;
          const swing = Math.min(0.32, speed * 0.72);
          const s = Math.sin(phase);
          const c = Math.cos(phase);
          action14[0] = state.cmd.vyaw * 0.16 * s;
          action14[1] = state.cmd.vy * 0.25 + 0.055 * c;
          action14[2] = swing * s;
          action14[3] = Math.max(0, -swing * c) * 1.15;
          action14[4] = -0.5 * swing * s;

          action14[9] = -state.cmd.vyaw * 0.16 * s;
          action14[10] = state.cmd.vy * 0.25 - 0.055 * c;
          action14[11] = swing * s;
          action14[12] = -Math.max(0, swing * c) * 1.15;
          action14[13] = -0.5 * swing * s;
        }

        // Update world odometry
        state.pose.yaw += state.cmd.vyaw * dt;
        const cosY = Math.cos(state.pose.yaw);
        const sinY = Math.sin(state.pose.yaw);
        const dx = (state.cmd.vx * cosY + state.cmd.vy * sinY) * dt;
        const dz = (-state.cmd.vx * sinY + state.cmd.vy * cosY) * dt;
        state.pose.x += dx;
        state.pose.z += dz;
        state.vetbook.metersWalked += Math.hypot(dx, dz);
      }

      // Seated posture
      if (state.isSeated) {
        action14[2] = -0.35;
        action14[3] = 1.25;
        action14[4] = -0.65;
        action14[11] = 0.35;
        action14[12] = -1.25;
        action14[13] = 0.65;
      }

      // Active one-shot skill overlay (Claude Endgegner Physics-Grounded Kinematics)
      if (state.activeSkill) {
        state.activeSkill.elapsed += dt;
        const p = Math.min(1, state.activeSkill.elapsed / state.activeSkill.duration);
        applySkillAnimation(state.activeSkill.name, p, action14, state);
        if (p >= 1) {
          state.pose.pitchFall = 0;
          state.pose.rollFall = 0;
          state.pose.liftY = 0;
          state.activeSkill = null;
        }
      }

      // Head commands (indices 5..8)
      action14[5] += state.cmd.neckPitch;
      action14[6] += state.cmd.headPitch;
      action14[7] += state.cmd.headYaw;
      action14[8] += state.cmd.headRoll;

      // Active Emotion keyframe override
      if (state.activeEmotion) {
        state.activeEmotion.elapsed += dt;
        const kf = sampleEmotion(state.activeEmotion, state.activeEmotion.elapsed);
        action14[5] = kf.neck || 0;
        action14[6] = kf.head_pitch || 0;
        action14[7] = kf.head_yaw || 0;
        action14[8] = kf.head_roll || 0;
        mouthOpen = kf.mouth || 0;
        if (state.activeEmotion.elapsed >= state.activeEmotion.duration) {
          state.activeEmotion = null;
        }
      }

      // Scatter 14 actions -> 15 joints
      let src = 0;
      for (let i = 0; i < 15; i++) {
        if (i === MOUTH_INDEX) {
          const next = mouthTarget(mouthOpen);
          state.velocities[i] = (next - state.joints[i]) / dt;
          state.joints[i] = next;
        } else {
          const next = Math.max(-Math.PI, Math.min(Math.PI, DEFAULT_POSITION[i] + action14[src]));
          state.velocities[i] = (next - state.joints[i]) / dt;
          state.joints[i] = next;
          src++;
        }
      }

      // Apply Custom Script Step & Overrides
      if (state.customScript.active && typeof state.customScript.fn === 'function') {
        try {
          state.customScript.fn(state.time, dt, duckApi);
          for (const [idxStr, val] of Object.entries(state.customJointOverrides)) {
            const jIdx = Number(idxStr);
            if (jIdx >= 0 && jIdx < 15 && Number.isFinite(val)) {
              state.joints[jIdx] = val;
            }
          }
        } catch (err) {
          state.customScript.active = false;
          state.customScript.lastError = err.message;
          const statusEl = document.getElementById("code-status-text");
          if (statusEl) {
            statusEl.textContent = `Error: ${err.message}`;
            statusEl.style.color = "var(--rose)";
          }
          const badge = document.getElementById("code-runner-badge");
          if (badge) {
            badge.textContent = "ERROR";
            badge.style.color = "var(--rose)";
          }
          pushLog("script/err", `Custom script halted: ${err.message}`);
        }
      }
    }

    // Populate exact 61-D observation vector
    const gravZ = state.isFallen ? -0.35 : -1.0;
    const gravX = state.isFallen ? 0.85 : 0.0;
    state.obs61[0] = 0;
    state.obs61[1] = 0;
    state.obs61[2] = state.cmd.vyaw;
    state.obs61[3] = gravX;
    state.obs61[4] = 0;
    state.obs61[5] = gravZ;
    let sIdx = 0;
    for (let i = 0; i < 15; i++) {
      if (i === MOUTH_INDEX) continue;
      state.obs61[6 + sIdx] = state.joints[i] - DEFAULT_POSITION[i];
      state.obs61[20 + sIdx] = state.velocities[i];
      state.obs61[34 + sIdx] = state.prevAction14[sIdx];
      sIdx++;
    }
    state.obs61[48] = state.cmd.vx;
    state.obs61[49] = state.cmd.vy;
    state.obs61[50] = state.cmd.vyaw;
    state.obs61[51] = state.cmd.neckPitch;
    state.obs61[52] = state.cmd.headPitch;
    state.obs61[53] = state.cmd.headYaw;
    state.obs61[54] = state.cmd.headRoll;
    state.obs61[55] = 0;
    state.obs61[56] = 0;
    state.obs61[57] = state.cmd.bodyZ;
    state.obs61[58] = state.cmd.bodyRoll;
    state.obs61[59] = state.cmd.bodyPitch;
    state.obs61[60] = 0;
    state.prevAction14 = [...action14];

    // Update Three.js transforms
    duckWorldRoot.position.set(state.pose.x, state.pose.liftY, state.pose.z);
    duckWorldRoot.rotation.set(state.pose.rollFall, state.pose.yaw, -state.pose.pitchFall);
    shadowDisc.position.set(state.pose.x, 0.002, state.pose.z);
    shadowDisc.material.opacity = Math.max(0.12, 0.45 - state.pose.liftY * 1.5);

    if (state.activeMesh === "cad") {
      applyJointsToCadGroups(cadBodyGroups, state.joints);
      const showRoller = state.locomotionMode === "roller" || state.rollerSkatesForced;
      for (let i = 0; i < rollerWheels.length; i++) {
        const w = rollerWheels[i];
        w.visible = showRoller;
        if (showRoller) {
          const moveSpeed = Math.abs(state.cmd.vx) > 0.005 ? state.cmd.vx : (speed > 0.01 ? speed : 0);
          if (Math.abs(moveSpeed) > 0.005) {
            w.rotateX(-(moveSpeed * dt) / Math.max(0.012, w.userData.radius || 0.015));
          }
        }
      }
    } else {
      fluxAiContainer.position.y = 0;
      fluxAiContainer.rotation.y = 0;
    }

    updateTofMatrix();
    updateFaceLeds();
  }

  function updateTofMatrix() {
    tofMarkersGroup.visible = state.showTofRays;
    const headYaw = state.pose.yaw + state.joints[7];
    const headPitch = 0.25 + state.joints[5] + state.joints[6];
    const originX = state.pose.x + Math.cos(headYaw) * 0.08;
    const originY = 0.21 + state.pose.liftY;
    const originZ = state.pose.z - Math.sin(headYaw) * 0.08;

    for (let r = 0; r < 8; r++) {
      for (let c = 0; c < 8; c++) {
        const idx = r * 8 + c;
        const azOff = ((c - 3.5) / 3.5) * 0.39; // ~45 deg FOV
        const elOff = ((r - 3.5) / 3.5) * 0.39;
        const rayYaw = headYaw - azOff;
        const rayPitch = headPitch + elOff;
        const dirY = -Math.sin(rayPitch);

        let dist = 1.8;
        let isHit = false;
        if (state.thereminOn && r >= 2 && r <= 5 && c >= 2 && c <= 5) {
          dist = Number(sliderTheremin.value);
          isHit = true;
        } else if (dirY < -0.05) {
          const floorDist = originY / -dirY;
          if (floorDist > 0.08 && floorDist < 2.2) {
            dist = floorDist;
          }
        }

        const px = originX + Math.cos(rayYaw) * Math.cos(rayPitch) * dist;
        const py = Math.max(0.006, originY + dirY * dist);
        const pz = originZ - Math.sin(rayYaw) * Math.cos(rayPitch) * dist;

        const m = tofMarkerMeshes[idx];
        m.position.set(px, py, pz);
        m.material.color.setHex(isHit ? 0xf5b700 : 0x22d3ee);

        const cell = tofCells[idx];
        if (dist > 1.7) {
          cell.style.background = "#0f172a";
          cell.style.color = "#475569";
          cell.textContent = "·";
        } else {
          const norm = Math.max(0, Math.min(1, (dist - 0.1) / 1.4));
          const hue = Math.round(20 + norm * 170);
          cell.style.background = `hsl(${hue}, 85%, 55%)`;
          cell.style.color = "#090d14";
          cell.textContent = dist.toFixed(1);
        }
      }
    }
  }

  function updateFaceLeds() {
    const ledStatus = document.getElementById("led-status");
    const ledNet = document.getElementById("led-net");
    const ledCam = document.getElementById("led-cam");
    const ledFlash = document.getElementById("led-flash");
    const ledBat1 = document.getElementById("led-bat1");
    const ledBat2 = document.getElementById("led-bat2");
    const ledBat3 = document.getElementById("led-bat3");

    ledStatus.style.background = state.isFallen ? "#f43f5e" : "#10b981";
    ledNet.style.background = state.choraleOn ? "#38bdf8" : "#10b981";
    ledCam.style.background = state.brainMode !== "manual" ? "#ef4444" : "#334155";
    ledFlash.style.background = state.laser.active ? "#f5b700" : "#334155";
    ledBat1.style.background = "#10b981";
    ledBat2.style.background = "#10b981";
    ledBat3.style.background = "#10b981";
  }

  function drawObsHeatmap() {
    const w = obsCanvas.width;
    const h = obsCanvas.height;
    obsCtx.fillStyle = "#060911";
    obsCtx.fillRect(0, 0, w, h);

    const cellW = w / 61;
    for (let i = 0; i < 61; i++) {
      const v = state.obs61[i];
      const mag = Math.min(1, Math.abs(v) / 1.5);
      obsCtx.fillStyle = v >= 0
        ? `rgba(34, 211, 238, ${0.15 + mag * 0.85})`
        : `rgba(244, 63, 94, ${0.15 + mag * 0.85})`;
      const barH = Math.max(4, mag * (h - 14));
      obsCtx.fillRect(i * cellW + 1, h - barH - 4, cellW - 2, barH);
    }
  }

  function updateDomTelemetry() {
    mOdom.textContent = `${state.pose.x.toFixed(2)}, ${state.pose.z.toFixed(2)} m`;
    mBrain.textContent = state.activeEmotion
      ? `EMOTION: ${state.activeEmotion.name.toUpperCase()}`
      : state.activeSkill
        ? `SKILL: ${state.activeSkill.name.toUpperCase()}`
        : `${state.brainMode.toUpperCase()} (${state.locomotionMode.toUpperCase()})`;

    if (state.isFallen) {
      mSafety.textContent = "FALLEN · LIMP";
      pillHealth.className = "pill pill-danger";
      pillHealth.textContent = "HEALTH: FALLEN (DEGRADED)";
    } else if (state.isPickedUp) {
      mSafety.textContent = "PICKED UP · PAUSE_POSE";
      pillHealth.className = "pill pill-warn";
      pillHealth.textContent = "HEALTH: HELD (63-D CNN)";
    } else {
      mSafety.textContent = `UPRIGHT · GUARD ${state.guardLevel}`;
      pillHealth.className = "pill pill-ok";
      pillHealth.textContent = "HEALTH: HEALTHY (50.0 Hz)";
    }

    mTrips.textContent = `${state.fallTrips} trips · ${state.fallCount} falls`;
    pillMode.textContent = `MODE: ${state.locomotionMode.toUpperCase()}`;

    for (let i = 0; i < 15; i++) {
      const deg = (state.joints[i] * 180) / Math.PI;
      jointRows[i].ang.textContent = `${deg.toFixed(1)}°`;
      jointRows[i].grd.textContent = String(state.impactGuardStates[i]);
    }

    drawObsHeatmap();
  }

  // ---------------------------------------------------------------------------
  // 8. Skills & Event Bindings
  // ---------------------------------------------------------------------------
  function updateLocomotionUi() {
    document.querySelectorAll("[data-skill]").forEach((b) => {
      const s = b.dataset.skill;
      if (s === "walk" || s === "roller") {
        const isActive = (s === state.locomotionMode);
        b.classList.toggle("btn-primary", isActive);
        b.classList.toggle("active-mode", isActive);
      }
    });
    if (skillStatusTag) {
      skillStatusTag.textContent = state.locomotionMode === "walk" ? "velstand.onnx" : "roller.onnx";
    }
    const btnToggleRollerSkates = document.getElementById("btn-toggle-roller-skates");
    if (btnToggleRollerSkates) {
      btnToggleRollerSkates.classList.toggle("active", state.locomotionMode === "roller" || state.rollerSkatesForced);
    }
  }

  function triggerSkill(name) {
    if (name === "walk" || name === "roller") {
      state.locomotionMode = name;
      state.isSeated = false;
      state.isResting = false;
      
      if (name === "roller") {
        state.rollerSkatesForced = true;
        // If speed is 0, auto-engage forward speed so the user immediately sees the roller skating strides!
        if (Math.abs(state.cmd.vx) < 0.05) {
          state.cmd.vx = 0.28;
          if (sliderVx) sliderVx.value = 0.28;
          if (valVx) valVx.textContent = "0.28 m/s";
        }
        pushLog("robotd", "Switched mode to Roller Skates (roller.onnx) - Dynamic leg push strides engaged.");
      } else {
        pushLog("robotd", "Switched mode to Bipedal Walk (velstand.onnx) - Bipedal walking cycle engaged.");
      }
      updateLocomotionUi();
      playProceduralQuack(name === "walk" ? 1.0 : 1.25);
      return;
    }
    if (name === "sit_toggle") {
      state.isSeated = !state.isSeated;
      state.isResting = false;
      pushLog("robotd", `sitstand.onnx -> ${state.isSeated ? "SEATED" : "STANDING"}`);
      return;
    }
    const durations = {
      ground_pick: 2.8,
      kick_left: 1.2,
      kick_right: 1.2,
      roulade: 1.6,
      polite_bow: 3.0,
      flamingo: 3.5,
    };
    state.activeSkill = { name, elapsed: 0, duration: durations[name] || 1.5 };
    pushLog("robotd", `robot.do "${name}" (${state.activeSkill.duration.toFixed(1)}s)`);
  }

  document.querySelectorAll("[data-skill]").forEach((btn) => {
    btn.addEventListener("click", () => triggerSkill(btn.dataset.skill));
  });

  document.querySelectorAll("[data-dpad]").forEach((btn) => {
    btn.addEventListener("click", () => {
      document.querySelectorAll("[data-dpad]").forEach((b) => b.classList.remove("active-mode"));
      btn.classList.add("active-mode");
      state.dpadMode = btn.dataset.dpad;
      pushLog("padd", `D-pad stick mapping -> ${state.dpadMode}`);
    });
  });

  sliderVx.addEventListener("input", () => {
    state.cmd.vx = Number(sliderVx.value);
    valVx.textContent = `${state.cmd.vx.toFixed(2)} m/s`;
  });
  sliderVy.addEventListener("input", () => {
    state.cmd.vy = Number(sliderVy.value);
    valVy.textContent = `${state.cmd.vy.toFixed(2)} m/s`;
  });
  sliderVyaw.addEventListener("input", () => {
    state.cmd.vyaw = Number(sliderVyaw.value);
    valVyaw.textContent = `${state.cmd.vyaw.toFixed(2)} rad/s`;
  });
  sliderHeadYaw.addEventListener("input", () => {
    state.cmd.headYaw = Number(sliderHeadYaw.value);
    valHeadYaw.textContent = `${state.cmd.headYaw.toFixed(2)} rad`;
  });
  sliderHeadPitch.addEventListener("input", () => {
    state.cmd.headPitch = Number(sliderHeadPitch.value);
    valHeadPitch.textContent = `${state.cmd.headPitch.toFixed(2)} rad`;
  });
  sliderMouth.addEventListener("input", () => {
    state.cmd.mouthOpen = Number(sliderMouth.value);
    valMouth.textContent = `${Math.round(state.cmd.mouthOpen * 100)}%`;
  });
  sliderTheremin.addEventListener("input", () => {
    valTheremin.textContent = `${Number(sliderTheremin.value).toFixed(2)} m`;
  });

  document.getElementById("btn-stop-vel").addEventListener("click", () => {
    state.cmd.vx = 0;
    state.cmd.vy = 0;
    state.cmd.vyaw = 0;
    sliderVx.value = 0;
    sliderVy.value = 0;
    sliderVyaw.value = 0;
    valVx.textContent = "0.00 m/s";
    valVy.textContent = "0.00 m/s";
    valVyaw.textContent = "0.00 rad/s";
    state.brainMode = "manual";
  });

  document.getElementById("btn-quack").addEventListener("click", () => playProceduralQuack(1.0));
  document.getElementById("btn-play-emotion").addEventListener("click", () => {
    playRealEmotion(document.getElementById("select-emotion").value);
  });

  selectGuard.addEventListener("change", () => {
    state.guardLevel = selectGuard.value;
    pushLog("bus", `Impact Guard profile set to Level ${state.guardLevel}`);
  });

  document.getElementById("btn-push-test").addEventListener("click", () => {
    if (state.guardLevel === "5") {
      state.fallTrips += 6;
      state.fallCount += 1;
      state.vetbook.falls += 1;
      state.isFallen = true;
      state.impactGuardStates[3] = 2;
      state.impactGuardStates[13] = 2;
      pushLog("ALERT", "Level 5 Impact Guard fired 6x on stance knees (500 mA soft hold) -> STANCE COLLAPSE!");
    } else {
      state.fallTrips += 1;
      state.impactGuardStates[3] = 1;
      setTimeout(() => { state.impactGuardStates[3] = 0; }, 250);
      pushLog("safety", `Level ${state.guardLevel} absorbed 7.5 rad/s push impulse cleanly (upright).`);
    }
  });

  document.getElementById("btn-pickup-toggle").addEventListener("click", () => {
    state.isPickedUp = !state.isPickedUp;
    if (state.isPickedUp) {
      state.pickupStartPose = [...state.joints];
      state.pickupElapsed = 0;
      state.vetbook.pickups += 1;
      pushLog("pickup", "picked up — pausing policy (300 ms ramp to PAUSE_POSE)");
    } else {
      pushLog("pickup", "put down — handing robot back to velstand.onnx");
    }
  });

  document.getElementById("btn-relax").addEventListener("click", () => {
    state.isResting = !state.isResting;
    state.isFallen = false;
    state.impactGuardStates.fill(0);
    pushLog("robotd", state.isResting ? "Easing to REST_POSITION" : "Standing back up to HOME");
  });

  document.getElementById("btn-reset-pos").addEventListener("click", () => {
    state.pose = { x: 0, z: 0, yaw: 0, liftY: 0, pitchFall: 0, rollFall: 0 };
    state.isFallen = false;
    state.isPickedUp = false;
    state.isSeated = false;
    state.isResting = false;
    state.joints = [...DEFAULT_POSITION];
    state.impactGuardStates.fill(0);
    state.brainMode = "manual";
    pushLog("robotd", "Reset robot to HOME pose at arena origin (0, 0).");
  });

  function toggleMeshMode() {
    if (state.activeMesh === "cad") {
      state.activeMesh = "flux_ai";
      duckCadContainer.visible = false;
      fluxAiContainer.visible = true;
      btnToggleMesh.textContent = "[M] Rig: XGO-Duck V5 STL Kit (50 STLs)";
      pushLog("view", "Switched to XGO-Duck V5 50-STL Printable Kit Platter.");
    } else {
      state.activeMesh = "cad";
      duckCadContainer.visible = true;
      fluxAiContainer.visible = false;
      btnToggleMesh.textContent = "[M] Rig: Articulated CAD (duck.bin)";
      pushLog("view", "Switched to Articulated CAD Rig (duck.bin).");
    }
  }
  btnToggleMesh.addEventListener("click", toggleMeshMode);

  const btnToggleRollerSkates = document.getElementById("btn-toggle-roller-skates");
  if (btnToggleRollerSkates) {
    btnToggleRollerSkates.addEventListener("click", () => {
      state.rollerSkatesForced = !state.rollerSkatesForced;
      btnToggleRollerSkates.classList.toggle("active", state.rollerSkatesForced);
      pushLog("STL", `Roller Skates (V5_AUDIO_VIDEO_TIRE_01.stl) ${state.rollerSkatesForced ? "MOUNTED" : "AUTO (Roller mode only)"}`);
    });
  }

  btnToggleTofRays.addEventListener("click", () => {
    state.showTofRays = !state.showTofRays;
    btnToggleTofRays.classList.toggle("active", state.showTofRays);
  });

  btnCamFollow.addEventListener("click", () => {
    state.followCam = !state.followCam;
    btnCamFollow.classList.toggle("active", state.followCam);
  });

  function setBrain(mode) {
    state.brainMode = state.brainMode === mode ? "manual" : mode;
    btnModeAutod.classList.toggle("active", state.brainMode === "autod");
    btnModeLaser.classList.toggle("active", state.brainMode === "laser");
    btnModeSoccer.classList.toggle("active", state.brainMode === "soccer");
    if (state.brainMode === "laser" && !state.laser.active) {
      state.laser.active = true;
      laserDotMesh.visible = true;
    }
    pushLog("brain", `Active brain -> ${state.brainMode}`);
  }
  btnModeAutod.addEventListener("click", () => setBrain("autod"));
  btnModeLaser.addEventListener("click", () => setBrain("laser"));
  btnModeSoccer.addEventListener("click", () => setBrain("soccer"));

  btnTheremin.addEventListener("click", () => {
    state.thereminOn = !state.thereminOn;
    btnTheremin.textContent = state.thereminOn ? "Disable Theremin" : "Enable ToF Theremin Audio";
    btnTheremin.classList.toggle("active-mode", state.thereminOn);
    if (!state.thereminOn) updateThereminAudio(false, 0);
    pushLog("tofd", `ToF Theremin ${state.thereminOn ? "ACTIVE" : "OFF"}`);
  });

  document.getElementById("btn-chorale").addEventListener("click", () => {
    if (state.choraleOn) return;
    state.choraleOn = true;
    choirGroup.visible = true;
    state.vetbook.songsSung += 1;
    let step = 0;
    const playStep = () => {
      if (!state.choraleOn) return;
      const chord = WISTFUL_CHORDS_HZ[step % WISTFUL_CHORDS_HZ.length];
      playChoraleChord(chord);
      state.choraleTick += 4;
      choraleBeat.textContent = `4 VOICES · BEAT ${state.choraleTick}`;
      state.cmd.mouthOpen = 0.6;
      setTimeout(() => { if (state.choraleOn) state.cmd.mouthOpen = 0.15; }, 500);
      step++;
    };
    playStep();
    choraleInterval = setInterval(playStep, 1200);
    pushLog("btd/chorale", "4-Duck BLE Chorale started: singing wistful.duckscore (±20ms sync).");
  });

  document.getElementById("btn-chorale-stop").addEventListener("click", () => {
    state.choraleOn = false;
    choirGroup.visible = false;
    if (choraleInterval) clearInterval(choraleInterval);
    choraleBeat.textContent = "IDLE";
    state.cmd.mouthOpen = 0;
    pushLog("btd/chorale", "Chorale stopped.");
  });

  // ---------------------------------------------------------------------------
  // 8.5. Live Code Sandbox, Presets & AI Motion Copilot
  // ---------------------------------------------------------------------------
  const CODE_PRESETS = {
    drunken_sailor: `// Preset 1: Drunken Sailor Wobble (Asymmetric gait & dynamic balance)
function onStep(t, dt, duck) {
  const waddleSpeed = 5.2;
  const phase = t * waddleSpeed;
  const sway = Math.sin(phase) * 0.22;
  const stagger = Math.sin(t * 1.3) * 0.15;

  duck.setTwist(0.18 + stagger, sway * 1.4);
  duck.setHead(Math.sin(t * 2.5) * 0.35, Math.cos(t * 3.1) * 0.18, -sway * 0.8);
  duck.setJoint('neck_pitch', 0.12 + 0.15 * Math.sin(t * 4.0));

  if (Math.sin(t * 1.8) > 0.85) {
    duck.setMouth(0.7);
  } else {
    duck.setMouth(0.0);
  }
}`,

    tof_beatbox: `// Preset 2: ToF Theremin Beatbox (Reactive to hand proximity)
let lastBeat = 0;
function onStep(t, dt, duck) {
  const tof = duck.readTof();
  if (tof.hand_present) {
    const proximity = Math.max(0, Math.min(1, 1.0 - (tof.distance_m - 0.08) / (0.60 - 0.08)));
    duck.setMouth(proximity);
    duck.setHead(0, (proximity - 0.5) * 0.4, Math.sin(t * 12.0) * proximity * 0.3);
    duck.setJoint('neck_pitch', 0.2 + 0.25 * Math.sin(t * 8.0));

    if (t - lastBeat > 0.35 && proximity > 0.4) {
      duck.playQuack(1.0 + proximity * 0.8);
      lastBeat = t;
    }
  } else {
    duck.setMouth(0.0);
    duck.setHead(0, 0, 0);
  }
}`,

    self_righting: `// Preset 3: Self-Righting Acrobat (Tilt detection & reflex recovery)
let stepCount = 0;
function onStep(t, dt, duck) {
  stepCount++;
  if (stepCount % 250 === 100) {
    duck.applyImpulse(7.5);
  }
  duck.setJoint('neck_pitch', 0.35 + 0.1 * Math.sin(t * 4));
  duck.setTwist(0.2, 0);
}`,

    laser_chase: `// Preset 4: Laser Dot Tracking
function onStep(t, dt, duck) {
  duck.setHead(Math.sin(t * 3.0) * 0.45, 0.25, 0.0);
  duck.setJoint('neck_pitch', 0.35);
  duck.setTwist(0.25, Math.sin(t * 2.0) * 0.5);
  duck.setMouth(Math.sin(t * 6.0) > 0.6 ? 0.6 : 0.0);
}`,

    python_style: `# Python-style Kinematics Routine
def on_step(t, dt, duck):
    # Waddle with animated head bob & beak chirps
    phase = t * 6.0
    duck.set_twist(0.22, math.sin(phase) * 0.45)
    duck.set_head(math.sin(t * 2.5) * 0.35, 0.2, 0.0)
    duck.set_mouth(abs(math.sin(t * 4.0)) * 0.8)
    duck.set_joint("neck_pitch", 0.25 + 0.15 * math.sin(phase))`
  };

  const codeEditorInput = document.getElementById("code-editor-input");
  const selectCodePreset = document.getElementById("select-code-preset");
  const btnLoadPreset = document.getElementById("btn-load-preset");
  const btnRunCode = document.getElementById("btn-run-code");
  const btnStopCode = document.getElementById("btn-stop-code");
  const codeStatusText = document.getElementById("code-status-text");
  const codeRunnerBadge = document.getElementById("code-runner-badge");
  const btnAiGenerate = document.getElementById("btn-ai-generate");
  const aiPromptInput = document.getElementById("ai-prompt-input");

  function transpilePythonToJs(rawCode) {
    if (!rawCode.includes("def on_step") && !rawCode.includes("def onStep")) {
      return rawCode;
    }
    let js = rawCode
      .replace(/#.*$/gm, "")
      .replace(/def\s+(?:on_step|onStep)\s*\([^)]*\)\s*:/g, "function onStep(t, dt, duck) {")
      .replace(/math\.sin/g, "Math.sin")
      .replace(/math\.cos/g, "Math.cos")
      .replace(/math\.abs/g, "Math.abs")
      .replace(/math\.pi/g, "Math.PI")
      .replace(/abs\(/g, "Math.abs(")
      .replace(/duck\.set_joint\(/g, "duck.setJoint(")
      .replace(/duck\.set_mouth\(/g, "duck.setMouth(")
      .replace(/duck\.set_twist\(/g, "duck.setTwist(")
      .replace(/duck\.set_head\(/g, "duck.setHead(")
      .replace(/duck\.read_tof\(/g, "duck.readTof(")
      .replace(/duck\.play_quack\(/g, "duck.playQuack(");
    js += "\n}";
    return js;
  }

  function compileAndRunUserScript(rawCode) {
    duckApi.reset();
    const executableCode = transpilePythonToJs(rawCode);

    try {
      const factory = new Function("duck", "console", "Math", `
        ${executableCode}
        if (typeof onStep === "function") return onStep;
        if (typeof on_step === "function") return on_step;
        return null;
      `);
      const stepFn = factory(duckApi, console, Math);
      if (typeof stepFn !== "function") {
        throw new Error("No 'function onStep(t, dt, duck)' or 'def on_step(t, dt, duck):' found.");
      }

      state.customScript.fn = stepFn;
      state.customScript.active = true;
      state.customScript.lastError = null;

      if (codeStatusText) {
        codeStatusText.textContent = "Status: Running at 50 Hz on MicroDuck 15-DOF rig";
        codeStatusText.className = "status-live";
      }
      if (codeRunnerBadge) {
        codeRunnerBadge.textContent = "LIVE 50 Hz";
        codeRunnerBadge.className = "mono-tag";
        codeRunnerBadge.style.color = "var(--emerald)";
        codeRunnerBadge.style.borderColor = "var(--emerald)";
      }
      pushLog("script/run", "Custom motion script started (50 Hz).");
    } catch (err) {
      state.customScript.active = false;
      state.customScript.fn = null;
      state.customScript.lastError = err.message;

      if (codeStatusText) {
        codeStatusText.textContent = `Compile Error: ${err.message}`;
        codeStatusText.className = "status-error";
      }
      if (codeRunnerBadge) {
        codeRunnerBadge.textContent = "SYNTAX ERROR";
        codeRunnerBadge.style.color = "var(--rose)";
      }
      pushLog("script/err", `Script compilation failed: ${err.message}`);
    }
  }

  if (codeEditorInput) {
    codeEditorInput.value = CODE_PRESETS.drunken_sailor;
  }

  if (btnLoadPreset && selectCodePreset && codeEditorInput) {
    btnLoadPreset.addEventListener("click", () => {
      const key = selectCodePreset.value;
      if (CODE_PRESETS[key]) {
        codeEditorInput.value = CODE_PRESETS[key];
        compileAndRunUserScript(codeEditorInput.value);
        pushLog("preset", `Loaded motion preset: ${key}`);
      }
    });
  }

  if (btnRunCode && codeEditorInput) {
    btnRunCode.addEventListener("click", () => {
      compileAndRunUserScript(codeEditorInput.value);
    });
  }

  if (btnStopCode) {
    btnStopCode.addEventListener("click", () => {
      state.customScript.active = false;
      duckApi.reset();
      if (codeStatusText) {
        codeStatusText.textContent = "Status: Stopped (Returned to policy control)";
        codeStatusText.className = "";
      }
      if (codeRunnerBadge) {
        codeRunnerBadge.textContent = "STANDBY";
        codeRunnerBadge.style.color = "var(--cyan)";
      }
      pushLog("script/stop", "Custom script stopped. Returned to default policy control.");
    });
  }

  if (btnAiGenerate && aiPromptInput && codeEditorInput) {
    btnAiGenerate.addEventListener("click", async () => {
      const promptText = aiPromptInput.value.trim();
      if (!promptText) return;

      btnAiGenerate.disabled = true;
      btnAiGenerate.textContent = "⏳ Generating...";
      if (codeStatusText) codeStatusText.textContent = "Connecting to local LLM / AI Copilot...";

      try {
        const sysPrompt = "You are an autonomous robotics motion engineer for the 15-DOF MicroDuck bipedal robot. Write ONLY an executable JavaScript function 'function onStep(t, dt, duck)' using the duck API: duck.setTwist(vx, vyaw), duck.setMouth(0..1), duck.setHead(yaw, pitch, roll), duck.setJoint(name, rad), duck.readTof(), duck.playQuack(). Do not include markdown code fences or explanations.";
        const resp = await fetch("http://127.0.0.1:11434/api/generate", {
          method: "POST",
          headers: { "Content-Type": "application/json" },
          body: JSON.stringify({
            model: "qwen3:30b-a3b",
            prompt: `${sysPrompt}\nUser Request: ${promptText}\nCode:`,
            stream: false,
            keep_alive: 0,
          }),
        });

        if (resp.ok) {
          const data = await resp.json();
          let generated = (data.response || "").replace(/```[a-z]*\n?/gi, "").trim();
          if (generated.includes("function onStep") || generated.includes("def on_step")) {
            codeEditorInput.value = generated;
            compileAndRunUserScript(generated);
            pushLog("AI", `Qwen 3 MoE generated routine for: "${promptText}"`);
            return;
          }
        }
        throw new Error("Local Ollama endpoint unavailable");
      } catch (e) {
        // Fallback procedural AI synthesizer
        const generated = `// AI Copilot Synthesis: "${promptText}"
function onStep(t, dt, duck) {
  // Wave neck and head in harmonic rhythm
  const harmonic = Math.sin(t * 4.5);
  duck.setHead(harmonic * 0.4, 0.25, Math.cos(t * 3.2) * 0.2);
  duck.setJoint('neck_pitch', 0.25 + 0.18 * Math.sin(t * 6.0));
  duck.setTwist(0.20, harmonic * 0.6);
  duck.setMouth(harmonic > 0.5 ? 0.8 : 0.0);
}`;
        codeEditorInput.value = generated;
        compileAndRunUserScript(generated);
        pushLog("AI", `Synthesized motion routine for: "${promptText}"`);
      } finally {
        btnAiGenerate.disabled = false;
        btnAiGenerate.textContent = "✨ Ask AI";
      }
    });
  }

  // Keyboard shortcuts
  window.addEventListener("keydown", (e) => {
    if (e.target.tagName === "INPUT" || e.target.tagName === "SELECT") return;
    const k = e.key.toLowerCase();
    if (k === "m") toggleMeshMode();
    if (k === "w" || e.key === "ArrowUp") state.cmd.vx = 0.30;
    if (k === "s" || e.key === "ArrowDown") state.cmd.vx = -0.20;
    if (k === "a" || e.key === "ArrowLeft") state.cmd.vyaw = 0.80;
    if (k === "d" || e.key === "ArrowRight") state.cmd.vyaw = -0.80;
    if (k === " ") {
      e.preventDefault();
      playProceduralQuack(1.0);
    }
  });
  window.addEventListener("keyup", (e) => {
    const k = e.key.toLowerCase();
    if (["w", "s", "arrowup", "arrowdown"].includes(k)) state.cmd.vx = 0;
    if (["a", "d", "arrowleft", "arrowright"].includes(k)) state.cmd.vyaw = 0;
  });

  // Modals (FLUX Gallery, Cheat Sheet, VetBook)
  const modalFlux = document.getElementById("modal-flux");
  const modalCheat = document.getElementById("modal-cheat");
  const modalVetbook = document.getElementById("modal-vetbook");

  document.getElementById("btn-open-flux").addEventListener("click", () => modalFlux.classList.add("open"));
  document.getElementById("btn-close-flux").addEventListener("click", () => modalFlux.classList.remove("open"));

  const selectColorwayEl = document.getElementById("select-colorway");
  if (selectColorwayEl) {
    selectColorwayEl.addEventListener("change", () => {
      applyColorway(duckCadContainer, selectColorwayEl.value);
      const label = HARDWARE_COLORWAYS[selectColorwayEl.value]?.label || "Raw CAD (duck.bin)";
      pushLog("CAD", `Hardware colorway -> ${label}`);
    });
  }

  document.getElementById("btn-open-cheat").addEventListener("click", async () => {
    modalCheat.classList.add("open");
    const pre = document.getElementById("cheat-pre-content");
    try {
      let r = await fetch("assets/CHEATSHEET_MASTER_GUIDE.md");
      if (!r.ok) r = await fetch("../docs/CHEATSHEET_MASTER_GUIDE.md");
      pre.textContent = r.ok ? await r.text() : "Cheat sheet not generated yet.";
    } catch {
      pre.textContent = "Run tri_council_microduck.py to populate CHEATSHEET_MASTER_GUIDE.md";
    }
  });
  document.getElementById("btn-close-cheat").addEventListener("click", () => modalCheat.classList.remove("open"));

  document.getElementById("btn-open-vetbook").addEventListener("click", () => {
    const v = state.vetbook;
    document.getElementById("vetbook-body").innerHTML = `
      <p><strong>Robot Name:</strong> ${v.name} (Alpha RK3566 · 15× XL330)</p>
      <p><strong>Distance Walked:</strong> ${v.metersWalked.toFixed(2)} m</p>
      <p><strong>Kicks:</strong> ${v.kicks} · <strong>Quacks:</strong> ${v.quacks} · <strong>Emotions:</strong> ${v.emotionsPlayed}</p>
      <p><strong>Chorale Songs:</strong> ${v.songsSung} · <strong>Pickups:</strong> ${v.pickups} · <strong>Falls:</strong> ${v.falls}</p>
      <p style="margin-top:10px; color:#10b981;">Mood: ${v.falls === 0 ? "Feeling great!" : "Vet check recommended (Impact Guard audit)"}</p>
    `;
    modalVetbook.classList.add("open");
  });
  document.getElementById("btn-close-vetbook").addEventListener("click", () => modalVetbook.classList.remove("open"));

  // ---------------------------------------------------------------------------
  // 9. Render Loop & Resize Handling
  // ---------------------------------------------------------------------------
  function resizeViewport() {
    const wrap = canvas3d.parentElement;
    const w = wrap.clientWidth || 800;
    const h = wrap.clientHeight || 540;
    renderer.setSize(w, h, false);
    camera.aspect = w / h;
    camera.updateProjectionMatrix();
  }
  window.addEventListener("resize", resizeViewport);
  resizeViewport();

  let lastMs = performance.now();
  let accum = 0;
  let uiTick = 0;

  function animate(nowMs) {
    requestAnimationFrame(animate);
    const frameDt = Math.min(0.05, (nowMs - lastMs) / 1000);
    lastMs = nowMs;
    accum += frameDt;

    while (accum >= state.dt) {
      stepSimulation(state.dt);
      accum -= state.dt;
      uiTick++;
      if (uiTick % 2 === 0) updateDomTelemetry(); // 25 Hz DOM updates
    }

    if (state.followCam) {
      controls.target.set(state.pose.x, 0.14 + state.pose.liftY * 0.5, state.pose.z);
    }
    controls.update();
    renderer.render(scene, camera);
  }


  // ===========================================================================
  // 10. Subsystem Inspector & Interactive Tooltips (Claude Architecture)
  // ===========================================================================
  const SUBSYSTEM_INFO = {
    "subsys-walk": {
      badge: "RL LOCOMOTION",
      tone: "cyan",
      title: "Bipedal Locomotion Policy (velstand.onnx)",
      text: "50 Hz neural network policy trained with PPO for stable dynamic limit-cycle walking, capturing phase transitions, contact timing, and continuous twist commands.",
      facts: ["50 Hz Loop", "PPO Policy", "Limit-Cycle Stride", "SE(3) Twist"]
    },
    "subsys-roller": {
      badge: "SKATE MODE",
      tone: "cyan",
      title: "Roller Skates Locomotion (roller.onnx)",
      text: "Passive wheel locomotion mode utilizing the 4 cyan polyurethane roller skate wheels mounted on both feet. Enables aerodynamic low crouch, lateral push-off carving, and fast gliding.",
      facts: ["V5 STL Tires", "4x PU Wheels", "Speed Skater Crouch", "Carving Glide"]
    },
    "subsys-sit": {
      badge: "POSTURE",
      tone: "green",
      title: "Sit / Stand Mode Transition",
      text: "Smooth compliant joint trajectory transitioning the robot between upright standing and grounded seated rest, preventing actuator current spikes.",
      facts: ["Smooth Transition", "Compliant Rest", "Actuator Unload"]
    },
    "subsys-pick": {
      badge: "MANIPULATION",
      tone: "green",
      title: "Ground Picking Routine",
      text: "Whole-body coordination lowering the center of mass to touch or grasp ground objects with beak mouth aperture control.",
      facts: ["Squat Depth", "Neck Extension", "Beak Trigger"]
    },
    "subsys-kick": {
      badge: "BALL DYNAMICS",
      tone: "amber",
      title: "Dynamic Single-Leg Kick",
      text: "High-velocity leg extension with single-support balance compensation on the stance foot, transferring dynamic momentum to the ball.",
      facts: ["Single Support", "Momentum Transfer", "Stance Stabilization"]
    },
    "subsys-choreo": {
      badge: "EXPRESSIVE",
      tone: "violet",
      title: "Choreography & Expressive Routines",
      text: "Polite bow, roulade 360° flip, flamingo single-leg balance, quacking and wing-flapping blended with whole-body balance control.",
      facts: ["Flamingo Balance", "Roulade Flip", "Polite Bow", "Personality"]
    },
    "subsys-emotion": {
      badge: "EXPRESSIVE",
      tone: "violet",
      title: "Real Emotion Playback (.wav + keyframes)",
      text: "Synchronized multi-track expressive playback: 12-track audio quacks, beak lip-sync, head gestures, and face board LED mood patterns.",
      facts: ["Lip-Sync Audio", "13 Emotion Clips", "LED Synchronization"]
    },
    "subsys-guard": {
      badge: "PROTECTION",
      tone: "amber",
      title: "Impact Guard & Velocity Limiter",
      text: "Continuous IMU shock, torque threshold, and collision monitoring. Dynamically limits joint velocities and current to protect gearboxes from impact damage.",
      facts: ["Hardened Profile R", "IMU Shock Detection", "Current Clamping"]
    },
    "subsys-impulse": {
      badge: "SAFETY TESTBED",
      tone: "red",
      title: "Push Impulse Disturbance Injection",
      text: "Injects sudden 7.5 rad/s angular impulse to evaluate recovery margins, capture steps, and Impact Guard compliance during external disturbance.",
      facts: ["7.5 rad/s Disturbance", "Capture Step Test", "Sim-to-Real Margin"]
    },
    "subsys-pickup": {
      badge: "SAFETY",
      tone: "amber",
      title: "Human Pick-up Detection",
      text: "Simulates a human lifting the robot off the floor: zero foot contact detection immediately ramps legs into compliant dangling mode.",
      facts: ["Dangling Detection", "Contact Loss", "Compliant Damping"]
    },
    "subsys-reset": {
      badge: "CALIBRATION",
      tone: "green",
      title: "Home Pose & Sensor Calibration",
      text: "Interpolates all 15 joints smoothly back to the mechanical zero-reference home pose and re-zeros odometry and velocity commands.",
      facts: ["Zero Offsets", "Smooth Interpolation", "Odometry Reset"]
    },
    "subsys-stop": {
      badge: "SAFETY",
      tone: "red",
      title: "Immediate E-Stop with Compliance",
      text: "Instantly zeroes policy commands and applies active damping to all 15 actuators, safely stopping motion without mechanical shock.",
      facts: ["Zero Command", "Active Damping", "Zero Gear Shock"]
    },
    "subsys-viewport": {
      badge: "3D DIGITAL TWIN",
      tone: "cyan",
      title: "Three.js 50 Hz Hardware Simulator",
      text: "Articulated CAD hierarchy rendered with authentic Lambertian materials, contact shadows, real-time kinematics, and interactive camera orbits.",
      facts: ["50 Hz Frame Rate", "Real CAD Meshes", "Contact Shadows"]
    },
    "subsys-tof": {
      badge: "PERCEPTION",
      tone: "cyan",
      title: "VL53L5CX 8×8 Time-of-Flight Depth Array",
      text: "Multizone 64-channel depth matrix measuring distance from 8 cm to 400 cm across a 63° diagonal FoV for spatial obstacle mapping and theremin audio.",
      facts: ["8×8 Zones", "63° FoV", "64 Range Channels", "Theremin Pitch"]
    },
    "subsys-obs61": {
      badge: "RL STATE",
      tone: "cyan",
      title: "61-D Policy Observation Vector",
      text: "Full state observation vector fed into the ONNX policy every 20 ms: joint positions & velocities (30), IMU orientation & rates (6), velocity commands (3), previous actions (14), gait phase (8).",
      facts: ["61 Dimensions", "50 Hz Update", "Proprioceptive State"]
    },
    "subsys-joints": {
      badge: "ACTUATION",
      tone: "green",
      title: "15-DOF Bus Servo Telemetry",
      text: "Live position, angular velocity, and load monitoring for all 15 serial bus servos across head (3), neck (1), mouth (1), and legs (10).",
      facts: ["15 Servos", "12-bit Encoder", "RS-485 Serial Bus"]
    },
    "subsys-kinematics": {
      badge: "KINEMATICS",
      tone: "cyan",
      title: "SE(3) Forward Kinematics & Base State",
      text: "Computes exact Cartesian positions, rotations, and Jacobians for the torso base frame and both foot contact centers via Lie group SE(3) transforms.",
      facts: ["SE(3) Lie Group", "Forward Kinematics", "Cartesian Base Pose"]
    },
    "subsys-telemetry": {
      badge: "SYSTEM HEALTH",
      tone: "amber",
      title: "Loop Frequency & Real-Time Jitter",
      text: "Monitors simulation loop rate (nominal 50 Hz), render FPS, and communication latency to ensure deterministic control timing.",
      facts: ["50 Hz Control Loop", "Zero Jitter Target", "WebSocket Link"]
    },
    "subsys-colorway": {
      badge: "VISUAL",
      tone: "violet",
      title: "Hardware Shell Colorway",
      text: "Custom 3D print shell presets: Classic White & Orange, Sky Blue & Orange, Lavender & Cyan, Stealth Slate & Purple, or Raw CAD duck.bin.",
      facts: ["5 Presets", "Real Shell Materials", "Accurate Beak/Feet Colors"]
    },
    "subsys-autod": {
      badge: "AUTONOMOUS",
      tone: "cyan",
      title: "autod Autonomous Behavior Daemon",
      text: "State machine cycling between random wandering, curiosity head tracking, grooming, and vocalizations without manual pilot intervention.",
      facts: ["State Machine", "Curiosity Tracking", "Autonomous Roam"]
    },
    "subsys-laser": {
      badge: "VISION TRACKING",
      tone: "red",
      title: "Laser Pointer Pursuit",
      text: "Tracks the interactive floor laser dot using visual servoing: computes heading error and drives locomotion towards the target.",
      facts: ["Target Acquisition", "Heading Tracking", "Interactive Target"]
    },
    "subsys-soccer": {
      badge: "GAME AI",
      tone: "emerald",
      title: "Autonomous Soccer Striker",
      text: "Perception and path planning loop: locates the ball, aligns approach heading, dribbles, and executes a dynamic kick into the goal area.",
      facts: ["Ball Seeking", "Approach Alignment", "Auto-Kick Trigger"]
    },
    "subsys-gamepad": {
      badge: "PILOT INPUT",
      tone: "cyan",
      title: "Virtual Gamepad & Twist Sliders",
      text: "Manual 50 Hz command interface for forward/strafe/yaw velocity (vx, vy, vyaw), neck/head orientation, and beak aperture. Also supports WASD / Arrow keys.",
      facts: ["WASD / Arrows", "SE(3) Velocity", "Beak Control"]
    },
    "subsys-quack": {
      badge: "AUDIO SYNTH",
      tone: "amber",
      title: "Procedural Formant Quack Synthesizer",
      text: "Real-time WebAudio formant synthesis generating expressive duck quacks synchronized with beak opening and pitch modulation.",
      facts: ["WebAudio API", "Formant Filter", "Beak Sync"]
    },
    "subsys-flux": {
      badge: "3D & STUDIO",
      tone: "violet",
      title: "FLUX.1 Gallery & 97-Part STL Browser",
      text: "Inspect official V2/V5 3D-printable STL components from LuwuDynamics/xgoduck_hardware alongside local FLUX.1 hardware colorway renders.",
      facts: ["97 Printable STLs", "V2 & V5 Kits", "FLUX.1 Renders"]
    },
    "subsys-cheatsheet": {
      badge: "DOCS",
      tone: "cyan",
      title: "Master Architecture Cheat Sheet",
      text: "Complete technical reference covering 15-DOF joint indices, 61-D observation layout, XL330 UART IDs, and safety invariants.",
      facts: ["15-DOF Map", "61-D Layout", "Safety Spec"]
    },
    "subsys-vetbook": {
      badge: "HEALTH LOG",
      tone: "green",
      title: "VetBook Telemetry & Diagnostics",
      text: "Tracks cumulative meters walked/skated, kicks, quacks, pickups, falls, and Impact Guard diagnostic health across the session.",
      facts: ["Session Metrics", "Fall Counter", "Health Score"]
    },
    "subsys-codelab": {
      badge: "50 HZ REPL",
      tone: "cyan",
      title: "Live Code Lab & AI Motion Copilot",
      text: "Execute custom JavaScript or Python-style kinematic routines at 50 Hz directly in the browser, or prompt the local LLM to generate new behaviors.",
      facts: ["50 Hz onStep()", "duck.* API", "Local AI Copilot"]
    }
  };

  function setupSubsystemInspector() {
    const doc = document;
    const MARGIN = 8;
    const OFFSET = 14;
    const IDLE_DELAY = 400;

    let tooltip = doc.querySelector(".md-tooltip");
    if (!tooltip) {
      tooltip = doc.createElement("div");
      tooltip.className = "md-tooltip";
      tooltip.setAttribute("role", "tooltip");
      doc.body.appendChild(tooltip);
    }

    const TAG_MAP = [
      ['[data-skill="walk"]', "subsys-walk"],
      ['[data-skill="roller"]', "subsys-roller"],
      ['[data-skill="sit_toggle"]', "subsys-sit"],
      ['[data-skill="ground_pick"]', "subsys-pick"],
      ['[data-skill="kick_left"], [data-skill="kick_right"]', "subsys-kick"],
      ['[data-skill="roulade"], [data-skill="polite_bow"], [data-skill="flamingo"]', "subsys-choreo"],
      ["#select-emotion, #btn-play-emotion", "subsys-emotion"],
      ["#select-guard", "subsys-guard"],
      ["#btn-push-test", "subsys-impulse"],
      ["#btn-pickup-toggle", "subsys-pickup"],
      ["#btn-relax", "subsys-sit"],
      ["#btn-reset-pos", "subsys-reset"],
      ["#btn-stop-vel", "subsys-stop"],
      ["#btn-toggle-mesh, .viewport-wrap", "subsys-viewport"],
      ["#select-colorway", "subsys-colorway"],
      ["#btn-toggle-roller-skates", "subsys-roller"],
      ["#btn-toggle-tof-rays", "subsys-tof"],
      ["#btn-mode-autod", "subsys-autod"],
      ["#btn-mode-laser", "subsys-laser"],
      ["#btn-mode-soccer", "subsys-soccer"],
      ["#tof-grid, #slider-theremin, #btn-theremin", "subsys-tof"],
      ["#obs-canvas", "subsys-obs61"],
      ["#joints-tbody, .joints-table", "subsys-joints"],
      ["#btn-chorale, #btn-chorale-stop", "subsys-choreo"],
      [".telemetry-card, .header-badges", "subsys-telemetry"],
      ["[data-dpad], #slider-vx, #slider-vy, #slider-vyaw, #slider-head-yaw, #slider-head-pitch, #slider-mouth", "subsys-gamepad"],
      ["#btn-quack", "subsys-quack"],
      ["#btn-open-flux", "subsys-flux"],
      ["#btn-open-cheat", "subsys-cheatsheet"],
      ["#btn-open-vetbook", "subsys-vetbook"],
      ["#select-code-preset, #btn-load-preset, #btn-run-code, #btn-stop-code, #ai-prompt-input, #btn-ai-generate", "subsys-codelab"]
    ];

    TAG_MAP.forEach(([sel, key]) => {
      try {
        doc.querySelectorAll(sel).forEach((el) => {
          if (!el.hasAttribute("data-info")) el.setAttribute("data-info", key);
        });
      } catch (e) {}
    });

    function getBar() { return doc.getElementById("subsystem-inspector-bar"); }
    function setElText(id, val) {
      const el = doc.getElementById(id);
      if (el && val != null) el.textContent = String(val);
    }

    function renderFacts(facts) {
      const list = doc.getElementById("si-facts");
      if (!list) return;
      while (list.firstChild) list.removeChild(list.firstChild);
      if (!facts) return;
      (Array.isArray(facts) ? facts : [facts]).forEach((fact) => {
        const d = doc.createElement("div");
        d.textContent = String(fact);
        list.appendChild(d);
      });
    }

    const IDLE = {
      tone: "cyan",
      badge: "INSPECTOR",
      title: "Subsystem Inspector Active",
      text: "Hover over any control, sensor, joint or skill to inspect live subsystem kinematics and telemetry.",
      facts: ["50 Hz Loop", "15-DOF", "SE(3) Kinematics"]
    };

    function setBarIdle() {
      const b = getBar();
      if (b) b.setAttribute("data-tone", IDLE.tone);
      setElText("si-badge", IDLE.badge);
      setElText("si-title", IDLE.title);
      setElText("si-text", IDLE.text);
      renderFacts(IDLE.facts);
    }

    function setBarInfo(info) {
      const b = getBar();
      if (b) b.setAttribute("data-tone", info.tone || "cyan");
      setElText("si-badge", info.badge || "INFO");
      setElText("si-title", info.title || "Subsystem");
      setElText("si-text", info.text || "");
      renderFacts(info.facts);
    }

    function fillTooltip(info) {
      while (tooltip.firstChild) tooltip.removeChild(tooltip.firstChild);
      if (info.title) {
        const t = doc.createElement("div");
        t.className = "md-tooltip-title";
        t.textContent = info.title;
        tooltip.appendChild(t);
      }
      if (info.text) {
        const d = doc.createElement("div");
        d.className = "md-tooltip-text";
        d.textContent = info.text;
        tooltip.appendChild(d);
      }
    }

    function positionTooltip(x, y, rect) {
      const vw = window.innerWidth || doc.documentElement.clientWidth;
      const vh = window.innerHeight || doc.documentElement.clientHeight;
      const tw = tooltip.offsetWidth || 260;
      const th = tooltip.offsetHeight || 80;
      let left, top;

      if (typeof x === "number" && typeof y === "number" && (x || y)) {
        left = x + OFFSET;
        top = y + OFFSET;
        if (left + tw + MARGIN > vw) left = x - OFFSET - tw;
        if (top + th + MARGIN > vh) top = y - OFFSET - th;
      } else if (rect) {
        left = rect.left + rect.width / 2 - tw / 2;
        top = rect.bottom + OFFSET;
        if (top + th + MARGIN > vh) top = rect.top - OFFSET - th;
      } else {
        left = MARGIN; top = MARGIN;
      }
      left = Math.max(MARGIN, Math.min(left, vw - tw - MARGIN));
      top = Math.max(MARGIN, Math.min(top, vh - th - MARGIN));
      tooltip.style.left = Math.round(left) + "px";
      tooltip.style.top = Math.round(top) + "px";
    }

    let idleTimer = null;
    let activeEl = null;

    function findInspectable(t) {
      return t && t.closest ? t.closest("[data-info], [data-tooltip]") : null;
    }

    doc.addEventListener("pointerover", (e) => {
      const el = findInspectable(e.target);
      if (!el) return;
      const key = el.getAttribute("data-info") || el.getAttribute("data-tooltip");
      const info = SUBSYSTEM_INFO[key];
      if (!info) return;

      if (idleTimer) { clearTimeout(idleTimer); idleTimer = null; }
      activeEl = el;
      setBarInfo(info);
      fillTooltip(info);
      tooltip.classList.add("is-visible");
      positionTooltip(e.clientX, e.clientY, el.getBoundingClientRect());
    });

    doc.addEventListener("pointermove", (e) => {
      if (!activeEl || !tooltip.classList.contains("is-visible")) return;
      positionTooltip(e.clientX, e.clientY, activeEl.getBoundingClientRect());
    });

    doc.addEventListener("pointerout", (e) => {
      const el = findInspectable(e.target);
      if (!el) return;
      const next = e.relatedTarget;
      if (next && el.contains(next)) return;

      tooltip.classList.remove("is-visible");
      if (activeEl === el) activeEl = null;

      const nextEl = findInspectable(next);
      if (nextEl && SUBSYSTEM_INFO[nextEl.getAttribute("data-info")]) return;

      if (idleTimer) clearTimeout(idleTimer);
      idleTimer = setTimeout(() => {
        idleTimer = null;
        if (!activeEl) setBarIdle();
      }, IDLE_DELAY);
    });

    doc.documentElement.addEventListener("mouseleave", () => {
      tooltip.classList.remove("is-visible");
      activeEl = null;
      setBarIdle();
    });
  }

  setupSubsystemInspector();
  updateLocomotionUi();

  pushLog("init", "MicroDuck 50 Hz Digital Twin initialized.");
  requestAnimationFrame(animate);
})();
