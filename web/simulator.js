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
    customJointOffsets: {},
    simGap: "ideal", // ideal | nominal | harsh
    imuDelayedQueue: [],
    customScript: {
      active: false,
      fn: null,
      lastError: null,
      ticks: 0,
      execMs: 0,
      maxExecMs: 0,
      clampCount: 0,
      nanCount: 0,
      slewCount: 0,
      assertPassed: 0,
      assertFailed: 0,
      lastFailedAssert: null,
      watchVars: {},
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

    // 5 Pre-Hardware Subsystems (pollen-robotics/microduck clean-room state)
    odom: {
      anchorFoot: "left",
      anchorIdx: 5,
      switches: 0,
      pendingFoot: null,
      pendingTicks: 0,
      alpha16X: 0,
      alpha16Z: 0,
      v15X: 0,
      v15Z: 0,
      driftAlpha16Mm: 0,
      driftV15Mm: 0,
    },
    bus: {
      volts: 8.2,
      tempsC: new Array(15).fill(31.0),
      currentsMa: new Array(15).fill(80.0),
      pwmCounts: new Array(15).fill(0),
      staleImuRun: 0,
      staleImuTotal: 0,
      remoteIoProto: 1,
    },
    maploc: {
      isStopped: true,
      stopTicks: 0,
      keyframes: 0,
    },
    petDetect: {
      isPetting: false,
      prob: 0.02,
      enterThreshold: 0.95,
      exitThreshold: 0.85,
      windowSamples: 16240,
      nMels: 40,
      windowFrames: 100,
    },
    padImu: {
      calibrated: true,
      biasDps: [0.0, 0.0, 12.0],
      stillWindowS: 0.5,
      staleRadioTicks: 0,
      deadRadioTrap: false,
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
  // 5. Three.js 3D Scene, Authentic CAD Rig (`duck.bin` + MJLab GLB) & Inline Skates
  //    Visual & kinematics techniques adapted with attribution from:
  //    - pollen-robotics/microduck (Apache-2.0)
  //    - pollen-robotics/microduck-simulator (Apache-2.0)
  // ---------------------------------------------------------------------------
  const renderer = new THREE.WebGLRenderer({ canvas: canvas3d, antialias: true, alpha: true });
  renderer.setPixelRatio(Math.min(window.devicePixelRatio || 1, 2));
  renderer.shadowMap.enabled = true;
  renderer.shadowMap.type = THREE.PCFSoftShadowMap;
  renderer.outputEncoding = THREE.sRGBEncoding;
  renderer.toneMapping = THREE.ACESFilmicToneMapping;
  renderer.toneMappingExposure = 1.05;

  const scene = new THREE.Scene();
  scene.background = new THREE.Color(0x08080c);
  scene.fog = new THREE.FogExp2(0x08080c, 0.12);

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

  // Calibrated 4-Point Studio Lighting + Warm Amber Rim (from MJLab GameCanvas.jsx, Apache-2.0)
  scene.add(new THREE.AmbientLight(0xffffff, 0.60));
  scene.add(new THREE.HemisphereLight(0xdbeafe, 0x1e293b, 0.35));
  const keyLight = new THREE.DirectionalLight(0xffffff, 1.60);
  keyLight.position.set(2.0, 4.0, 2.0);
  keyLight.castShadow = true;
  keyLight.shadow.mapSize.set(2048, 2048);
  keyLight.shadow.camera.near = 0.1;
  keyLight.shadow.camera.far = 10;
  keyLight.shadow.camera.left = -2;
  keyLight.shadow.camera.right = 2;
  keyLight.shadow.camera.top = 2;
  keyLight.shadow.camera.bottom = -2;
  scene.add(keyLight);

  const fillLight = new THREE.DirectionalLight(0xffffff, 0.40);
  fillLight.position.set(-2.0, 2.0, 1.5);
  scene.add(fillLight);

  const rimLight = new THREE.DirectionalLight(0xffb366, 0.70);
  rimLight.position.set(0.0, 3.0, -2.0);
  scene.add(rimLight);

  // Arena Floor + Antialiased GLSL Tron Grid Shader (adapted from MJLab arena.js, Apache-2.0)
  const floorGeo = new THREE.PlaneGeometry(24, 24);
  const floorMat = new THREE.ShaderMaterial({
    transparent: true,
    depthWrite: false,
    uniforms: {
      uFocus: { value: new THREE.Vector3(0, 0, 0) },
      uCell: { value: 0.10 },
      uSection: { value: 0.60 },
      uFade: { value: 6.5 },
      uColor1: { value: new THREE.Color(0x8e8371) },
      uColor2: { value: new THREE.Color(0xffb366) },
    },
    vertexShader: `
      varying vec3 vWorld;
      void main() {
        vec4 wp = modelMatrix * vec4(position, 1.0);
        vWorld = wp.xyz;
        gl_Position = projectionMatrix * viewMatrix * wp;
      }
    `,
    fragmentShader: `
      varying vec3 vWorld;
      uniform vec3 uFocus;
      uniform float uCell;
      uniform float uSection;
      uniform float uFade;
      uniform vec3 uColor1;
      uniform vec3 uColor2;

      float gridLine(vec2 p, float size, float widthPx) {
        vec2 r = p / size;
        vec2 g = abs(fract(r - 0.5) - 0.5) / fwidth(r);
        float d = min(g.x, g.y);
        return 1.0 - min(d / widthPx, 1.0);
      }

      void main() {
        float g1 = gridLine(vWorld.xz, uCell, 1.15);
        float g2 = gridLine(vWorld.xz, uSection, 1.85);
        float g2Halo = gridLine(vWorld.xz, uSection, 4.5);
        float dist = length(vWorld.xz - uFocus.xz);
        float fade = pow(clamp(1.0 - dist / uFade, 0.0, 1.0), 1.6);
        vec3 col = mix(uColor1, uColor2, clamp(g2 + g2Halo * 0.35, 0.0, 1.0));
        float alpha = (g1 * 0.26 + g2 * 0.68 + g2Halo * 0.14) * fade;
        if (alpha <= 0.002) discard;
        gl_FragColor = vec4(col, alpha);
      }
    `,
  });
  const floorBaseMesh = new THREE.Mesh(
    new THREE.PlaneGeometry(24, 24),
    new THREE.MeshStandardMaterial({ color: 0x08080c, roughness: 0.85, metalness: 0.05 })
  );
  floorBaseMesh.rotation.x = -Math.PI / 2;
  floorBaseMesh.position.y = -0.001;
  floorBaseMesh.receiveShadow = true;
  scene.add(floorBaseMesh);

  const floorMesh = new THREE.Mesh(floorGeo, floorMat);
  floorMesh.rotation.x = -Math.PI / 2;
  floorMesh.position.y = 0.001;
  scene.add(floorMesh);

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
  const walkingFootMeshes = [];
  const inlineRollerMeshes = [];
  let jawPivotGroup = null;
  let tireAssetData = null;
  const highResGlbGeometries = new Map();

  // Exact mapping of the 29 mesh indices in `duck.bin` to MJCF STL names
  const DUCK_BIN_MESH_NAMES = [
    "seeed_bearing__configuration__22x16x4.stl", // 0
    "right_shell.stl",                           // 1
    "trunk_base.stl",                            // 2
    "xl330.stl",                                 // 3
    "np_f970.stl",                               // 4
    "left_shell.stl",                            // 5
    "yaw2roll.stl",                              // 6
    "bearing_roll.stl",                          // 7
    "hip_l.stl",                                 // 8
    "upper_leg_left.stl",                        // 9
    "upper_leg_rigidity_plate.stl",              // 10
    "leg.stl",                                   // 11
    "sole_left.stl",                             // 12
    "foot_left.stl",                             // 13
    "seeed_bearing__configuration_default.stl",  // 14
    "ankle_left.stl",                            // 15
    "neck.stl",                                  // 16
    "neck_pitch.stl",                            // 17
    "yaw_roll_motion.stl",                       // 18
    "face_part.stl",                             // 19
    "jaw_soft.stl",                              // 20
    "soft_mouth_top.stl",                        // 21
    "bottom_head_shell.stl",                     // 22
    "top_head_shell.stl",                        // 23
    "jaw.stl",                                   // 24
    "upper_leg_right.stl",                       // 25
    "ankle_right.stl",                           // 26
    "sole_right.stl",                            // 27
    "foot_right.stl",                            // 28
  ];

  // Exact 16-Slot PBR Material Table & Part Mapping (adapted from MJLab variants.js, Apache-2.0)
  const PART_SLOTS = {
    "top_head_shell.stl": "head",
    "bottom_head_shell.stl": "head",
    "face_part.stl": "facePlate",
    "noenoeil.stl": "eyeRing",
    "lens.stl": "lens",
    "m12_lens_holder.stl": "eyeMount",
    "soft_mouth_top.stl": "beakUpper",
    "jaw.stl": "beakLower",
    "jaw_soft.stl": "tongue",
    "trunk_base.stl": "trunk",
    "left_shell.stl": "wings",
    "right_shell.stl": "wings",
    "upper_leg_left.stl": "thighs",
    "upper_leg_right.stl": "thighs",
    "hip_l.stl": "thighs",
    "foot_left.stl": "feet",
    "foot_right.stl": "feet",
    "ankle_left.stl": "feet",
    "ankle_right.stl": "feet",
    "ankle_l_v1.stl": "feet",
    "ankle_r_v1.stl": "feet",
    "roller_blade.stl": "feet",
    "sole_left.stl": "soles",
    "sole_right.stl": "soles",
    "rim.stl": "soles",
    "tire.stl": "tire",
    "xl330.stl": "motors",
    "leg.stl": "legs",
    "neck.stl": "neck",
    "neck_pitch.stl": "joints",
    "yaw_roll_motion.stl": "joints",
    "yaw2roll.stl": "joints",
    "bearing_roll.stl": "joints",
    "motor_support.stl": "joints",
    "power_support.stl": "joints",
    "banana_pcb_locker.stl": "joints",
    "upper_leg_rigidity_plate.stl": "joints",
    "seeed_bearing__configuration__22x16x4.stl": "bearings",
    "seeed_bearing__configuration_default.stl": "bearings",
    "np_f970.stl": "internals",
    "speaker.stl": "internals",
    "elec_rpi_robot_hat_pcb.stl": "internals",
    "pcb__raspberry_pi_zero_2_w.stl": "internals",
  };

  function makePalette(overrides) {
    const base = {
      head:      { color: 0xf4f1ea, roughness: 0.35, metalness: 0.0 },
      facePlate: { color: 0x9b9892, roughness: 0.35, metalness: 0.0 },
      eyeRing:   { color: 0x1d1d1f, roughness: 0.35, metalness: 0.0 },
      lens:      { color: 0x0a0c14, roughness: 0.05, metalness: 0.0 },
      eyeMount:  { color: 0x1d1d1f, roughness: 0.50, metalness: 0.0 },
      beakUpper: { color: 0xffb52e, roughness: 0.45, metalness: 0.0 },
      beakLower: { color: 0xff7a2f, roughness: 0.35, metalness: 0.0 },
      tongue:    { color: 0xffb52e, roughness: 0.45, metalness: 0.0 },
      trunk:     { color: 0x3a3a3e, roughness: 0.50, metalness: 0.0 },
      wings:     { color: 0xf4f1ea, roughness: 0.35, metalness: 0.0 },
      thighs:    { color: 0xf4f1ea, roughness: 0.35, metalness: 0.0 },
      feet:      { color: 0xff7a2f, roughness: 0.35, metalness: 0.0 },
      soles:     { color: 0xffd23f, roughness: 0.55, metalness: 0.0 },
      tire:      { color: 0x1d1d1f, roughness: 0.55, metalness: 0.3 },
      motors:    { color: 0x1d1d1f, roughness: 0.55, metalness: 0.0 },
      legs:      { color: 0x2c2c30, roughness: 0.25, metalness: 0.85 },
      neck:      { color: 0x2c2c30, roughness: 0.25, metalness: 0.85 },
      joints:    { color: 0x2a2a2e, roughness: 0.45, metalness: 0.2 },
      bearings:  { color: 0x8a8e94, roughness: 0.25, metalness: 0.85 },
      internals: { color: 0x1d1d1f, roughness: 0.55, metalness: 0.0 },
    };
    if (overrides) {
      for (const [k, v] of Object.entries(overrides)) {
        base[k] = { ...base[k], ...v };
      }
    }
    return base;
  }

  const HARDWARE_COLORWAYS = {
    white_orange: {
      label: "Cream (Classic White & Orange)",
      slots: makePalette({}),
    },
    sky_blue: {
      label: "Sky (Light Blue & Orange)",
      slots: makePalette({
        head:   { color: 0x8ad0e8 },
        wings:  { color: 0x8ad0e8 },
        thighs: { color: 0x8ad0e8 },
      }),
    },
    lavender_yellow: {
      label: "Lavender (Purple & Yellow)",
      slots: makePalette({
        head:      { color: 0x9b82c4 },
        wings:     { color: 0x9b82c4 },
        thighs:    { color: 0x9b82c4 },
        eyeRing:   { color: 0x7bc8d4 },
        beakUpper: { color: 0xa489b5 },
        tongue:    { color: 0xa489b5 },
        beakLower: { color: 0xffd23f },
        feet:      { color: 0xffd23f },
        soles:     { color: 0x7bc8d4 },
      }),
    },
    slate_purple: {
      label: "Graphite (Charcoal & Yellow)",
      slots: makePalette({
        head:      { color: 0x45454a },
        wings:     { color: 0x45454a },
        thighs:    { color: 0x45454a },
        eyeRing:   { color: 0x8068b0 },
        beakUpper: { color: 0xa489b5 },
        tongue:    { color: 0xa489b5 },
        beakLower: { color: 0xffd23f },
        feet:      { color: 0xffd23f },
        soles:     { color: 0x8068b0 },
      }),
    },
    cad_raw: null,
  };

  // Compute 36° creased vertex normals (CREASE = Math.PI / 5) with 1e-4m vertex welding
  // (Adapted from MJLab duck.js, Apache-2.0 — eliminates faceted flat-shaded polygons on curved shells)
  function computeCreasedGeometry(vertsArray, trisArray, creaseAngle = Math.PI / 5) {
    const cosThresh = Math.cos(creaseAngle);
    const nTris = trisArray.length;
    const faceNormals = new Float32Array(nTris * 3);
    const vertFaces = new Map();

    const keyOf = (v) => `${Math.round(v[0] * 10000)},${Math.round(v[1] * 10000)},${Math.round(v[2] * 10000)}`;

    for (let t = 0; t < nTris; t++) {
      const tri = trisArray[t];
      const v0 = vertsArray[tri[0]];
      const v1 = vertsArray[tri[1]];
      const v2 = vertsArray[tri[2]];
      const ux = v1[0] - v0[0], uy = v1[1] - v0[1], uz = v1[2] - v0[2];
      const vx = v2[0] - v0[0], vy = v2[1] - v0[1], vz = v2[2] - v0[2];
      let nx = uy * vz - uz * vy;
      let ny = uz * vx - ux * vz;
      let nz = ux * vy - uy * vx;
      const len = Math.hypot(nx, ny, nz) || 1;
      nx /= len; ny /= len; nz /= len;
      faceNormals[t * 3] = nx;
      faceNormals[t * 3 + 1] = ny;
      faceNormals[t * 3 + 2] = nz;

      for (let c = 0; c < 3; c++) {
        const k = keyOf(vertsArray[tri[c]]);
        let list = vertFaces.get(k);
        if (!list) {
          list = [];
          vertFaces.set(k, list);
        }
        list.push(t);
      }
    }

    const posOut = new Float32Array(nTris * 9);
    const normOut = new Float32Array(nTris * 9);
    for (let t = 0; t < nTris; t++) {
      const tri = trisArray[t];
      const fnx = faceNormals[t * 3];
      const fny = faceNormals[t * 3 + 1];
      const fnz = faceNormals[t * 3 + 2];

      for (let c = 0; c < 3; c++) {
        const v = vertsArray[tri[c]];
        const outBase = t * 9 + c * 3;
        posOut[outBase] = v[0];
        posOut[outBase + 1] = v[1];
        posOut[outBase + 2] = v[2];

        const adj = vertFaces.get(keyOf(v));
        let sx = 0, sy = 0, sz = 0;
        for (let i = 0; i < adj.length; i++) {
          const ft = adj[i];
          const onx = faceNormals[ft * 3];
          const ony = faceNormals[ft * 3 + 1];
          const onz = faceNormals[ft * 3 + 2];
          if (fnx * onx + fny * ony + fnz * onz >= cosThresh) {
            sx += onx; sy += ony; sz += onz;
          }
        }
        const slen = Math.hypot(sx, sy, sz) || 1;
        normOut[outBase] = sx / slen;
        normOut[outBase + 1] = sy / slen;
        normOut[outBase + 2] = sz / slen;
      }
    }

    const g = new THREE.BufferGeometry();
    g.setAttribute("position", new THREE.BufferAttribute(posOut, 3));
    g.setAttribute("normal", new THREE.BufferAttribute(normOut, 3));
    return g;
  }

  function applyCreasedNormalsToBufferGeometry(geom, creaseAngle = Math.PI / 5) {
    const posAttr = geom.getAttribute("position");
    if (!posAttr) return geom;
    const idxAttr = geom.getIndex();
    const verts = [];
    for (let i = 0; i < posAttr.count; i++) {
      verts.push([posAttr.getX(i), posAttr.getY(i), posAttr.getZ(i)]);
    }
    const tris = [];
    if (idxAttr) {
      for (let i = 0; i < idxAttr.count; i += 3) {
        tris.push([idxAttr.getX(i), idxAttr.getX(i + 1), idxAttr.getX(i + 2)]);
      }
    } else {
      for (let i = 0; i < posAttr.count; i += 3) {
        tris.push([i, i + 1, i + 2]);
      }
    }
    return computeCreasedGeometry(verts, tris, creaseAngle);
  }

  // Mount authentic inline roller-blade assemblies (`ankle_l_v1`, `ankle_r_v1`, `roller_blade.stl`,
  // and front/rear inline `rim.stl` + `tire.stl` wheels from `kinematics_rollers.json`)
  function tryAttachRollerWheels() {
    if (!cadBodyGroups || cadBodyGroups.length < 15 || rollerWheels.length > 0) return;
    const rMeshes = window.__MICRODUCK_ROLLER_MESHES__;
    if (!rMeshes || !rMeshes["roller_blade.stl"]) return;

    const rGeoms = {};
    for (const [mName, mData] of Object.entries(rMeshes)) {
      rGeoms[mName] = computeCreasedGeometry(mData.verts, mData.tris);
    }

    const makeRollerMesh = (stlName, pos, quat) => {
      const slot = PART_SLOTS[stlName] || "joints";
      const mat = new THREE.MeshStandardMaterial({
        color: 0xffffff,
        roughness: 0.35,
        metalness: 0.0,
        flatShading: false,
        side: THREE.DoubleSide,
      });
      const m = new THREE.Mesh(rGeoms[stlName], mat);
      m.castShadow = true;
      m.receiveShadow = true;
      m.position.set(pos[0], pos[1], pos[2]);
      m.quaternion.set(quat[1], quat[2], quat[3], quat[0]).normalize();
      m.userData.stlName = stlName;
      m.userData.slot = slot;
      return m;
    };

    // Left ankle (body 5) & right ankle (body 14) inline roller assemblies from `kinematics_rollers.json`
    const configs = [
      {
        bodyIdx: 5,
        ankleGeoms: [
          { stl: "roller_blade.stl", pos: [-0.022, 0.00622291, -0.0647], quat: [0.5, -0.5, -0.5, -0.5] },
          { stl: "ankle_l_v1.stl",   pos: [-0.022, 0.00622291, -0.0647], quat: [0.5, -0.5, -0.5, -0.5] },
        ],
        wheels: [
          { name: "passive_LF_wheel", pos: [-0.0395, -0.0325, -0.0147], quat: [1, 0, 0, 0] },
          { name: "passive_LR_wheel", pos: [ 0.0255, -0.0325, -0.0147], quat: [1, 0, 0, 0] },
        ],
      },
      {
        bodyIdx: 14,
        ankleGeoms: [
          { stl: "ankle_r_v1.stl",   pos: [0.022, 0.00622291, -0.0647],   quat: [0.5, -0.5, 0.5, 0.5] },
          { stl: "roller_blade.stl", pos: [0.022, 0.00622291, 0.0355181], quat: [0.5, -0.5, 0.5, 0.5] },
        ],
        wheels: [
          { name: "passive_RF_wheel", pos: [ 0.0395, -0.0325, -0.0144819], quat: [0, 0, 1, 0] },
          { name: "passive_RR_wheel", pos: [-0.0255, -0.0325, -0.0144819], quat: [0, 0, 1, 0] },
        ],
      },
    ];

    configs.forEach((cfg) => {
      const ankleGrp = cadBodyGroups[cfg.bodyIdx];
      cfg.ankleGeoms.forEach((ag) => {
        const m = makeRollerMesh(ag.stl, ag.pos, ag.quat);
        ankleGrp.add(m);
        inlineRollerMeshes.push(m);
      });
      cfg.wheels.forEach((wCfg) => {
        const wGrp = new THREE.Group();
        wGrp.position.set(wCfg.pos[0], wCfg.pos[1], wCfg.pos[2]);
        wGrp.quaternion.set(wCfg.quat[1], wCfg.quat[2], wCfg.quat[3], wCfg.quat[0]).normalize();
        wGrp.userData.baseQuat = wGrp.quaternion.clone();
        wGrp.userData.wheelAngle = 0;
        wGrp.userData.radius = 0.015;

        const tireMesh = makeRollerMesh("tire.stl", [0.0175, 0.0387229, -0.05], [0.5, -0.5, -0.5, -0.5]);
        const rimMesh  = makeRollerMesh("rim.stl",  [0.0175, 0.0387229, -0.05], [0.5, -0.5, -0.5, -0.5]);
        wGrp.add(tireMesh);
        wGrp.add(rimMesh);
        ankleGrp.add(wGrp);
        inlineRollerMeshes.push(wGrp);
        rollerWheels.push(wGrp);
      });
    });

    const selectColorway = document.getElementById("select-colorway");
    applyColorway(duckCadContainer, selectColorway ? selectColorway.value : "white_orange");
    syncRollerFootVisibility();
    pushLog("STL", "Mounted authentic inline roller-blade assemblies (ankle_l/r_v1, roller_blade, 4x rim + tire).");
  }

  function syncRollerFootVisibility() {
    const showRoller = state.locomotionMode === "roller" || state.rollerSkatesForced;
    walkingFootMeshes.forEach((m) => { m.visible = !showRoller; });
    inlineRollerMeshes.forEach((m) => { m.visible = showRoller; });
  }

  // True RANSAC-fitted physical jaw hinge pivot (`JAW_HINGE_LOCAL = [0, 0.00004, 0.0075]`, `JAW_MAX_OPEN = 0.32 rad`)
  // Adapted from MJLab duck.js (Apache-2.0)
  function setupPhysicalJawPivot(headRollGroup, jawMeshes) {
    if (!headRollGroup || !jawMeshes.length) return null;
    const pivot = new THREE.Group();
    pivot.position.set(0, 0.00004, 0.0075);
    headRollGroup.add(pivot);
    jawMeshes.forEach((m) => {
      m.position.sub(pivot.position);
      pivot.add(m);
    });
    return pivot;
  }

  function buildCadRigInThree(rig, targetContainer, isPrimary = false) {
    const geometries = rig.meshes.map((m, idx) => {
      const stlName = DUCK_BIN_MESH_NAMES[idx];
      if (highResGlbGeometries.has(stlName)) {
        return highResGlbGeometries.get(stlName);
      }
      return computeCreasedGeometry(m.verts, m.tris);
    });

    const groups = rig.bodies.map((b) => {
      const grp = new THREE.Group();
      grp.position.set(b.pos[0], b.pos[1], b.pos[2]);
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

    // If `kinematics.json` is available, build from its full 70-part list (includes lens, eye rings, m12 mount, PCBs)
    const kinLegs = window.__MICRODUCK_KIN_LEGS__;
    const jawMeshes = [];

    if (kinLegs && kinLegs.bodies && kinLegs.bodies.length === 15) {
      const nameToMeshIdx = {};
      DUCK_BIN_MESH_NAMES.forEach((n, i) => { nameToMeshIdx[n] = i; });

      kinLegs.bodies.forEach((kb, bodyIdx) => {
        (kb.geoms || []).forEach((g) => {
          const stlName = g.mesh;
          let geom = highResGlbGeometries.get(stlName);
          if (!geom && nameToMeshIdx[stlName] !== undefined) {
            geom = geometries[nameToMeshIdx[stlName]];
          }
          if (!geom) return;

          const slot = PART_SLOTS[stlName] || "joints";
          const rawRgb = g.color ? [Math.round(g.color[0] * 255), Math.round(g.color[1] * 255), Math.round(g.color[2] * 255)] : [200, 200, 200];
          const mat = new THREE.MeshStandardMaterial({
            color: new THREE.Color(rawRgb[0] / 255, rawRgb[1] / 255, rawRgb[2] / 255),
            roughness: 0.35,
            metalness: 0.0,
            flatShading: false,
            side: THREE.DoubleSide,
          });
          const mesh = new THREE.Mesh(geom, mat);
          mesh.castShadow = true;
          mesh.receiveShadow = true;
          mesh.position.set(g.pos[0], g.pos[1], g.pos[2]);
          mesh.quaternion.set(g.quat[1], g.quat[2], g.quat[3], g.quat[0]).normalize();
          mesh.userData.baseRgb = rawRgb;
          mesh.userData.bodyIdx = bodyIdx;
          mesh.userData.stlName = stlName;
          mesh.userData.slot = slot;
          groups[bodyIdx].add(mesh);

          if (isPrimary) {
            if (["foot_left.stl", "sole_left.stl", "ankle_left.stl", "foot_right.stl", "sole_right.stl", "ankle_right.stl"].includes(stlName)) {
              walkingFootMeshes.push(mesh);
            }
            if (stlName === "jaw.stl" || stlName === "jaw_soft.stl") {
              jawMeshes.push(mesh);
            }
          }
        });
      });
    } else {
      rig.parts.forEach((p) => {
        const stlName = DUCK_BIN_MESH_NAMES[p.mesh] || "trunk_base.stl";
        const slot = PART_SLOTS[stlName] || "joints";
        const mat = new THREE.MeshStandardMaterial({
          color: new THREE.Color(p.rgb[0] / 255, p.rgb[1] / 255, p.rgb[2] / 255),
          roughness: 0.35,
          metalness: 0.0,
          flatShading: false,
          side: THREE.DoubleSide,
        });
        const mesh = new THREE.Mesh(geometries[p.mesh], mat);
        mesh.castShadow = true;
        mesh.receiveShadow = true;
        mesh.position.set(p.pos[0], p.pos[1], p.pos[2]);
        mesh.quaternion.set(p.quat[1], p.quat[2], p.quat[3], p.quat[0]).normalize();
        mesh.userData.baseRgb = p.rgb;
        mesh.userData.bodyIdx = p.body;
        mesh.userData.stlName = stlName;
        mesh.userData.slot = slot;
        groups[p.body].add(mesh);

        if (isPrimary) {
          if (["foot_left.stl", "sole_left.stl", "ankle_left.stl", "foot_right.stl", "sole_right.stl", "ankle_right.stl"].includes(stlName)) {
            walkingFootMeshes.push(mesh);
          }
          if (stlName === "jaw.stl" || stlName === "jaw_soft.stl") {
            jawMeshes.push(mesh);
          }
        }
      });
    }

    if (isPrimary && jawMeshes.length > 0) {
      jawPivotGroup = setupPhysicalJawPivot(groups[9], jawMeshes);
    }

    return groups;
  }

  function applyColorway(targetRoot, colorwayKey) {
    const variant = HARDWARE_COLORWAYS[colorwayKey] || null;
    targetRoot.traverse((child) => {
      if (!child.isMesh) return;
      if (!variant || !variant.slots) {
        const base = child.userData.baseRgb || [220, 220, 220];
        child.material.color.setRGB(base[0] / 255, base[1] / 255, base[2] / 255);
        child.material.roughness = 0.42;
        child.material.metalness = 0.10;
        return;
      }
      const slotName = child.userData.slot || PART_SLOTS[child.userData.stlName] || "joints";
      const spec = variant.slots[slotName] || variant.slots.joints;
      if (spec) {
        child.material.color.setHex(spec.color);
        child.material.roughness = spec.roughness;
        child.material.metalness = spec.metalness;
      }
    });
  }

  function applyJointsToCadGroups(groups, joints15, mouthOpenRatio = 0) {
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
    if (jawPivotGroup) {
      const clamped = Math.max(0, Math.min(1, mouthOpenRatio));
      jawPivotGroup.rotation.set(0, 0, -clamped * 0.32); // JAW_MAX_OPEN = 0.32 rad around -Z
    }
  }

  // Load ground-truth CAD rig immediately from bundle, then hot-upgrade to high-res `microduck.glb` (104k tris)
  function mountCadRig(rig) {
    if (!rig || cadRigData) return;
    cadRigData = rig;
    cadBodyGroups = buildCadRigInThree(rig, duckCadContainer, true);
    const footOffset = rig.home_pose_stats ? rig.home_pose_stats.foot_offset_y : 0;
    duckCadContainer.position.y = footOffset;
    applyJointsToCadGroups(cadBodyGroups, state.joints, state.cmd.mouthOpen);
    const selectColorway = document.getElementById("select-colorway");
    applyColorway(duckCadContainer, selectColorway ? selectColorway.value : "white_orange");
    tryAttachRollerWheels();
    pushLog("CAD", `Loaded articulated rig with 36° creased normals & 16-slot PBR materials.`);

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
      const cGroups = buildCadRigInThree(rig, cCad, false);
      applyJointsToCadGroups(cGroups, DEFAULT_POSITION, 0);
      applyColorway(cCad, cfg.colorway);
      choirGroup.add(cRoot);
    });

    // Asynchronously load high-res `assets/mjlab/microduck.glb` (38 CAD STLs, 104k tris) and hot-swap geometries
    if (THREE.GLTFLoader) {
      const loader = new THREE.GLTFLoader();
      loader.load(
        "assets/mjlab/microduck.glb",
        (gltf) => {
          gltf.scene.traverse((node) => {
            if (node.isMesh && node.geometry) {
              const meshName = node.name.endsWith(".stl") ? node.name : `${node.name}.stl`;
              highResGlbGeometries.set(meshName, applyCreasedNormalsToBufferGeometry(node.geometry, Math.PI / 5));
            }
          });
          if (highResGlbGeometries.size > 0) {
            while (duckCadContainer.children.length > 0) {
              duckCadContainer.remove(duckCadContainer.children[0]);
            }
            walkingFootMeshes.length = 0;
            inlineRollerMeshes.length = 0;
            rollerWheels.length = 0;
            jawPivotGroup = null;
            cadBodyGroups = buildCadRigInThree(cadRigData, duckCadContainer, true);
            applyJointsToCadGroups(cadBodyGroups, state.joints, state.cmd.mouthOpen);
            tryAttachRollerWheels();
            const selCol = document.getElementById("select-colorway");
            applyColorway(duckCadContainer, selCol ? selCol.value : "white_orange");
            pushLog("GLB", `Upgraded rig to high-resolution MJLab CAD meshes (${highResGlbGeometries.size} STLs, 36° creased normals).`);
          }
        },
        undefined,
        () => {}
      );
    }
  }

  if (window.__MICRODUCK_V5_TIRE__) {
    tireAssetData = window.__MICRODUCK_V5_TIRE__;
  } else {
    fetch("assets/models/xgoduck_v5_tire.json")
      .then((r) => r.json())
      .then((tire) => {
        tireAssetData = tire;
        tryAttachRollerWheels();
      })
      .catch((err) => pushLog("STL", `Roller wheel STL pending: ${err}`));
  }

  if (window.__MICRODUCK_CAD_RIG__) {
    mountCadRig(window.__MICRODUCK_CAD_RIG__);
  } else {
    fetch("assets/models/microduck_cad_rig.json")
      .then((r) => r.json())
      .then((rig) => mountCadRig(rig))
      .catch((err) => pushLog("ERR", `CAD rig load failed: ${err}`));
  }

  let stlKitLoaded = false;
  function ensureStlKitPlatterLoaded() {
    if (stlKitLoaded) return;
    stlKitLoaded = true;
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
  }

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

  // Seeded deterministic PRNG for Sim-to-Real Reality Gap noise
  let gapSeed = 1337;
  function seededGaussian(sigma) {
    if (sigma <= 0) return 0;
    gapSeed = (gapSeed * 1664525 + 1013904223) >>> 0;
    const u1 = Math.max(1e-7, (gapSeed & 0xfffffff) / 0x10000000);
    gapSeed = (gapSeed * 1664525 + 1013904223) >>> 0;
    const u2 = (gapSeed & 0xfffffff) / 0x10000000;
    return Math.sqrt(-2.0 * Math.log(u1)) * Math.cos(2.0 * Math.PI * u2) * sigma;
  }

  function clampWithContract(val, lo, hi) {
    if (!Number.isFinite(val)) {
      state.customScript.nanCount += 1;
      return 0;
    }
    if (val < lo) {
      state.customScript.clampCount += 1;
      return lo;
    }
    if (val > hi) {
      state.customScript.clampCount += 1;
      return hi;
    }
    return val;
  }

  const duckApi = {
    setJoint(nameOrIdx, rad) {
      let idx = typeof nameOrIdx === "string" ? JOINT_NAMES.indexOf(nameOrIdx) : nameOrIdx;
      if (idx >= 0 && idx < 15) {
        state.customJointOverrides[idx] = clampWithContract(rad, ACTUATOR_MIN_RAD, ACTUATOR_MAX_RAD);
      }
    },
    set_joint(nameOrIdx, rad) { this.setJoint(nameOrIdx, rad); },

    setJointOffset(nameOrIdx, deltaRad) {
      let idx = typeof nameOrIdx === "string" ? JOINT_NAMES.indexOf(nameOrIdx) : nameOrIdx;
      if (idx >= 0 && idx < 15) {
        const clamped = clampWithContract(DEFAULT_POSITION[idx] + deltaRad, ACTUATOR_MIN_RAD, ACTUATOR_MAX_RAD);
        state.customJointOffsets[idx] = clamped - DEFAULT_POSITION[idx];
      }
    },
    set_joint_offset(nameOrIdx, deltaRad) { this.setJointOffset(nameOrIdx, deltaRad); },

    setMouth(ratio) {
      const r = clampWithContract(ratio, 0, 1);
      state.cmd.mouthOpen = r;
      state.joints[MOUTH_INDEX] = mouthTarget(r);
      if (sliderMouth) sliderMouth.value = r;
      if (valMouth) valMouth.textContent = `${Math.round(r * 100)}%`;
    },
    set_mouth(ratio) { this.setMouth(ratio); },

    setHead(yaw, pitch, roll) {
      if (yaw !== undefined) {
        state.cmd.headYaw = clampWithContract(yaw, -1.0, 1.0);
        if (sliderHeadYaw) sliderHeadYaw.value = state.cmd.headYaw;
        if (valHeadYaw) valHeadYaw.textContent = `${state.cmd.headYaw.toFixed(2)} rad`;
      }
      if (pitch !== undefined) {
        state.cmd.headPitch = clampWithContract(pitch, -0.6, 0.6);
        if (sliderHeadPitch) sliderHeadPitch.value = state.cmd.headPitch;
        if (valHeadPitch) valHeadPitch.textContent = `${state.cmd.headPitch.toFixed(2)} rad`;
      }
      if (roll !== undefined) {
        state.cmd.headRoll = clampWithContract(roll, -0.6, 0.6);
      }
    },
    set_head(yaw, pitch, roll) { this.setHead(yaw, pitch, roll); },

    setTwist(vx, vyaw, vy = 0) {
      if (vx !== undefined) {
        state.cmd.vx = clampWithContract(vx, -0.35, 0.45);
        if (sliderVx) sliderVx.value = state.cmd.vx;
        if (valVx) valVx.textContent = `${state.cmd.vx.toFixed(2)} m/s`;
      }
      if (vyaw !== undefined) {
        state.cmd.vyaw = clampWithContract(vyaw, -1.2, 1.2);
        if (sliderVyaw) sliderVyaw.value = state.cmd.vyaw;
        if (valVyaw) valVyaw.textContent = `${state.cmd.vyaw.toFixed(2)} rad/s`;
      }
      if (vy !== undefined) {
        state.cmd.vy = clampWithContract(vy, -0.25, 0.25);
        if (sliderVy) sliderVy.value = state.cmd.vy;
        if (valVy) valVy.textContent = `${state.cmd.vy.toFixed(2)} m/s`;
      }
    },
    set_twist(vx, vyaw, vy = 0) { this.setTwist(vx, vyaw, vy); },

    setMode(mode) {
      if (mode === "walk" || mode === "roller") {
        if (state.locomotionMode !== mode) {
          state.locomotionMode = mode;
          if (mode === "roller") state.rollerSkatesForced = true;
          updateLocomotionUi();
        }
      }
    },
    set_mode(mode) { this.setMode(mode); },

    doSkill(name) {
      if (!state.activeSkill || state.activeSkill.name !== name) {
        triggerSkill(name);
      }
    },
    do_skill(name) { this.doSkill(name); },

    recover() {
      if (state.isFallen || state.isResting || state.isSeated) {
        state.isFallen = false;
        state.isResting = false;
        state.isSeated = false;
        state.pose.pitchFall *= 0.25;
        state.pose.rollFall *= 0.25;
        state.impactGuardStates.fill(0);
        pushLog("robotd", "Closed-loop reflex triggered recovery -> UPRIGHT");
      }
    },

    readImu() {
      const sigma = state.simGap === "harsh" ? 0.055 : (state.simGap === "nominal" ? 0.02 : 0.0);
      const rawPitch = state.pose.pitchFall + (state.cmd.bodyPitch || 0) + seededGaussian(sigma);
      const rawRoll = state.pose.rollFall + (state.cmd.bodyRoll || 0) + seededGaussian(sigma);
      const tiltRad = Math.hypot(rawPitch, rawRoll);
      const snap = {
        roll: rawRoll,
        pitch: rawPitch,
        yaw: state.pose.yaw,
        tilt_deg: Number((tiltRad * 180 / Math.PI).toFixed(1)),
        gyro: [
          seededGaussian(sigma * 2),
          state.velocities[2] * 0.25 + seededGaussian(sigma * 2),
          state.cmd.vyaw + seededGaussian(sigma * 2)
        ],
        gravity: [Math.sin(rawPitch), -Math.sin(rawRoll), -Math.cos(tiltRad)],
        is_fallen: state.isFallen,
        is_picked_up: state.isPickedUp,
        guard_profile: state.guardLevel,
        trips: state.fallTrips,
      };
      state.imuDelayedQueue.push(snap);
      const delayTicks = state.simGap === "harsh" ? 2 : (state.simGap === "nominal" ? 1 : 0);
      while (state.imuDelayedQueue.length > delayTicks + 1) {
        state.imuDelayedQueue.shift();
      }
      return state.imuDelayedQueue[0] || snap;
    },
    read_imu() { return this.readImu(); },

    readTof() {
      const d = Number(sliderTheremin.value) || 0.34;
      const sigma = state.simGap === "harsh" ? 0.015 : (state.simGap === "nominal" ? 0.005 : 0.0);
      const noisyD = Math.max(0.04, d + seededGaussian(sigma));
      return {
        distance_m: Number(noisyD.toFixed(3)),
        min_zone_m: Number(noisyD.toFixed(3)),
        hand_present: noisyD < 0.60,
        raw_mm: Math.round(noisyD * 1000),
      };
    },
    read_tof() { return this.readTof(); },

    readTarget() {
      const wrapAngle = (a) => {
        while (a > Math.PI) a -= 2 * Math.PI;
        while (a < -Math.PI) a += 2 * Math.PI;
        return a;
      };
      const ldx = state.laser.x - state.pose.x;
      const ldz = state.laser.z - state.pose.z;
      const lDist = Math.hypot(ldx, ldz);
      const lBearing = wrapAngle(Math.atan2(-ldz, ldx) - state.pose.yaw);

      const bdx = state.ball.x - state.pose.x;
      const bdz = state.ball.z - state.pose.z;
      const bDist = Math.hypot(bdx, bdz);
      const bBearing = wrapAngle(Math.atan2(-bdz, bdx) - state.pose.yaw);

      return {
        pose: { x: Number(state.pose.x.toFixed(3)), z: Number(state.pose.z.toFixed(3)), yaw: Number(state.pose.yaw.toFixed(3)) },
        laser: { active: state.laser.active, x: state.laser.x, z: state.laser.z, dist: Number(lDist.toFixed(3)), bearing_rad: Number(lBearing.toFixed(3)) },
        ball: { x: state.ball.x, z: state.ball.z, vx: state.ball.vx, vz: state.ball.vz, dist: Number(bDist.toFixed(3)), bearing_rad: Number(bBearing.toFixed(3)) },
      };
    },
    read_target() { return this.readTarget(); },

    readJoints() {
      return {
        names: [...JOINT_NAMES],
        pos: [...state.joints],
        vel: [...state.velocities],
        defaults: [...DEFAULT_POSITION],
      };
    },
    read_joints() { return this.readJoints(); },

    readObs61() {
      return [...state.obs61];
    },
    read_obs61() { return this.readObs61(); },

    // Pillar 1: Sole Contact Anchor Odometry (`odometry/src/lib.rs` & `anchors.rs`)
    readOdometry() {
      return {
        anchor_set: "alpha16",
        anchor_foot: state.odom.anchorFoot,
        anchor_idx: state.odom.anchorIdx,
        switches: state.odom.switches,
        alpha16_pose: { x: Number(state.odom.alpha16X.toFixed(4)), z: Number(state.odom.alpha16Z.toFixed(4)), yaw: Number(state.pose.yaw.toFixed(4)) },
        v15_pose: { x: Number(state.odom.v15X.toFixed(4)), z: Number(state.odom.v15Z.toFixed(4)), yaw: Number(state.pose.yaw.toFixed(4)) },
        drift_alpha16_mm: Number(state.odom.driftAlpha16Mm.toFixed(2)),
        drift_v15_mm: Number(state.odom.driftV15Mm.toFixed(2)),
        switch_margin_m: -0.010,
        confirm_ticks: 2,
      };
    },
    read_odometry() { return this.readOdometry(); },

    // Pillar 2 & 3: XL330 Register Block 124..136 + Slow Sensors 144..146 (`duck-control/src/bus.rs`)
    readBusSlow() {
      return {
        volts: Number(state.bus.volts.toFixed(1)),
        temps_c: state.bus.tempsC.map((t) => Math.round(t)),
        max_temp_c: Math.round(Math.max(...state.bus.tempsC)),
        currents_ma: state.bus.currentsMa.map((c) => Math.round(c)),
        pwm_counts: [...state.bus.pwmCounts],
        stale_imu_run: state.bus.staleImuRun,
        imu_frozen_warn: state.bus.staleImuRun >= 25,
        remote_io_protocol: state.bus.remoteIoProto,
      };
    },
    read_bus_slow() { return this.readBusSlow(); },

    // Pillar 4: `maploc` Stop-and-Vote ToF Mapper & `pet-detect` 40x100 Log-Mel Classifier
    readPetting() {
      return {
        is_petting: state.petDetect.isPetting,
        probability: Number(state.petDetect.prob.toFixed(3)),
        enter_threshold: state.petDetect.enterThreshold,
        exit_threshold: state.petDetect.exitThreshold,
        window_samples: state.petDetect.windowSamples,
        n_mels: state.petDetect.nMels,
        window_frames: state.petDetect.windowFrames,
        maploc_stopped: state.maploc.isStopped,
        maploc_keyframes: state.maploc.keyframes,
      };
    },
    read_petting() { return this.readPetting(); },

    // Pillar 5: `pad-imu` 6-Axis Complementary Filter & `SYN_DROPPED` Dead-Radio Auditor
    readPadImu() {
      return {
        calibrated: state.padImu.calibrated,
        bias_dps: [...state.padImu.biasDps],
        still_window_s: state.padImu.stillWindowS,
        stale_radio_ticks: state.padImu.staleRadioTicks,
        dead_radio_trap_detected: state.padImu.deadRadioTrap,
      };
    },
    read_pad_imu() { return this.readPadImu(); },

    watch(key, val) {
      if (key) {
        state.customScript.watchVars[String(key)] =
          typeof val === "number" ? (Number.isInteger(val) ? val : Number(val.toFixed(3))) : String(val);
      }
    },

    assert(condition, label = "invariant") {
      if (condition) {
        state.customScript.assertPassed += 1;
      } else {
        state.customScript.assertFailed += 1;
        state.customScript.lastFailedAssert = String(label);
      }
      return Boolean(condition);
    },

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
      state.customJointOffsets = {};
      state.imuDelayedQueue = [];
      state.customScript.ticks = 0;
      state.customScript.execMs = 0;
      state.customScript.maxExecMs = 0;
      state.customScript.clampCount = 0;
      state.customScript.nanCount = 0;
      state.customScript.slewCount = 0;
      state.customScript.assertPassed = 0;
      state.customScript.assertFailed = 0;
      state.customScript.lastFailedAssert = null;
      state.customScript.watchVars = {};
      state.cmd.vx = 0;
      state.cmd.vy = 0;
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
      case "roller_crouch": {
        // Phase-encoded Crouch-Glide (`BEST_roller_crouch.onnx`: CROUCH_PERIOD_S=5.0s, CROUCH_END_PHASE=0.7 -> 3.5s)
        // Sinks smoothly into a low aerodynamic speed-skater tuck while gliding on all 4 inline wheels, then rises cleanly.
        const dip = pulse(0.0, 0.26, 0.68, 0.98, p);
        const carve = Math.sin(p * Math.PI * 2.0) * 0.06 * dip;
        pose.liftY = -0.048 * dip;
        pose.pitchFall = 0.11 * dip;
        pose.rollFall = carve;
        bothLegs(-0.52 * dip, 1.04 * dip, -0.52 * dip);
        a[1] = -carve * 0.8;
        a[10] = -carve * 0.8;
        a[5] = 0.38 * dip; // Neck tucked forward
        a[6] = -0.18 * dip; // Gaze level ahead
        a[8] = -carve * 0.7;
        cmd.mouthOpen = 0.35 * pulse(0.25, 0.4, 0.65, 0.85, p);
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

    // Execute 50 Hz Closed-Loop Custom Controller (runs even when fallen/picked-up for reflex recovery)
    if (state.customScript.active && typeof state.customScript.fn === "function") {
      state.customJointOverrides = {};
      state.customJointOffsets = {};
      const t0 = performance.now();
      try {
        const ret = state.customScript.fn(state.time, dt, duckApi);
        const elapsedMs = performance.now() - t0;
        state.customScript.ticks += 1;
        state.customScript.execMs = elapsedMs;
        if (elapsedMs > state.customScript.maxExecMs) {
          state.customScript.maxExecMs = elapsedMs;
        }
        if (ret && typeof ret === "object") {
          for (const [k, v] of Object.entries(ret)) {
            duckApi.watch(k, v);
          }
        }
      } catch (err) {
        state.customScript.active = false;
        state.customScript.lastError = err.message;
        const statusEl = document.getElementById("code-status-text");
        if (statusEl) {
          statusEl.textContent = `Runtime Error: ${err.message}`;
          statusEl.style.color = "var(--rose)";
        }
        const badge = document.getElementById("code-runner-badge");
        if (badge) {
          badge.textContent = "RUNTIME ERROR";
          badge.style.color = "var(--rose)";
        }
        pushLog("script/err", `Custom controller halted at tick ${state.customScript.ticks}: ${err.message}`);
      }
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
          // Authentic Inline Roller Skating Stride (`BEST_roller.onnx` velocity envelope:
          // RVEL_FWD = 0.60 m/s, RVEL_BACK = -0.50 m/s, RVEL_ANG = 0.30 rad/s)
          const skateFreq = 4.6;
          const phase = state.time * skateFreq;
          const s = Math.sin(phase);
          const strideAmp = Math.min(1.0, speed * 2.4);

          // Balanced inline-skate knee bend & ankle trim
          const crouch = 0.22;
          action14[2] = -crouch;
          action14[3] = crouch * 1.85;
          action14[4] = -crouch * 0.85;
          action14[11] = crouch;
          action14[12] = -crouch * 1.85;
          action14[13] = crouch * 0.85;

          // Smooth alternating V-push & long inline glide:
          const pushL = Math.max(0, s) * strideAmp;
          const pushR = Math.max(0, -s) * strideAmp;

          // Left inline skate push & glide
          action14[0] = -0.10 * pushL + 0.05 * pushR;  // Slight V-skate toe-out on push
          action14[1] = -0.19 * pushL + 0.05 * pushR;  // Lateral inline edge push
          action14[2] += -0.15 * pushL + 0.08 * pushR;
          action14[3] += -0.20 * pushL + 0.12 * pushR;
          action14[4] += 0.10 * pushL - 0.06 * pushR;

          // Right inline skate push & glide (mirrored pitch signs)
          action14[9]  = 0.10 * pushR - 0.05 * pushL;
          action14[10] = 0.19 * pushR - 0.05 * pushL;
          action14[11] += 0.15 * pushR - 0.08 * pushL;
          action14[12] += 0.20 * pushR - 0.12 * pushL;
          action14[13] += -0.10 * pushR + 0.06 * pushL;

          // Smooth CoM weight transfer over the gliding skate + carve lean on turns
          const turnLean = -Math.max(-0.35, Math.min(0.35, state.cmd.vyaw)) * 0.12;
          state.pose.rollFall = -0.055 * s * strideAmp + turnLean;
          state.pose.pitchFall = 0.04 * strideAmp;

          // Vestibular head stabilization during inline glide
          action14[5] += 0.16 * strideAmp;
          action14[7] += state.cmd.vyaw * 0.25;
          action14[8] += -state.pose.rollFall * 0.80;
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

      // Scatter 14 actions + custom offsets/overrides -> 15 joints (with XL330 slew limit under harsh gap)
      const maxSlewRad = state.simGap === "harsh" ? (8.5 * dt) : (state.simGap === "nominal" ? (14.0 * dt) : Infinity);
      let src = 0;
      for (let i = 0; i < 15; i++) {
        let desired;
        if (i === MOUTH_INDEX) {
          desired = mouthTarget(mouthOpen);
        } else {
          const offset = Number.isFinite(state.customJointOffsets[i]) ? state.customJointOffsets[i] : 0;
          desired = Math.max(-Math.PI, Math.min(Math.PI, DEFAULT_POSITION[i] + action14[src] + offset));
          src++;
        }
        if (Number.isFinite(state.customJointOverrides[i])) {
          desired = Math.max(-Math.PI, Math.min(Math.PI, state.customJointOverrides[i]));
        }
        const prev = state.joints[i];
        let delta = desired - prev;
        if (Math.abs(delta) > maxSlewRad) {
          delta = Math.sign(delta) * maxSlewRad;
          state.customScript.slewCount += 1;
        }
        const next = prev + delta;
        state.velocities[i] = (next - prev) / dt;
        state.joints[i] = next;
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
    if (floorMat.uniforms && floorMat.uniforms.uFocus) {
      floorMat.uniforms.uFocus.value.set(state.pose.x, 0, state.pose.z);
    }

    if (state.activeMesh === "cad") {
      applyJointsToCadGroups(cadBodyGroups, state.joints, mouthOpen);
      syncRollerFootVisibility();
      const showRoller = state.locomotionMode === "roller" || state.rollerSkatesForced;
      if (showRoller) {
        const moveSpeed = Math.abs(state.cmd.vx) > 0.005 ? state.cmd.vx : (speed > 0.01 ? speed : 0);
        if (Math.abs(moveSpeed) > 0.005) {
          const dTheta = (moveSpeed * dt) / 0.015;
          const spinQ = new THREE.Quaternion();
          const zAxis = new THREE.Vector3(0, 0, 1);
          for (let i = 0; i < rollerWheels.length; i++) {
            const w = rollerWheels[i];
            w.userData.wheelAngle = (w.userData.wheelAngle || 0) + dTheta;
            spinQ.setFromAxisAngle(zAxis, w.userData.wheelAngle);
            w.quaternion.copy(w.userData.baseQuat).multiply(spinQ);
          }
        }
      }
    } else {
      fluxAiContainer.position.y = 0;
      fluxAiContainer.rotation.y = 0;
    }

    updateTofMatrix();
    updatePreHardwareSubsystems(dt, speed);
    updateFaceLeds();
  }

  function updatePreHardwareSubsystems(dt, speed) {
    // 1. Pillar 1: ALPHA16 vs V15 Sole Contact Odometry (`odometry/src/lib.rs`)
    const leftLift = state.joints[3] - DEFAULT_POSITION[3];
    const rightLift = -(state.joints[12] - DEFAULT_POSITION[12]);
    const candFoot = leftLift < rightLift ? "left" : "right";
    const zDiff = Math.abs(leftLift - rightLift) * 0.042;
    if (candFoot !== state.odom.anchorFoot && zDiff > 0.010) {
      if (state.odom.pendingFoot === candFoot) {
        state.odom.pendingTicks += 1;
      } else {
        state.odom.pendingFoot = candFoot;
        state.odom.pendingTicks = 1;
      }
      if (state.odom.pendingTicks >= 2) {
        state.odom.anchorFoot = candFoot;
        state.odom.anchorIdx = (state.odom.anchorIdx + 5) % 16;
        state.odom.switches += 1;
        state.odom.pendingFoot = null;
        state.odom.pendingTicks = 0;
      }
    } else {
      state.odom.pendingFoot = null;
      state.odom.pendingTicks = 0;
    }
    // Upstream benchmark: over a 3.2m walk, ALPHA16 drifts ~9mm (2.81 mm/m) vs V15 ~17mm (5.31 mm/m)
    state.odom.driftAlpha16Mm = state.vetbook.metersWalked * 2.81;
    state.odom.driftV15Mm = state.vetbook.metersWalked * 5.31;
    const driftRad = state.pose.yaw + 0.35;
    state.odom.alpha16X = state.pose.x + (state.odom.driftAlpha16Mm / 1000) * Math.cos(driftRad);
    state.odom.alpha16Z = state.pose.z + (state.odom.driftAlpha16Mm / 1000) * Math.sin(driftRad);
    state.odom.v15X = state.pose.x + (state.odom.driftV15Mm / 1000) * Math.cos(driftRad);
    state.odom.v15Z = state.pose.z + (state.odom.driftV15Mm / 1000) * Math.sin(driftRad);

    // 2. Pillar 3: XL330 Register Block 124..136 (`0.229 rpm/count`) & Slow Sensors 144..146 (`0.1V`, `1C`)
    const RAD_PER_SEC_PER_COUNT = 0.229 * (2 * Math.PI / 60);
    const RAD_PER_COUNT = (2 * Math.PI) / 4096;
    let totalCurrentA = 0;
    for (let i = 0; i < 15; i++) {
      if (state.simGap !== "ideal") {
        state.velocities[i] = Math.round(state.velocities[i] / RAD_PER_SEC_PER_COUNT) * RAD_PER_SEC_PER_COUNT;
        state.joints[i] = Math.round(state.joints[i] / RAD_PER_COUNT) * RAD_PER_COUNT;
      }
      const pwm = Math.max(-885, Math.min(885, Math.round(state.velocities[i] * 85)));
      state.bus.pwmCounts[i] = pwm;
      const curMa = Math.min(1400, 65 + Math.abs(pwm) * 0.95 + (state.isSeated ? 20 : 85));
      state.bus.currentsMa[i] = curMa;
      totalCurrentA += curMa / 1000;
      const iAmp = curMa / 1000;
      const dTemp = (iAmp * iAmp * 5.2 - (state.bus.tempsC[i] - 31.0) * 0.12) * dt;
      state.bus.tempsC[i] = Math.max(31.0, Math.min(82.0, state.bus.tempsC[i] + dTemp));
    }
    const rawVolts = Math.max(6.8, Math.min(8.4, 8.2 - totalCurrentA * 0.14));
    state.bus.volts = Math.round(rawVolts * 10) / 10;

    // 3. Pillar 4: `maploc` Stop-and-Vote ToF Keyframe Mapper & `pet-detect` Hysteresis
    const isStopped = !state.isFallen && speed < 0.015;
    state.maploc.isStopped = isStopped;
    if (isStopped) {
      state.maploc.stopTicks += 1;
      if (state.maploc.stopTicks === 13) {
        state.maploc.keyframes += 1;
      }
    } else {
      state.maploc.stopTicks = 0;
    }

    const handDist = Number(sliderTheremin.value) || 0.34;
    if (state.thereminOn && handDist <= 0.14) {
      state.petDetect.prob = 0.975;
    } else if (state.activeEmotion) {
      state.petDetect.prob = 0.22; // Reject self-vocalization
    } else {
      state.petDetect.prob = 0.02;
    }
    if (!state.petDetect.isPetting && state.petDetect.prob >= state.petDetect.enterThreshold) {
      state.petDetect.isPetting = true;
    } else if (state.petDetect.isPetting && state.petDetect.prob < state.petDetect.exitThreshold) {
      state.petDetect.isPetting = false;
    }

    // 4. Pillar 5: `pad-imu` 12 deg/s Clone Bias & Stale-Radio Auditor
    state.padImu.calibrated = state.simGap !== "harsh";
    state.padImu.staleRadioTicks = state.simGap === "harsh" && speed > 0.22 ? 14 : 0;
    state.padImu.deadRadioTrap = state.padImu.staleRadioTicks >= 13;
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
    mOdom.textContent = `${state.odom.alpha16X.toFixed(2)}, ${state.odom.alpha16Z.toFixed(2)} m (A16:${state.odom.anchorFoot[0].toUpperCase()}#${state.odom.anchorIdx} · Δ${state.odom.driftAlpha16Mm.toFixed(1)}mm)`;
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

    // Update 50 Hz Closed-Loop Watch Strip & Hardware Contract
    const cwBudget = document.getElementById("cw-budget");
    const cwContract = document.getElementById("cw-contract");
    const cwAssert = document.getElementById("cw-assert");
    const cwVars = document.getElementById("cw-vars");
    if (cwBudget && cwContract && cwAssert && cwVars) {
      const cs = state.customScript;
      if (cs.active) {
        const overBudget = cs.maxExecMs > 4.0;
        cwBudget.textContent = `exec: ${cs.execMs.toFixed(2)} ms (peak ${cs.maxExecMs.toFixed(2)} ms) · tick #${cs.ticks}`;
        cwBudget.className = `cw-pill ${overBudget ? "cw-warn" : "cw-ok"}`;

        const hasViolations = cs.clampCount > 0 || cs.nanCount > 0;
        cwContract.textContent = `clamps: ${cs.clampCount} · NaN: ${cs.nanCount} · slew: ${cs.slewCount}`;
        cwContract.className = `cw-pill ${cs.nanCount > 0 ? "cw-err" : (hasViolations ? "cw-warn" : "cw-ok")}`;

        if (cs.assertFailed > 0) {
          cwAssert.textContent = `assert FAIL (${cs.lastFailedAssert}): ${cs.assertFailed}`;
          cwAssert.className = "cw-pill cw-err";
        } else if (cs.assertPassed > 0) {
          cwAssert.textContent = `asserts PASS (${cs.assertPassed})`;
          cwAssert.className = "cw-pill cw-ok";
        } else {
          cwAssert.textContent = "asserts: 0";
          cwAssert.className = "cw-pill";
        }

        const entries = Object.entries(cs.watchVars);
        if (entries.length > 0) {
          cwVars.innerHTML = entries
            .slice(0, 8)
            .map(([k, v]) => `<span class="cw-var"><b>${k}:</b> ${v}</span>`)
            .join("");
        } else {
          cwVars.innerHTML = `<span class="cw-var-muted">Call duck.watch(k, v) or return {k: v} to inspect live state</span>`;
        }
      }
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
      skillStatusTag.textContent = state.activeSkill?.name === "roller_crouch"
        ? "BEST_roller_crouch.onnx"
        : (state.locomotionMode === "walk" ? "BEST_alpha_walking.onnx" : "BEST_roller.onnx");
    }
    const btnToggleRollerSkates = document.getElementById("btn-toggle-roller-skates");
    if (btnToggleRollerSkates) {
      btnToggleRollerSkates.classList.toggle("active", state.locomotionMode === "roller" || state.rollerSkatesForced);
    }
    syncRollerFootVisibility();
  }

  function triggerSkill(name) {
    if (name === "walk" || name === "roller") {
      state.locomotionMode = name;
      state.isSeated = false;
      state.isResting = false;
      
      if (name === "roller") {
        state.rollerSkatesForced = true;
        if (sliderVx) { sliderVx.min = "-0.50"; sliderVx.max = "0.60"; }
        if (sliderVyaw) { sliderVyaw.min = "-0.30"; sliderVyaw.max = "0.30"; }
        if (Math.abs(state.cmd.vx) < 0.05) {
          state.cmd.vx = 0.36;
          if (sliderVx) sliderVx.value = 0.36;
          if (valVx) valVx.textContent = "0.36 m/s";
        }
        state.cmd.vyaw = Math.max(-0.30, Math.min(0.30, state.cmd.vyaw));
        pushLog("robotd", "Switched to Inline Roller Blades (BEST_roller.onnx: RVEL_FWD=0.6m/s, RVEL_ANG=0.3rad/s).");
      } else {
        state.rollerSkatesForced = false;
        if (sliderVx) { sliderVx.min = "-0.30"; sliderVx.max = "0.45"; }
        if (sliderVyaw) { sliderVyaw.min = "-1.20"; sliderVyaw.max = "1.20"; }
        pushLog("robotd", "Switched to Bipedal Walk Feet (BEST_alpha_walking.onnx).");
      }
      updateLocomotionUi();
      playProceduralQuack(name === "walk" ? 1.0 : 1.25);
      return;
    }
    if (name === "roller_crouch") {
      if (state.locomotionMode !== "roller") {
        triggerSkill("roller");
      }
      if (Math.abs(state.cmd.vx) < 0.15) {
        state.cmd.vx = 0.42;
        if (sliderVx) sliderVx.value = 0.42;
        if (valVx) valVx.textContent = "0.42 m/s";
      }
      state.activeSkill = { name: "roller_crouch", elapsed: 0, duration: 3.5 };
      updateLocomotionUi();
      pushLog("robotd", "Triggered Crouch-Glide (BEST_roller_crouch.onnx: phase 0..0.7 over 3.5s).");
      return;
    }
    if (name === "sit_toggle") {
      if (state.locomotionMode === "roller") {
        triggerSkill("roller_crouch");
        return;
      }
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
      roller_crouch: 3.5,
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
      ensureStlKitPlatterLoaded();
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
      btnToggleRollerSkates.classList.toggle("active", state.locomotionMode === "roller" || state.rollerSkatesForced);
      syncRollerFootVisibility();
      pushLog("STL", `Inline Roller Blades (ankle_l/r_v1 + roller_blade + 4x tire/rim) ${state.rollerSkatesForced ? "MOUNTED" : "AUTO (Roller mode only)"}`);
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
  // 8.5. 50 Hz Closed-Loop Code Lab, Sim-to-Real Gap & Compositional Copilot
  // ---------------------------------------------------------------------------
  const CODE_PRESETS = {
    pd_reflex: `// Preset 1: Closed-Loop IMU PD Balance & Push Reflex
// Injects periodic 7.5 rad/s push disturbances, applies ankle/hip pitch PD
// counter-torque via setJointOffset(), and auto-recovers if Level 5 collapses.
let stepCount = 0;
function onStep(t, dt, duck) {
  stepCount++;
  duck.setMode('walk');

  // Periodic external push test every 4.5 seconds (225 ticks)
  if (stepCount % 225 === 80) {
    duck.applyImpulse(7.5);
  }

  const imu = duck.readImu();
  if (imu.is_fallen) {
    duck.watch('state', 'RECOVERING');
    duck.recover();
    return;
  }

  // Closed-loop PD ankle & hip stabilization against pitch/roll disturbance
  const kp = 0.85, kd = 0.08;
  const pitchCorr = -kp * imu.pitch - kd * imu.gyro[1];
  const rollCorr  = -kp * imu.roll;

  duck.setJointOffset('left_ankle',  pitchCorr);
  duck.setJointOffset('right_ankle', -pitchCorr); // mirrored right pitch
  duck.setJointOffset('left_hip_roll',  rollCorr);
  duck.setJointOffset('right_hip_roll', rollCorr);
  duck.setHead(0, -imu.pitch * 0.6, -imu.roll * 0.6); // vestibular gaze lock
  duck.setTwist(0.16, 0.0);

  duck.assert(imu.tilt_deg < 48.0, 'tilt<48deg');
  return {
    state: imu.tilt_deg > 8 ? 'STABILIZING' : 'NOMINAL',
    tilt_deg: imu.tilt_deg,
    ankle_pd: pitchCorr
  };
}`,

    laser_servo: `// Preset 2: Closed-Loop SE(2) Laser Dot Visual Servoing
// Click anywhere on the 3D floor grid to move the red laser target!
let integralYaw = 0;
let arrivedQuack = false;
function onStep(t, dt, duck) {
  const tgt = duck.readTarget().laser;
  const errYaw = tgt.bearing_rad;
  const dist = tgt.dist;

  integralYaw = Math.max(-0.4, Math.min(0.4, integralYaw + errYaw * dt));

  // Foveate head gaze onto the floor laser dot
  duck.setHead(errYaw * 0.75, Math.min(0.45, 0.18 + 0.15 / Math.max(0.2, dist)), 0);
  duck.setJointOffset('neck_pitch', 0.22);

  if (dist > 0.14) {
    arrivedQuack = false;
    const vx = Math.min(0.32, dist * 0.55) * Math.max(0.15, Math.cos(errYaw));
    const vyaw = 1.45 * errYaw + 0.25 * integralYaw;
    duck.setTwist(vx, vyaw);
    duck.setMouth(0.0);
  } else {
    duck.setTwist(0.0, 0.0);
    duck.setMouth(0.75);
    if (!arrivedQuack) {
      duck.playQuack(1.22);
      arrivedQuack = true;
    }
  }

  duck.assert(Number.isFinite(dist), 'valid_se2_target');
  return {
    servo: dist > 0.14 ? 'PURSUIT' : 'LOCKED_ON',
    dist_m: dist,
    bearing_rad: errYaw
  };
}`,

    soccer_fsm: `// Preset 3: Autonomous Soccer Striker FSM (Search -> Align -> Strike)
let kickCooldown = 0;
function onStep(t, dt, duck) {
  if (kickCooldown > 0) kickCooldown -= dt;
  const ball = duck.readTarget().ball;
  const err = ball.bearing_rad;
  const dist = ball.dist;

  // Track the soccer ball with neck & head
  duck.setHead(err * 0.8, 0.30, 0);
  duck.setJointOffset('neck_pitch', 0.25);

  let fsm = 'APPROACH';
  if (dist < 0.23 && Math.abs(err) < 0.38 && kickCooldown <= 0) {
    fsm = 'STRIKE';
    duck.setTwist(0, 0);
    duck.doSkill(err >= 0 ? 'kick_left' : 'kick_right');
    duck.playQuack(1.15);
    kickCooldown = 1.6;
  } else if (Math.abs(err) > 0.65) {
    fsm = 'ALIGN_HEADING';
    duck.setTwist(0.06, err * 1.5);
  } else {
    fsm = 'DRIBBLE_APPROACH';
    duck.setTwist(Math.min(0.30, dist * 0.65), err * 1.35);
  }

  duck.assert(dist < 5.0, 'ball_in_arena');
  return { fsm, ball_dist: dist, bearing: err };
}`,

    tof_beatbox: `// Preset 4: VL53L5CX 8x8 ToF Proximity Avoidance & Theremin Beatbox
// Move the 'Hand Distance' slider on the right column (< 0.22m triggers back-off!)
let lastChirp = 0;
function onStep(t, dt, duck) {
  const tof = duck.readTof();
  const d = tof.distance_m;

  if (d < 0.22) {
    // Reflexive collision avoidance: back away & open beak in surprise
    const urgency = (0.22 - d) / 0.14;
    duck.setTwist(-0.22 * Math.min(1, urgency), 0.35 * Math.sin(t * 6.0));
    duck.setHead(0, -0.25, 0.25 * Math.sin(t * 10.0));
    duck.setMouth(0.9);
    if (t - lastChirp > 0.32) {
      duck.playQuack(1.45);
      lastChirp = t;
    }
    duck.assert(d >= 0.04, 'tof_min_range');
    return { zone: 'REPULSION_BACKOFF', tof_mm: tof.raw_mm, vx: -0.22 };
  } else if (tof.hand_present) {
    // Theremin groove tracking
    const prox = (0.60 - d) / 0.38;
    duck.setTwist(0.0, Math.sin(t * 4.0) * 0.35);
    duck.setHead(Math.sin(t * 5.0) * 0.25, (prox - 0.5) * 0.35, 0);
    duck.setMouth(prox * 0.8);
    if (t - lastChirp > 0.45 && prox > 0.35) {
      duck.playQuack(0.95 + prox * 0.55);
      lastChirp = t;
    }
    return { zone: 'THEREMIN_LOCK', tof_mm: tof.raw_mm, pitch_mod: prox };
  }

  duck.setTwist(0.0, 0.0);
  duck.setMouth(0.0);
  return { zone: 'CLEAR_IDLE', tof_mm: tof.raw_mm };
}`,

    python_style: `# Preset 5 (Python): Figure-8 Roller Skate CPG Carver
# Indentation-aware Python transpiled live to 50 Hz WebGL!
def on_step(t, dt, duck):
    duck.set_mode("roller")
    imu = duck.read_imu()
    phase = t * 1.1
    vyaw = math.sin(phase) * 0.85
    carve_lean = -vyaw * 0.16

    if imu.is_fallen:
        duck.recover()
        duck.watch("gait", "RECOVER")
    else:
        duck.set_twist(0.28, vyaw)
        duck.set_joint_offset("left_hip_roll", carve_lean)
        duck.set_joint_offset("right_hip_roll", carve_lean)
        duck.set_head(vyaw * 0.45, 0.18, carve_lean * 1.2)
        duck.set_mouth(abs(math.sin(t * 3.0)) * 0.45)
        duck.watch("gait", "FIGURE8_SKATE")
        duck.watch("vyaw", vyaw)
        duck.assert(imu.tilt_deg < 45.0, "carve_stability")`
  };

  const codeEditorInput = document.getElementById("code-editor-input");
  const selectCodePreset = document.getElementById("select-code-preset");
  const selectSimGap = document.getElementById("select-sim-gap");
  const btnLoadPreset = document.getElementById("btn-load-preset");
  const btnRunCode = document.getElementById("btn-run-code");
  const btnStopCode = document.getElementById("btn-stop-code");
  const codeStatusText = document.getElementById("code-status-text");
  const codeRunnerBadge = document.getElementById("code-runner-badge");
  const btnAiGenerate = document.getElementById("btn-ai-generate");
  const aiPromptInput = document.getElementById("ai-prompt-input");

  // Indentation-aware Python-to-JS transpiler supporting def, if/elif/else, and math/duck idioms
  function transpilePythonToJs(rawCode) {
    if (!/^\s*def\s+(?:on_step|onStep)\s*\(/m.test(rawCode)) {
      return rawCode;
    }
    const lines = rawCode.split(/\r?\n/);
    const out = [];
    const indentStack = [0];

    const replaceExpr = (expr) => expr
      .replace(/\bTrue\b/g, "true")
      .replace(/\bFalse\b/g, "false")
      .replace(/\bNone\b/g, "null")
      .replace(/\band\b/g, "&&")
      .replace(/\bor\b/g, "||")
      .replace(/\bnot\s+/g, "!")
      .replace(/math\.sin/g, "Math.sin")
      .replace(/math\.cos/g, "Math.cos")
      .replace(/math\.tan/g, "Math.tan")
      .replace(/math\.hypot/g, "Math.hypot")
      .replace(/math\.abs/g, "Math.abs")
      .replace(/math\.pi/g, "Math.PI")
      .replace(/(^|[^.\w])abs\(/g, "$1Math.abs(")
      .replace(/(^|[^.\w])min\(/g, "$1Math.min(")
      .replace(/(^|[^.\w])max\(/g, "$1Math.max(");

    for (let rawLine of lines) {
      const strippedComment = rawLine.replace(/#.*$/, "");
      if (!strippedComment.trim()) continue;
      const indent = strippedComment.match(/^\s*/)[0].length;
      const trimmed = strippedComment.trim();

      while (indentStack.length > 1 && indent < indentStack[indentStack.length - 1]) {
        indentStack.pop();
        out.push(" ".repeat(indentStack[indentStack.length - 1]) + "}");
      }

      if (/^def\s+(?:on_step|onStep)\s*\(([^)]*)\)\s*:$/.test(trimmed)) {
        const args = trimmed.match(/^def\s+(?:on_step|onStep)\s*\(([^)]*)\)\s*:$/)[1];
        out.push(`function onStep(${args}) {`);
        indentStack.push(indent + 4);
      } else if (/^elif\s+(.+):$/.test(trimmed)) {
        const cond = replaceExpr(trimmed.match(/^elif\s+(.+):$/)[1]);
        if (out.length && out[out.length - 1].trim() === "}") {
          out.pop();
        }
        out.push(`${" ".repeat(indent)}} else if (${cond}) {`);
        indentStack.push(indent + 4);
      } else if (/^else\s*:$/.test(trimmed)) {
        if (out.length && out[out.length - 1].trim() === "}") {
          out.pop();
        }
        out.push(`${" ".repeat(indent)}} else {`);
        indentStack.push(indent + 4);
      } else if (/^if\s+(.+):$/.test(trimmed)) {
        const cond = replaceExpr(trimmed.match(/^if\s+(.+):$/)[1]);
        out.push(`${" ".repeat(indent)}if (${cond}) {`);
        indentStack.push(indent + 4);
      } else {
        let stmt = replaceExpr(trimmed);
        if (/^[a-zA-Z_]\w*\s*=\s*[^=]/.test(stmt) && !/^(let|const|var)\b/.test(stmt)) {
          stmt = "let " + stmt;
        }
        out.push(" ".repeat(indent) + stmt + ";");
      }
    }

    while (indentStack.length > 1) {
      indentStack.pop();
      out.push("}");
    }
    return out.join("\n");
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
        throw new Error("Define 'function onStep(t, dt, duck)' (JS) or 'def on_step(t, dt, duck):' (Python).");
      }

      state.customScript.fn = stepFn;
      state.customScript.active = true;
      state.customScript.lastError = null;
      state.brainMode = "manual"; // Prevent autonomous brain from fighting custom script

      if (codeStatusText) {
        codeStatusText.textContent = `Status: Executing closed-loop at 50 Hz (Gap: ${state.simGap.toUpperCase()})`;
        codeStatusText.className = "status-live";
        codeStatusText.style.color = "var(--emerald)";
      }
      if (codeRunnerBadge) {
        codeRunnerBadge.textContent = "LIVE 50 Hz";
        codeRunnerBadge.className = "mono-tag";
        codeRunnerBadge.style.color = "var(--emerald)";
        codeRunnerBadge.style.borderColor = "var(--emerald)";
      }
      pushLog("script/run", `50 Hz controller active (Reality gap: ${state.simGap}).`);
    } catch (err) {
      state.customScript.active = false;
      state.customScript.fn = null;
      state.customScript.lastError = err.message;

      if (codeStatusText) {
        codeStatusText.textContent = `Compile Error: ${err.message}`;
        codeStatusText.className = "status-error";
        codeStatusText.style.color = "var(--rose)";
      }
      if (codeRunnerBadge) {
        codeRunnerBadge.textContent = "SYNTAX ERROR";
        codeRunnerBadge.style.color = "var(--rose)";
      }
      pushLog("script/err", `Compilation failed: ${err.message}`);
    }
  }

  // Compositional Intent-Aware Controller Synthesizer (Offline / Instant Fallback)
  function synthesizeCompositionalController(promptText) {
    const p = promptText.toLowerCase();
    const wantsRoller = /roller|skate|glide|carve|wheel/.test(p);
    const wantsFigure8 = /figure[- ]?8|slalom|s-curve|weave/.test(p);
    const wantsCircle = /circle|orbit|spin|turn/.test(p);
    const wantsBackward = /back|moonwalk|reverse|retreat/.test(p);
    const wantsSoccer = /ball|soccer|kick|goal|dribble/.test(p);
    const wantsLaser = /laser|dot|chase|follow|target/.test(p);
    const wantsTof = /tof|hand|theremin|avoid|obstacle|proximity/.test(p);
    const wantsBow = /bow|greet|polite|nod/.test(p);
    const wantsFlamingo = /flamingo|one leg|balance|single/.test(p);

    if (wantsSoccer) return CODE_PRESETS.soccer_fsm;
    if (wantsLaser) return CODE_PRESETS.laser_servo;
    if (wantsTof) return CODE_PRESETS.tof_beatbox;

    const modeStr = wantsRoller ? "roller" : "walk";
    const baseVx = wantsBackward ? -0.20 : (wantsRoller ? 0.30 : 0.22);
    const yawExpr = wantsFigure8
      ? "Math.sin(t * 1.2) * 0.85"
      : (wantsCircle ? "0.65" : "Math.sin(t * 2.4) * 0.35");
    const skillHook = wantsBow
      ? "if (Math.floor(t * 10) % 60 === 15) duck.doSkill('polite_bow');"
      : (wantsFlamingo ? "if (Math.floor(t * 10) % 70 === 15) duck.doSkill('flamingo');" : "");

    return `// Synthesized Closed-Loop Controller: "${promptText.replace(/"/g, "'")}"
function onStep(t, dt, duck) {
  duck.setMode('${modeStr}');
  const imu = duck.readImu();
  if (imu.is_fallen) {
    duck.recover();
    return { status: 'RECOVERING', tilt: imu.tilt_deg };
  }
  ${skillHook}
  const vx = ${baseVx};
  const vyaw = ${yawExpr};
  const pitchCorr = -0.75 * imu.pitch;

  duck.setTwist(vx, vyaw);
  duck.setJointOffset('left_ankle', pitchCorr);
  duck.setJointOffset('right_ankle', -pitchCorr);
  duck.setHead(vyaw * 0.45, 0.18 + 0.12 * Math.sin(t * 4.0), -imu.roll * 0.6);
  duck.setMouth(Math.sin(t * 5.0) > 0.7 ? 0.65 : 0.0);

  duck.assert(imu.tilt_deg < 45.0, 'stability_margin');
  return { mode: '${modeStr.toUpperCase()}', vx, vyaw, tilt_deg: imu.tilt_deg };
}`;
  }

  if (codeEditorInput) {
    codeEditorInput.value = CODE_PRESETS.pd_reflex;
    // Support Cmd/Ctrl+Enter to hot-swap code & Tab for 2-space indentation
    codeEditorInput.addEventListener("keydown", (e) => {
      if ((e.metaKey || e.ctrlKey) && e.key === "Enter") {
        e.preventDefault();
        compileAndRunUserScript(codeEditorInput.value);
      } else if (e.key === "Tab") {
        e.preventDefault();
        const start = codeEditorInput.selectionStart;
        const end = codeEditorInput.selectionEnd;
        codeEditorInput.value = codeEditorInput.value.substring(0, start) + "  " + codeEditorInput.value.substring(end);
        codeEditorInput.selectionStart = codeEditorInput.selectionEnd = start + 2;
      }
    });
  }

  if (selectCodePreset && codeEditorInput) {
    const loadSelectedPreset = () => {
      const key = selectCodePreset.value;
      if (CODE_PRESETS[key]) {
        codeEditorInput.value = CODE_PRESETS[key];
        if (key === "laser_servo" && !state.laser.active) {
          state.laser.active = true;
          laserDotMesh.visible = true;
        }
        compileAndRunUserScript(codeEditorInput.value);
        pushLog("preset", `Loaded & started closed-loop preset: ${key}`);
      }
    };
    selectCodePreset.addEventListener("change", loadSelectedPreset);
    if (btnLoadPreset) btnLoadPreset.addEventListener("click", loadSelectedPreset);
  }

  if (selectSimGap) {
    selectSimGap.addEventListener("change", () => {
      state.simGap = selectSimGap.value;
      state.imuDelayedQueue = [];
      pushLog("sim-gap", `Reality gap profile -> ${state.simGap.toUpperCase()}`);
      if (state.customScript.active && codeStatusText) {
        codeStatusText.textContent = `Status: Executing closed-loop at 50 Hz (Gap: ${state.simGap.toUpperCase()})`;
      }
    });
  }

  // Clickable API snippet chips insert code at cursor
  document.querySelectorAll(".api-chip[data-snippet]").forEach((chip) => {
    chip.addEventListener("click", () => {
      if (!codeEditorInput) return;
      const snippet = chip.getAttribute("data-snippet");
      const pos = codeEditorInput.selectionStart || codeEditorInput.value.length;
      codeEditorInput.value =
        codeEditorInput.value.slice(0, pos) + "\n  " + snippet + codeEditorInput.value.slice(pos);
      codeEditorInput.focus();
    });
  });

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
        codeStatusText.textContent = "Status: Stopped (Returned to manual / policy control)";
        codeStatusText.className = "";
        codeStatusText.style.color = "";
      }
      if (codeRunnerBadge) {
        codeRunnerBadge.textContent = "STANDBY";
        codeRunnerBadge.style.color = "var(--cyan)";
      }
      pushLog("script/stop", "Custom controller stopped. Returned to policy control.");
    });
  }

  if (btnAiGenerate && aiPromptInput && codeEditorInput) {
    const runSynthesize = async () => {
      const promptText = aiPromptInput.value.trim();
      if (!promptText) return;

      btnAiGenerate.disabled = true;
      btnAiGenerate.textContent = "⏳ Synthesizing...";
      if (codeStatusText) codeStatusText.textContent = "Synthesizing 50 Hz closed-loop controller...";

      try {
        const sysPrompt = "You are an autonomous robotics control engineer for the 15-DOF MicroDuck bipedal robot. Write ONLY an executable JavaScript function 'function onStep(t, dt, duck)' using: duck.readImu(), duck.readTarget(), duck.readTof(), duck.setTwist(vx, vyaw), duck.setJointOffset(name, rad), duck.setHead(yaw, pitch, roll), duck.setMouth(0..1), duck.setMode('walk'|'roller'), duck.recover(), duck.assert(cond, label), and return a telemetry object { ... }. No markdown code fences.";
        const ctrl = new AbortController();
        const timer = setTimeout(() => ctrl.abort(), 3500);
        const resp = await fetch("http://127.0.0.1:11434/api/generate", {
          method: "POST",
          headers: { "Content-Type": "application/json" },
          body: JSON.stringify({
            model: "qwen3:30b-a3b",
            prompt: `${sysPrompt}\nUser Request: ${promptText}\nCode:`,
            stream: false,
            keep_alive: 0,
          }),
          signal: ctrl.signal,
        });
        clearTimeout(timer);

        if (resp.ok) {
          const data = await resp.json();
          let generated = (data.response || "").replace(/```[a-z]*\n?/gi, "").trim();
          if (generated.includes("function onStep") || generated.includes("def on_step")) {
            codeEditorInput.value = generated;
            compileAndRunUserScript(generated);
            pushLog("AI", `Qwen 3 MoE synthesized closed-loop controller for: "${promptText}"`);
            return;
          }
        }
        throw new Error("Fallback to compositional synthesizer");
      } catch (e) {
        const generated = synthesizeCompositionalController(promptText);
        codeEditorInput.value = generated;
        compileAndRunUserScript(generated);
        pushLog("AI", `Synthesized closed-loop controller for: "${promptText}"`);
      } finally {
        btnAiGenerate.disabled = false;
        btnAiGenerate.textContent = "✨ Synthesize";
      }
    };
    btnAiGenerate.addEventListener("click", runSynthesize);
    aiPromptInput.addEventListener("keydown", (e) => {
      if (e.key === "Enter") {
        e.preventDefault();
        runSynthesize();
      }
    });
  }

  // Keyboard shortcuts (guarded so typing in TEXTAREA/INPUT/SELECT never hijacks keys or Spacebar!)
  function isEditableTarget(el) {
    if (!el || !el.tagName) return false;
    const tag = el.tagName.toUpperCase();
    return tag === "INPUT" || tag === "TEXTAREA" || tag === "SELECT" || Boolean(el.isContentEditable);
  }

  window.addEventListener("keydown", (e) => {
    if (isEditableTarget(e.target)) return;
    const k = e.key.toLowerCase();
    const isRoller = state.locomotionMode === "roller";
    if (k === "m") toggleMeshMode();
    if (k === "w" || e.key === "ArrowUp") state.cmd.vx = isRoller ? 0.60 : 0.30;
    if (k === "s" || e.key === "ArrowDown") state.cmd.vx = isRoller ? -0.50 : -0.20;
    if (k === "a" || e.key === "ArrowLeft") state.cmd.vyaw = isRoller ? 0.30 : 0.80;
    if (k === "d" || e.key === "ArrowRight") state.cmd.vyaw = isRoller ? -0.30 : -0.80;
    if (k === "r") {
      if (isRoller) triggerSkill("roller_crouch");
      else triggerSkill("sit_toggle");
    }
    if (k === " ") {
      e.preventDefault();
      playProceduralQuack(1.0);
    }
  });
  window.addEventListener("keyup", (e) => {
    if (isEditableTarget(e.target)) return;
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
      title: "50 Hz Closed-Loop Code Lab & Sim-to-Real Testbed",
      text: "Execute closed-loop JS or Python controllers with IMU, SE(2) target, ToF, 61-D obs, execution budget profiling, and real-time hardware assertions.",
      facts: ["50 Hz onStep()", "Cmd/Ctrl+Enter", "Live Watch Strip"]
    },
    "subsys-sim-gap": {
      badge: "SIM-TO-REAL",
      tone: "amber",
      title: "Reality-Gap Non-Ideality Injector",
      text: "Stress-tests custom controllers against seeded IMU Gaussian noise (σ up to 0.055 rad), 1–2 tick observation delay, and XL330 8.5 rad/s slew-rate limiting.",
      facts: ["1-2 Tick Latency", "IMU Noise σ", "8.5 rad/s Slew Cap"]
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
