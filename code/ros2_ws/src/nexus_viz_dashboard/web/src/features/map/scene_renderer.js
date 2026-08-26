import * as THREE from "../../../public/vendor/three/build/three.module.js";
import { OrbitControls } from "../../../public/vendor/three/examples/jsm/controls/OrbitControls.js";

const COLORS = {
  road: 0x414542, silver: 0x8e9598, orange: 0xd86624, purple: 0x79518a,
  blue_red: 0x397ca1, green: 0x4d8a5a, yellow: 0xc9a442, white: 0xc4c8c6,
  white_yellow: 0xd0b66d, beige: 0xaa9270, glass: 0x5a8da1,
  blue_gray: 0x61717b, grass: 0x7c9878, tree: 0x47704f, concrete: 0x757f7d,
  road_white: 0xf1f0e7, road_yellow: 0xebae1f,
};

function colorFor(name) { return COLORS[name] || 0x8c918d; }

const CATEGORY_LABELS = {
  sandbox: "沙盘任务空间", road: "道路", road_marking: "道路标线", building: "工业建筑",
  tank: "储罐", tree: "绿化树木", industrial_yard: "工业场地", traffic_light: "交通灯",
  traffic_cone: "交通锥", uwb_anchor: "UWB 基站", sim_uav: "模拟无人机",
  gazebo_camera: "Gazebo 相机", uwb_tag: "机载 UWB 标签",
};

const UWB_ANCHORS = [
  ["UWB_ANCHOR_SW", "UWB 基站 · 西南角", 0.15, 0.15, 0.70],
  ["UWB_ANCHOR_SE", "UWB 基站 · 东南角", 3.85, 0.15, 0.70],
  ["UWB_ANCHOR_NE", "UWB 基站 · 东北角", 3.85, 4.55, 0.70],
  ["UWB_ANCHOR_NW", "UWB 基站 · 西北角", 0.15, 4.55, 0.70],
];

function semanticData({ id, category, position, label, note }) {
  return { id, category, position: [...position], label: label || CATEGORY_LABELS[category] || category, note };
}

function addObject(group, object) {
  if (!object.position || !object.size) return;
  const [sx, sy, sz] = object.size;
  const material = new THREE.MeshStandardMaterial({
    color: colorFor(object.color), roughness: object.category === "tank" ? 0.28 : 0.78,
    metalness: object.category === "tank" ? 0.42 : 0.04,
    transparent: object.color === "glass", opacity: object.color === "glass" ? 0.68 : 1,
  });
  const geometry = object.shape === "cylinder"
    ? new THREE.CylinderGeometry(Math.min(sx, sy) / 2, Math.min(sx, sy) / 2, sz, 24)
    : new THREE.BoxGeometry(sx, sy, sz);
  const mesh = new THREE.Mesh(geometry, material);
  mesh.position.set(...object.position);
  if (object.shape === "cylinder") mesh.rotation.x = Math.PI / 2;
  mesh.castShadow = !["road", "road_marking", "industrial_yard"].includes(object.category);
  mesh.receiveShadow = true;
  mesh.userData.semantic = semanticData({
    id: object.id, category: object.category, position: object.position,
    label: object.id === "sandbox_deck" ? "沙盘任务空间" : undefined,
    note: object.confidence ? `布局置信度 ${object.confidence}；坐标源自沙盘参数化模型` : "坐标源自沙盘参数化模型",
  });
  group.add(mesh);
}

function makeMarker(color, radius = 0.045) {
  const marker = new THREE.Mesh(new THREE.SphereGeometry(radius, 16, 12), new THREE.MeshStandardMaterial({ color, emissive: color, emissiveIntensity: 0.18 }));
  marker.castShadow = true; marker.visible = false;
  return marker;
}

function addUwbAnchor(group, id, label, x, y, z) {
  const anchor = new THREE.Group();
  const bodyMaterial = new THREE.MeshStandardMaterial({ color: 0x2d6fa3, metalness: 0.28, roughness: 0.42 });
  const post = new THREE.Mesh(new THREE.CylinderGeometry(0.025, 0.025, z, 12), bodyMaterial);
  post.rotation.x = Math.PI / 2;
  post.position.z = z / 2;
  const head = new THREE.Mesh(new THREE.BoxGeometry(0.16, 0.10, 0.09), bodyMaterial);
  head.position.z = z + 0.025;
  anchor.add(post, head); anchor.position.set(x, y, 0);
  const semantic = semanticData({ id, category: "uwb_anchor", position: [x, y, z], label, note: "四角部署示意；标称安装高度 0.70 m，需以现场标定值替换" });
  anchor.traverse((child) => { child.userData.semantic = semantic; });
  group.add(anchor);
}

function addSimUav(group) {
  const uav = new THREE.Group();
  const bodyMaterial = new THREE.MeshStandardMaterial({ color: 0xebae1f, metalness: 0.34, roughness: 0.38 });
  const cameraMaterial = new THREE.MeshStandardMaterial({ color: 0x202422, metalness: 0.55, roughness: 0.26 });
  const tagMaterial = new THREE.MeshStandardMaterial({ color: 0x79518a, emissive: 0x371a48, emissiveIntensity: 0.3 });
  const body = new THREE.Mesh(new THREE.BoxGeometry(0.34, 0.24, 0.09), bodyMaterial); uav.add(body);
  [[0.28, 0.22], [0.28, -0.22], [-0.28, 0.22], [-0.28, -0.22]].forEach(([x, y]) => {
    const arm = new THREE.Mesh(new THREE.BoxGeometry(0.30, 0.035, 0.026), bodyMaterial); arm.position.set(x / 2, y / 2, 0); arm.rotation.z = Math.atan2(y, x); uav.add(arm);
    const rotor = new THREE.Mesh(new THREE.CylinderGeometry(0.095, 0.095, 0.018, 18), cameraMaterial); rotor.rotation.x = Math.PI / 2; rotor.position.set(x, y, 0.03); uav.add(rotor);
  });
  const camera = new THREE.Mesh(new THREE.BoxGeometry(0.10, 0.12, 0.08), cameraMaterial); camera.position.set(0.17, 0, -0.08); uav.add(camera);
  const tag = new THREE.Mesh(new THREE.BoxGeometry(0.09, 0.09, 0.028), tagMaterial); tag.position.set(0, 0, 0.06); uav.add(tag);
  const uavSemantic = semanticData({ id: "SIM_UAV", category: "sim_uav", position: [2.0, 2.35, 1.20], label: "模拟无人机", note: "Gazebo 飞行平台；实时 /nexus/gazebo/uav/odom 可更新其位置" });
  const cameraSemantic = semanticData({ id: "SIM_UAV_CAMERA", category: "gazebo_camera", position: [2.17, 2.35, 1.12], label: "机载 Gazebo 相机", note: "IMX219 仿真相机；坐标为无人机静态示意位姿下的相机外形位置" });
  const tagSemantic = semanticData({ id: "SIM_UAV_UWB_TAG", category: "uwb_tag", position: [2.0, 2.35, 1.26], label: "机载 UWB 标签", note: "用于与四角 UWB 基站测距；标签外参需现场标定" });
  body.userData.semantic = uavSemantic;
  uav.children.filter((child) => child !== camera && child !== tag).forEach((child) => { child.userData.semantic = uavSemantic; });
  camera.userData.semantic = cameraSemantic; tag.userData.semantic = tagSemantic;
  uav.position.set(2.0, 2.35, 1.20); group.add(uav);
  return uav;
}

function formatPosition(position) {
  const [x, y, z] = position;
  return `x ${x.toFixed(2)} m · y ${y.toFixed(2)} m · z ${z.toFixed(2)} m`;
}

export function createSceneRenderer(canvas, wrapper, store) {
  const scene = new THREE.Scene();
  scene.background = new THREE.Color(0xf5f4f0);
  const camera = new THREE.PerspectiveCamera(42, 1, 0.01, 100);
  camera.up.set(0, 0, 1); camera.position.set(6.2, -6.8, 6.1);
  const renderer = new THREE.WebGLRenderer({ canvas, antialias: true, alpha: false });
  renderer.setPixelRatio(Math.min(window.devicePixelRatio || 1, 2));
  renderer.shadowMap.enabled = true; renderer.shadowMap.type = THREE.PCFSoftShadowMap;
  renderer.outputColorSpace = THREE.SRGBColorSpace;
  scene.add(new THREE.HemisphereLight(0xfaf8ef, 0x68716d, 2.0));
  const key = new THREE.DirectionalLight(0xfff5df, 3.0);
  key.position.set(-3, -4, 8); key.castShadow = true; key.shadow.mapSize.set(2048, 2048);
  key.shadow.camera.left = -5; key.shadow.camera.right = 5; key.shadow.camera.top = 6; key.shadow.camera.bottom = -5;
  scene.add(key);
  const axes = new THREE.AxesHelper(0.7); axes.rotation.x = Math.PI / 2; scene.add(axes);
  const controls = new OrbitControls(camera, canvas);
  controls.enableDamping = true; controls.dampingFactor = 0.08; controls.minDistance = 2.1; controls.maxDistance = 20;
  controls.minPolarAngle = 0.05; controls.maxPolarAngle = Math.PI * 0.95; controls.target.set(2, 2.35, 0);
  const modelGroup = new THREE.Group(); const equipmentGroup = new THREE.Group(); const overlayGroup = new THREE.Group(); scene.add(modelGroup, equipmentGroup, overlayGroup);
  const targetMarker = makeMarker(0x67507f, 0.065); const uwbMarker = makeMarker(0x2d6fa3); const visionMarker = makeMarker(0x2b817d); const platformMarker = makeMarker(0xebae1f, 0.08);
  overlayGroup.add(targetMarker, uwbMarker, visionMarker, platformMarker);
  UWB_ANCHORS.forEach((anchor) => addUwbAnchor(equipmentGroup, ...anchor));
  const simUav = addSimUav(equipmentGroup);
  const defaultUavPosition = new THREE.Vector3(2.0, 2.35, 1.20);
  let trajectoryLine = null;
  const raycaster = new THREE.Raycaster(); const pointer = new THREE.Vector2();
  const objectName = document.getElementById("map-object-name");
  const objectType = document.getElementById("map-object-type");
  const objectPosition = document.getElementById("map-object-position");
  const objectNote = document.getElementById("map-object-note");
  let hovered = null;

  const showObjectInfo = (semantic) => {
    if (!semantic) {
      objectName.textContent = "沙盘任务空间";
      objectType.textContent = "地图坐标系：原点在西南角";
      objectPosition.textContent = "x 0.00–4.00 · y 0.00–4.70 · z 向上";
      objectNote.textContent = "悬停查看物体语义与标称 map 坐标（厘米显示）";
      return;
    }
    objectName.textContent = semantic.label;
    objectType.textContent = `${semantic.id} · ${CATEGORY_LABELS[semantic.category] || semantic.category}`;
    objectPosition.textContent = formatPosition(semantic.position);
    objectNote.textContent = semantic.note;
  };
  const updateHover = (event) => {
    const bounds = canvas.getBoundingClientRect();
    pointer.x = ((event.clientX - bounds.left) / bounds.width) * 2 - 1;
    pointer.y = -((event.clientY - bounds.top) / bounds.height) * 2 + 1;
    raycaster.setFromCamera(pointer, camera);
    const hit = raycaster.intersectObjects([modelGroup, equipmentGroup], true).find((entry) => entry.object.userData.semantic);
    const semantic = hit?.object.userData.semantic || null;
    if (semantic?.id === hovered?.id) return;
    hovered = semantic; showObjectInfo(semantic);
    wrapper.classList.toggle("has-map-object-hover", Boolean(semantic));
  };
  canvas.addEventListener("pointermove", updateHover);
  canvas.addEventListener("pointerleave", () => { hovered = null; showObjectInfo(null); wrapper.classList.remove("has-map-object-hover"); });

  const loadScene = async () => {
    try {
      const response = await fetch("assets/sandbox/sandbox_scene.json");
      if (!response.ok) throw new Error(`scene ${response.status}`);
      const data = await response.json();
      (data.objects || []).forEach((object) => addObject(modelGroup, object));
      camera.lookAt(2, 2.35, 0);
    } catch (error) { console.warn("NEXUS scene unavailable", error); }
  };
  const updateOverlays = () => {
    const state = store.getState();
    const setMarker = (marker, point) => {
      const valid = point && [point.x, point.y, point.z].every(Number.isFinite);
      marker.visible = Boolean(valid); if (valid) marker.position.set(point.x, point.y, point.z);
    };
    setMarker(targetMarker, state.pose); setMarker(uwbMarker, state.observations.uwb); setMarker(visionMarker, state.observations.vision); setMarker(platformMarker, state.platform);
    const livePlatform = state.platform && [state.platform.x, state.platform.y, state.platform.z].every(Number.isFinite);
    simUav.position.copy(livePlatform ? state.platform : defaultUavPosition);
    simUav.traverse((child) => {
      const semantic = child.userData.semantic;
      if (!semantic || !semantic.id.startsWith("SIM_UAV")) return;
      if (semantic.id === "SIM_UAV") {
        semantic.position = [simUav.position.x, simUav.position.y, simUav.position.z];
      } else {
        const worldPosition = new THREE.Vector3(); child.getWorldPosition(worldPosition);
        semantic.position = [worldPosition.x, worldPosition.y, worldPosition.z];
      }
    });
    if (trajectoryLine) overlayGroup.remove(trajectoryLine);
    const points = state.trajectory.filter((p) => [p.x, p.y, p.z].every(Number.isFinite));
    if (points.length > 1) {
      const geometry = new THREE.BufferGeometry().setFromPoints(points.map((p) => new THREE.Vector3(p.x, p.y, p.z)));
      trajectoryLine = new THREE.Line(geometry, new THREE.LineBasicMaterial({ color: 0x67507f })); overlayGroup.add(trajectoryLine);
    } else trajectoryLine = null;
  };
  const resize = () => {
    const width = Math.max(1, wrapper.clientWidth); const height = Math.max(1, wrapper.clientHeight);
    renderer.setSize(width, height, false); camera.aspect = width / height; camera.updateProjectionMatrix();
  };
  const draw = () => { resize(); updateOverlays(); controls.update(); renderer.render(scene, camera); requestAnimationFrame(draw); };
  window.addEventListener("resize", resize); loadScene(); draw();
}
