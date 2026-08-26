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
  mesh.userData.objectId = object.id;
  group.add(mesh);
}

function makeMarker(color, radius = 0.045) {
  const marker = new THREE.Mesh(new THREE.SphereGeometry(radius, 16, 12), new THREE.MeshStandardMaterial({ color, emissive: color, emissiveIntensity: 0.18 }));
  marker.castShadow = true; marker.visible = false;
  return marker;
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
  const modelGroup = new THREE.Group(); const overlayGroup = new THREE.Group(); scene.add(modelGroup, overlayGroup);
  const targetMarker = makeMarker(0x67507f, 0.065); const uwbMarker = makeMarker(0x2d6fa3); const visionMarker = makeMarker(0x2b817d);
  overlayGroup.add(targetMarker, uwbMarker, visionMarker);
  let trajectoryLine = null;

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
    setMarker(targetMarker, state.pose); setMarker(uwbMarker, state.observations.uwb); setMarker(visionMarker, state.observations.vision);
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
