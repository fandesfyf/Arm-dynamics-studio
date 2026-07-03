import {
  ArrowHelper,
  AxesHelper,
  BufferGeometry,
  DoubleSide,
  Float32BufferAttribute,
  Line,
  LineBasicMaterial,
  Mesh,
  MeshPhongMaterial,
  Object3D,
  SphereGeometry,
  Vector3,
  type Material,
} from 'three';
import type { URDFRobot } from 'urdf-loader';
import { URDFCollider } from 'urdf-loader';
import {
  collectAllReferenceTfFrames,
  collectFullVizChainFrames,
} from '../../ik/ik-chain-utils';

/** Traverse visual meshes (URDFVisual + async-loaded STL fallback, skip collision). */
export function forEachVisualMesh(robot: URDFRobot, fn: (mesh: Mesh) => void): void {
  robot.traverse((obj) => {
    if (!(obj as Mesh).isMesh) return;
    const mesh = obj as Mesh;
    let parent: Object3D | null = mesh;
    while (parent) {
      if ((parent as URDFCollider).isURDFCollider) return;
      parent = parent.parent;
    }
    fn(mesh);
  });
}

/** MuJoCo simulate collision group tint */
const COLLISION_COLOR = 0x33cc66;
const COLLISION_OPACITY = 0.3;

/** MuJoCo-style inertia marker */
const INERTIA_COLOR = 0xff9922;
const INERTIA_OPACITY = 0.5;

const JOINT_FRAME_SIZE = 0.1;

const visualPrepared = new WeakSet<URDFRobot>();
const collisionPrepared = new WeakSet<URDFRobot>();

function cloneMeshMaterials(mesh: Mesh): void {
  if (Array.isArray(mesh.material)) {
    mesh.material = mesh.material.map((m) => m.clone());
  } else {
    mesh.material = mesh.material.clone();
  }
}

function forEachMeshMaterial(mesh: Mesh, fn: (mat: Material) => void): void {
  const materials = Array.isArray(mesh.material) ? mesh.material : [mesh.material];
  for (const material of materials) {
    fn(material);
  }
}

/** Clone materials on visual meshes (including async STL loads). */
export function prepareVisualMeshMaterials(robot: URDFRobot): void {
  forEachVisualMesh(robot, (mesh) => {
    if (mesh.userData.__vizMatPrepared) return;
    cloneMeshMaterials(mesh);
    mesh.userData.__vizMatPrepared = true;
  });
}

/** Clone visual materials once so opacity changes do not fight shared refs. */
export function ensureVisualMaterialsPrepared(robot: URDFRobot): void {
  prepareVisualMeshMaterials(robot);
  visualPrepared.add(robot);
}

/** Replace collision meshes with MuJoCo-style green translucent fill. */
export function ensureCollisionMaterialsPrepared(robot: URDFRobot): void {
  if (collisionPrepared.has(robot)) return;
  robot.traverse((child) => {
    if (!(child as URDFCollider).isURDFCollider) return;
    child.visible = false;
    child.traverse((obj) => {
      if (!(obj as Mesh).isMesh) return;
      const mesh = obj as Mesh;
      mesh.material = new MeshPhongMaterial({
        color: COLLISION_COLOR,
        transparent: true,
        opacity: COLLISION_OPACITY,
        depthWrite: false,
        wireframe: false,
        side: DoubleSide,
        shininess: 8,
      });
      mesh.renderOrder = 2;
    });
  });
  collisionPrepared.add(robot);
}

/** Target preview ghost: green tint */
export const GHOST_GREEN = 0x44cc66;

function applyStyleToVisualMeshes(
  robot: URDFRobot,
  opacity: number,
  ghost: boolean,
): void {
  prepareVisualMeshMaterials(robot);
  const isTransparent = opacity < 0.999;
  forEachVisualMesh(robot, (mesh) => {
    forEachMeshMaterial(mesh, (material) => {
      if (ghost && 'color' in material && (material as MeshPhongMaterial).color) {
        (material as MeshPhongMaterial).color.setHex(GHOST_GREEN);
      }
      material.transparent = isTransparent;
      material.opacity = opacity;
      material.depthWrite = !isTransparent;
      material.needsUpdate = true;
    });
    mesh.renderOrder = ghost ? 1 : 0;
  });
}

export function applyGhostVisualStyle(robot: URDFRobot, opacity: number): void {
  applyStyleToVisualMeshes(robot, opacity, true);
}

export function applyVisualOpacity(robot: URDFRobot, opacity: number): void {
  applyStyleToVisualMeshes(robot, opacity, false);
}

export function applyCollisionVisibility(robot: URDFRobot, visible: boolean): void {
  ensureCollisionMaterialsPrepared(robot);
  robot.traverse((child) => {
    if (!(child as URDFCollider).isURDFCollider) return;
    child.visible = visible;
  });
}

/** Equivalent inertia ellipsoid radii (diagonal inertia, MuJoCo-style scale). */
function inertiaEllipsoidRadii(
  mass: number,
  ixx: number,
  iyy: number,
  izz: number,
): [number, number, number] {
  const m = Math.max(mass, 1e-6);
  const scale = 0.22;
  const minR = 0.008;
  return [
    Math.max(minR, Math.sqrt(Math.max(ixx / m, 0)) * scale),
    Math.max(minR, Math.sqrt(Math.max(iyy / m, 0)) * scale),
    Math.max(minR, Math.sqrt(Math.max(izz / m, 0)) * scale),
  ];
}

/** RGB frame axes at each actuated joint (MuJoCo joint frame display). */
export function attachJointAxisMarkers(robot: URDFRobot): Object3D[] {
  const markers: Object3D[] = [];
  for (const joint of Object.values(robot.joints)) {
    if (
      joint.jointType !== 'revolute' &&
      joint.jointType !== 'continuous' &&
      joint.jointType !== 'prismatic'
    ) {
      continue;
    }

    const frame = new Object3D();
    const axes = new AxesHelper(JOINT_FRAME_SIZE);
    axes.renderOrder = 3;
    frame.add(axes);

    if (joint.jointType === 'revolute' || joint.jointType === 'continuous') {
      const axis =
        joint.axis.lengthSq() > 0 ? joint.axis.clone().normalize() : new Vector3(0, 0, 1);
      const arrow = new ArrowHelper(
        axis,
        new Vector3(0, 0, 0),
        JOINT_FRAME_SIZE * 1.35,
        0xffcc00,
        JOINT_FRAME_SIZE * 0.22,
        JOINT_FRAME_SIZE * 0.12,
      );
      arrow.line.material = arrow.line.material as Material;
      (arrow.line.material as Material).depthTest = false;
      arrow.cone.material = arrow.cone.material as Material;
      (arrow.cone.material as Material).depthTest = false;
      arrow.renderOrder = 4;
      frame.add(arrow);
    }

    joint.add(frame);
    markers.push(frame);
  }
  return markers;
}

/** Hide every mesh under a URDF robot (skeleton-only TF overlay). */
export function hideAllRobotMeshes(robot: URDFRobot): void {
  robot.traverse((obj) => {
    if ((obj as import('three').Mesh).isMesh) {
      obj.visible = false;
    }
  });
}

export interface ReferenceTfFrameOptions {
  frameSize: number;
  showChainLines: boolean;
}

const CHAIN_LINE_COLOR = 0x33ddaa;
const CHAIN_LINE_COLOR_DIM = 0x228866;

function detectDualArmEndLinks(robot: URDFRobot): string[] {
  const names = Object.keys(robot.links ?? {});
  const result: string[] = [];
  const left = names.find((n) => /zarm_l\d+_end_effector$/i.test(n));
  const right = names.find((n) => /zarm_r\d+_end_effector$/i.test(n));
  if (left) result.push(left);
  if (right) result.push(right);
  if (result.length > 0) return result;

  const eeLinks = names.filter((n) => /end_effector$/i.test(n)).sort();
  if (eeLinks.length > 0) return eeLinks;

  const armTips = names.filter((n) => /^zarm_[lr]\d+_link$/i.test(n)).sort();
  const leftTip = armTips.find((n) => /zarm_l/i.test(n));
  const rightTip = armTips.find((n) => /zarm_r/i.test(n));
  if (leftTip) result.push(leftTip);
  if (rightTip) result.push(rightTip);
  return result;
}

function addChainLinesBetweenLinks(
  robot: URDFRobot,
  chainLinks: Object3D[],
  color: number,
  opacity: number,
  markers: Object3D[],
): void {
  if (chainLinks.length < 2) return;

  const worldA = new Vector3();
  const worldB = new Vector3();

  for (let i = 0; i < chainLinks.length - 1; i++) {
    chainLinks[i]!.getWorldPosition(worldA);
    chainLinks[i + 1]!.getWorldPosition(worldB);

    const p0 = robot.worldToLocal(worldA.clone());
    const p1 = robot.worldToLocal(worldB.clone());
    const geometry = new BufferGeometry();
    geometry.setAttribute(
      'position',
      new Float32BufferAttribute([p0.x, p0.y, p0.z, p1.x, p1.y, p1.z], 3),
    );
    const material = new LineBasicMaterial({
      color,
      depthTest: false,
      transparent: true,
      opacity,
    });
    const line = new Line(geometry, material);
    line.renderOrder = 4;
    robot.add(line);
    markers.push(line);
  }
}

export function attachReferenceTfFrames(
  robot: URDFRobot,
  endEffectorLink: string,
  options: ReferenceTfFrameOptions,
): Object3D[] {
  const markers: Object3D[] = [];
  const { frameSize, showChainLines } = options;

  robot.updateMatrixWorld(true);
  const { joints, links } = collectAllReferenceTfFrames(robot);

  for (const link of links) {
    const axes = new AxesHelper(frameSize * 0.9);
    axes.renderOrder = 5;
    link.add(axes);
    markers.push(axes);
  }

  for (const joint of joints) {
    const axes = new AxesHelper(frameSize);
    axes.renderOrder = 6;
    joint.add(axes);
    markers.push(axes);
  }

  if (showChainLines) {
    const armEnds = detectDualArmEndLinks(robot);
    const targets = armEnds.length > 0 ? armEnds : [endEffectorLink];

    for (const eeLink of targets) {
      const frames = collectFullVizChainFrames(robot, eeLink);
      const chainLinks = frames.links;
      if (chainLinks.length < 2) continue;
      const selected = eeLink === endEffectorLink;
      addChainLinesBetweenLinks(
        robot,
        chainLinks,
        selected ? CHAIN_LINE_COLOR : CHAIN_LINE_COLOR_DIM,
        selected ? 0.95 : 0.45,
        markers,
      );
    }
  }

  return markers;
}

export function attachInertiaMarkers(robot: URDFRobot): Object3D[] {
  const markers: Object3D[] = [];
  for (const link of Object.values(robot.links)) {
    const inertial = link.inertial;
    if (!inertial || inertial.mass <= 0) continue;

    const [x, y, z] = inertial.origin.xyz;
    const [r, p, yaw] = inertial.origin.rpy;
    const { ixx, iyy, izz } = inertial.inertia;

    const marker = new Object3D();
    marker.position.set(x, y, z);
    marker.rotation.set(r, p, yaw, 'ZYX');

    const [rx, ry, rz] = inertiaEllipsoidRadii(inertial.mass, ixx, iyy, izz);
    const geometry = new SphereGeometry(1, 14, 10);
    const material = new MeshPhongMaterial({
      color: INERTIA_COLOR,
      transparent: true,
      opacity: INERTIA_OPACITY,
      depthWrite: false,
      wireframe: false,
      side: DoubleSide,
      shininess: 12,
    });
    const ellipsoid = new Mesh(geometry, material);
    ellipsoid.scale.set(rx, ry, rz);
    ellipsoid.renderOrder = 3;

    marker.add(ellipsoid);
    link.add(marker);
    markers.push(marker);
  }
  return markers;
}

export function disposeOverlayMarkers(markers: Object3D[]): void {
  for (const marker of markers) {
    marker.parent?.remove(marker);
    marker.traverse((obj) => {
      if (obj instanceof AxesHelper) {
        obj.dispose();
        return;
      }
      if (obj instanceof ArrowHelper) {
        obj.dispose();
        return;
      }
      if (obj instanceof Line) {
        obj.geometry?.dispose();
        const mat = obj.material;
        if (Array.isArray(mat)) {
          for (const m of mat) m.dispose();
        } else {
          mat.dispose();
        }
        return;
      }
      if (!(obj as Mesh).isMesh) return;
      const mesh = obj as Mesh;
      mesh.geometry?.dispose();
      const materials = Array.isArray(mesh.material) ? mesh.material : [mesh.material];
      for (const material of materials) {
        material.dispose();
      }
    });
  }
}
