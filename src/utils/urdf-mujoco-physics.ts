/**
 * MuJoCo 动力学加载用 URDF：去掉仅用于显示的 mesh visual。
 * 碰撞 primitive / inertial 保留；mesh 由 Three.js 查看器单独加载。
 */
export function stripMeshVisualsForMujoco(urdfText: string): string {
  return urdfText.replace(/<visual>\s*[\s\S]*?<mesh\b[\s\S]*?<\/visual>/gi, '');
}

/** 是否仍引用 mesh 文件（collision 或遗漏的 visual） */
export function urdfReferencesMeshFiles(urdfText: string): boolean {
  return /filename="[^"]+\.(stl|dae|obj|ply)"/i.test(urdfText);
}
