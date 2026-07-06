import { extractRobotFromZip, type RobotAssetExtract } from './robot-asset-loader';
import { prepareUrdfForMujocoLoad } from './urdf-sanitize';

const TEST_ARM_ZIP_URL = '/robots/test_arm.zip';
const TEST_ARM_URDF_URL = '/robots/test_arm.urdf';

let cached: RobotAssetExtract | null = null;
let inFlight: Promise<RobotAssetExtract> | null = null;

/** 从 public/robots 加载内置 test_arm（优先 zip 含 mesh，回退扁平 urdf） */
export async function loadBundledTestArm(): Promise<RobotAssetExtract> {
  if (cached) return cached;
  if (inFlight) return inFlight;

  inFlight = (async () => {
    const zipRes = await fetch(TEST_ARM_ZIP_URL);
    if (zipRes.ok) {
      const bundle = await extractRobotFromZip(await zipRes.arrayBuffer());
      cached = bundle;
      return bundle;
    }

    const urdfRes = await fetch(TEST_ARM_URDF_URL);
    if (!urdfRes.ok) {
      throw new Error(`无法加载 test_arm: HTTP ${urdfRes.status}`);
    }
    const bundle: RobotAssetExtract = {
      urdfText: prepareUrdfForMujocoLoad(await urdfRes.text()),
      urdfFileName: 'test_arm.urdf',
      meshes: new Map(),
    };
    cached = bundle;
    return bundle;
  })();

  try {
    return await inFlight;
  } finally {
    inFlight = null;
  }
}

export function prefetchBundledTestArm(): void {
  void loadBundledTestArm().catch(() => undefined);
}

/** 生产环境无本机 biped STL；开发环境可通过 Vite /biped-assets 提供 */
export function shouldUseBundledTestArmAsDefault(): boolean {
  return import.meta.env.PROD;
}
