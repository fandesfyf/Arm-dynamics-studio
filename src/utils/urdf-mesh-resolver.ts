import { findMeshBytes, resolveAssetPath } from './robot-asset-loader';

/** 为 urdf-loader / THREE 创建 mesh URL 解析器 */
export function createMeshUrlModifier(
  urdfFileName: string,
  meshes: Map<string, Uint8Array>,
): (url: string) => string {
  const blobUrls = new Map<string, string>();

  return (url: string) => {
    let ref = url;
    if (ref.startsWith('file://')) {
      ref = ref.slice(7);
    }
    if (ref.startsWith('./')) {
      ref = ref.slice(2);
    }
    if (ref.startsWith('/')) {
      ref = ref.slice(1);
    }

    const resolved = resolveAssetPath(urdfFileName, ref);
    const bytes = findMeshBytes(meshes, resolved);
    if (!bytes) {
      return url;
    }

    const cacheKey = resolved;
    if (!blobUrls.has(cacheKey)) {
      blobUrls.set(
        cacheKey,
        URL.createObjectURL(new Blob([bytes])),
      );
    }
    return blobUrls.get(cacheKey)!;
  };
}

export function revokeMeshBlobUrls(modifier: (url: string) => string): void {
  void modifier;
}
