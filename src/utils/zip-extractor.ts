export {
  extractRobotFromZip,
  extractRobotFromFiles,
  extractRobotFromFileList,
  pickUrdfPath,
  stripPackageRoot,
  resolveAssetPath,
  findMeshBytes,
  type RobotAssetExtract,
} from './robot-asset-loader';

/** @deprecated 使用 RobotAssetExtract */
export type ZipExtractResult = import('./robot-asset-loader').RobotAssetExtract;
