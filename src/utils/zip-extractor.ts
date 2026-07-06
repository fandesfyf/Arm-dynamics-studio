export {
  extractRobotFromZip,
  extractRobotFromFiles,
  extractRobotFromFileList,
  listUrdfPathsFromZip,
  listUrdfPathsFromFiles,
  listUrdfCandidates,
  prepareFolderFiles,
  pickUrdfPath,
  stripPackageRoot,
  resolveAssetPath,
  findMeshBytes,
  type RobotAssetExtract,
  type PreparedFolderFiles,
} from './robot-asset-loader';

/** @deprecated 使用 RobotAssetExtract */
export type ZipExtractResult = import('./robot-asset-loader').RobotAssetExtract;
