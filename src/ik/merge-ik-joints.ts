/** Preserve off-chain joint values from IK seed after single-arm solve. */
export function mergeIkJointsWithChain(
  jointNames: string[],
  chainJointNames: string[],
  seedJoints: number[],
  solvedJoints: number[],
): number[] {
  const chainSet = new Set(chainJointNames);
  return jointNames.map((name, i) =>
    chainSet.has(name) ? (solvedJoints[i] ?? seedJoints[i] ?? 0) : (seedJoints[i] ?? 0),
  );
}
