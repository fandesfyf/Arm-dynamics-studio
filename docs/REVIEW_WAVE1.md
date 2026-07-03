# Wave 1 代码评审

> 评审时间：2026-07-01  
> 命令：`cd web && npm test`（26/26）· `npm run build`（成功）

## 总评

**通过，可进入 Wave 2。** 架构符合计划：控制用 MuJoCo `mj_inverse`，pinocchio 仅 IK/加载。核心算法与仿真循环质量可接受。

---

## 分项评审

### ✅ T1 MuJoCo/Pinocchio 加载

| 检查项 | 结果 |
|--------|------|
| VFS + `mjDSBL_CONTACT` | ✅ `mujoco/loader.ts` |
| JointMap 按名对齐 | ✅ `pinocchio/joint-map.ts` |
| `model.delete()` / `data.delete()` | ✅ `robot-session.dispose()` |
| vite WASM 配置 | ✅ manualChunks 分 mujoco/pinocchio |

**备注**：build 时 `fs`/`path`/`module` externalize 警告，已知可接受。

### ✅ T2 控制 + 仿真

| 检查项 | 结果 |
|--------|------|
| 前馈 `mj_inverse` + `qfrc_applied` 清零 | ✅ `controller.ts` |
| PD + `angleDiff` + 力矩限幅 | ✅ |
| `mj_fullM` 对角增益，回退策略 | ✅ 务实 |
| `CONTROL_DT=0.002` | ✅ |
| `runToTarget` / `runToEeTarget` / `runTrajectory` | ✅ `simulation.ts` |

**待修（Wave 2）**：
- `robot-session.ts` 中 `createForwardKinematics` 仍为 **stub**（恒返回原点）
- `IKSolver` 未在 session 层封装，需 T6 接入 `InverseKinematics`

### ✅ T3 轨迹 / 录制 / CSV

| 检查项 | 结果 |
|--------|------|
| CSV 列顺序 §5.8 | ✅ 测试覆盖 |
| DataRecorder 字段 §5.6 | ✅ |
| CubicSpline + SLERP | ✅ 自实现，无多余依赖 |

### ✅ T4 IK

| 检查项 | 结果 |
|--------|------|
| DLS + 多初值 | ✅ |
| 输出 MuJoCo qpos（JointMap） | ✅ |
| 未用 `pin.rnea` 控制 | ✅ |

### ❌ T5 UI

| 检查项 | 结果 |
|--------|------|
| `App.tsx` / `components/` / `stores/` | ❌ **未交付** |
| 入口仍为 P0 验收页 | ❌ `main.tsx` |

---

## 必须修复项（Wave 2 优先级）

1. **P0**：补全 T5 UI + 将 `main.tsx` 切到 `App`
2. **P0**：实现真实 FK（pinocchio `getJointPlacement`），替换 stub
3. **P0**：T6 集成 — `useSimulation` + 面板驱动 `runToTarget` / `runToEeTarget` / `runTrajectory`
4. **P1**：uPlot 实时曲线（T7）
5. **P1**：ZIP 上传 + mass-editor（T8）
6. **P2**：`tsconfig` 去掉对 core 的 exclude（若仍存在）；Pinocchio 真机 WASM 加载冒烟

## 无问题项

- 双引擎控制分工正确
- 测试覆盖核心算法
- 类型定义清晰（`types/robot.ts`, `types/simulation.ts`, `types/mujoco.ts`）
