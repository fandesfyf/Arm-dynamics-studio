# Wave 2 代码评审

> 评审时间：2026-07-01  
> 命令：`cd web && npm test`（30/30）· `npm run build`（成功）

## 总评

**Wave 2 集成完成，可交付使用。** T5 UI 壳、T6 仿真接线、T7 uPlot 曲线、T8 ZIP/质量编辑均已落地；`main.tsx` 已切换至 `App`。控制回路仍仅用 MuJoCo `mj_inverse`，未改动 core 算法逻辑。

---

## 集成后状态

| 模块 | 状态 | 说明 |
|------|------|------|
| T5 UI 布局 | ✅ | `App.tsx` 顶栏 + 左侧面板 + 3D Viewer + 底栏图表 |
| T6 useSimulation | ✅ | 加载 / 关节目标 / 末端 IK / 轨迹 / CSV 导出 / 取消 |
| T6 FK | ✅ | `robot-session.ts` 使用 pinocchio `getJointPlacement`，Wave 1 stub 已替换 |
| T7 uPlot | ✅ | `SimCharts.tsx`：q/v/τ 指令虚线 vs 实际实线 |
| T8 ZIP | ✅ | `zip-extractor.ts` + `RobotUpload` 面板 |
| T8 mass-editor | ✅ | `MassEditorPanel` + `reloadUrdf` 重建模型 |
| 测试 | ✅ | 7 文件 30 测试（含 mass-editor 4 项） |
| 构建 | ✅ | `dist/` 静态资源，mujoco/pinocchio/three 分 chunk |

---

## 已知限制

1. **基座 / 末端 link 选择**：`session-store` 默认 `base_link` / `ee_link`，尚无 UI 下拉选择；非标准命名 URDF 需改 store 默认值或后续补面板。
2. **首包体积**：Three.js chunk ~1 MB gzip，MuJoCo WASM ~10 MB；首屏需等待引擎加载。
3. **pinocchio-js 构建警告**：`fs`/`path`/`module` externalize 为已知可接受项（与 Wave 1 一致）。
4. **物理线程**：当前主线程定时器仿真（计划 P3 可选 Worker）。
5. **IK 精度**：复杂姿态 / 奇异位形可能不收敛，控制台会 warn。

---

## §11 验收清单（WEB_IMPLEMENTATION_PLAN.md）

| # | 项 | 结果 | 备注 |
|---|-----|------|------|
| 1 | 上传 ZIP（URDF+stl）可加载并显示 | ✅ | `.urdf` 单文件亦可；mesh 经 VFS |
| 2 | 可选基座 link、末端 link | ⚠️ | 代码支持 `endEffectorLink`，默认硬编码；**无 UI 选择器** |
| 3 | 关节角目标驱动：平滑到达、自动停止 | ✅ | `JointTargetPanel` + `runToTarget` |
| 4 | 末端 XYZ 目标驱动：IK + 到达 | ✅ | `EeTargetPanel` + DLS IK |
| 5 | ≥2 轨迹关键点：插值播放 | ✅ | UI 禁用 `<2` 点；CubicSpline + SLERP |
| 6 | 编辑 link 质量/惯量后仿真仍稳定 | ✅ | `MassEditorPanel` + 模型重建 |
| 7 | 实时曲线：指令虚线 vs 实际实线（q/v/τ） | ✅ | uPlot `SimCharts` |
| 8 | CSV 导出列格式与旧版一致 | ✅ | `csv-exporter.test.ts` 覆盖 §5.8 |
| 9 | `npm run build` 静态部署可运行 | ✅ | `dist/` 构建通过 |
| 10 | `test_arm.urdf` 回归：无 mesh 依赖，CI 可用 | ✅ | `public/robots/test_arm.urdf` + Vitest |

**勾选汇总**：9/10 完全通过，1 项部分通过（link 选择缺 UI）。

---

## Wave 2 分项检查（主 agent 检查项）

| 检查项 | 结果 |
|--------|------|
| 双引擎：控制仅用 `mj_inverse` | ✅ |
| JointMap 跨引擎按 name 映射 | ✅ |
| MuJoCo Embind `.delete()` | ✅ `dispose()` |
| CSV 列顺序 §5.8 | ✅ |
| 构建无 WASM 路径错误 | ✅ |
| 未修改非所有权 core 算法 | ✅ T9 仅文档与验收 |
