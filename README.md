# URDF 单臂动力学仿真工具（Web 版）

浏览器端单臂 URDF 动力学仿真：MuJoCo WASM 物理步进 + pinocchio-js 运动学/IK，纯静态部署。

## 快速开始

```bash
npm install
npm run dev
```

浏览器访问 Vite 提示的本地地址（默认 `http://localhost:5173`）。首次加载会下载 MuJoCo WASM（约 10 MB），请稍候。

## 构建与测试

```bash
npm run build    # 输出 dist/，可部署至任意静态托管
npm run preview  # 本地预览生产构建
npm test         # Vitest 单元测试
npm run test:watch  # 监听模式
```

## 功能

| 能力 | 说明 |
|------|------|
| URDF / ZIP 导入 | 上传 `.urdf` 或含 mesh 的 `.zip`，写入 MuJoCo VFS 后加载 |
| 内置 test_arm | 一键加载 `public/robots/test_arm.urdf`（无 mesh，适合 CI） |
| 3D 可视化 | Three.js + urdf-loader 实时显示关节状态 |
| 动力学仿真 | `mj_step` 积分，`mj_inverse` 前馈 + PD 反馈 |
| 关节角目标 | 平滑插值到达目标角，自动停止 |
| 末端 XYZ 目标 | pinocchio DLS IK → 关节驱动 |
| 轨迹关键点 | ≥2 个关键点 CubicSpline + SLERP 插值播放 |
| 质量 / 惯量编辑 | 修改 link 惯性参数后重建模型 |
| 实时曲线 | uPlot：指令（虚线）vs 实际（实线），q / v / τ |
| CSV 导出 | 时序数据列格式对齐旧版 Python 桌面版 |

## 状态

- **Wave 2 已完成**（2026-07-01）：UI 集成、实时曲线、ZIP 上传与质量编辑。详见 [REVIEW_WAVE2.md](./docs/REVIEW_WAVE2.md)。
- **旧版 Python 实现**：已归档至上级仓库 `_archive/legacy-python/`（日常开发不参考）。

## 文档

| 文件 | 说明 |
|------|------|
| [WEB_IMPLEMENTATION_PLAN.md](./docs/WEB_IMPLEMENTATION_PLAN.md) | Web 版完整实施计划（架构、阶段、模块、风险） |
| [TASK_DIVISION.md](./docs/TASK_DIVISION.md) | 任务分工与 Wave 进度 |
| [REVIEW_WAVE1.md](./docs/REVIEW_WAVE1.md) | Wave 1 代码评审 |
| [REVIEW_WAVE2.md](./docs/REVIEW_WAVE2.md) | Wave 2 集成评审与验收清单 |

## 许可证

MIT
