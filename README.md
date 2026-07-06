# Arm Dynamics Studio

浏览器端 URDF 单臂动力学仿真工具。基于 MuJoCo WASM 物理步进与 pinocchio-js 运动学/IK，纯前端静态部署，无需后端服务。

## 目录

- [功能概览](#功能概览)
- [技术栈](#技术栈)
- [环境要求](#环境要求)
- [快速开始](#快速开始)
- [构建与测试](#构建与测试)
- [项目结构](#项目结构)
- [许可证](#许可证)

## 功能概览

| 能力 | 说明 |
|------|------|
| URDF / ZIP 导入 | 上传 `.urdf` 或含 mesh 的 `.zip`，写入 MuJoCo VFS 后加载 |
| 内置 test_arm | 一键加载 kuavo `biped_v3_arm`（`public/robots/test_arm.zip`，含 STL mesh） |
| 3D 可视化 | Three.js + urdf-loader 实时显示关节状态 |
| 动力学仿真 | `mj_step` 积分，`mj_inverse` 前馈 + PD 反馈 |
| 关节角目标 | 平滑插值到达目标角，自动停止 |
| 末端 XYZ 目标 | pinocchio DLS IK → 关节驱动 |
| 轨迹关键点 | ≥2 个关键点 CubicSpline + SLERP 插值播放 |
| 质量 / 惯量编辑 | 修改 link 惯性参数后重建模型 |
| 实时曲线 | uPlot：指令（虚线）vs 实际（实线），q / v / τ |
| CSV 导出 | 时序数据列格式对齐旧版 Python 桌面版 |

## 技术栈

| 组件 | 用途 |
|------|------|
| React + Vite | 应用框架与构建 |
| Three.js / R3F | 三维渲染与交互 |
| @mujoco/mujoco | WASM 物理仿真 |
| pinocchio-js | 运动学、关节映射、DLS IK |
| closed-chain-ik | 末端位姿 IK 求解 |
| uPlot | 实时仿真曲线 |
| Zustand | 全局状态管理 |

## 环境要求

- Node.js 18+（推荐 20+）
- 支持 WebGL 的现代浏览器
- 首次加载需下载 MuJoCo WASM（约 10 MB）

## 快速开始

```bash
npm install
npm run dev
```

浏览器访问终端提示的本地地址（默认 `http://localhost:5173`）。

## 构建与测试

```bash
npm run build       # 输出 dist/，可部署至静态托管
npm run preview     # 本地预览生产构建
npm test            # Vitest 单元测试
npm run test:watch  # 监听模式
```

## Cloudflare Workers 部署

与 [robot_motion_editor](https://github.com/fandesfyf/robot_motion_editor) 相同：纯静态 SPA，通过 Workers **Static Assets** 托管（见根目录 `wrangler.jsonc`）。

在 **Workers & Pages → 你的 Worker → Settings → Builds** 中配置：

| 项 | 值 |
|----|-----|
| 根目录 | `/`（仓库根即项目根，含 `package.json`） |
| **Build command** | `npm run build` |
| **Deploy command** | `npx wrangler deploy`（默认） |

注意：**Dashboard 中的 Worker 名称**须与 `wrangler.jsonc` 里 `"name"` 一致（当前为 `arm-dynamics-studio`），否则部署会失败。SPA 路由由 `assets.not_found_handling` 处理，勿使用 `public/_redirects`（Workers 上会触发无限循环）。

可选环境变量：`NODE_VERSION=20`、`PLAYWRIGHT_SKIP_BROWSER_DOWNLOAD=1`。

构建日志中 `fs` / `path` externalized 警告可忽略。`public/_headers` 已为 `.wasm` 设置 MIME 类型。

本机手动发布：`npm run build` 后执行 `npx wrangler deploy`（首次需 `npx wrangler login`）。

> 线上默认加载 `test_arm`（`public/robots/test_arm.zip`，源自 kuavo `biped_s56` 的 `biped_v3_arm.urdf` + STL mesh）；开发环境默认 `biped_s70_upper_body`（依赖本机 `/biped-assets`）。重新打包：`npm run package:test-arm-zip`（源路径见脚本 `BIPED_S56_ROOT`）。

## 项目结构

```
.
├── docs/                    # 设计与评审文档
├── public/
│   └── robots/              # 内置 URDF 模型
├── scripts/                 # 构建辅助脚本
├── src/
│   ├── components/
│   │   ├── Viewer/          # 三维视口、URDF 模型、运动目标标记
│   │   ├── charts/          # uPlot 实时曲线与导出
│   │   ├── layout/          # 菜单栏、侧边栏、可停靠面板
│   │   ├── panels/          # 模型、仿真、控制、IK、轨迹等功能面板
│   │   └── ui/              # 通用 UI 组件与浮动面板
│   ├── contexts/            # React 上下文（如末端 IK）
│   ├── core/                # 仿真核心：控制器、轨迹、规划、载荷编辑
│   ├── export/              # CSV 时序数据导出
│   ├── fixtures/            # 测试用 URDF 样例
│   ├── hooks/               # useSimulation 等仿真生命周期钩子
│   ├── ik/                  # closed-chain-ik 桥接与末端控制
│   ├── mujoco/              # MuJoCo 加载、步进、外力
│   ├── pinocchio/           # pinocchio-js 封装与 IK
│   ├── stores/              # Zustand 会话 / UI / 可视化状态
│   ├── types/               # 共享类型定义
│   ├── utils/               # URDF 解析、资源加载、ZIP 解压
│   └── viewer/              # 末端运动学、Gizmo 同步
├── index.html
├── package.json
├── vite.config.ts
└── vitest.config.ts
```

## 许可证

MIT
