# Web 版实施计划：MuJoCo WASM + pinocchio-js

> 文档版本：1.0  
> 创建日期：2026-07-01  
> 目标：在浏览器中实现单臂 URDF 动力学仿真工具，功能对齐已归档的 Python 桌面版行为规格。

---

## 1. 项目目标

### 1.1 核心能力（与旧版功能对齐）

| 能力 | 说明 |
|------|------|
| URDF 导入 | 支持任意单臂 URDF；可指定基座 link、末端 link |
| 实时 3D 可视化 | Three.js 渲染，仿真中以 MuJoCo 状态为准 |
| 动力学仿真 | `mj_step` 时间积分，真实 q / v / τ |
| 逆运动学 | 末端 XYZ 目标 → 关节角（姿态可选保持当前） |
| 运动驱动 | 关节角目标模式；末端位置目标模式；末端轨迹关键点播放 |
| 关节/惯量编辑 | 修改质量、惯量、质心、关节限位与力矩限幅，重建模型 |
| 计算力矩控制 | `mj_inverse` 前馈 + 自适应 PD 反馈 |
| 实时曲线 | 指令（虚线）vs 实际（实线）：q / v / τ |
| 录制与导出 | CSV 时序数据，含指令与实际 q/v/τ |

### 1.2 非目标（首版不做）

- 多臂 / 移动基座 / 全身机器人
- 碰撞交互与抓取（接触全局关闭，与旧版一致）
- Drake / ROS / rosbag
- 服务端仿真（首版纯客户端静态部署）
- Pyodide 跑 Python

### 1.3 交付形态

- `npm run build` → `dist/` 静态资源
- 可部署至 GitHub Pages / 任意静态托管，无 Node 运行时
- 用户通过 ZIP（URDF + mesh）或文件夹上传，数据不出本机

---

## 2. 技术栈

| 层级 | 选型 | 版本参考 | 职责 |
|------|------|----------|------|
| 构建 | Vite + TypeScript | Vite 5.x | 打包、dev server、WASM 资源 |
| UI | React 18 | — | 面板、状态、交互 |
| 3D | Three.js + React Three Fiber + drei | three 0.16x | URDF 可视化 |
| 物理仿真 | `@mujoco/mujoco` | 3.10.x | `mj_step`、`mj_inverse`、`mj_fullM` |
| 运动学/IK | `pinocchio-js` | 1.2.x | FK、Jacobian、DLS IK、可选 RNEA 校验 |
| URDF 显示 | `urdf-loader` | 0.10.x | mesh 加载与关节树 |
| 图表 | uPlot 或 ECharts | — | 实时 q/v/τ 曲线 |
| 状态 | Zustand | — | 仿真状态、UI 状态分离 |
| 轨迹插值 | `cubic-spline` 或自写 | — | 位置 CubicSpline + 姿态 SLERP |

### 2.1 依赖体积预估

| 包 | 解压体积 | 加载策略 |
|----|----------|----------|
| `@mujoco/mujoco` | ~19 MB | `manualChunks` 懒加载 + 进度条 |
| `pinocchio-js` | ~2 MB | 与 MuJoCo 并行初始化 |
| Three.js 生态 | ~2 MB | 与 UI 同 chunk |

首屏建议显示「正在加载仿真引擎…」，避免白屏。

---

## 3. 架构设计

### 3.1 总体数据流

```
用户上传 URDF+mesh (ZIP)
        │
        ├─► MuJoCo FS (VFS) ──► MjModel / MjData     ← 仿真真值
        ├─► pinocchio-js URDF parser ──► Model / Data  ← IK / Jacobian
        └─► urdf-loader ──► Three.js 场景              ← 显示

仿真循环 (Worker 或主线程定时器):
  Planner → q_d, v_d, a_d
  Controller(mj_inverse + PD) → τ
  applyTorque → mj_step
  readState(qpos/qvel live view) → Recorder + Chart + Three.js sync

导出: Recorder → CSV Blob 下载
```

### 3.2 双引擎分工（关键原则）

| 职责 | 引擎 | 原因 |
|------|------|------|
| 物理步进、状态真值 | **MuJoCo** | 与旧版 `simulation.py` 一致 |
| 逆动力学前馈 | **MuJoCo `mj_inverse`** | 旧版 `controller.py` 已改用 MuJoCo，保证 τ 与仿真模型一致 |
| 质量矩阵对角（增益初始化） | **MuJoCo `mj_fullM`** | 与旧版 `_mass_matrix_diagonal` 一致 |
| IK、Jacobian、末端 FK | **pinocchio-js** | 替代 Drake IK；无 Drake WASM |
| 可选交叉验证 | pinocchio `rnea` vs `mj_inverse` | 仅开发调试，不进入控制回路 |

**禁止**：控制回路中混用 `pin.rnea` 出力矩再 `mj_step`，会导致模型不一致。

### 3.3 关节映射表（JointMap）

MuJoCo 与 Pinocchio 从同一 URDF 构建时，`nq`/`nv` 可能因 fixed 关节处理不同而不一致。加载时构建：

```typescript
interface JointMapping {
  name: string;           // URDF 关节名
  mj_qposadr: number;     // MuJoCo qpos 下标
  mj_dofadr: number;      // MuJoCo qvel 下标
  pin_vidx: number;       // Pinocchio 速度空间下标（-1 表示无对应）
}
```

所有跨引擎数据交换必须经过 `JointMap`，按 **name** 对齐，禁止按下标硬编码。

### 3.4 线程模型

| 方案 | 适用 | 说明 |
|------|------|------|
| **A. 主线程定时器** | P0–P2 MVP | 实现简单；UI 卡顿会影响仿真 |
| **B. Web Worker 物理** | P3 推荐 | 主线程渲染，Worker 跑 `mj_step` + 控制 |
| **C. MuJoCo MT + COOP/COEP** | 可选优化 | 多线程 WASM，部署需 HTTP 头 |

首版用方案 A 验证；功能稳定后迁移到 B。

---

## 4. 目录结构（目标）

```
arm_dynamics_sim/
├── README.md
├── WEB_IMPLEMENTATION_PLAN.md      # 本文件
├── _archive/legacy-python/         # 已归档，日常不参考
│
└── web/                            # Web 应用根目录（待创建）
    ├── package.json
    ├── vite.config.ts
    ├── tsconfig.json
    ├── index.html
    ├── public/
    │   └── robots/
    │       └── test_arm.urdf       # 从归档复制，内置测试模型
    └── src/
        ├── main.tsx
        ├── App.tsx
        ├── types/
        │   ├── robot.ts
        │   └── simulation.ts
        ├── mujoco/
        │   ├── loader.ts           # VFS 挂载、MjModel 加载
        │   ├── step.ts             # mj_step 封装
        │   └── state.ts            # qpos/qvel 读取
        ├── pinocchio/
        │   ├── loader.ts           # buildPinocchioModel
        │   ├── ik.ts               # DLS IK
        │   └── joint-map.ts        # JointMap 构建与转换
        ├── core/
        │   ├── robot-session.ts    # 会话：双模型 + Three.js
        │   ├── controller.ts       # ComputedTorqueController
        │   ├── simulation.ts       # 仿真循环
        │   ├── trajectory.ts       # CubicSpline + SLERP
        │   ├── planner.ts          # 目标/轨迹规划
        │   ├── mass-editor.ts      # URDF DOM 编辑
        │   ├── data-recorder.ts
        │   └── inverse-kinematics.ts
        ├── export/
        │   └── csv-exporter.ts
        ├── components/
        │   ├── Viewer/             # R3F 场景
        │   ├── panels/             # 关节、IK、轨迹、导出等
        │   └── charts/             # 实时曲线
        ├── stores/
        │   └── session-store.ts
        └── workers/
            └── physics.worker.ts   # P3 可选
```

---

## 5. 模块规格（对照旧版行为）

### 5.1 `mujoco/loader.ts` — 对应 `robot_model.py`（MuJoCo 部分）

**输入**：URDF 文本 + `Map<相对路径, Uint8Array>` mesh 文件

**流程**：
1. `await loadMujoco()`
2. `FS.mkdir('/robot')`，写入 URDF 与 mesh
3. `MjModel.from_xml_path('/robot/xxx.urdf')`
4. `opt.disableflags |= mjDSBL_CONTACT`（关闭接触，与旧版一致）
5. `new MjData(model)`

**输出**：`{ model, data, jointNames, nq, nv, nu }`

**基座指定**：解析 URDF，将用户选择的 base link 与世界之间的 fixed 关节作为根；若无 world link，自动插入。

**注意**：
- `package://` 路径需规范化为 VFS 相对路径
- URDF 无 `<actuator>` 时 `nu=0`，力矩走 `qfrc_applied`

### 5.2 `pinocchio/loader.ts` — 对应 `robot_model.py`（Drake 部分 → 替换）

使用 `pinocchio-js/src/urdf-parser.mjs`：
- `parseURDF(xml)` → `buildPinocchioModel(pin, urdfData)`

提取：`jointNames`（非 fixed）、`lowerLimits`、`upperLimits`、`neutralConfiguration`。

### 5.3 `core/controller.ts` — 对应 `controller.py`

**控制律**：

```
τ = mj_inverse(q, v, a_d)     // 前馈
  + Kp * angleDiff(q_d, q)    // PD 位置
  + Kd * (v_d - v)            // PD 速度
τ = clip(τ, ±effort_limit)
```

**增益初始化**（与旧版一致）：
- `ω = 30 rad/s`
- `diag_M = diag(mj_fullM(q=0))`
- `Kp[i] = max(ω² * diag_M[i], 0.5)`
- `Kd[i] = 2 * sqrt(Kp[i] * diag_M[i])`

**`mj_inverse` 注意**：调用前 `qfrc_applied` 必须清零（旧版注释中的正反馈陷阱）。

### 5.4 `core/inverse-kinematics.ts` — 对应 `inverse_kinematics.py`

| 旧版（Drake） | Web 版（pinocchio-js） |
|---------------|------------------------|
| `AddPositionConstraint` + SNOPT | DLS 迭代 Jacobian |
| 多初值种子 | 保留多 seed 策略 |
| 可配置末端 link | `endEffectorJointId` 可配置 |
| 仅位置，姿态保持当前 | 首版同：仅 XYZ |

**增强（P2+）**：关节限位 null-space 投影；可选接入 Kinex WASM SQP。

### 5.5 `core/trajectory.ts` — 对应 `trajectory_interpolator.py`

- 末端位置：`CubicSpline` 过关键点 (t, x/y/z)
- 末端姿态：`SLERP` 四元数插值（轨迹模式用；目标模式保持当前姿态）
- 输出：每个采样时刻的 `(ee_pos, ee_quat)` → IK → `q_d(t)`

### 5.6 `core/simulation.ts` — 对应 `simulation.py`

三个公开 API（行为对齐旧版）：

| 方法 | 行为 |
|------|------|
| `runToTarget(q_target)` | 关节角目标，`q_d` 恒定，`v_d=0`，到达阈值停止 |
| `runToEeTarget(pos)` | IK 求 `q_target` 后调用 `runToTarget` |
| `runTrajectory(interpolator, duration)` | 按时间采样轨迹，逐帧控制 |

**参数**：
- `control_dt = 0.002`（500 Hz，与旧版默认一致）
- `physics_dt = model.opt.timestep`

**每步记录字段**（供 DataRecorder）：

```typescript
{
  time, qpos, qvel, tau,           // 实际
  q_desired, qvel_desired,         // 指令
  tau_commanded,                   // 控制器输出
  ee_pos, ee_quat                  // 末端（Pinocchio FK）
}
```

### 5.7 `core/mass-editor.ts` — 对应 `mass_editor.py`

- 解析 URDF 为 DOM
- 修改 `<inertial>`（mass、origin、inertia ixx…izz）
- 修改 `<limit>`（lower/upper/effort/velocity）
- 序列化 XML → 触发 `robot-session.reload(xml)`

### 5.8 `core/data-recorder.ts` + `export/csv-exporter.ts`

**CSV 列顺序**（与旧版兼容）：

```
time,
{joint}_pos..., {joint}_vel..., {joint}_torque...,
{joint}_pos_desired..., {joint}_vel_desired..., {joint}_torque_cmd...
```

浏览器导出：`new Blob([csv], {type:'text/csv'})` + `<a download>`。

---

## 6. UI 规格

### 6.1 布局

```
┌─────────────────────────────────────────────────────────┐
│  顶栏：模型名 | DOF | 仿真状态 | 导出 | 上传 URDF        │
├──────────────┬──────────────────────────────────────────┤
│  左侧面板     │           Three.js 3D 视图               │
│  ┌ 模型信息  │                                          │
│  ├ 关节目标  │                                          │
│  ├ 末端目标  │                                          │
│  ├ 轨迹关键点│                                          │
│  ├ 惯量编辑  │                                          │
│  └ 基座/末端 │                                          │
├──────────────┴──────────────────────────────────────────┤
│  底部：实时曲线 tabs [位置 | 速度 | 力矩]  指令虚线/实际实线 │
└─────────────────────────────────────────────────────────┘
```

### 6.2 交互流程（对齐旧版 README 操作流程）

1. 上传 URDF（ZIP 含 mesh，或仅几何 URDF）
2. 选择基座 link、末端 link（下拉，默认 root 与最后一个活动关节子 link）
3. 「运动到目标」：关节模式 / 末端 XYZ 模式
4. 或添加 ≥2 个轨迹关键点 → 「运行轨迹仿真」
5. 仿真中曲线实时更新
6. 「导出 CSV」

---

## 7. 实施阶段

### P0：引擎验证（3–5 天）

**目标**：证明 MuJoCo WASM 能加载 URDF 并步进。

| 任务 | 验收标准 |
|------|----------|
| 初始化 Vite + React + TS 工程于 `web/` | `npm run dev` 可访问 |
| 集成 `@mujoco/mujoco`，配置 WASM 资源 | 无加载错误 |
| 内置 `test_arm.urdf` 挂载 VFS 并 `from_xml_path` | `nq`/`nv` 与预期一致 |
| `mj_step` 1000 步 + `qfrc_applied` 恒定力矩 | 关节角变化合理 |
| `mj_inverse` 单点验证 | 与 Python 同 URDF 趋势一致（允许小数值差） |
| 集成 `pinocchio-js`，构建 JointMap | 活动关节名一一对应 |

**交付物**：`web/` 脚手架 + P0 验收页面（可删除或并入主 App）。

### P1：可视化 + 单关节控制（5–7 天）

| 任务 | 验收标准 |
|------|----------|
| urdf-loader Three.js 显示 | mesh/基本几何可见 |
| MuJoCo `xpos`/`xquat` 同步 Three.js | 仿真时模型跟随 |
| 实现 `ComputedTorqueController` | 单关节阶跃到目标 |
| `runToTarget` 单关节模式 | 到达后停止，无发散 |
| 关闭接触 `mjDSBL_CONTACT` | 与旧版行为一致 |

### P2：完整驱动 + IK + 轨迹（5–7 天）

| 任务 | 验收标准 |
|------|----------|
| pinocchio DLS IK + 多初值 | 末端 XYZ 可达工作空间内点 |
| `runToEeTarget` | 末端收敛 |
| `trajectory.ts` CubicSpline + SLERP | 2+ 关键点平滑播放 |
| `runTrajectory` | 整段轨迹跑完 |
| 基座/末端 link 可配置 | 下拉切换生效 |

### P3：录制、曲线、导出（2–3 天）

| 任务 | 验收标准 |
|------|----------|
| `DataRecorder` 时序缓存 | 每控制周期一条 |
| uPlot 实时曲线（q/v/τ，指令 vs 实际） | 仿真中 60fps 刷新 |
| CSV 导出格式与旧版兼容 | 列名、顺序一致 |
| 可选：Physics Worker | UI 卡顿不拖慢仿真 |

### P4：属性编辑 + 打磨（3–5 天）

| 任务 | 验收标准 |
|------|----------|
| 质量/惯量/关节限位编辑 UI | 修改后重建模型，仿真可用 |
| 力矩限幅从 URDF effort 读取 | 与旧版一致 |
| 加载进度、错误提示、空状态 | 用户体验完整 |
| `npm run build` 静态部署验证 | `dist/` 可离线打开（或 GH Pages） |

### P5：测试与文档（2 天）

| 任务 | 验收标准 |
|------|----------|
| 单元测试：controller、trajectory、joint-map、csv | Jest/Vitest 通过 |
| `test_arm.urdf` 回归场景清单 | 文档化预期行为 |
| 更新根 `README.md` 运行说明 | 含 build/deploy 步骤 |

**总工期估算**：约 3–4 周（单人，熟悉 React/Three.js）。

---

## 8. Vite 配置要点

```typescript
// web/vite.config.ts 关键项
export default defineConfig({
  plugins: [react()],
  assetsInclude: ['**/*.wasm'],
  optimizeDeps: {
    exclude: ['@mujoco/mujoco', 'pinocchio-js'],
  },
  build: {
    rollupOptions: {
      output: {
        manualChunks: {
          mujoco: ['@mujoco/mujoco'],
          pinocchio: ['pinocchio-js'],
          three: ['three', '@react-three/fiber', '@react-three/drei'],
        },
      },
    },
  },
  server: {
    headers: {
      // 仅在使用 @mujoco/mujoco/mt 时需要：
      // 'Cross-Origin-Opener-Policy': 'same-origin',
      // 'Cross-Origin-Embedder-Policy': 'require-corp',
    },
  },
});
```

**pinocchio-js CommonJS**：Vite 通常可直接预打包；若报错，加 `ssr.noExternal` 或 `commonjsOptions`。

---

## 9. 风险与缓解

| 风险 | 等级 | 缓解措施 |
|------|------|----------|
| MuJoCo/Pinocchio `nq` 不一致 | 高 | JointMap 按名映射；控制只用 MuJoCo |
| URDF mesh 路径失败 | 中 | ZIP 上传 + 路径规范化；文档说明目录结构 |
| URDF 无 actuator | 中 | 使用 `qfrc_applied` |
| WASM 首包过大 | 中 | 分 chunk + 加载 UI + 可选 CDN |
| MuJoCo WASM 内存泄漏 | 中 | 模型重载时 `model.delete()`/`data.delete()` |
| IK 精度不足 | 中 | 多初值；P2+ 评估 Kinex |
| 浏览器定时精度 | 低 | Worker 物理线程；`control_dt ≥ 2ms` |
| 官方 MuJoCo WASM 仍 WIP | 低 | 锁定版本 3.10.x；关注 changelog |

---

## 10. 参考资源（外部）

| 资源 | 用途 |
|------|------|
| [@mujoco/mujoco npm](https://www.npmjs.com/package/@mujoco/mujoco) | 官方 WASM 绑定 |
| [MuJoCo wasm README](https://github.com/google-deepmind/mujoco/blob/main/wasm/README.md) | API、内存、VFS |
| [pinocchio-js](https://github.com/Mostafasaad1/pinocchio-js) | 动力学/IK 算法 |
| [robot-analyzer-js](https://github.com/Mostafasaad1/robot-analyzer-js) | Pinocchio+Vite+React 集成参考（无仿真） |
| [mjswan](https://github.com/ttktjmt/mjswan) | MuJoCo 浏览器仿真循环参考 |
| [robot_viewer](http://viewer.robotsfan.com/) | URDF+MuJoCo Web 查看器 |

> 仓库内 `_archive/legacy-python/` 仅作历史快照，实施时以本计划第 5 节行为规格为准。

---

## 11. 验收清单（最终）

- [ ] 上传 ZIP（URDF+stl）可加载并显示
- [ ] 可选基座 link、末端 link
- [ ] 关节角目标驱动：平滑到达、自动停止
- [ ] 末端 XYZ 目标驱动：IK + 到达
- [ ] ≥2 轨迹关键点：插值播放
- [ ] 编辑 link 质量/惯量后仿真仍稳定
- [ ] 实时曲线：指令虚线 vs 实际实线（q/v/τ）
- [ ] CSV 导出列格式与旧版一致
- [ ] `npm run build` 静态部署可运行
- [ ] `test_arm.urdf` 回归：无 mesh 依赖，CI 可用

---

## 12. 下一步行动

1. 在 `web/` 创建 Vite 工程（`npm create vite@latest`）
2. 从 `_archive/legacy-python/test_arm.urdf` 复制到 `web/public/robots/`
3. 执行 **P0** 任务清单，完成 MuJoCo + pinocchio 双引擎加载验证
4. P0 通过后按 P1→P4 顺序推进，每阶段结束对照第 11 节勾选子项
