# 模型来源

全身 29 自由度的 G1 模型，供 `仿真/g1_mujoco_sim.py` 和
`PC端/dual_arm_viz.py` 共用。**两份文件都是上游原样拷贝，未修改。**

## `g1_29dof.urdf`

给 PC 端的 OpenGL 渲染器（它只认 URDF）。

- 仓库：<https://github.com/unitreerobotics/unitree_ros>
- 上游路径：`robots/g1_description/g1_29dof_rev_1_0.urdf`
- 上游修订：`278b222a3ca04f684c764c53ae82a70c87ff3044`
- 取得日期：2026-07-27
- 许可证：BSD 3-Clause，见同目录 `LICENSE`

与同目录 `g1_description/`（上半身模型）来自同一仓库、同一修订。

选 `_rev_1_0` 而不是 `g1_29dof.urdf`：两者对应不同硬件版本，只有 `_rev_1_0`
与下面那份 MJCF 完全一致（非 rev 版本有一处关节原点相差 1.9e-2 m）。

## `g1_29dof.xml`

给 MuJoCo。原样拷贝自本仓库内的 HumDex 上游检出：

- 上游路径：`HumDex_IMU融合实验/upstream/HumDex/assets/g1/g1_sim2sim_29dof.xml`
- 具体来源与修订见 `HumDex_IMU融合实验/UPSTREAM_VERSIONS.md`
- 拷贝日期：2026-07-27

选它而不是宇树官方的 `g1_29dof_rev_1_0.xml`，是因为
`HumDex_IMU融合实验/bridge/twist2_dynamic_sim.py` 已经在用这一份 —— 跑策略
的模型和画面里的模型必须是同一个。

在这里再放一份而不是跨目录引用 `upstream/`，是为了让 `仿真/` 和 `PC端/`
不依赖那个上游检出的存在与否。

## `g1_29dof_rev_1_0_with_inspire_hand_FTP.urdf`

带因时 FTP 灵巧手的版本，给手部遥操用。**上游原样拷贝，未修改。**

- 仓库：<https://github.com/unitreerobotics/unitree_ros>
- 上游路径：`robots/g1_description/g1_29dof_rev_1_0_with_inspire_hand_FTP.urdf`
- 上游修订：`278b222a3ca04f684c764c53ae82a70c87ff3044`（与上面两份同一修订）
- 取得日期：2026-07-28
- 许可证：BSD 3-Clause，见同目录 `LICENSE`

与 `g1_29dof.urdf` 是同一台机器人加上双手：97 link / 96 joint，53 个转动关节
= 车身 29 + 双手 24。**别选 `_DFQ` 那一版**，那是因时的另一款手。

每只手 12 个转动关节，但**独立自由度只有 6 个**，正好对上手套的 6 个通道和
因时角度寄存器的 6 个通道，不需要 retargeting。另外 6 个是 `<mimic>` 联动，
剩下的力传感器关节全是 fixed。通道 ↔ 关节的对应表、联动倍率和行程限制在
`PC端/hand_mapping.py`，那里有单测钉住。

两个坑：

- **`<mimic>` 没人自动帮你算。** `PC端/g1_urdf_renderer.py` 不解析 mimic，
  MuJoCo 也不认（转 MJCF 时要写成 `<equality joint>` 约束）。两边都得靠
  `hand_mapping.expand_mimic()` 把 6 个驱动关节展开成 12 个再喂进去。
- **拇指是两级链**：`thumb_2 → thumb_3`（×0.8024）`→ thumb_4`（×0.9487），
  是逐级相乘，不是都乘 `thumb_2`。四指都只有一级（×1.0843）。

## `meshes/`

124 个 STL。其中 64 个与 `g1_29dof.xml` 一同拷自 HumDex 上游，另外 60 个是
因时 FTP 版 URDF 需要的手部网格，拷自上面那个 unitree_ros 修订。文件名与
宇树官方 `meshes/` 一致，三份模型共用同一批网格。

拷手部网格时逐个比对过：**60 个手部网格在 `278b222a` 和 2026-07-28 的 master
(`ac771482`) 之间字节一致**，但车身网格里 `left_ankle_roll_link.STL` 和
`right_shoulder_roll_link.STL` 这两个上游改过。所以本目录统一钉在
`278b222a`，**不要从 master 补拉单个网格** —— 混修订不会报错，只会让画面和
仿真悄悄错位，正是下面那条一致性检查要防的事。

## 一致性

URDF 和 MJCF 出自**不同上游**，不能假定它们描述同一台机器人；不一致不会报
错，只会让上位机画的姿态和仿真里跑的姿态悄悄错位。`仿真/test_model_consistency.py`
在随机关节角下逐 link 对拍两者的正运动学（偏差须小于 1e-6 m），并核对全部
29 个关节的限位。换上游模型后跑这个测试。
