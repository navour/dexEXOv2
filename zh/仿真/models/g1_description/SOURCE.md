# 模型来源

本目录中的 `g1_dual_arm.urdf`、`g1_dual_arm.xml` 以及 `meshes/` 中被
URDF 引用的 STL 文件来自宇树官方仓库，文件内容未修改。

- 仓库：<https://github.com/unitreerobotics/unitree_ros>
- 上游路径：`robots/g1_description/`
- 上游修订：`278b222a3ca04f684c764c53ae82a70c87ff3044`
- 取得日期：2026-07-21
- 许可证：BSD 3-Clause，见同目录 `LICENSE`

`meshes/` 只保留 `g1_dual_arm.urdf` 实际引用的 22 个网格，未复制
官方包中与腿部、灵巧手或其他 G1 版本相关的网格。
