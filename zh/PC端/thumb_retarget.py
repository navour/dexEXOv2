#!/usr/bin/env python3
"""因时 FTP 拇指的**指腹位置重定向**, 纯几何, 不碰手套也不碰 Modbus。

## 为什么拇指要单独一条路

四指是一个自由度对一个自由度, 人手弯多少映射多少就够用了(实测人手弯曲幅度
132~157°, 因时 ∠α 跨度 156°, 接近 1:1)。拇指不是: 因时拇指只有 2 个自由度
(对掌 β + 弯曲 θ), 人的拇指有 4 个以上, 而且这两个动作在人手的**旋转特征
空间**里高度耦合 —— 实测两条标定轴夹角只有 31.5°(右手)/36.8°(左手), 相关
系数 0.853/0.801, 正好卡在退化门槛 0.90 的下沿。在这种近共线的基底上做最小
二乘分解是病态的: 标定的那两个纯姿势上精确, 但抓杯子这种弯曲和对掌同时发生
的动作, 分出来的比例对噪声极敏感, 表现就是"拇指位置不对"。

同样两个自由度换到**位置空间**看, 夹角是 60.2°, 相关系数 0.498 —— 条件数
好一个量级。所以改成在位置空间做分解:

    人手指腹(掌心系) --手套端-→ 归一化(u,v) --消费端-→ 拇指两个通道

**收益全在第一步**, 也就是手套端 ``thumb_normalized_uv()`` 那个最小二乘:
把"这个动作有几成弯曲、几成对掌"从病态的旋转特征基底换到良态的位置基底。
第二步只是把结果落到通道上, 是参数对参数的直通。

曾经在第二步里做过"在三维空间里插值出目标点再解最近可达点", **那是错的**:
角点位移 ``q_opp``/``q_flex`` 是曲面上的弦向量, 它们张成的平面一离开角点就
脱离曲面 —— 实测最远偏离 77mm, 而整个工作空间跨度才 90mm; 投影回曲面后映射
会错乱(u=0.75,v=0.25 解出对掌 0.975/弯曲 0.233)。三个标定点只够建人手曲面的
仿射图, 建不出曲率, 凭空插值不会产生本来就没有的信息。
``test_chord_basis_leaves_the_surface`` 把这个教训留成了回归。

要真吃掉非线性, 得让人手标定采一整片扫掠而不是三个点 —— 那是另一件事。

## 这里的 FK 和求解器还留着做什么

不在运行时路径上。它们是分析和校验工具: 证明工作空间确实是可解的二维曲面、
证明位置空间的条件数优于旋转空间、以及 ``export_table`` 给"把求解搬进 C++"
留的后路(``standalone_inspire_bridge.py`` 那条直连真手的老路要用时)。
运行时只用模块级的 ``uv_to_closures``, 纯算术。

## 指腹取哪个点

``*_thumb_force_sensor_4`` —— 因时的力传感器就装在指腹上, 所以它天然是"指腹"
这个语义点, 不用在末端自己猜一个偏移量。人手那一侧取节点 3(拇指末)相对节点
0(手背), 并转进手背局部系, 这样整只手怎么挥动都不影响这个量。

## 谁是权威

运动学常数全部从 URDF 现读 (``仿真/models/.../*_with_inspire_hand_FTP.urdf``),
不在这里抄一份。抄一份的话 URDF 换了修订版这边不会报错, 只会悄悄画错。
"""

import math
import os
import re
import xml.etree.ElementTree as ET

import numpy as np


_MY_DIR = os.path.dirname(os.path.abspath(__file__))
DEFAULT_URDF = os.path.join(
    os.path.dirname(_MY_DIR), "仿真", "models", "g1_29dof",
    "g1_29dof_rev_1_0_with_inspire_hand_FTP.urdf")

# 拇指链: base → thumb_1(对掌) → thumb_2(弯曲) → thumb_3 → thumb_4 → 指腹
THUMB_CHAIN = ("thumb_1_joint", "thumb_2_joint", "thumb_3_joint",
               "thumb_4_joint")
PAD_JOINT = "thumb_force_sensor_4_joint"

# 查表分辨率。41×41 时相邻点最小间距 0.256mm, 远小于人手重复精度, 再细没有
# 意义; 而且这张表要传给 C++ 遥操线程, 每帧扫一遍 1681 个点也才几十微秒。
GRID_N = 41


# 三个标定手势各自对应的因时拇指姿态 (对掌 u, 弯曲 v)。
#
# 用**语义手势**而不是"纯对掌/纯弯曲"这类抽象动作: 后者人做不干净, 实测右手
# 采出来的对掌轴方向和抓握时的实际移动方向近乎相反(抓握位在该轴上的分量
# -0.88, 左手同一动作却是 +0.11)。这三个手势各有明确形状或触觉终点 ——
# 拇指贴到掌心、拇指碰到食指 —— 人做得准也可重复。
#
# OK 的目标是从 URDF 正运动学扫出来的, 不是拍脑袋: 见
# test_thumb_retarget.GestureTargetTest。C++ 侧 kThumbPalmTarget /
# kThumbPinchTarget 必须与这里一致。
GESTURE_TARGETS = {
    "比赞": (0.0, 0.0),        # 拇指零位, 作原点
    "横贴掌心": (1.0, 0.0),     # 对掌满行程, 不弯曲
    "OK": (1.0, 0.5),          # 拇指指腹碰食指指腹
}


def uv_to_closures(u, v, overshoot=1.0):
    """归一化 (对掌 u, 弯曲 v) → (拇指弯曲闭合度, 拇指对掌闭合度)。

    这是**运行时唯一需要的东西**, 纯算术, 不碰 URDF 也不碰 numpy —— 所以
    手套线程里不用为它建网格。本模块其余部分是分析和校验工具: 证明工作空间是
    可解的二维曲面、证明位置空间的条件数确实优于旋转空间、以及给 C++ 备用的
    导表。

    ``overshoot`` 允许 u/v 略微超过 1 —— 人手能做的比因时大, 超出部分顶到
    边界即可。非有限输入返回 ``None``, 调用方据此退回线性投影。
    """
    try:
        u = float(u)
        v = float(v)
    except (TypeError, ValueError):
        return None
    if not (math.isfinite(u) and math.isfinite(v)):
        return None
    u = min(max(u, 0.0), overshoot)
    v = min(max(v, 0.0), overshoot)
    return (min(v, 1.0), min(u, 1.0))


def _rpy_matrix(roll, pitch, yaw):
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ])


def _transform(xyz, rot):
    matrix = np.eye(4)
    matrix[:3, :3] = rot
    matrix[:3, 3] = xyz
    return matrix


def _rotation_about(axis, angle):
    axis = np.asarray(axis, dtype=float)
    norm = np.linalg.norm(axis)
    if norm < 1e-12:
        return np.eye(3)
    axis = axis / norm
    x, y, z = axis
    c, s = math.cos(angle), math.sin(angle)
    return np.array([
        [c + x * x * (1 - c), x * y * (1 - c) - z * s, x * z * (1 - c) + y * s],
        [y * x * (1 - c) + z * s, c + y * y * (1 - c), y * z * (1 - c) - x * s],
        [z * x * (1 - c) - y * s, z * y * (1 - c) + x * s, c + z * z * (1 - c)],
    ])


class ThumbKinematics:
    """从 URDF 读出来的一只手的拇指链。"""

    def __init__(self, side="right", urdf_path=DEFAULT_URDF):
        if side not in ("right", "left"):
            raise ValueError("side 只能是 'right' 或 'left'")
        self.side = side
        self.urdf_path = urdf_path
        root = ET.parse(urdf_path).getroot()
        joints = {j.get("name"): j for j in root.iter("joint")}

        self._links = []
        for name in THUMB_CHAIN:
            joint = joints.get(f"{side}_{name}")
            if joint is None:
                raise KeyError(f"URDF 缺少关节 {side}_{name}")
            self._links.append(self._parse(joint))
        pad = joints.get(f"{side}_{PAD_JOINT}")
        if pad is None:
            raise KeyError(f"URDF 缺少指腹力传感器关节 {side}_{PAD_JOINT}")
        self._pad = self._parse(pad)

        # 驱动关节的上限: thumb_1 = 对掌 β, thumb_2 = 弯曲 θ。
        self.beta_max = self._links[0]["upper"]
        self.theta_max = self._links[1]["upper"]
        # thumb_3 / thumb_4 是 mimic, 倍率逐级相乘 —— 与 hand_mapping 的
        # expand_mimic() 必须一致, 测试里钉住了这一条。
        self.mimic = (1.0, self._links[2]["multiplier"],
                      self._links[2]["multiplier"] * self._links[3]["multiplier"])

    @staticmethod
    def _parse(joint):
        origin = joint.find("origin")
        xyz = [float(v) for v in (origin.get("xyz", "0 0 0").split())]
        rpy = [float(v) for v in (origin.get("rpy", "0 0 0").split())]
        axis_node = joint.find("axis")
        axis = ([float(v) for v in axis_node.get("xyz").split()]
                if axis_node is not None else [0.0, 0.0, 1.0])
        limit = joint.find("limit")
        upper = float(limit.get("upper")) if limit is not None else 0.0
        mimic = joint.find("mimic")
        multiplier = (float(mimic.get("multiplier", "1"))
                      if mimic is not None else 1.0)
        return {"xyz": xyz, "rpy": rpy, "axis": axis, "upper": upper,
                "multiplier": multiplier, "fixed": joint.get("type") == "fixed"}

    def pad_position(self, beta, theta):
        """(对掌 β, 弯曲 θ) → 指腹在 ``{side}_base_link`` 系里的坐标, 单位 m。"""
        angles = (beta, theta, self.mimic[1] * theta, self.mimic[2] * theta)
        matrix = np.eye(4)
        for link, angle in zip(self._links, angles):
            matrix = matrix @ _transform(link["xyz"],
                                         _rpy_matrix(*link["rpy"]))
            matrix = matrix @ _transform(
                [0, 0, 0], _rotation_about(link["axis"], angle))
        matrix = matrix @ _transform(self._pad["xyz"],
                                     _rpy_matrix(*self._pad["rpy"]))
        return matrix[:3, 3]

    def workspace(self, n=GRID_N):
        """(β, θ) 网格 → 指腹位置, 返回 (betas, thetas, n×n×3 数组)。"""
        betas = np.linspace(0.0, self.beta_max, n)
        thetas = np.linspace(0.0, self.theta_max, n)
        grid = np.empty((n, n, 3))
        for i, beta in enumerate(betas):
            for j, theta in enumerate(thetas):
                grid[i, j] = self.pad_position(beta, theta)
        return betas, thetas, grid


class FingerKinematics:
    """四指之一的指腹位置。结构比拇指简单: 一个驱动关节 + 一个 mimic。

    存在的理由不是驱动, 而是**校验抓握几何**: 拇指该走到多深, 取决于它要去
    够的四指在哪。标定时"只缩放对掌、不缩放弯曲"这个决定就建立在这上面。
    """

    #: 每根手指: (驱动关节, mimic 关节, 指腹力传感器)
    FINGERS = {
        "index": ("index_1_joint", "index_2_joint",
                  "index_force_sensor_3_joint"),
        "middle": ("middle_1_joint", "middle_2_joint",
                   "middle_force_sensor_3_joint"),
        "ring": ("ring_1_joint", "ring_2_joint", "ring_force_sensor_3_joint"),
        "little": ("little_1_joint", "little_2_joint",
                   "little_force_sensor_3_joint"),
    }

    def __init__(self, finger="index", side="right", urdf_path=DEFAULT_URDF):
        if finger not in self.FINGERS:
            raise ValueError(f"finger 只能是 {tuple(self.FINGERS)}")
        driver, passive, pad = self.FINGERS[finger]
        root = ET.parse(urdf_path).getroot()
        joints = {j.get("name"): j for j in root.iter("joint")}
        self._links = [ThumbKinematics._parse(joints[f"{side}_{driver}"]),
                       ThumbKinematics._parse(joints[f"{side}_{passive}"])]
        self._pad = ThumbKinematics._parse(joints[f"{side}_{pad}"])
        self.upper = self._links[0]["upper"]
        self.mimic = self._links[1]["multiplier"]

    def pad_position(self, closure):
        """闭合度 (0 伸直, 1 满弯) → 指腹坐标 (m)。"""
        angle = min(max(float(closure), 0.0), 1.0) * self.upper
        matrix = np.eye(4)
        for link, value in zip(self._links, (angle, self.mimic * angle)):
            matrix = matrix @ _transform(link["xyz"],
                                         _rpy_matrix(*link["rpy"]))
            matrix = matrix @ _transform(
                [0, 0, 0], _rotation_about(link["axis"], value))
        matrix = matrix @ _transform(self._pad["xyz"],
                                     _rpy_matrix(*self._pad["rpy"]))
        return matrix[:3, 3]


class ThumbSolver:
    """给定目标指腹位置, 求最接近的 (β, θ)。

    先在网格上找最近点, 再在相邻格内做一次二次细化。网格无折叠(相邻点最小
    间距 0.256mm), 所以最近点是良定义的, 不会出现两组解来回跳。
    """

    def __init__(self, kinematics=None, side="right", n=GRID_N):
        self.k = kinematics or ThumbKinematics(side)
        self.betas, self.thetas, self.grid = self.k.workspace(n)
        self._flat = self.grid.reshape(-1, 3)
        self.n = n
        # 因时侧的三个参考点, retarget_uv 用它们张成目标位置。
        self.q_open = self.k.pad_position(0.0, 0.0)
        self.q_opp = self.k.pad_position(self.k.beta_max, 0.0) - self.q_open
        self.q_flex = self.k.pad_position(0.0, self.k.theta_max) - self.q_open

    def solve(self, target):
        """→ (beta, theta, 残差 m)。目标在工作空间外时给最接近的可达点。"""
        target = np.asarray(target, dtype=float)
        if target.shape != (3,) or not np.all(np.isfinite(target)):
            return 0.0, 0.0, float("inf")
        distances = np.linalg.norm(self._flat - target, axis=1)
        index = int(np.argmin(distances))
        i, j = divmod(index, self.n)
        beta, theta = self._refine(target, i, j)
        residual = float(np.linalg.norm(self.k.pad_position(beta, theta)
                                        - target))
        return beta, theta, residual

    def retarget_uv(self, u, v, overshoot=1.0):
        """归一化 (对掌 u, 弯曲 v) → (弯曲闭合度, 对掌闭合度), 均在 [0, 1]。

        人手侧的标定在手套那一端(它才有手套数据), 送过来的就是这个 (u, v);
        这里只做"因时那一半"。

        **这里按参数对应, 不在三维空间里插值目标点。** 曾经的写法是
        ``target = q_open + u*q_opp + v*q_flex`` 再解最近可达点, 那是错的:
        ``q_opp``/``q_flex`` 是曲面上的**弦向量**, 它们张成的平面一离开角点就
        脱离曲面 —— 实测最远偏离 77mm, 而整个工作空间跨度才 90mm。投影回曲面
        之后映射会错乱(实测 u=0.75,v=0.25 解出对掌 0.975、弯曲 0.233)。

        用三个标定点能建立的只是人手曲面的**仿射图**, 建不出曲率; 因时这边
        同样只能用它自己的 (β, θ) 参数化。所以正确的对应就是参数对参数。

        方案 C 真正的收益在**上游**: 把"这个动作有几成弯曲、几成对掌"的分解
        从旋转特征空间(两轴夹角 31.5 度, 病态)搬到位置空间(60.2 度)。那一步在
        手套端的 thumb_normalized_uv() 里, 这里只负责把结果落到通道上。

        要再吃掉非线性, 得让人手标定采一整片扫掠而不是三个点 —— 那是另一件
        事, 不能靠在这里插值凭空变出来。

        实现在模块级的 ``uv_to_closures``: 运行时不需要网格, 这里只是保留一个
        方法入口给已有调用方。
        """
        return uv_to_closures(u, v, overshoot=overshoot)

    def _refine(self, target, i, j):
        """在最近格点周围做一次局部细分, 把量化误差压到网格步长以下。"""
        best = (self.betas[i], self.thetas[j])
        best_distance = float(np.linalg.norm(self.grid[i, j] - target))
        beta_step = (self.k.beta_max / (self.n - 1)) if self.n > 1 else 0.0
        theta_step = (self.k.theta_max / (self.n - 1)) if self.n > 1 else 0.0
        # 收缩步长搜索: 不改善时**继续收缩**而不是退出 —— 退出的话搜索会停在
        # 网格量化误差上(实测残差卡在 0.55mm 下不去), 收缩才是找到极小点的
        # 手段。
        #
        # 9 次折半把步长从一个网格步(0.029 rad)压到约 1e-4 rad。再细没有意义:
        # 因时角度寄存器 1000 counts 铺满整个行程, 1 count 已经是 1.2e-3 rad,
        # 比这一步还粗一个量级。迭代次数直接决定这段的耗时(每轮 8 次位置解算),
        # 曾经设成 20 次, 单手 6.3ms —— 纯属为了微米精度买单。
        for _ in range(9):
            for d_beta in (-beta_step, 0.0, beta_step):
                for d_theta in (-theta_step, 0.0, theta_step):
                    if d_beta == 0.0 and d_theta == 0.0:
                        continue
                    beta = min(max(best[0] + d_beta, 0.0), self.k.beta_max)
                    theta = min(max(best[1] + d_theta, 0.0), self.k.theta_max)
                    distance = float(np.linalg.norm(
                        self.k.pad_position(beta, theta) - target))
                    if distance < best_distance - 1e-15:
                        best, best_distance = (beta, theta), distance
            beta_step *= 0.5
            theta_step *= 0.5
        return best


class ThumbCalibration:
    """人手拇指指腹 → 因时目标指腹位置。

    人手侧只需要三个姿势, 正好是 mapcal 已经在做的:

        张开(P-pose) / 只做拇指弯曲 / 只做拇指对掌

    这三点在掌心系里张成一个平面片; 当前指腹位置在这个基底上的坐标 (u, v)
    就是"这个动作做了几成弯曲、几成对掌"。同一个 (u, v) 拿到因时那边的同名
    基底上, 就得到目标位置。

    垂直于该平面的分量被丢掉 —— 这是有意的: 因时拇指只有两个自由度, 平面外
    的动作它做不出来, 硬塞进去只会让两个通道乱动。
    """

    def __init__(self, open_pos, flex_pos, opp_pos, solver=None, side="right"):
        self.solver = solver or ThumbSolver(side=side)
        self.open_pos = np.asarray(open_pos, dtype=float)
        self.flex_axis = np.asarray(flex_pos, dtype=float) - self.open_pos
        self.opp_axis = np.asarray(opp_pos, dtype=float) - self.open_pos
        self._basis = np.stack([self.opp_axis, self.flex_axis], axis=1)

    @property
    def conditioning_deg(self):
        """人手两条基底的夹角。太小(<30°)说明标定姿势没做纯, 分解不可靠。"""
        a, b = self.opp_axis, self.flex_axis
        denominator = np.linalg.norm(a) * np.linalg.norm(b)
        if denominator < 1e-9:
            return 0.0
        cosine = min(1.0, abs(float(a @ b)) / denominator)
        return math.degrees(math.acos(cosine))

    def normalized(self, pad_position):
        """当前人手指腹 → (u, v), 各自 0=张开 1=标定到的满行程。"""
        delta = np.asarray(pad_position, dtype=float) - self.open_pos
        solution, *_ = np.linalg.lstsq(self._basis, delta, rcond=None)
        return float(solution[0]), float(solution[1])

    def retarget(self, pad_position, overshoot=1.0):
        """人手指腹位置 → (对掌闭合度, 弯曲闭合度), 均在 [0, 1]。

        ``overshoot`` 允许 u/v 超过 1 一点再交给求解器 —— 人手比因时能做的
        范围大, 超出的部分本来就该顶到因时的可达边界, 而不是提前夹死。
        """
        u, v = self.normalized(pad_position)
        return self.solver.retarget_uv(u, v, overshoot=overshoot)


def export_table(path, side="right", n=GRID_N):
    """把 (β, θ) → 指腹位置的表写成纯文本, 给 C++ 遥操线程加载。

    C++ 那边不重新解析 URDF: 运动学常数只有一个权威来源, 抄过去就会漂。
    表是生成物, 测试会重新生成一遍并逐点比对。
    """
    kinematics = ThumbKinematics(side)
    betas, thetas, grid = kinematics.workspace(n)
    with open(path, "w", encoding="utf-8") as handle:
        handle.write(f"# 因时 FTP {side} 拇指指腹工作空间, 由 "
                     f"PC端/thumb_retarget.py 从 URDF 生成, 不要手改。\n")
        handle.write(f"# 源: {os.path.relpath(kinematics.urdf_path)}\n")
        handle.write("# grid_n beta_max theta_max\n")
        # 一律先转成 Python float 再 repr —— numpy 2.x 的 repr 会写成
        # "np.float64(0.0)", C++ 那边解析不了, 而且只有跑到才发现。
        handle.write(f"{n} {float(kinematics.beta_max)!r} "
                     f"{float(kinematics.theta_max)!r}\n")
        handle.write("# 每行: beta theta x y z (单位 rad / m)\n")
        for i, beta in enumerate(betas):
            for j, theta in enumerate(thetas):
                x, y, z = (float(v) for v in grid[i, j])
                handle.write(f"{float(beta)!r} {float(theta)!r} "
                             f"{x!r} {y!r} {z!r}\n")
    return path


if __name__ == "__main__":
    for which in ("right", "left"):
        kin = ThumbKinematics(which)
        solver = ThumbSolver(kin)
        flat = solver.grid.reshape(-1, 3)
        print(f"[{which}] β_max={kin.beta_max:.4f} θ_max={kin.theta_max:.4f} "
              f"mimic={tuple(round(m, 4) for m in kin.mimic)}")
        print(f"         指腹工作空间跨度 (mm): "
              + "  ".join(f"{(flat[:, k].max() - flat[:, k].min()) * 1000:.1f}"
                          for k in range(3)))
