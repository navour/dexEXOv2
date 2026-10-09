#!/usr/bin/env python3
"""使用带软铁交叉轴和异常点的合成数据验证完整椭球拟合。"""

from __future__ import annotations

import numpy as np

from full_ellipsoid_calibration import fit_full_ellipsoid


def _sphere_samples(count: int) -> np.ndarray:
    index = np.arange(count, dtype=np.float64)
    z = 1.0 - 2.0 * (index + 0.5) / count
    radius = np.sqrt(1.0 - z * z)
    phi = index * (np.pi * (3.0 - np.sqrt(5.0)))
    return np.column_stack((radius * np.cos(phi), radius * np.sin(phi), z))


def main() -> None:
    rng = np.random.default_rng(20260721)
    field_norm = 49.0
    hard_iron = np.array((17.5, -10.0, 6.5))
    distortion = np.array(
        (
            (1.24, 0.18, -0.07),
            (0.18, 0.82, 0.13),
            (-0.07, 0.13, 1.09),
        )
    )

    ideal = field_norm * _sphere_samples(1200)
    raw_clean = ideal @ distortion.T + hard_iron
    raw_clean += rng.normal(0.0, 0.12, raw_clean.shape)

    # 加入约 3% 的局部磁场异常点，验证 Huber 鲁棒权重。
    raw = raw_clean.copy()
    outlier_index = rng.choice(len(raw), size=36, replace=False)
    raw[outlier_index] += rng.normal(0.0, 35.0, (len(outlier_index), 3))

    # 复现旧固件的 min/max + 对角缩放，作为同一数据集上的基线。
    minimum = raw.min(axis=0)
    maximum = raw.max(axis=0)
    diagonal_center = 0.5 * (minimum + maximum)
    diagonal_radius = 0.5 * (maximum - minimum)
    diagonal_scale = diagonal_radius.mean() / diagonal_radius
    diagonal_clean = (raw_clean - diagonal_center) * diagonal_scale
    diagonal_norms = np.linalg.norm(diagonal_clean, axis=1)
    diagonal_relative_std = float(np.std(diagonal_norms) / np.mean(diagonal_norms))

    result = fit_full_ellipsoid(raw)
    corrected_clean = result.apply(raw_clean)
    norms = np.linalg.norm(corrected_clean, axis=1)
    relative_std = float(np.std(norms) / np.mean(norms))
    bias_error = float(np.linalg.norm(result.hard_iron - hard_iron))

    print(result.to_c_initializer("synthetic_cal"))
    print(f"旧对角算法模长相对标准差: {diagonal_relative_std:.3%}")
    print(f"干净验证集模长相对标准差: {relative_std:.3%}")
    print(f"硬铁偏置误差: {bias_error:.3f} µT")
    print(f"内部质量判定: {'通过' if result.quality.passed else '未通过'}")

    if relative_std >= 0.01:
        raise SystemExit("验证失败：完整矩阵校正后的模长误差过大")
    if relative_std >= diagonal_relative_std * 0.25:
        raise SystemExit("验证失败：相对旧对角算法的改善不足")
    if bias_error >= 1.0:
        raise SystemExit("验证失败：硬铁偏置误差过大")
    if not result.quality.passed:
        raise SystemExit("验证失败：质量门限未通过")


if __name__ == "__main__":
    main()
