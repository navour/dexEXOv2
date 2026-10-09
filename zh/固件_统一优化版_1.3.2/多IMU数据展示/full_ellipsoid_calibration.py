#!/usr/bin/env python3
"""面向 ICM-20948/AK09916 的鲁棒完整三维椭球校准。

输入为 N×3 的磁力计原始数据（本项目中单位为 µT），输出：

    calibrated = matrix @ (raw - hard_iron)

其中 ``matrix`` 是包含非对角元素的完整 3×3 软铁校正矩阵。
拟合使用归一化代数椭球作为初值，再用 Huber 权重的几何残差迭代精修。
"""

from __future__ import annotations

import argparse
import json
import math
import re
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Iterable

import numpy as np


ALGORITHM_VERSION = "1.0.0"


class CalibrationError(RuntimeError):
    """样本不足、覆盖不完整或拟合退化。"""


@dataclass(frozen=True)
class CalibrationQuality:
    sample_count: int
    inlier_count: int
    rejected_count: int
    inlier_ratio: float
    coverage: float
    relative_std: float
    p95_radial_error: float
    max_radial_error: float
    matrix_condition: float
    passed: bool
    warnings: tuple[str, ...]

    def to_dict(self) -> dict[str, Any]:
        return {
            "sample_count": self.sample_count,
            "inlier_count": self.inlier_count,
            "rejected_count": self.rejected_count,
            "inlier_ratio": self.inlier_ratio,
            "coverage": self.coverage,
            "relative_std": self.relative_std,
            "p95_radial_error": self.p95_radial_error,
            "max_radial_error": self.max_radial_error,
            "matrix_condition": self.matrix_condition,
            "passed": self.passed,
            "warnings": list(self.warnings),
        }


@dataclass(frozen=True)
class CalibrationResult:
    hard_iron: np.ndarray
    soft_iron: np.ndarray
    field_norm: float
    quality: CalibrationQuality

    def apply(self, samples: np.ndarray | Iterable[Iterable[float]]) -> np.ndarray:
        raw = _as_samples(samples)
        return (raw - self.hard_iron) @ self.soft_iron.T

    def to_dict(self) -> dict[str, Any]:
        return {
            "algorithm": "robust_full_3d_ellipsoid",
            "algorithm_version": ALGORITHM_VERSION,
            "input_unit": "uT",
            "formula": "calibrated = soft_iron @ (raw - hard_iron)",
            "hard_iron": self.hard_iron.tolist(),
            "soft_iron": self.soft_iron.tolist(),
            "field_norm": self.field_norm,
            "quality": self.quality.to_dict(),
        }

    def to_c_initializer(self, variable_name: str = "mag_cal") -> str:
        hi = ", ".join(f"{value:.9g}f" for value in self.hard_iron)
        si = ", ".join(f"{value:.9g}f" for value in self.soft_iron.ravel())
        return (
            f"mag_cal_params_t {variable_name} = {{\n"
            f"    .hard_iron = {{{hi}}},\n"
            f"    .soft_iron = {{{si}}},\n"
            f"    .field_norm = {self.field_norm:.9g}f,\n"
            f"    .magic = MAG_CAL_MAGIC,\n"
            f"}};"
        )


def _as_samples(samples: np.ndarray | Iterable[Iterable[float]]) -> np.ndarray:
    array = np.asarray(samples, dtype=np.float64)
    if array.ndim != 2 or array.shape[1] != 3:
        raise CalibrationError("磁力计样本必须是 N×3 数组")
    if not np.all(np.isfinite(array)):
        raise CalibrationError("磁力计样本包含 NaN 或无穷大")
    return array


def _mad(values: np.ndarray) -> float:
    median = float(np.median(values))
    return 1.4826 * float(np.median(np.abs(values - median))) + 1e-12


def _matrix_sqrt(matrix: np.ndarray) -> np.ndarray:
    eigenvalues, eigenvectors = np.linalg.eigh(matrix)
    if float(eigenvalues.min()) <= 1e-8:
        raise CalibrationError("椭球二次型不是正定矩阵，请增加三维方向覆盖")
    return (eigenvectors * np.sqrt(eigenvalues)) @ eigenvectors.T


def _algebraic_fit(samples: np.ndarray, weights: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """以 trace(Q)=1 约束拟合 (x-b)^T M (x-b)=1。"""
    x, y, z = samples.T
    design = np.column_stack(
        (
            x * x - z * z,
            y * y - z * z,
            2.0 * x * y,
            2.0 * x * z,
            2.0 * y * z,
            x,
            y,
            z,
            np.ones_like(x),
        )
    )
    target = -(z * z)
    sqrt_weights = np.sqrt(np.clip(weights, 1e-6, 1.0))
    coefficients, _, rank, _ = np.linalg.lstsq(
        design * sqrt_weights[:, None], target * sqrt_weights, rcond=None
    )
    if rank < design.shape[1]:
        raise CalibrationError("样本方向退化，无法确定完整椭球")

    qxx, qyy, qxy, qxz, qyz, lx, ly, lz, constant = coefficients
    quadratic = np.array(
        (
            (qxx, qxy, qxz),
            (qxy, qyy, qyz),
            (qxz, qyz, 1.0 - qxx - qyy),
        ),
        dtype=np.float64,
    )
    linear = np.array((lx, ly, lz), dtype=np.float64)

    eigenvalues = np.linalg.eigvalsh(quadratic)
    if float(eigenvalues.min()) <= 1e-8:
        raise CalibrationError("拟合结果不是封闭椭球，请排除干扰点并补齐方向")

    center = -0.5 * np.linalg.solve(quadratic, linear)
    radius_term = float(center @ quadratic @ center - constant)
    if radius_term <= 1e-8:
        raise CalibrationError("椭球半径无效，请重新采集样本")

    shape = quadratic / radius_term
    return center, _matrix_sqrt(shape)


def _initial_model(samples: np.ndarray, huber_k: float) -> tuple[np.ndarray, np.ndarray]:
    """代数拟合并用径向残差做 IRLS，得到非线性优化初值。"""
    weights = np.ones(len(samples), dtype=np.float64)
    center: np.ndarray | None = None
    correction: np.ndarray | None = None

    for _ in range(12):
        center, correction = _algebraic_fit(samples, weights)
        calibrated = (samples - center) @ correction.T
        residual = np.linalg.norm(calibrated, axis=1) - 1.0
        threshold = max(0.015, huber_k * _mad(residual))
        magnitude = np.abs(residual)
        new_weights = np.ones_like(weights)
        mask = magnitude > threshold
        new_weights[mask] = threshold / magnitude[mask]
        new_weights = np.clip(new_weights, 0.01, 1.0)
        if float(np.max(np.abs(new_weights - weights))) < 1e-3:
            break
        weights = new_weights

    if center is None or correction is None:
        raise CalibrationError("无法生成椭球初值")
    return center, correction


def _pack_model(center: np.ndarray, matrix: np.ndarray) -> np.ndarray:
    return np.array(
        (
            center[0], center[1], center[2],
            matrix[0, 0], matrix[1, 1], matrix[2, 2],
            matrix[0, 1], matrix[0, 2], matrix[1, 2],
        ),
        dtype=np.float64,
    )


def _unpack_model(parameters: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    center = parameters[:3]
    a00, a11, a22, a01, a02, a12 = parameters[3:]
    matrix = np.array(
        ((a00, a01, a02), (a01, a11, a12), (a02, a12, a22)),
        dtype=np.float64,
    )
    return center, matrix


def _residual_and_jacobian(
    samples: np.ndarray, parameters: np.ndarray
) -> tuple[np.ndarray, np.ndarray]:
    center, matrix = _unpack_model(parameters)
    delta = samples - center
    transformed = delta @ matrix.T
    norms = np.linalg.norm(transformed, axis=1)
    norms = np.clip(norms, 1e-12, None)
    unit = transformed / norms[:, None]
    residual = norms - 1.0

    jacobian = np.empty((len(samples), 9), dtype=np.float64)
    jacobian[:, :3] = -(unit @ matrix)
    jacobian[:, 3] = unit[:, 0] * delta[:, 0]
    jacobian[:, 4] = unit[:, 1] * delta[:, 1]
    jacobian[:, 5] = unit[:, 2] * delta[:, 2]
    jacobian[:, 6] = unit[:, 0] * delta[:, 1] + unit[:, 1] * delta[:, 0]
    jacobian[:, 7] = unit[:, 0] * delta[:, 2] + unit[:, 2] * delta[:, 0]
    jacobian[:, 8] = unit[:, 1] * delta[:, 2] + unit[:, 2] * delta[:, 1]
    return residual, jacobian


def _refine_geometric(
    samples: np.ndarray,
    center: np.ndarray,
    matrix: np.ndarray,
    huber_k: float,
    max_iterations: int,
) -> tuple[np.ndarray, np.ndarray]:
    """以真实径向残差做 Huber-IRLS Levenberg-Marquardt 精修。"""
    parameters = _pack_model(center, matrix)
    damping = 1e-3

    for _ in range(max_iterations):
        residual, jacobian = _residual_and_jacobian(samples, parameters)
        threshold = max(0.008, huber_k * _mad(residual))
        magnitude = np.abs(residual)
        weights = np.ones_like(residual)
        mask = magnitude > threshold
        weights[mask] = threshold / magnitude[mask]
        weights = np.clip(weights, 0.005, 1.0)

        normal = jacobian.T @ (weights[:, None] * jacobian)
        gradient = jacobian.T @ (weights * residual)
        diagonal = np.maximum(np.diag(normal), 1e-9)
        system = normal + damping * np.diag(diagonal)
        try:
            step = np.linalg.solve(system, -gradient)
        except np.linalg.LinAlgError as exc:
            raise CalibrationError("几何精修矩阵奇异，请补齐方向") from exc

        old_cost = float(np.sum(weights * residual * residual))
        accepted = False
        step_scale = 1.0
        for _ in range(12):
            candidate = parameters + step_scale * step
            _, candidate_matrix = _unpack_model(candidate)
            if float(np.linalg.eigvalsh(candidate_matrix).min()) > 1e-6:
                candidate_residual, _ = _residual_and_jacobian(samples, candidate)
                new_cost = float(np.sum(weights * candidate_residual * candidate_residual))
                if new_cost < old_cost:
                    parameters = candidate
                    accepted = True
                    damping = max(damping * 0.4, 1e-8)
                    break
            step_scale *= 0.5

        if not accepted:
            damping = min(damping * 10.0, 1e8)
        if float(np.linalg.norm(step_scale * step)) < 1e-9:
            break

    return _unpack_model(parameters)


def _fibonacci_directions(count: int) -> np.ndarray:
    index = np.arange(count, dtype=np.float64)
    z = 1.0 - 2.0 * (index + 0.5) / count
    radius = np.sqrt(np.maximum(0.0, 1.0 - z * z))
    phi = index * (math.pi * (3.0 - math.sqrt(5.0)))
    return np.column_stack((radius * np.cos(phi), radius * np.sin(phi), z))


def _coverage(directions: np.ndarray, bin_count: int) -> float:
    centers = _fibonacci_directions(bin_count)
    bins = np.argmax(directions @ centers.T, axis=1)
    return float(len(np.unique(bins)) / bin_count)


def _assess_quality(
    raw: np.ndarray,
    hard_iron: np.ndarray,
    soft_iron: np.ndarray,
    field_norm: float,
    coverage_bins: int,
) -> CalibrationQuality:
    calibrated = (raw - hard_iron) @ soft_iron.T
    norms = np.linalg.norm(calibrated, axis=1)
    median_norm = float(np.median(norms))
    absolute_relative = np.abs(norms / max(median_norm, 1e-12) - 1.0)
    rejection_limit = min(
        0.25,
        max(0.05, float(np.median(absolute_relative)) + 4.5 * _mad(absolute_relative)),
    )
    inlier_mask = absolute_relative <= rejection_limit
    if int(np.count_nonzero(inlier_mask)) < 50:
        raise CalibrationError("有效样本不足，磁干扰点过多")

    inlier_vectors = calibrated[inlier_mask]
    inlier_norms = np.linalg.norm(inlier_vectors, axis=1)
    mean_norm = float(np.mean(inlier_norms))
    relative_error = np.abs(inlier_norms - mean_norm) / max(mean_norm, 1e-12)
    directions = inlier_vectors / np.clip(inlier_norms[:, None], 1e-12, None)

    coverage = _coverage(directions, coverage_bins)
    relative_std = float(np.std(inlier_norms) / max(mean_norm, 1e-12))
    p95_error = float(np.percentile(relative_error, 95))
    max_error = float(np.max(relative_error))
    condition = float(np.linalg.cond(soft_iron))
    inlier_count = int(np.count_nonzero(inlier_mask))
    inlier_ratio = inlier_count / len(raw)

    warnings: list[str] = []
    if coverage < 0.65:
        warnings.append(f"球面覆盖不足：{coverage:.1%} < 65%")
    if relative_std > 0.03:
        warnings.append(f"校正后模长相对标准差偏大：{relative_std:.1%} > 3%")
    if p95_error > 0.05:
        warnings.append(f"95%径向误差偏大：{p95_error:.1%} > 5%")
    if inlier_ratio < 0.80:
        warnings.append(f"有效样本比例过低：{inlier_ratio:.1%} < 80%")
    if condition > 10.0:
        warnings.append(f"软铁矩阵条件数过大：{condition:.2f} > 10")
    if not 15.0 <= field_norm <= 100.0:
        warnings.append(
            f"磁场模长 {field_norm:.2f} µT 超出常见范围，请检查单位或附近磁源"
        )
    axis_min = directions.min(axis=0)
    axis_max = directions.max(axis=0)
    if np.any(axis_min > -0.55) or np.any(axis_max < 0.55):
        warnings.append("至少一个轴的正负方向覆盖不足")

    return CalibrationQuality(
        sample_count=len(raw),
        inlier_count=inlier_count,
        rejected_count=len(raw) - inlier_count,
        inlier_ratio=inlier_ratio,
        coverage=coverage,
        relative_std=relative_std,
        p95_radial_error=p95_error,
        max_radial_error=max_error,
        matrix_condition=condition,
        passed=not warnings,
        warnings=tuple(warnings),
    )


def fit_full_ellipsoid(
    samples: np.ndarray | Iterable[Iterable[float]],
    *,
    min_samples: int = 200,
    huber_k: float = 1.5,
    max_iterations: int = 40,
    coverage_bins: int = 80,
) -> CalibrationResult:
    """拟合完整三维椭球并返回可直接供固件使用的校准参数。"""
    raw = _as_samples(samples)
    if len(raw) < min_samples:
        raise CalibrationError(f"样本不足：{len(raw)} < {min_samples}")
    if huber_k <= 0:
        raise ValueError("huber_k 必须大于 0")
    if coverage_bins < 20:
        raise ValueError("coverage_bins 至少为 20")

    # 使用稳健位置和尺度归一化，避免 µT 数值导致正规方程病态。
    location = np.median(raw, axis=0)
    scale = float(np.median(np.linalg.norm(raw - location, axis=1)))
    if scale <= 1e-6:
        raise CalibrationError("样本变化范围过小")
    normalized = (raw - location) / scale

    center, unit_matrix = _initial_model(normalized, huber_k)
    center, unit_matrix = _refine_geometric(
        normalized, center, unit_matrix, huber_k, max_iterations
    )

    eigenvalues = np.linalg.eigvalsh(unit_matrix)
    if float(eigenvalues.min()) <= 1e-6:
        raise CalibrationError("精修后的校正矩阵不是正定矩阵")

    determinant = float(np.linalg.det(unit_matrix))
    if determinant <= 1e-12:
        raise CalibrationError("精修后的校正矩阵行列式无效")

    # 椭球拟合无法独立辨识绝对磁场强度。令校正矩阵 det=1，保留 AK09916
    # 已有的物理量程标定，并以三条椭球半轴的几何平均值作为磁场模长。
    determinant_scale = determinant ** (1.0 / 3.0)
    hard_iron = location + scale * center
    soft_iron = unit_matrix / determinant_scale
    field_norm = scale / determinant_scale

    quality = _assess_quality(
        raw, hard_iron, soft_iron, field_norm, coverage_bins
    )
    return CalibrationResult(
        hard_iron=hard_iron,
        soft_iron=soft_iron,
        field_norm=field_norm,
        quality=quality,
    )


def load_samples(path: str | Path) -> np.ndarray:
    """读取逗号、分号或空白分隔的 x/y/z 文本，允许首行表头和 # 注释。"""
    rows: list[list[float]] = []
    source = Path(path)
    for line_number, line in enumerate(source.read_text(encoding="utf-8").splitlines(), 1):
        stripped = line.strip()
        if not stripped or stripped.startswith("#"):
            continue
        fields = [field for field in re.split(r"[,;\s]+", stripped) if field]
        if len(fields) < 3:
            raise CalibrationError(f"{source}:{line_number} 少于三列")
        try:
            rows.append([float(fields[0]), float(fields[1]), float(fields[2])])
        except ValueError:
            if not rows:
                continue
            raise CalibrationError(f"{source}:{line_number} 包含非数字数据") from None
    if not rows:
        raise CalibrationError(f"{source} 中没有磁力计样本")
    return _as_samples(rows)


def _print_result(result: CalibrationResult) -> None:
    q = result.quality
    print("完整三维椭球校准结果")
    print("硬铁偏置 (µT):", np.array2string(result.hard_iron, precision=6))
    print("软铁矩阵:")
    print(np.array2string(result.soft_iron, precision=8, suppress_small=False))
    print(f"校正后磁场模长: {result.field_norm:.4f} µT")
    print(
        f"质量: {'通过' if q.passed else '未通过'}，覆盖率={q.coverage:.1%}，"
        f"相对标准差={q.relative_std:.2%}，P95径向误差={q.p95_radial_error:.2%}，"
        f"剔除={q.rejected_count}/{q.sample_count}"
    )
    for warning in q.warnings:
        print("警告:", warning)


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="AK09916 鲁棒完整三维椭球校准")
    parser.add_argument("samples", help="包含 mx,my,mz 的 CSV/文本文件")
    parser.add_argument("-o", "--output", help="输出校准 JSON 文件")
    parser.add_argument("--c-output", help="输出 mag_cal_params_t C 初始化代码")
    parser.add_argument("--min-samples", type=int, default=200)
    parser.add_argument("--allow-poor-quality", action="store_true")
    args = parser.parse_args(argv)

    try:
        result = fit_full_ellipsoid(
            load_samples(args.samples), min_samples=args.min_samples
        )
    except (CalibrationError, OSError) as exc:
        parser.error(str(exc))

    _print_result(result)
    if args.output:
        Path(args.output).write_text(
            json.dumps(result.to_dict(), ensure_ascii=False, indent=2) + "\n",
            encoding="utf-8",
        )
    if args.c_output:
        Path(args.c_output).write_text(result.to_c_initializer() + "\n", encoding="utf-8")

    if not result.quality.passed and not args.allow_poor_quality:
        return 2
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
