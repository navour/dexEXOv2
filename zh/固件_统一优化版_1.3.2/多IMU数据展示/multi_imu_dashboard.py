#!/usr/bin/env python3
"""Multi-IMU dashboard UI that consumes APIs from multi_imu_core."""

from __future__ import annotations

import math
import os
import time
import threading
from pathlib import Path
from typing import List, Optional, Tuple
import pygame
from pygame.locals import (
    DOUBLEBUF, OPENGL, QUIT, KEYDOWN,
    K_ESCAPE, K_UP, K_DOWN, K_BACKSPACE, K_SPACE,
    K_c, K_x, K_e, K_o, K_u,
)
from OpenGL.GL import *
from OpenGL.GLU import *
import numpy as np

from multi_imu_core import MultiImuService
from full_ellipsoid_calibration import (
    CalibrationError,
    CalibrationResult,
    fit_full_ellipsoid,
)

WIN_W, WIN_H = 1200, 760
DASHBOARD_VERSION = "1.5.0"
OTA_FIRMWARE_PATH = str(
    Path(__file__).resolve().parent.parent
    / "ESP32C3固件" / "build_unified_1_3_2" / "vqf_esp32.bin"
)

CHINESE_FONT_REGULAR = "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc"
CHINESE_FONT_BOLD = "/usr/share/fonts/opentype/noto/NotoSansCJK-Bold.ttc"

CAL_GUIDE_STEP_SEC = 5.0
CAL_GUIDE_STEPS = (
    ("开始自由翻转", "从元件面朝上开始，连续缓慢滚动设备"),
    ("覆盖六个面", "让正反面和四个侧面都经过朝上方向"),
    ("补齐斜方向", "缓慢画 8 字并观察球面覆盖提示"),
    ("等待质量通过", "出现绿色提示后按 X，拟合并无线保存"),
)

CAL_STATE_ZH = {
    "idle": "空闲",
    "start": "开始采集",
    "progress": "采集中",
    "collecting": "采集中",
    "computing": "正在计算",
    "uploading": "正在下发完整参数",
    "set_ok": "完整校准已保存",
    "cancelled": "已取消采集",
    "done": "校准完成",
    "fail": "校准失败",
    "stopped": "已停止",
    "erased": "已清除",
}


def _format_remain(minutes: int) -> str:
    """Format remaining battery time for display."""
    if minutes < 0:
        return "--"
    if minutes >= 60:
        h = minutes // 60
        m = minutes % 60
        return f"{h}小时{m:02d}分"
    return f"{minutes}分"


def _load_chinese_font(size: int, bold: bool = False) -> pygame.font.Font:
    """优先加载明确支持中文的字体，避免界面出现方框字。"""
    preferred = CHINESE_FONT_BOLD if bold else CHINESE_FONT_REGULAR
    if os.path.exists(preferred):
        return pygame.font.Font(preferred, size)

    matched = pygame.font.match_font(
        "notosanscjksc,notosanscjk,droidsansfallback,wqyzenhei"
    )
    if matched:
        return pygame.font.Font(matched, size)
    return pygame.font.SysFont(None, size, bold=bold)


def _cal_state_zh(state: str) -> str:
    return CAL_STATE_ZH.get(state.lower(), state or "--")


def _supports_full_calibration(version: str) -> bool:
    """CAL_SET 从固件 1.3.0 开始支持。未知版本按不支持处理。"""
    try:
        parts = tuple(int(part) for part in version.lstrip("vV").split(".")[:3])
    except ValueError:
        return False
    return len(parts) == 3 and parts >= (1, 3, 0)


def _power_source_zh(source: str, charging: bool) -> str:
    if charging:
        return "USB 充电中"
    if source == "USB":
        return "USB 供电"
    if source == "BAT":
        return "电池供电"
    return "供电未知"


def _run_ota(svc: MultiImuService, node_id: str, path: str, status: dict) -> None:
    """Background thread: OTA upload, update status dict."""
    status[node_id] = "OTA:上传中..."
    try:
        ok = svc.ota_update(node_id, path, timeout=90)
        status[node_id] = "OTA:成功 ✓" if ok else "OTA:失败 ✗"
    except Exception as e:
        status[node_id] = f"OTA:异常 {e}"


def quat_to_euler(q):
    w, x, y, z = q
    sinr = 2.0 * (w * x + y * z)
    cosr = 1.0 - 2.0 * (x * x + y * y)
    roll = math.degrees(math.atan2(sinr, cosr))

    sinp = 2.0 * (w * y - z * x)
    pitch = math.degrees(math.asin(max(-1.0, min(1.0, sinp))))

    siny = 2.0 * (w * z + x * y)
    cosy = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.degrees(math.atan2(siny, cosy))
    return roll, pitch, yaw


def quat_to_gl_matrix(q):
    w, x, y, z = q
    return [
        1 - 2 * (y * y + z * z), 2 * (x * y + w * z), 2 * (x * z - w * y), 0,
        2 * (x * y - w * z), 1 - 2 * (x * x + z * z), 2 * (y * z + w * x), 0,
        2 * (x * z + w * y), 2 * (y * z - w * x), 1 - 2 * (x * x + y * y), 0,
        0, 0, 0, 1,
    ]


def draw_board():
    hx, hy, hz = 1.0, 0.6, 0.07
    glBegin(GL_QUADS)
    glColor3f(0.06, 0.58, 0.26)
    glVertex3f(-hx, -hy, hz); glVertex3f(hx, -hy, hz); glVertex3f(hx, hy, hz); glVertex3f(-hx, hy, hz)
    glColor3f(0.02, 0.35, 0.12)
    glVertex3f(-hx, -hy, -hz); glVertex3f(-hx, hy, -hz); glVertex3f(hx, hy, -hz); glVertex3f(hx, -hy, -hz)
    glEnd()


def draw_text(x, y, text, font, color=(240, 240, 240)):
    s = font.render(text, True, color).convert_alpha()
    data = pygame.image.tostring(s, "RGBA", True)
    w, h = s.get_size()
    glPixelStorei(GL_UNPACK_ALIGNMENT, 1)
    glWindowPos2f(float(x), float(WIN_H - y - h))
    glDrawPixels(w, h, GL_RGBA, GL_UNSIGNED_BYTE, data)


def draw_cal_guide_panel(step: int, seconds_left: int, guidance: str,
                         stop_ok: bool, font, font_lg, font_xl) -> None:
    """绘制校准分步向导面板。"""
    x, y, w, h = 680, 270, 500, 250
    title, instruction = CAL_GUIDE_STEPS[step]

    glDisable(GL_DEPTH_TEST)
    glEnable(GL_BLEND)
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA)
    _setup_ortho()

    glColor4f(0.06, 0.08, 0.14, 0.94)
    glBegin(GL_QUADS)
    glVertex2f(x, y)
    glVertex2f(x + w, y)
    glVertex2f(x + w, y + h)
    glVertex2f(x, y + h)
    glEnd()
    glLineWidth(1.0)

    border = (0.2, 0.85, 0.45) if stop_ok else (0.25, 0.55, 0.95)
    glColor3f(*border)
    glLineWidth(2.0)
    glBegin(GL_LINE_LOOP)
    glVertex2f(x, y)
    glVertex2f(x + w, y)
    glVertex2f(x + w, y + h)
    glVertex2f(x, y + h)
    glEnd()

    progress = step / max(len(CAL_GUIDE_STEPS) - 1, 1)
    glColor3f(0.16, 0.18, 0.24)
    glBegin(GL_QUADS)
    glVertex2f(x + 18, y + 66)
    glVertex2f(x + w - 18, y + 66)
    glVertex2f(x + w - 18, y + 76)
    glVertex2f(x + 18, y + 76)
    glEnd()
    glColor3f(0.2, 0.8, 0.45)
    glBegin(GL_QUADS)
    glVertex2f(x + 18, y + 66)
    glVertex2f(x + 18 + (w - 36) * progress, y + 66)
    glVertex2f(x + 18 + (w - 36) * progress, y + 76)
    glVertex2f(x + 18, y + 76)
    glEnd()
    _restore_projection()

    color = (80, 255, 120) if stop_ok else (100, 200, 255)
    draw_text(x + 18, y + 12,
              f"磁力计校准向导  {step + 1}/{len(CAL_GUIDE_STEPS)}",
              font_lg, color)
    draw_text(x + 18, y + 86, title, font_xl, (255, 230, 120))
    draw_text(x + 18, y + 128, instruction, font, (235, 235, 240))

    if step < len(CAL_GUIDE_STEPS) - 1:
        draw_text(x + 18, y + 160,
                  f"本步剩余 {seconds_left} 秒，请缓慢转动",
                  font_lg, (130, 210, 255))
    elif stop_ok:
        draw_text(x + 18, y + 160, "数据已达标：按 X 计算并保存校准",
                  font_lg, (80, 255, 120))
    else:
        draw_text(x + 18, y + 160, guidance, font_lg, (180, 200, 255))

    draw_text(x + 18, y + 210,
              "空格：下一步    退格：上一步    X：完成并保存",
              font, (170, 175, 190))


# ============================================================================
# Calibration visualization helpers
# ============================================================================

THETA_BINS = 12   # elevation bins (0..π)
PHI_BINS = 24     # azimuth bins (0..2π)
COVERAGE_THRESHOLD = 0.65
NORM_STD_THRESHOLD = 0.03  # 3% of mean


def _samples_to_np(samples: List[List[float]]) -> np.ndarray:
    if not samples:
        return np.empty((0, 3))
    return np.array(samples, dtype=np.float32)


def _compute_norms(raw: np.ndarray) -> np.ndarray:
    if len(raw) == 0:
        return np.empty(0)
    return np.linalg.norm(raw, axis=1)


def _compute_coverage(raw: np.ndarray) -> Tuple[np.ndarray, float]:
    """Return (heatmap[THETA_BINS, PHI_BINS], coverage_ratio)."""
    hmap = np.zeros((THETA_BINS, PHI_BINS), dtype=np.int32)
    if len(raw) == 0:
        return hmap, 0.0
    norms = np.linalg.norm(raw, axis=1, keepdims=True)
    norms = np.clip(norms, 1e-9, None)
    unit = raw / norms
    theta = np.arccos(np.clip(unit[:, 2], -1, 1))            # 0..π
    phi = np.arctan2(unit[:, 1], unit[:, 0]) + np.pi         # 0..2π
    ti = np.clip((theta / np.pi * THETA_BINS).astype(int), 0, THETA_BINS - 1)
    pi_ = np.clip((phi / (2 * np.pi) * PHI_BINS).astype(int), 0, PHI_BINS - 1)
    for t, p in zip(ti, pi_):
        hmap[t, p] += 1
    filled = np.count_nonzero(hmap)
    coverage = filled / (THETA_BINS * PHI_BINS)
    return hmap, coverage


def _lowest_coverage_direction(hmap: np.ndarray) -> str:
    """Find empty/lowest bin region and return human-readable rotation hint."""
    if hmap.sum() == 0:
        return "缓慢旋转设备，覆盖所有方向"
    region_sums = {}
    half_t = THETA_BINS // 2
    half_p = PHI_BINS // 2
    region_sums["上 (Z+)"] = hmap[:half_t // 2, :].sum()
    region_sums["下 (Z-)"] = hmap[-half_t // 2:, :].sum()
    region_sums["前 (Y+)"] = hmap[:, half_p // 2: half_p].sum()
    region_sums["后 (Y-)"] = hmap[:, -half_p // 2:].sum()
    region_sums["左 (X-)"] = hmap[:, : half_p // 4].sum()
    region_sums["右 (X+)"] = hmap[:, half_p - half_p // 4: half_p + half_p // 4].sum()
    weakest = min(region_sums, key=region_sums.get)
    return f"请向 {weakest} 方向旋转"


def _setup_ortho():
    """Switch to 2D orthographic projection for HUD drawing."""
    glMatrixMode(GL_PROJECTION)
    glPushMatrix()
    glLoadIdentity()
    glOrtho(0, WIN_W, WIN_H, 0, -1, 1)
    glMatrixMode(GL_MODELVIEW)
    glPushMatrix()
    glLoadIdentity()


def _restore_projection():
    glMatrixMode(GL_PROJECTION)
    glPopMatrix()
    glMatrixMode(GL_MODELVIEW)
    glPopMatrix()


def draw_cal_3d_pointcloud(raw: np.ndarray, calibrated: Optional[np.ndarray],
                           cam_angle: float):
    """Draw 3D magnetometer point cloud in the left viewport area."""
    glViewport(0, 200, 660, 560)
    glMatrixMode(GL_PROJECTION)
    glLoadIdentity()
    gluPerspective(45, 660 / 560, 0.1, 200.0)
    glMatrixMode(GL_MODELVIEW)
    glLoadIdentity()

    dist = 5.0
    cx = dist * math.cos(math.radians(cam_angle)) * math.cos(math.radians(25))
    cy = dist * math.sin(math.radians(cam_angle)) * math.cos(math.radians(25))
    cz = dist * math.sin(math.radians(25))
    gluLookAt(cx, cy, cz, 0, 0, 0, 0, 0, 1)

    # Axis lines
    glBegin(GL_LINES)
    glColor3f(0.7, 0.2, 0.2); glVertex3f(0, 0, 0); glVertex3f(2, 0, 0)
    glColor3f(0.2, 0.7, 0.2); glVertex3f(0, 0, 0); glVertex3f(0, 2, 0)
    glColor3f(0.2, 0.2, 0.7); glVertex3f(0, 0, 0); glVertex3f(0, 0, 2)
    glEnd()

    glPointSize(3.0)
    if len(raw) > 0:
        # auto-scale to fit
        scale = 1.0
        maxv = np.abs(raw).max()
        if maxv > 0:
            scale = 2.0 / maxv

        # Raw points (gray)
        glBegin(GL_POINTS)
        glColor3f(0.55, 0.55, 0.55)
        for p in raw:
            glVertex3f(p[0] * scale, p[1] * scale, p[2] * scale)
        glEnd()

        # Calibrated points (green)
        if calibrated is not None and len(calibrated) > 0:
            glBegin(GL_POINTS)
            glColor3f(0.2, 0.9, 0.3)
            for p in calibrated:
                glVertex3f(p[0] * scale, p[1] * scale, p[2] * scale)
            glEnd()

    # Restore full viewport
    glViewport(0, 0, WIN_W, WIN_H)


def draw_cal_heatmap(hmap: np.ndarray, coverage: float, font, font_lg):
    """Draw spherical coverage heatmap as 2D colored grid in top-right area."""
    ox, oy = 700, 60
    cell_w = 18
    cell_h = 14
    max_count = max(hmap.max(), 1)

    glDisable(GL_LIGHTING)
    glDisable(GL_DEPTH_TEST)
    _setup_ortho()

    for ti in range(THETA_BINS):
        for pi in range(PHI_BINS):
            v = hmap[ti, pi]
            if v == 0:
                r, g, b = 0.12, 0.12, 0.18
            else:
                t = min(v / max_count, 1.0)
                # blue → cyan → green → yellow
                if t < 0.33:
                    f = t / 0.33
                    r, g, b = 0.0, f * 0.6, 0.8 - f * 0.3
                elif t < 0.66:
                    f = (t - 0.33) / 0.33
                    r, g, b = f * 0.8, 0.6 + f * 0.3, 0.5 - f * 0.5
                else:
                    f = (t - 0.66) / 0.34
                    r, g, b = 0.8 + f * 0.2, 0.9 - f * 0.3, 0.0

            x1 = ox + pi * cell_w
            y1 = oy + ti * cell_h
            glColor3f(r, g, b)
            glBegin(GL_QUADS)
            glVertex2f(x1, y1)
            glVertex2f(x1 + cell_w - 1, y1)
            glVertex2f(x1 + cell_w - 1, y1 + cell_h - 1)
            glVertex2f(x1, y1 + cell_h - 1)
            glEnd()

    # Border
    bw = PHI_BINS * cell_w
    bh = THETA_BINS * cell_h
    glColor3f(0.4, 0.4, 0.4)
    glBegin(GL_LINE_LOOP)
    glVertex2f(ox, oy)
    glVertex2f(ox + bw, oy)
    glVertex2f(ox + bw, oy + bh)
    glVertex2f(ox, oy + bh)
    glEnd()

    _restore_projection()

    # Labels
    cov_color = (80, 255, 120) if coverage >= COVERAGE_THRESHOLD else (255, 200, 100)
    draw_text(ox, oy - 24, f"球面覆盖热力图  覆盖率: {coverage*100:.1f}%", font_lg, cov_color)
    draw_text(ox, oy + bh + 4, "← φ (方位角 0~360°) →", font, (160, 160, 160))

    glEnable(GL_DEPTH_TEST)


def draw_cal_norm_curve(norms: np.ndarray, font, font_lg):
    """Draw magnetometer norm time-series in bottom area."""
    ox, oy = 40, 560
    w, h = 500, 140

    if len(norms) == 0:
        draw_text(ox, oy, "范数曲线: 等待数据...", font, (160, 160, 160))
        return

    mean_n = norms.mean()
    std_n = norms.std()
    rel_std = (std_n / mean_n * 100) if mean_n > 0 else 0.0

    glDisable(GL_LIGHTING)
    glDisable(GL_DEPTH_TEST)
    _setup_ortho()

    # Background
    glColor4f(0.08, 0.08, 0.14, 0.85)
    glBegin(GL_QUADS)
    glVertex2f(ox, oy); glVertex2f(ox + w, oy)
    glVertex2f(ox + w, oy + h); glVertex2f(ox, oy + h)
    glEnd()

    # Border
    glColor3f(0.3, 0.3, 0.4)
    glBegin(GL_LINE_LOOP)
    glVertex2f(ox, oy); glVertex2f(ox + w, oy)
    glVertex2f(ox + w, oy + h); glVertex2f(ox, oy + h)
    glEnd()

    # Plot norms
    n_show = min(len(norms), 300)
    plot_norms = norms[-n_show:]
    lo = plot_norms.min() * 0.95
    hi = plot_norms.max() * 1.05
    if hi - lo < 0.01:
        lo -= 0.05
        hi += 0.05
    margin = 5

    # Mean line
    my = oy + h - margin - (mean_n - lo) / (hi - lo) * (h - 2 * margin)
    glColor3f(0.3, 0.6, 0.3)
    glBegin(GL_LINES)
    glVertex2f(ox, my); glVertex2f(ox + w, my)
    glEnd()

    # Norm curve
    glColor3f(0.3, 0.85, 1.0)
    glBegin(GL_LINE_STRIP)
    for i, n in enumerate(plot_norms):
        x = ox + margin + i * (w - 2 * margin) / max(n_show - 1, 1)
        y = oy + h - margin - (n - lo) / (hi - lo) * (h - 2 * margin)
        glVertex2f(x, y)
    glEnd()

    _restore_projection()

    # Labels
    std_color = (80, 255, 120) if rel_std < (NORM_STD_THRESHOLD * 100) else (255, 200, 100)
    draw_text(ox, oy - 24, f"‖m‖ 范数曲线  均值={mean_n:.3f}  标准差={std_n:.4f}  相对偏差={rel_std:.1f}%",
              font_lg, std_color)
    draw_text(ox + w + 10, oy, f"最大={plot_norms.max():.3f}", font, (160, 160, 160))
    draw_text(ox + w + 10, oy + h - 16, f"最小={plot_norms.min():.3f}", font, (160, 160, 160))

    glEnable(GL_DEPTH_TEST)


def main():
    svc = MultiImuService()
    svc.start()

    pygame.init()
    pygame.font.init()
    pygame.display.set_mode((WIN_W, WIN_H), DOUBLEBUF | OPENGL)
    pygame.display.set_caption("无线 IMU 上位机")

    glEnable(GL_DEPTH_TEST)
    glEnable(GL_LIGHTING)
    glEnable(GL_LIGHT0)
    glEnable(GL_COLOR_MATERIAL)
    glClearColor(0.07, 0.08, 0.12, 1.0)

    font = _load_chinese_font(17)
    font_lg = _load_chinese_font(22, bold=True)
    font_xl = _load_chinese_font(30, bold=True)

    clock = pygame.time.Clock()
    selected_node_id = None
    cam_angle = 30.0  # auto-rotating camera for cal 3D view
    cal_view_node_id = None
    cal_view_active = False
    cal_guide_step = 0
    cal_guide_step_started_at = 0.0
    cal_preview: Optional[CalibrationResult] = None
    cal_preview_sample_count = 0
    cal_preview_updated_at = 0.0
    cal_preview_error = "等待至少 200 个样本"
    ota_status: dict = {}  # node_id -> status string
    ui_notice = ""
    ui_notice_until = 0.0

    running = True
    while running:
        devices = svc.list_devices()

        # Resolve selected index from node_id
        selected_idx = 0
        if selected_node_id:
            for i, d in enumerate(devices):
                if d.node_id == selected_node_id:
                    selected_idx = i
                    break
            else:
                # Previously selected device disappeared; reset
                selected_node_id = None
        if devices:
            selected_idx = max(0, min(selected_idx, len(devices) - 1))
            selected_node_id = devices[selected_idx].node_id
            svc.select_device(selected_node_id)
        selected = svc.get_selected()

        for ev in pygame.event.get():
            if ev.type == QUIT:
                if cal_view_active and cal_view_node_id:
                    svc.send_cal_stop(cal_view_node_id)
                running = False
            elif ev.type == KEYDOWN:
                if ev.key == K_ESCAPE:
                    if cal_view_active and cal_view_node_id:
                        svc.send_cal_stop(cal_view_node_id)
                        cal_view_active = False
                        cal_view_node_id = None
                        ui_notice = "已取消本次校准，原有参数保持不变"
                        ui_notice_until = time.time() + 5.0
                    else:
                        running = False
                elif ev.key == K_UP and devices:
                    selected_idx = max(0, selected_idx - 1)
                    selected_node_id = devices[selected_idx].node_id
                elif ev.key == K_DOWN and devices:
                    selected_idx = min(len(devices) - 1, selected_idx + 1)
                    selected_node_id = devices[selected_idx].node_id
                elif ev.key == K_c and selected and not cal_view_active:
                    if not _supports_full_calibration(selected.firmware_version):
                        ui_notice = (
                            f"设备固件 {selected.firmware_version or '未知'} 不支持完整校准，"
                            "请先按 U 升级到 1.3.2"
                        )
                        ui_notice_until = time.time() + 7.0
                    elif svc.send_cal_start(selected.node_id):
                        cal_view_node_id = selected.node_id
                        cal_view_active = True
                        cal_guide_step = 0
                        cal_guide_step_started_at = time.time()
                        cal_preview = None
                        cal_preview_sample_count = 0
                        cal_preview_updated_at = 0.0
                        cal_preview_error = "等待至少 200 个样本"
                    else:
                        ui_notice = "校准启动失败：请确认设备显示“已连接”"
                        ui_notice_until = time.time() + 5.0
                elif ev.key == K_SPACE and cal_view_active:
                    cal_guide_step = min(
                        len(CAL_GUIDE_STEPS) - 1, cal_guide_step + 1
                    )
                    cal_guide_step_started_at = time.time()
                elif ev.key == K_BACKSPACE and cal_view_active:
                    cal_guide_step = max(0, cal_guide_step - 1)
                    cal_guide_step_started_at = time.time()
                elif ev.key == K_x:
                    target = None
                    if cal_view_node_id:
                        for d in devices:
                            if d.node_id == cal_view_node_id:
                                target = d
                                break
                    if target and cal_view_active:
                        try:
                            final_result = fit_full_ellipsoid(
                                _samples_to_np(target.cal_raw_samples),
                                min_samples=200,
                                max_iterations=40,
                            )
                            if not final_result.quality.passed:
                                details = "；".join(final_result.quality.warnings[:2])
                                ui_notice = f"数据质量未通过，请继续翻转：{details}"
                            elif svc.send_cal_set(
                                target.node_id,
                                final_result.hard_iron.tolist(),
                                final_result.soft_iron.tolist(),
                                final_result.field_norm,
                            ):
                                cal_view_active = False
                                cal_view_node_id = None
                                ui_notice = "完整 3×3 参数已发送，等待设备写入 NVS"
                            else:
                                ui_notice = "参数下发失败：设备连接已断开"
                        except CalibrationError as exc:
                            ui_notice = f"暂不能完成校准：{exc}"
                        ui_notice_until = time.time() + 7.0
                elif ev.key == K_e and selected:
                    if cal_view_active and cal_view_node_id:
                        svc.send_cal_stop(cal_view_node_id)
                        cal_view_active = False
                        cal_view_node_id = None
                    if svc.send_cal_erase(selected.node_id):
                        ui_notice = "已发送清除校准参数命令"
                    else:
                        ui_notice = "清除失败：设备未连接"
                    ui_notice_until = time.time() + 5.0
                elif ev.key == K_o and selected:
                    svc.send_shutdown(selected.node_id)
                elif ev.key == K_u and selected:
                    # OTA upgrade: firmware must be at OTA_FIRMWARE_PATH
                    node = selected.node_id
                    if ota_status.get(node, "") != "OTA:上传中...":
                        t = threading.Thread(
                            target=_run_ota,
                            args=(svc, node, OTA_FIRMWARE_PATH, ota_status),
                            daemon=True)
                        t.start()

        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)

        # 校准界面完全由本地 UI 状态控制，避免任何自动退出。
        cal_device = None
        if cal_view_node_id:
            for d in devices:
                if d.node_id == cal_view_node_id:
                    cal_device = d
                    break
        cal_active = cal_view_active

        if cal_active:
            # === Calibration visualization mode ===
            cam_angle += 0.3  # auto-rotate

            guide_now = time.time()
            if (cal_guide_step < len(CAL_GUIDE_STEPS) - 1
                    and guide_now - cal_guide_step_started_at >= CAL_GUIDE_STEP_SEC):
                cal_guide_step += 1
                cal_guide_step_started_at = guide_now

            raw_np = _samples_to_np(cal_device.cal_raw_samples) if cal_device else np.empty((0, 3))
            cal_np = None
            if (len(raw_np) >= 200
                    and len(raw_np) != cal_preview_sample_count
                    and guide_now - cal_preview_updated_at >= 0.75):
                try:
                    cal_preview = fit_full_ellipsoid(
                        raw_np, min_samples=200, max_iterations=20
                    )
                    cal_preview_error = ""
                except CalibrationError as exc:
                    cal_preview = None
                    cal_preview_error = str(exc)
                cal_preview_sample_count = len(raw_np)
                cal_preview_updated_at = guide_now

            if cal_preview is not None:
                cal_np = cal_preview.apply(raw_np)
                display_vectors = cal_np
            elif len(raw_np) > 0:
                display_vectors = raw_np - np.median(raw_np, axis=0)
            else:
                display_vectors = raw_np
            norms = _compute_norms(display_vectors)
            hmap, coverage = _compute_coverage(display_vectors)

            # 1) 3D point cloud (left area)
            glDisable(GL_LIGHTING)
            draw_cal_3d_pointcloud(raw_np, cal_np, cam_angle)

            # 2) Heatmap (top-right)
            draw_cal_heatmap(hmap, coverage, font, font_lg)

            # 3) Norm curve (bottom-left)
            draw_cal_norm_curve(norms, font, font_lg)

            # HUD text overlay
            glDisable(GL_LIGHTING)
            glDisable(GL_DEPTH_TEST)
            glEnable(GL_BLEND)
            glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA)

            # Device list (compact)
            draw_text(14, 10, "设备列表（↑/↓ 选择）", font_lg)
            y = 36
            now = time.time()
            for i, d in enumerate(devices):
                marker = ">" if i == selected_idx else " "
                conn = "已连接" if d.connected else "未连接"
                line = f"{marker} {d.node_id}  {conn}"
                color = (80, 255, 120) if d.connected else (255, 130, 130)
                draw_text(14, y, line, font, color)
                # OTA status on same row
                ota_s = ota_status.get(d.node_id, "")
                if ota_s:
                    ota_col = (255, 200, 80) if "上传" in ota_s else ((80, 255, 120) if "成功" in ota_s else (255, 100, 100))
                    draw_text(200, y, ota_s, font, ota_col)
                y += 20
            if cal_device:
                draw_text(14, 505,
                          f"校准状态：{_cal_state_zh(cal_device.cal_state)}  "
                          f"样本：{len(cal_device.cal_raw_samples)}  "
                          f"采集进度：{cal_device.cal_progress_pct}%",
                          font_lg, (100, 200, 255))
            else:
                draw_text(14, 505,
                          "校准视图保持中：设备暂时离线，等待重连……",
                          font_lg, (255, 200, 100))

            # 最终停止条件来自完整椭球模块的等面积覆盖和误差门限。
            stop_ok = bool(cal_preview and cal_preview.quality.passed)
            if cal_preview is not None:
                q = cal_preview.quality
                stop_msg = (
                    f"{'✓ 数据质量已达标，可以按 X 保存' if stop_ok else '继续翻转'}  "
                    f"等面积覆盖={q.coverage:.0%}  模长偏差={q.relative_std:.1%}"
                )
            elif len(raw_np) < 200:
                stop_msg = f"继续翻转：样本 {len(raw_np)}/200"
            else:
                stop_msg = f"继续采集：{cal_preview_error}"
            if stop_ok:
                draw_text(14, 720, stop_msg, font_lg, (80, 255, 120))
            elif stop_msg:
                draw_text(14, 720, stop_msg, font, (255, 200, 100))

            if cal_preview is not None and cal_preview.quality.warnings:
                guidance = cal_preview.quality.warnings[0]
            else:
                guidance = _lowest_coverage_direction(hmap)
            seconds_left = max(
                0,
                math.ceil(CAL_GUIDE_STEP_SEC - (guide_now - cal_guide_step_started_at)),
            )
            draw_cal_guide_panel(
                cal_guide_step, seconds_left, guidance, stop_ok,
                font, font_lg, font_xl,
            )

            draw_text(14, 450, "操作：X=完成并保存  E=清除校准  U=无线升级", font)
            draw_text(14, 476,
                      f"上位机 v{DASHBOARD_VERSION}  设备固件：{cal_device.firmware_version if cal_device else '--'}",
                      font, (140, 140, 180))
            glDisable(GL_BLEND)

        else:
            # === Normal mode ===
            # 3D selected IMU
            glMatrixMode(GL_PROJECTION)
            glLoadIdentity()
            gluPerspective(45, WIN_W / WIN_H, 0.1, 60.0)
            glMatrixMode(GL_MODELVIEW)
            glLoadIdentity()
            gluLookAt(3.2, 2.4, 2.8, 0, 0, 0, 0, 0, 1)

            if selected:
                glPushMatrix()
                glMultMatrixf(quat_to_gl_matrix(selected.quat))
                draw_board()
                glPopMatrix()

            # HUD
            glDisable(GL_LIGHTING)
            glDisable(GL_DEPTH_TEST)
            glEnable(GL_BLEND)
            glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA)

            draw_text(14, 10, "设备列表（↑/↓ 选择）", font_lg)
            y = 40
            now = time.time()
            for i, d in enumerate(devices):
                age = now - d.last_seen
                marker = ">" if i == selected_idx else " "
                conn = "已连接" if d.connected else "未连接"
                batt = f"{d.battery_percent}%" if d.battery_percent is not None else "--"
                power = _power_source_zh(d.power_source, d.charging)
                remain = _format_remain(d.battery_remain_min)
                hz = f"{d.pkt_rate_hz:.0f}Hz" if d.pkt_rate_hz > 0 else "--Hz"
                fw = d.firmware_version or "--"
                line = (f"{marker} {d.node_id}  {d.ip}  {conn}  {hz}  "
                        f"电量:{batt}  剩余:{remain}  {power}  固件:{fw}  延迟:{age:3.1f}秒")
                color = (80, 255, 120) if d.connected else (255, 130, 130)
                draw_text(14, y, line, font, color)
                # OTA status
                ota_s = ota_status.get(d.node_id, "")
                if ota_s:
                    ota_col = (255, 200, 80) if "上传" in ota_s else ((80, 255, 120) if "成功" in ota_s else (255, 100, 100))
                    draw_text(14, y + 13, ota_s, font, ota_col)
                    y += 13
                y += 24

            y = 420
            draw_text(14, y,
                      "操作：C=开始磁力计校准  E=清除校准  O=设备关机  U=无线升级  ESC=退出",
                      font)
            y += 26

            if selected:
                roll, pitch, yaw = quat_to_euler(selected.quat)
                draw_text(14, y, f"当前设备：{selected.node_id}  ({selected.ip})", font_lg)
                y += 32
                draw_text(14, y,
                          f"姿态角：横滚={roll:+7.2f}°  俯仰={pitch:+7.2f}°  航向={yaw:+7.2f}°",
                          font)
                y += 24
                voltage = (f"{selected.battery_voltage:.3f}V"
                           if selected.battery_voltage is not None else "--V")
                percent = (f"{selected.battery_percent}%"
                           if selected.battery_percent is not None else "--")
                draw_text(14, y,
                          f"电池：{voltage}  {percent}  预计剩余：{_format_remain(selected.battery_remain_min)}  "
                          f"{_power_source_zh(selected.power_source, selected.charging)}",
                          font)
                y += 24
                key_voltage = (f"{selected.key_voltage:.3f}V"
                               if selected.key_voltage is not None else "--V")
                key_state = "已按下" if selected.key_pressed else "未按下"
                draw_text(14, y,
                          f"数据速率：{selected.pkt_rate_hz:.1f} Hz  "
                          f"固件：{selected.firmware_version or '--'}  "
                          f"按键：{key_voltage} ({key_state})",
                          font)
                y += 24
                draw_text(14, y,
                          f"校准状态：{_cal_state_zh(selected.cal_state)}  "
                          f"设备消息：{selected.cal_message or '--'}",
                          font)
            else:
                draw_text(14, y, "尚未发现 IMU，请确认设备与电脑在同一局域网……",
                          font_lg, (255, 200, 100))

            if ui_notice and time.time() < ui_notice_until:
                draw_text(14, 650, ui_notice, font_lg, (255, 220, 100))

            glEnable(GL_DEPTH_TEST)
            glDisable(GL_BLEND)
        pygame.display.flip()
        clock.tick(60)

    svc.stop()
    pygame.quit()


if __name__ == "__main__":
    main()
