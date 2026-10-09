"""轻量级 G1 URDF/STL OpenGL 渲染器。

不引入新的第三方依赖：使用标准库解析 URDF 和二进制/ASCII STL，
由 PyOpenGL 顶点数组绘制官方 G1 双臂模型。
"""

from __future__ import annotations

from dataclasses import dataclass
import os
import struct
import xml.etree.ElementTree as ET

import numpy as np
from OpenGL.GL import (
    GL_AMBIENT,
    GL_AMBIENT_AND_DIFFUSE,
    GL_COLOR_MATERIAL,
    GL_DIFFUSE,
    GL_FLOAT,
    GL_FRONT_AND_BACK,
    GL_LIGHT0,
    GL_LIGHTING,
    GL_NORMAL_ARRAY,
    GL_POSITION,
    GL_SHININESS,
    GL_SPECULAR,
    GL_TRIANGLES,
    GL_VERTEX_ARRAY,
    glColor4f,
    glColorMaterial,
    glDisable,
    glDisableClientState,
    glDrawArrays,
    glEnable,
    glEnableClientState,
    glLightfv,
    glMaterialf,
    glMaterialfv,
    glMultMatrixf,
    glNormalPointer,
    glPopMatrix,
    glPushMatrix,
    glVertexPointer,
)


def _parse_vector(text, length=3, default=0.0):
    if not text:
        return np.full(length, default, dtype=float)
    values = [float(v) for v in text.split()]
    if len(values) != length:
        raise ValueError(f"向量长度应为 {length}: {text!r}")
    return np.asarray(values, dtype=float)


def _rpy_matrix(rpy):
    roll, pitch, yaw = rpy
    cr, sr = np.cos(roll), np.sin(roll)
    cp, sp = np.cos(pitch), np.sin(pitch)
    cy, sy = np.cos(yaw), np.sin(yaw)
    rx = np.array([[1., 0., 0.], [0., cr, -sr], [0., sr, cr]])
    ry = np.array([[cp, 0., sp], [0., 1., 0.], [-sp, 0., cp]])
    rz = np.array([[cy, -sy, 0.], [sy, cy, 0.], [0., 0., 1.]])
    return rz @ ry @ rx


def _transform(xyz=None, rpy=None):
    result = np.eye(4)
    result[:3, :3] = _rpy_matrix(np.zeros(3) if rpy is None else rpy)
    result[:3, 3] = np.zeros(3) if xyz is None else xyz
    return result


def _axis_angle_matrix(axis, angle):
    axis = np.asarray(axis, dtype=float)
    norm = float(np.linalg.norm(axis))
    if norm < 1e-12 or abs(angle) < 1e-12:
        return np.eye(4)
    x, y, z = axis / norm
    c, s = np.cos(angle), np.sin(angle)
    one_c = 1.0 - c
    result = np.eye(4)
    result[:3, :3] = np.array([
        [c + x*x*one_c, x*y*one_c - z*s, x*z*one_c + y*s],
        [y*x*one_c + z*s, c + y*y*one_c, y*z*one_c - x*s],
        [z*x*one_c - y*s, z*y*one_c + x*s, c + z*z*one_c],
    ])
    return result


# 全身模型的固定站姿。这里只是画个造型 —— 腿不参与遥操, 也不参与任何解算;
# 取值与 仿真/g1_mujoco_sim.STANDING_LEG_POSE 和 HumDex 的 DEFAULT_JOINTS
# 一致, 三处画面里的站姿才是同一个。上半身模型没有这些关节, FK 会忽略。
STANDING_LEG_POSE = {
    "left_hip_pitch_joint": -0.2,
    "left_knee_joint": 0.4,
    "left_ankle_pitch_joint": -0.2,
    "right_hip_pitch_joint": -0.2,
    "right_knee_joint": 0.4,
    "right_ankle_pitch_joint": -0.2,
}

# 判断一个 link 属不属于手臂。全身模型里 left_/right_ 前缀腿也有, 只靠前缀
# 会把腿当成手臂高亮。
_ARM_LINK_PARTS = ("shoulder", "elbow", "wrist", "hand", "rubber")


@dataclass
class TriangleMesh:
    vertices: np.ndarray
    normals: np.ndarray

    @property
    def triangle_count(self):
        return len(self.vertices) // 3


@dataclass
class Visual:
    link: str
    mesh_path: str
    origin: np.ndarray
    scale: np.ndarray
    color: np.ndarray
    material_name: str


@dataclass
class Joint:
    name: str
    joint_type: str
    parent: str
    child: str
    origin: np.ndarray
    axis: np.ndarray


def load_stl(path):
    """读取二进制或 ASCII STL，返回展开后的三角形顶点和法线。"""
    file_size = os.path.getsize(path)
    with open(path, "rb") as stream:
        header = stream.read(84)
        if len(header) >= 84:
            triangle_count = struct.unpack("<I", header[80:84])[0]
            is_binary = 84 + triangle_count * 50 == file_size
        else:
            is_binary = False

        if is_binary:
            dtype = np.dtype([
                ("normal", "<f4", (3,)),
                ("vertices", "<f4", (3, 3)),
                ("attribute", "<u2"),
            ])
            records = np.fromfile(stream, dtype=dtype, count=triangle_count)
            face_normals = np.asarray(records["normal"], dtype=np.float32)
            vertices = np.asarray(records["vertices"].reshape(-1, 3),
                                  dtype=np.float32)
        else:
            stream.seek(0)
            lines = stream.read().decode("utf-8", errors="ignore").splitlines()
            face_normals = []
            triangles = []
            current_vertices = []
            current_normal = np.array([0., 0., 1.], dtype=np.float32)
            for line in lines:
                fields = line.strip().split()
                if fields[:2] == ["facet", "normal"] and len(fields) >= 5:
                    current_normal = np.asarray(fields[2:5], dtype=np.float32)
                elif fields[:1] == ["vertex"] and len(fields) >= 4:
                    current_vertices.append([float(v) for v in fields[1:4]])
                    if len(current_vertices) == 3:
                        triangles.extend(current_vertices)
                        face_normals.append(current_normal)
                        current_vertices = []
            vertices = np.asarray(triangles, dtype=np.float32)
            face_normals = np.asarray(face_normals, dtype=np.float32)

    if len(vertices) == 0 or len(vertices) % 3:
        raise ValueError(f"STL 没有有效三角形: {path}")

    # 某些 STL 法线为空，直接从顶点重新计算。
    triangles = vertices.reshape(-1, 3, 3)
    generated = np.cross(triangles[:, 1] - triangles[:, 0],
                         triangles[:, 2] - triangles[:, 0])
    generated_norm = np.linalg.norm(generated, axis=1, keepdims=True)
    generated = generated / np.maximum(generated_norm, 1e-12)
    face_norm = np.linalg.norm(face_normals, axis=1, keepdims=True)
    valid = face_norm[:, 0] > 1e-8
    face_normals = np.where(valid[:, None],
                            face_normals / np.maximum(face_norm, 1e-12),
                            generated).astype(np.float32)
    normals = np.repeat(face_normals, 3, axis=0)
    return TriangleMesh(np.ascontiguousarray(vertices),
                        np.ascontiguousarray(normals))


class G1UrdfModel:
    """解析 G1 URDF 并计算完整链式 FK；该类不依赖 OpenGL 上下文。"""

    def __init__(self, urdf_path):
        self.urdf_path = os.path.abspath(urdf_path)
        self.base_dir = os.path.dirname(self.urdf_path)
        self.materials = {}
        self.visuals = []
        self.joints = []
        self.children = {}
        self.links = set()
        self.root_link = None
        self._parse()

    @staticmethod
    def _origin(element):
        origin = element.find("origin") if element is not None else None
        if origin is None:
            return np.eye(4)
        return _transform(_parse_vector(origin.get("xyz")),
                          _parse_vector(origin.get("rpy")))

    def _parse(self):
        robot = ET.parse(self.urdf_path).getroot()
        for material in robot.findall("material"):
            color = material.find("color")
            if color is not None:
                self.materials[material.get("name", "")] = _parse_vector(
                    color.get("rgba"), length=4, default=1.0)

        for link in robot.findall("link"):
            link_name = link.get("name")
            self.links.add(link_name)
            for visual_element in link.findall("visual"):
                mesh = visual_element.find("geometry/mesh")
                if mesh is None:
                    continue
                filename = mesh.get("filename", "")
                if filename.startswith("package://"):
                    filename = filename.split("/", 3)[-1]
                mesh_path = os.path.normpath(os.path.join(self.base_dir, filename))
                material = visual_element.find("material")
                material_name = material.get("name", "") if material is not None else ""
                inline_color = material.find("color") if material is not None else None
                color = (self.materials.get(material_name, np.array([.72, .75, .80, 1.]))
                         if inline_color is None else
                         _parse_vector(inline_color.get("rgba"), length=4, default=1.0))
                self.visuals.append(Visual(
                    link_name, mesh_path, self._origin(visual_element),
                    _parse_vector(mesh.get("scale"), default=1.0),
                    np.asarray(color, dtype=float), material_name))

        child_links = set()
        for joint_element in robot.findall("joint"):
            parent = joint_element.find("parent").get("link")
            child = joint_element.find("child").get("link")
            axis_element = joint_element.find("axis")
            axis = (_parse_vector(axis_element.get("xyz"))
                    if axis_element is not None else np.array([1., 0., 0.]))
            joint = Joint(
                joint_element.get("name"), joint_element.get("type", "fixed"),
                parent, child, self._origin(joint_element), axis)
            self.joints.append(joint)
            self.children.setdefault(parent, []).append(joint)
            child_links.add(child)

        roots = sorted(self.links - child_links)
        if len(roots) != 1:
            raise ValueError(f"URDF 根 link 数量异常: {roots}")
        self.root_link = roots[0]

    def link_transforms(self, joint_values=None):
        joint_values = joint_values or {}
        transforms = {self.root_link: np.eye(4)}
        pending = [self.root_link]
        while pending:
            parent = pending.pop()
            for joint in self.children.get(parent, []):
                motion = np.eye(4)
                if joint.joint_type in ("revolute", "continuous"):
                    motion = _axis_angle_matrix(
                        joint.axis, float(joint_values.get(joint.name, 0.0)))
                transforms[joint.child] = transforms[parent] @ joint.origin @ motion
                pending.append(joint.child)
        return transforms


class G1UrdfRenderer:
    """使用官方 mesh 实时绘制 G1 上半身。"""

    def __init__(self, urdf_path):
        self.model = G1UrdfModel(urdf_path)
        self.meshes = {}
        self.loaded = False
        self.load_error = None

    @property
    def triangle_count(self):
        return sum(mesh.triangle_count for mesh in self.meshes.values())

    def load_meshes(self):
        if self.loaded:
            return
        try:
            for visual in self.model.visuals:
                if visual.mesh_path not in self.meshes:
                    self.meshes[visual.mesh_path] = load_stl(visual.mesh_path)
            self.loaded = True
        except Exception as exc:
            self.load_error = str(exc)
            raise

    @staticmethod
    def _joint_values(right_angles, left_angles, waist_yaw=0.0,
                      hand_joints=None):
        values = dict(STANDING_LEG_POSE)
        # 腰只驱动 yaw; roll/pitch 恒定回中, 与真机接收端一致。上半身模型
        # 没有这些关节, 多余的键会被 FK 忽略。
        values["waist_yaw_joint"] = float(waist_yaw)
        values["waist_roll_joint"] = 0.0
        values["waist_pitch_joint"] = 0.0
        for side, angles in (("right", right_angles), ("left", left_angles)):
            if angles is None:
                continue
            values.update({
                f"{side}_shoulder_pitch_joint": float(angles.shoulder_pitch),
                f"{side}_shoulder_roll_joint": float(angles.shoulder_roll),
                f"{side}_shoulder_yaw_joint": float(angles.shoulder_yaw),
                f"{side}_elbow_joint": float(angles.elbow),
                f"{side}_wrist_roll_joint": float(angles.wrist_roll),
                f"{side}_wrist_pitch_joint": 0.0,
                f"{side}_wrist_yaw_joint": 0.0,
            })
        # 灵巧手：调用方给的是 hand_mapping.expand_mimic() 展开后的 12 个
        # 关节角 —— 本渲染器不解析 URDF 的 <mimic>, 所以联动关节必须由
        # 调用方算好一起传进来, 只传 6 个驱动关节的话手指会只弯一半。
        # 无手模型上这些键会被 FK 直接忽略。
        if hand_joints:
            values.update({name: float(angle)
                           for name, angle in hand_joints.items()})
        return values

    @staticmethod
    def _is_arm_link(link):
        """区分手臂与腿 —— 全身模型里 left_/right_ 前缀两者都有。

        高亮只应落在正在跟随的那条手臂上; 按前缀判断会把同侧的腿一起点亮。
        """
        return any(part in link for part in _ARM_LINK_PARTS)

    @classmethod
    def _display_color(cls, visual, active_side, following):
        color = visual.color.copy()
        # 按实际 rgba 判深色件, 不认材质名 —— 官方上半身 URDF 用的是
        # "dark"/"white", 生成的全身 URDF 用的是 mat_0/mat_1。
        if float(np.mean(color[:3])) < 0.35:
            color[:3] = np.array([0.055, 0.07, 0.09])
        elif (visual.link.startswith(f"{active_side}_")
                and cls._is_arm_link(visual.link)):
            accent = np.array([0.34, 0.72, 0.86]) if following else np.array([0.58, 0.67, 0.76])
            color[:3] = color[:3] * 0.58 + accent * 0.42
        elif visual.link.startswith(("left_", "right_")) and cls._is_arm_link(visual.link):
            color[:3] = color[:3] * 0.72
        elif visual.link.startswith(("left_", "right_")):
            # 腿不参与遥操, 压暗让画面重心留在双臂上。
            color[:3] = np.array([0.42, 0.45, 0.50])
        else:
            color[:3] = np.array([0.66, 0.69, 0.74])
        color[3] = 1.0
        return color

    def render(self, root_transform, right_angles, left_angles,
               active_side="right", following=False, waist_yaw=0.0,
               hand_joints=None):
        if not self.loaded:
            self.load_meshes()
        transforms = self.model.link_transforms(
            self._joint_values(right_angles, left_angles, waist_yaw,
                               hand_joints))

        glEnable(GL_LIGHTING)
        glEnable(GL_LIGHT0)
        glEnable(GL_COLOR_MATERIAL)
        glColorMaterial(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE)
        glLightfv(GL_LIGHT0, GL_POSITION, (2.5, -1.5, 3.5, 1.0))
        glLightfv(GL_LIGHT0, GL_AMBIENT, (0.22, 0.24, 0.28, 1.0))
        glLightfv(GL_LIGHT0, GL_DIFFUSE, (0.92, 0.95, 1.0, 1.0))
        glLightfv(GL_LIGHT0, GL_SPECULAR, (0.35, 0.38, 0.42, 1.0))
        glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, (0.28, 0.30, 0.34, 1.0))
        glMaterialf(GL_FRONT_AND_BACK, GL_SHININESS, 24.0)
        glEnableClientState(GL_VERTEX_ARRAY)
        glEnableClientState(GL_NORMAL_ARRAY)

        for visual in self.model.visuals:
            link_transform = transforms.get(visual.link)
            mesh = self.meshes.get(visual.mesh_path)
            if link_transform is None or mesh is None:
                continue
            model_matrix = root_transform @ link_transform @ visual.origin
            if not np.allclose(visual.scale, 1.0):
                scale = np.eye(4)
                scale[0, 0], scale[1, 1], scale[2, 2] = visual.scale
                model_matrix = model_matrix @ scale
            color = self._display_color(visual, active_side, following)
            glColor4f(*color)
            glPushMatrix()
            glMultMatrixf(np.asarray(model_matrix.T, dtype=np.float32))
            glVertexPointer(3, GL_FLOAT, 0, mesh.vertices)
            glNormalPointer(GL_FLOAT, 0, mesh.normals)
            glDrawArrays(GL_TRIANGLES, 0, len(mesh.vertices))
            glPopMatrix()

        glDisableClientState(GL_NORMAL_ARRAY)
        glDisableClientState(GL_VERTEX_ARRAY)
        glDisable(GL_COLOR_MATERIAL)
        glDisable(GL_LIGHT0)
        glDisable(GL_LIGHTING)
