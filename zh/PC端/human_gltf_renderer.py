"""带骨骼人体 GLB 的轻量级 OpenGL 渲染器。

仅依赖 numpy 与 PyOpenGL：解析 glTF 2.0 二进制文件、执行 CPU 蒙皮，
并把 IMU 解算出的上臂/前臂方向写入人体骨骼。
"""

from __future__ import annotations

from dataclasses import dataclass
import json
import os
import struct

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
    GL_NORMALIZE,
    GL_POSITION,
    GL_SHININESS,
    GL_SPECULAR,
    GL_TRIANGLES,
    GL_UNSIGNED_BYTE,
    GL_UNSIGNED_INT,
    GL_UNSIGNED_SHORT,
    GL_VERTEX_ARRAY,
    glColor4f,
    glColorMaterial,
    glDisable,
    glDisableClientState,
    glDrawElements,
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


_COMPONENT_DTYPES = {
    5120: np.int8,
    5121: np.uint8,
    5122: np.int16,
    5123: np.uint16,
    5125: np.uint32,
    5126: np.float32,
}
_TYPE_COMPONENTS = {
    "SCALAR": 1,
    "VEC2": 2,
    "VEC3": 3,
    "VEC4": 4,
    "MAT2": 4,
    "MAT3": 9,
    "MAT4": 16,
}
_GL_INDEX_TYPES = {
    np.dtype(np.uint8): GL_UNSIGNED_BYTE,
    np.dtype(np.uint16): GL_UNSIGNED_SHORT,
    np.dtype(np.uint32): GL_UNSIGNED_INT,
}


def _quaternion_matrix(quaternion):
    """glTF 四元数 [x, y, z, w] → 4x4 旋转矩阵。"""
    x, y, z, w = np.asarray(quaternion, dtype=float)
    norm = x*x + y*y + z*z + w*w
    if norm < 1e-16:
        return np.eye(4)
    scale = 2.0 / norm
    result = np.eye(4)
    result[:3, :3] = np.array([
        [1-scale*(y*y+z*z), scale*(x*y-z*w), scale*(x*z+y*w)],
        [scale*(x*y+z*w), 1-scale*(x*x+z*z), scale*(y*z-x*w)],
        [scale*(x*z-y*w), scale*(y*z+x*w), 1-scale*(x*x+y*y)],
    ])
    return result


def _node_matrix(node):
    if "matrix" in node:
        # glTF 矩阵以列优先保存。
        return np.asarray(node["matrix"], dtype=float).reshape(4, 4).T
    translation = np.asarray(node.get("translation", [0., 0., 0.]), dtype=float)
    rotation = _quaternion_matrix(node.get("rotation", [0., 0., 0., 1.]))
    scale = np.asarray(node.get("scale", [1., 1., 1.]), dtype=float)
    result = rotation @ np.diag([scale[0], scale[1], scale[2], 1.])
    result[:3, 3] = translation
    return result


def _rotation_only(matrix):
    """从可能含统一缩放的矩阵中提取稳定的正交旋转。"""
    u, _, vh = np.linalg.svd(np.asarray(matrix, dtype=float)[:3, :3])
    result = u @ vh
    if np.linalg.det(result) < 0.0:
        u[:, -1] *= -1.0
        result = u @ vh
    return result


def _rotation_between(source, target):
    source = np.asarray(source, dtype=float)
    target = np.asarray(target, dtype=float)
    source /= max(float(np.linalg.norm(source)), 1e-12)
    target /= max(float(np.linalg.norm(target)), 1e-12)
    cross = np.cross(source, target)
    sine = float(np.linalg.norm(cross))
    cosine = float(np.clip(np.dot(source, target), -1.0, 1.0))
    if sine < 1e-9:
        if cosine > 0.0:
            return np.eye(3)
        basis = np.array([1., 0., 0.]) if abs(source[0]) < 0.8 else np.array([0., 1., 0.])
        axis = np.cross(source, basis)
        axis /= np.linalg.norm(axis)
        return _axis_angle(axis, np.pi)
    axis = cross / sine
    angle = np.arctan2(sine, cosine)
    return _axis_angle(axis, angle)


def _axis_angle(axis, angle):
    axis = np.asarray(axis, dtype=float)
    axis /= max(float(np.linalg.norm(axis)), 1e-12)
    x, y, z = axis
    cosine, sine = np.cos(angle), np.sin(angle)
    one_c = 1.0 - cosine
    return np.array([
        [cosine+x*x*one_c, x*y*one_c-z*sine, x*z*one_c+y*sine],
        [y*x*one_c+z*sine, cosine+y*y*one_c, y*z*one_c-x*sine],
        [z*x*one_c-y*sine, z*y*one_c+x*sine, cosine+z*z*one_c],
    ])


@dataclass
class GltfPrimitive:
    positions: np.ndarray
    normals: np.ndarray
    joints: np.ndarray
    weights: np.ndarray
    indices: np.ndarray
    material: int

    @property
    def triangle_count(self):
        return len(self.indices) // 3


class HumanGltfModel:
    """解析人体 GLB，并在无 OpenGL 上下文时也能进行骨骼姿态测试。"""

    BONE_CHAINS = {
        "left": ("DEF-upper_arm.L", "DEF-forearm.L", "DEF-hand.L"),
        "right": ("DEF-upper_arm.R", "DEF-forearm.R", "DEF-hand.R"),
    }

    def __init__(self, glb_path):
        self.glb_path = os.path.abspath(glb_path)
        self.document, self.binary = self._read_glb(self.glb_path)
        self.nodes = self.document.get("nodes", [])
        self.parents = {
            child: parent
            for parent, node in enumerate(self.nodes)
            for child in node.get("children", [])
        }
        self.node_by_name = {
            node.get("name", f"node_{index}"): index
            for index, node in enumerate(self.nodes)
        }
        self.bind_locals = [_node_matrix(node) for node in self.nodes]
        self.skin = self.document["skins"][0]
        self.skin_joints = np.asarray(self.skin["joints"], dtype=np.int32)
        inverse_bind = self.accessor(self.skin["inverseBindMatrices"])
        self.inverse_bind = inverse_bind.reshape(-1, 4, 4).transpose(0, 2, 1)
        self.materials = self._read_materials()
        self.primitives = self._read_primitives()
        self.mesh_node = self._find_mesh_node()
        self.last_globals = self.global_transforms(self.bind_locals)

        for chain in self.BONE_CHAINS.values():
            missing = [name for name in chain if name not in self.node_by_name]
            if missing:
                raise ValueError(f"人体模型缺少骨骼: {', '.join(missing)}")

    @staticmethod
    def _read_glb(path):
        with open(path, "rb") as stream:
            payload = stream.read()
        if len(payload) < 20:
            raise ValueError("GLB 文件过短")
        magic, version, total_length = struct.unpack_from("<4sII", payload, 0)
        if magic != b"glTF" or version != 2 or total_length != len(payload):
            raise ValueError("仅支持有效的 glTF 2.0 GLB 文件")
        offset = 12
        json_length, json_type = struct.unpack_from("<II", payload, offset)
        offset += 8
        if json_type != 0x4E4F534A:
            raise ValueError("GLB 首个区块不是 JSON")
        document = json.loads(payload[offset:offset+json_length].decode("utf-8"))
        offset = (offset + json_length + 3) & ~3
        binary_length, binary_type = struct.unpack_from("<II", payload, offset)
        offset += 8
        if binary_type != 0x004E4942:
            raise ValueError("GLB 缺少二进制区块")
        binary = memoryview(payload)[offset:offset+binary_length]
        return document, binary

    def accessor(self, index):
        accessor = self.document["accessors"][index]
        if "sparse" in accessor:
            raise ValueError("暂不支持 sparse accessor")
        view = self.document["bufferViews"][accessor["bufferView"]]
        dtype = np.dtype(_COMPONENT_DTYPES[accessor["componentType"]]).newbyteorder("<")
        component_count = _TYPE_COMPONENTS[accessor["type"]]
        byte_offset = view.get("byteOffset", 0) + accessor.get("byteOffset", 0)
        item_size = dtype.itemsize * component_count
        stride = view.get("byteStride", item_size)
        values = np.ndarray(
            (accessor["count"], component_count), dtype=dtype,
            buffer=self.binary, offset=byte_offset,
            strides=(stride, dtype.itemsize),
        )
        return np.ascontiguousarray(values)

    def _read_materials(self):
        colors = []
        for material in self.document.get("materials", []):
            pbr = material.get("pbrMetallicRoughness", {})
            colors.append(np.asarray(
                pbr.get("baseColorFactor", [.72, .75, .80, 1.]), dtype=float))
        return colors or [np.array([.72, .75, .80, 1.])]

    def _read_primitives(self):
        result = []
        for mesh in self.document.get("meshes", []):
            for primitive in mesh.get("primitives", []):
                if primitive.get("mode", 4) != 4:
                    continue
                attributes = primitive["attributes"]
                required = {"POSITION", "NORMAL", "JOINTS_0", "WEIGHTS_0"}
                if not required.issubset(attributes):
                    raise ValueError("人体网格缺少蒙皮所需属性")
                weights = self.accessor(attributes["WEIGHTS_0"]).astype(np.float32)
                weights /= np.maximum(weights.sum(axis=1, keepdims=True), 1e-12)
                result.append(GltfPrimitive(
                    positions=self.accessor(attributes["POSITION"]).astype(np.float32),
                    normals=self.accessor(attributes["NORMAL"]).astype(np.float32),
                    joints=self.accessor(attributes["JOINTS_0"]).astype(np.int32),
                    weights=weights,
                    indices=self.accessor(primitive["indices"]).reshape(-1),
                    material=int(primitive.get("material", 0)),
                ))
        if not result:
            raise ValueError("GLB 中没有可绘制的人体三角网格")
        return result

    def _find_mesh_node(self):
        for index, node in enumerate(self.nodes):
            if "mesh" in node and "skin" in node:
                return index
        raise ValueError("GLB 中没有蒙皮网格节点")

    def global_transforms(self, locals_):
        globals_ = [None] * len(self.nodes)

        def resolve(index):
            if globals_[index] is not None:
                return globals_[index]
            parent = self.parents.get(index)
            globals_[index] = (locals_[index].copy() if parent is None
                               else resolve(parent) @ locals_[index])
            return globals_[index]

        for index in range(len(self.nodes)):
            resolve(index)
        return globals_

    @staticmethod
    def _replace_local_rotation(local, rotation):
        result = local.copy()
        scales = np.linalg.norm(result[:3, :3], axis=0)
        result[:3, :3] = rotation @ np.diag(scales)
        return result

    def _aim_bone(self, locals_, globals_, bone, child, target, twist=0.0):
        bone_index = self.node_by_name[bone]
        child_index = self.node_by_name[child]
        parent_index = self.parents[bone_index]
        current = globals_[child_index][:3, 3] - globals_[bone_index][:3, 3]
        desired = np.asarray(target, dtype=float)
        if np.linalg.norm(current) < 1e-10 or np.linalg.norm(desired) < 1e-10:
            return globals_
        desired /= np.linalg.norm(desired)
        delta = _rotation_between(current, desired)
        desired_rotation = delta @ _rotation_only(globals_[bone_index])
        if abs(twist) > 1e-8:
            desired_rotation = _axis_angle(desired, float(twist)) @ desired_rotation
        parent_rotation = _rotation_only(globals_[parent_index])
        local_rotation = parent_rotation.T @ desired_rotation
        locals_[bone_index] = self._replace_local_rotation(
            locals_[bone_index], local_rotation)
        return self.global_transforms(locals_)

    def pose(self, arms=None):
        """返回指定双臂方向后的节点全局矩阵。

        arms 格式：{"right": (upper_dir, forearm_dir, twist), ...}，
        所有方向均使用模型坐标系。
        """
        locals_ = [matrix.copy() for matrix in self.bind_locals]
        globals_ = self.global_transforms(locals_)
        for side, values in (arms or {}).items():
            if side not in self.BONE_CHAINS or values is None:
                continue
            upper_dir, forearm_dir, twist = values
            upper, forearm, hand = self.BONE_CHAINS[side]
            globals_ = self._aim_bone(
                locals_, globals_, upper, forearm, upper_dir)
            globals_ = self._aim_bone(
                locals_, globals_, forearm, hand, forearm_dir, twist)
        self.last_globals = globals_
        return globals_

    def skinned_arrays(self, globals_):
        """计算每个 primitive 的 CPU 蒙皮顶点与法线。"""
        joint_matrices = np.asarray([
            globals_[node] @ self.inverse_bind[index]
            for index, node in enumerate(self.skin_joints)
        ])
        result = []
        for primitive in self.primitives:
            matrices = joint_matrices[primitive.joints]
            homogeneous = np.concatenate([
                primitive.positions,
                np.ones((len(primitive.positions), 1), dtype=np.float32),
            ], axis=1)
            influenced = np.einsum("nvij,nj->nvi", matrices, homogeneous)
            positions = np.sum(
                influenced * primitive.weights[:, :, None], axis=1)[:, :3]
            normal_parts = np.einsum(
                "nvij,nj->nvi", matrices[:, :, :3, :3], primitive.normals)
            normals = np.sum(
                normal_parts * primitive.weights[:, :, None], axis=1)
            normals /= np.maximum(
                np.linalg.norm(normals, axis=1, keepdims=True), 1e-12)
            result.append((
                np.ascontiguousarray(positions, dtype=np.float32),
                np.ascontiguousarray(normals, dtype=np.float32),
            ))
        return result

    @property
    def triangle_count(self):
        return sum(primitive.triangle_count for primitive in self.primitives)


class HumanGltfRenderer:
    """实时绘制带 IMU 骨骼姿态的人体数字替身。"""

    # glTF: X=人体左右、Y=向上、Z=前后；显示帧: X=右、Y=前、Z=上。
    MODEL_TO_DISPLAY_ROTATION = np.array([
        [1., 0., 0.],
        [0., 0., -1.],
        [0., 1., 0.],
    ])

    def __init__(self, glb_path):
        self.model = HumanGltfModel(glb_path)
        self.last_globals = self.model.last_globals

    @staticmethod
    def root_transform(offset, scale=0.44):
        result = np.eye(4)
        result[:3, :3] = HumanGltfRenderer.MODEL_TO_DISPLAY_ROTATION * scale
        result[:3, 3] = np.asarray(offset, dtype=float)
        return result

    @classmethod
    def direction_to_model(cls, direction):
        direction = np.asarray(direction, dtype=float)
        return cls.MODEL_TO_DISPLAY_ROTATION.T @ direction

    @staticmethod
    def _display_color(material_index, active_side, following):
        # 冷灰蓝主体 + 深色关节，比素材原始橙紫色更贴合仿真界面。
        if material_index == 1:
            color = np.array([0.075, 0.105, 0.145, 1.0])
        else:
            color = np.array([0.32, 0.50, 0.61, 1.0])
            if following:
                color[:3] = color[:3] * 0.72 + np.array([0.12, 0.70, 0.78]) * 0.28
        return color

    def render(self, root_transform, arms=None,
               active_side="right", following=False):
        model_arms = {}
        for side, values in (arms or {}).items():
            if values is None:
                continue
            upper, forearm, twist = values
            model_arms[side] = (
                self.direction_to_model(upper),
                self.direction_to_model(forearm),
                float(twist),
            )
        globals_ = self.model.pose(model_arms)
        self.last_globals = globals_
        arrays = self.model.skinned_arrays(globals_)

        glEnable(GL_LIGHTING)
        glEnable(GL_LIGHT0)
        glEnable(GL_COLOR_MATERIAL)
        glEnable(GL_NORMALIZE)
        glColorMaterial(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE)
        glLightfv(GL_LIGHT0, GL_POSITION, (2.5, -1.5, 3.5, 1.0))
        glLightfv(GL_LIGHT0, GL_AMBIENT, (0.20, 0.23, 0.27, 1.0))
        glLightfv(GL_LIGHT0, GL_DIFFUSE, (0.92, 0.96, 1.0, 1.0))
        glLightfv(GL_LIGHT0, GL_SPECULAR, (0.30, 0.34, 0.38, 1.0))
        glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, (0.22, 0.28, 0.32, 1.0))
        glMaterialf(GL_FRONT_AND_BACK, GL_SHININESS, 20.0)
        glEnableClientState(GL_VERTEX_ARRAY)
        glEnableClientState(GL_NORMAL_ARRAY)

        glPushMatrix()
        glMultMatrixf(np.asarray(root_transform.T, dtype=np.float32))
        for primitive, (positions, normals) in zip(self.model.primitives, arrays):
            color = self._display_color(
                primitive.material, active_side, following)
            glColor4f(*color)
            glVertexPointer(3, GL_FLOAT, 0, positions)
            glNormalPointer(GL_FLOAT, 0, normals)
            glDrawElements(
                GL_TRIANGLES, len(primitive.indices),
                _GL_INDEX_TYPES[primitive.indices.dtype], primitive.indices)
        glPopMatrix()

        glDisableClientState(GL_NORMAL_ARRAY)
        glDisableClientState(GL_VERTEX_ARRAY)
        glDisable(GL_COLOR_MATERIAL)
        glDisable(GL_NORMALIZE)
        glDisable(GL_LIGHT0)
        glDisable(GL_LIGHTING)

    def bone_position(self, root_transform, bone_name):
        index = self.model.node_by_name[bone_name]
        point = root_transform @ self.last_globals[index] @ np.array([0., 0., 0., 1.])
        return point[:3]

    def segment_midpoint(self, root_transform, bone_name, child_name):
        start = self.bone_position(root_transform, bone_name)
        end = self.bone_position(root_transform, child_name)
        return (start + end) * 0.5, end - start
