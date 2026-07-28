from dataclasses import dataclass

import numpy as np

# Right arm origin RPY (from URDF)
SHOULDER_PITCH_ORIGIN_RPY_X = -0.27931
SHOULDER_ROLL_ORIGIN_RPY_X = 0.27925

# Left arm origin RPY (mirrored from right, from URDF)
LEFT_SHOULDER_PITCH_ORIGIN_RPY_X = 0.27931
LEFT_SHOULDER_ROLL_ORIGIN_RPY_X = -0.27925

# --- Right Arm ---
RIGHT_ARM_JOINT_LIMITS = {
    "right_shoulder_pitch": (-3.0892, 2.6704),
    "right_shoulder_roll": (-2.2515, 1.5882),
    "right_shoulder_yaw": (-2.6180, 2.6180),
    "right_elbow": (-1.0472, 2.0944),
    "right_wrist_roll": (-1.9722, 1.9722),
}

RIGHT_ARM_MOTOR_INDEX = {
    "right_shoulder_pitch": 22,
    "right_shoulder_roll": 23,
    "right_shoulder_yaw": 24,
    "right_elbow": 25,
    "right_wrist_roll": 26,
}

RIGHT_ARM_LINK_OFFSETS = {
    "shoulder_roll_origin_xyz": np.array([0.0, -0.038, -0.013831], dtype=float),
    "shoulder_yaw_origin_xyz": np.array([0.0, -0.00624, -0.1032], dtype=float),
    "elbow_origin_xyz": np.array([0.015783, 0.0, -0.080518], dtype=float),
    "wrist_roll_origin_xyz": np.array([0.100, -0.00188791, -0.010], dtype=float),
}

# --- Left Arm ---
LEFT_ARM_JOINT_LIMITS = {
    "left_shoulder_pitch": (-3.0892, 2.6704),
    "left_shoulder_roll": (-1.5882, 2.2515),
    "left_shoulder_yaw": (-2.6180, 2.6180),
    "left_elbow": (-1.0472, 2.0944),
    "left_wrist_roll": (-1.9722, 1.9722),
}

LEFT_ARM_MOTOR_INDEX = {
    "left_shoulder_pitch": 16,
    "left_shoulder_roll": 17,
    "left_shoulder_yaw": 18,
    "left_elbow": 19,
    "left_wrist_roll": 20,
}

LEFT_ARM_LINK_OFFSETS = {
    "shoulder_roll_origin_xyz": np.array([0.0, 0.038, -0.013831], dtype=float),
    "shoulder_yaw_origin_xyz": np.array([0.0, 0.00624, -0.1032], dtype=float),
    "elbow_origin_xyz": np.array([0.015783, 0.0, -0.080518], dtype=float),
    "wrist_roll_origin_xyz": np.array([0.100, 0.00188791, -0.010], dtype=float),
}

HUMAN_ARM_DEFAULT_LENGTH = {
    "upper": 0.30,
    "forearm": 0.26,
}

# --- IMU Fixed IDs ---
RIGHT_UPPER_ARM_FIXED_ID = "right_upper_arm"
RIGHT_FOREARM_FIXED_ID = "right_forearm"
LEFT_UPPER_ARM_FIXED_ID = "left_upper_arm"
LEFT_FOREARM_FIXED_ID = "left_forearm"
CHEST_FIXED_ID = "chest"


def get_joint_limits(side: str) -> dict:
    return LEFT_ARM_JOINT_LIMITS if side == "left" else RIGHT_ARM_JOINT_LIMITS


def get_motor_index(side: str) -> dict:
    return LEFT_ARM_MOTOR_INDEX if side == "left" else RIGHT_ARM_MOTOR_INDEX


def get_link_offsets(side: str) -> dict:
    return LEFT_ARM_LINK_OFFSETS if side == "left" else RIGHT_ARM_LINK_OFFSETS


@dataclass
class JointAngles:
    shoulder_pitch: float
    shoulder_roll: float
    shoulder_yaw: float
    elbow: float
    wrist_roll: float = 0.0
    side: str = "right"

    def as_dict(self) -> dict:
        return {
            f"{self.side}_shoulder_pitch": self.shoulder_pitch,
            f"{self.side}_shoulder_roll": self.shoulder_roll,
            f"{self.side}_shoulder_yaw": self.shoulder_yaw,
            f"{self.side}_elbow": self.elbow,
            f"{self.side}_wrist_roll": self.wrist_roll,
        }

    def as_array(self) -> np.ndarray:
        return np.array(
            [self.shoulder_pitch, self.shoulder_roll, self.shoulder_yaw,
             self.elbow, self.wrist_roll],
            dtype=float,
        )

    def with_side(self, side: str) -> "JointAngles":
        return JointAngles(
            shoulder_pitch=self.shoulder_pitch,
            shoulder_roll=self.shoulder_roll,
            shoulder_yaw=self.shoulder_yaw,
            elbow=self.elbow,
            wrist_roll=self.wrist_roll,
            side=side,
        )
