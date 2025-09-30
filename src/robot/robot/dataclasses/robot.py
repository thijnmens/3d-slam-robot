import math
from dataclasses import dataclass

from .wheel import Wheel


@dataclass
class Robot:
    front_left: Wheel
    front_right: Wheel
    rear_left: Wheel
    rear_right: Wheel
    robot_width: int
    robot_length: int
    wheel_diameter: int
    pulses_per_rev: int
    gear_ratio: float

    def __init__(self):
        # Robot details
        self.robot_width: int = 210  # mm
        self.robot_length: int = 200  # mm
        self.wheel_diameter: int = 60  # mm

        # Encoder details
        self.pulses_per_rev: int = 11
        self.gear_ratio: float = 36 / 1

        # Motor details
        self.front_left = Wheel(
            "front_left",
            True,
            18,
            24,
            23,
            25,
            27,
            (1.5, 2, 0.0)
        )
        self.front_right = Wheel(
            "front_right",
            False,
            12,
            1,
            22,
            16,
            20,
            (0.8, 2, 0.0)
        )
        self.rear_left = Wheel(
            "rear_left",
            False,
            19,
            21,
            26,
            0,
            11,
            (0.8, 2, 0.0)
        )
        self.rear_right = Wheel(
            "rear_right",
            True,
            13,
            6,
            5,
            9,
            10,
            (1.2, 2, 0.0)
        )

        # Parameters
        self.max_wheel_speed = 1
        self.max_pwm = 1
        self.min_pwm = -1

    def __iter__(self):
        yield self.front_left
        yield self.front_right
        yield self.rear_left
        yield self.rear_right
