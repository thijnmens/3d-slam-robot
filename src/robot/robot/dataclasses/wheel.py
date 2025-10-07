import math
from dataclasses import dataclass
from typing import Tuple

@dataclass
class Wheel:
    """
    Keeps data on each wheel and contains functions for wheel calculations
    """
    forward: bool
    pulses: int
    pwm: int
    in1: int
    in2: int
    encoder_a: int
    encoder_b: int
    pid: Tuple[float, float, float]

    def __init__(self, name:str, forward: bool, pwm: int, in1: int, in2: int, encoder_a: int, encoder_b: int, pid: Tuple[float, float, float]):
        self.name = name
        self.forward = forward
        self.pulses = 0
        self.pwm = pwm
        self.in1 = in1
        self.in2 = in2
        self.encoder_a = encoder_a
        self.encoder_b = encoder_b
        self.pid = pid