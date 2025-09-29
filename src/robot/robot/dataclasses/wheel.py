import math
from dataclasses import dataclass

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

    def __init__(self, forward: bool, pwm: int, in1: int, in2: int, encoder_a: int, encoder_b: int):
        self.forward = forward
        self.pulses = 0
        self.pwm = pwm
        self.in1 = in1
        self.in2 = in2
        self.encoder_a = encoder_a
        self.encoder_b = encoder_b