"""Self-contained three-state nominal bicycle model."""

from __future__ import annotations

import os
import yaml
from dataclasses import dataclass
import math
import numpy as np


@dataclass(frozen=True)
class BicycleParameters:
    """Physical parameters of the simplified roll-steer bicycle model."""

    rear_contact_to_com: float = 0.117
    wheelbase: float = 0.28223
    trail: float = 0.0140
    com_height: float = 0.105
    mass: float = 2.218
    roll_inertia: float = 0.02445
    steering_axis_angle: float = 0.179770
    gravity: float = 9.81

    @classmethod
    def from_yaml(cls, file_name="bicycle_params.yaml"):
        file_path = os.path.join(os.path.dirname(__file__), file_name)
        if os.path.exists(file_path):
            with open(file_path, 'r') as f:
                data = yaml.safe_load(f)
            params = data.get('bicycle_parameters', {})
            return cls(**params)
        return cls()

    @property
    def effective_roll_inertia(self) -> float:
        if self.roll_inertia is not None:
            return float(self.roll_inertia)
        return 4.0 / 3.0 * self.mass * self.com_height**2


@dataclass(frozen=True)
class LinearBicycleSystem:
    """Matrix snapshot consumed by ``NominalBikeController``."""

    A: np.ndarray
    Bu: np.ndarray
    Bd: np.ndarray
    Cm: np.ndarray
    Co: np.ndarray
    dt: float

    @property
    def nx(self) -> int:
        return self.A.shape[0]

    @property
    def nu(self) -> int:
        return self.Bu.shape[1]

    @property
    def no(self) -> int:
        return self.Co.shape[0]


class LinearBicycleModel:
    """Speed-scheduled linear roll-steer bicycle model."""

    def __init__(
        self,
        dt: float,
        parameters: BicycleParameters | None = None,
        *,
        track_roll: bool = False,
        min_forward_speed: float = 1.0,
        forward_speed: float = 2.0,
    ) -> None:
        self.dt = float(dt)
        self.parameters = parameters or BicycleParameters.from_yaml()
        self.track_roll = bool(track_roll)
        self.update_system_parameters(
            forward_speed, min_forward_speed=min_forward_speed
        )

    def update_system_parameters(
        self,
        forward_speed: float,
        *,
        gravity: float | None = None,
        min_forward_speed: float = 1.0,
    ) -> LinearBicycleSystem:
        p = self.parameters
        g = p.gravity if gravity is None else float(gravity)
        speed = float(forward_speed)
        minimum = abs(float(min_forward_speed))
        direction = -1.0 if speed < 0.0 else 1.0
        scheduled_speed = direction * max(minimum, abs(speed))

        a = p.rear_contact_to_com
        b = p.wheelbase
        c = p.trail
        h = p.com_height
        m = p.mass
        inertia = p.effective_roll_inertia
        cos_lambda = math.cos(p.steering_axis_angle)

        a1 = m * a * h * scheduled_speed * cos_lambda / (b * inertia)
        a2 = (m * scheduled_speed**2 * h - m * a * c * g) * cos_lambda / (
            b * inertia
        )
        a4 = m * g * h / inertia

        self.forward_speed = speed
        self.scheduled_speed = scheduled_speed
        self.min_forward_speed = minimum
        self.A = np.array(
            [[0.0, 0.0, 0.0], [0.0, 0.0, 1.0], [a2, a4, 0.0]]
        )
        self.Bu = np.array([[1.0], [0.0], [a1]])
        self.Bd = np.array([[0.0], [0.0], [1.0]])
        self.Cm = np.eye(3)
        self.Co = (
            np.array([[0.0, 1.0, 0.0]])
            if self.track_roll
            else np.array([[1.0, 0.0, 0.0]])
        )
        self.sys = LinearBicycleSystem(
            A=self.A.copy(),
            Bu=self.Bu.copy(),
            Bd=self.Bd.copy(),
            Cm=self.Cm.copy(),
            Co=self.Co.copy(),
            dt=self.dt,
        )
        return self.sys

    def updateSysParam(
        self,
        forw_vel: float,
        g: float = 9.81,
        min_forw_vel: float = 1.0,
    ) -> LinearBicycleSystem:
        """Compatibility alias using the names from the original model."""
        return self.update_system_parameters(
            forw_vel, gravity=g, min_forward_speed=min_forw_vel
        )


ScaleBikeModel = LinearBicycleModel