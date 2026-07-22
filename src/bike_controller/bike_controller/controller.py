"""Nominal continuous-time state-feedback controllers."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Iterable

import numpy as np
from scipy.linalg import solve_continuous_are
from scipy.signal import place_poles


ArrayLike = np.ndarray | Iterable[float] | float


@dataclass(frozen=True)
class _Matrices:
    A: np.ndarray
    B: np.ndarray
    Cm: np.ndarray
    Co: np.ndarray
    dt: float


@dataclass(frozen=True)
class EquilibriumPoint:
    """A dynamically consistent nominal steady-state operating point."""

    state: np.ndarray
    control: np.ndarray
    output: np.ndarray
    residual_norm: float
    is_unique: bool


class NominalStateFeedbackController:
    """Full-state feedback controller without disturbance observation."""

    def __init__(
        self,
        sys: Any,
        method: str = "place_multiple_poles",
        *,
        u_min: ArrayLike | None = None,
        u_max: ArrayLike | None = None,
        equilibrium_tolerance: float = 1e-8,
        **kwargs: Any,
    ) -> None:
        self.method = method
        self.gain_kwargs = dict(kwargs)
        self.equilibrium_tolerance = float(equilibrium_tolerance)
        self._set_matrices(sys)
        self.u_min = self._as_limit(u_min, "u_min")
        self.u_max = self._as_limit(u_max, "u_max")
        if np.any(self.u_min > self.u_max):
            raise ValueError("u_min must not be greater than u_max.")
        self.updateGain(method, **kwargs)
        self.resetState()

    def _set_matrices(self, sys: Any) -> None:
        A = np.asarray(sys.A, dtype=float)
        B = np.asarray(sys.Bu, dtype=float)
        Cm = np.asarray(sys.Cm, dtype=float)
        Co = np.asarray(sys.Co, dtype=float)
        dt = float(getattr(sys, "dt", 0.0))

        if A.ndim != 2 or A.shape[0] != A.shape[1]:
            raise ValueError("A must be a square matrix.")
        nx = A.shape[0]
        if B.ndim != 2 or B.shape[0] != nx:
            raise ValueError("Bu must have shape (number of states, number of inputs).")
        if Cm.shape != (nx, nx) or np.linalg.matrix_rank(Cm) != nx:
            raise ValueError("Cm must be a square, invertible full-state measurement map.")
        if Co.ndim != 2 or Co.shape[1] != nx:
            raise ValueError("Co must have one column per state.")

        self.matrices = _Matrices(A.copy(), B.copy(), Cm.copy(), Co.copy(), dt)
        self.A, self.Bu, self.Cm, self.Co, self.dt = (
            self.matrices.A,
            self.matrices.B,
            self.matrices.Cm,
            self.matrices.Co,
            self.matrices.dt,
        )
        self.nx, self.nu, self.no = nx, B.shape[1], Co.shape[0]

    def _as_limit(self, value: ArrayLike | None, name: str) -> np.ndarray:
        if value is None:
            fill = -np.inf if name == "u_min" else np.inf
            return np.full((self.nu, 1), fill)
        array = np.asarray(value, dtype=float)
        if array.size == 1:
            return np.full((self.nu, 1), float(array.reshape(-1)[0]))
        try:
            return array.reshape(self.nu, 1)
        except ValueError as exc:
            raise ValueError(f"{name} must be scalar or contain {self.nu} values.") from exc

    @staticmethod
    def _controllability_matrix(A: np.ndarray, B: np.ndarray) -> np.ndarray:
        return np.hstack(
            [np.linalg.matrix_power(A, power) @ B for power in range(A.shape[0])]
        )

    @classmethod
    def _ackermann_siso(
        cls, A: np.ndarray, B: np.ndarray, poles: np.ndarray
    ) -> np.ndarray:
        n = A.shape[0]
        controllability = cls._controllability_matrix(A, B)
        coefficients = np.poly(poles)
        phi_A = np.linalg.matrix_power(A, n)
        for index in range(1, n + 1):
            phi_A += coefficients[index] * np.linalg.matrix_power(A, n - index)

        last_row = np.zeros((1, n))
        last_row[0, -1] = 1.0
        return np.linalg.solve(controllability.T, last_row.T).T @ phi_A

    def updateGain(self, method: str | None = None, **kwargs: Any) -> tuple[np.ndarray, np.ndarray]:
        if method is not None:
            self.method = method
        if kwargs:
            self.gain_kwargs = dict(kwargs)
        options = self.gain_kwargs

        controllability = self._controllability_matrix(self.A, self.Bu)
        if np.linalg.matrix_rank(controllability) != self.nx:
            raise ValueError("The nominal system is not controllable.")

        if self.method == "place_multiple_poles":
            wc = options.get("wc")
            if wc is None:
                raise ValueError("'wc' is required for place_multiple_poles.")
            desired_poles = np.full(self.nx, complex(wc))
            self.K = np.real_if_close(
                self._ackermann_siso(self.A, self.Bu, desired_poles)
            ).astype(float)

        elif self.method == "place_distinct_poles":
            desired_poles = np.asarray(options.get("poles_ctr", ()), dtype=complex)
            self.K = np.real_if_close(
                place_poles(self.A, self.Bu, desired_poles).gain_matrix
            ).astype(float)

        elif self.method == "lqr":
            Q = options.get("Qc")
            R = options.get("Rc")
            if Q is None or R is None:
                raise ValueError("'Qc' and 'Rc' are required for lqr.")
            Q = np.asarray(Q, dtype=float)
            R = np.asarray(R, dtype=float)
            P = solve_continuous_are(self.A, self.Bu, Q, R)
            self.K = np.linalg.solve(R, self.Bu.T @ P)
            desired_poles = np.linalg.eigvals(self.A - self.Bu @ self.K)

        else:
            raise ValueError("Unknown method. Use 'place_multiple_poles' or 'lqr'.")

        self.poles = np.asarray(desired_poles)
        self.closed_loop_poles = np.linalg.eigvals(self.A - self.Bu @ self.K)
        return self.K.copy(), self.closed_loop_poles.copy()

    def updateSysAndGain(
        self, sys: Any, method: str | None = None, **kwargs: Any
    ) -> tuple[np.ndarray, np.ndarray]:
        old_nu = self.nu
        old_min, old_max = self.u_min.copy(), self.u_max.copy()
        self._set_matrices(sys)
        if self.nu != old_nu:
            raise ValueError("The number of control inputs cannot change.")
        self.u_min, self.u_max = old_min, old_max
        return self.updateGain(method, **kwargs)

    def solve_equilibrium(self, reference: ArrayLike) -> EquilibriumPoint:
        reference = np.asarray(reference, dtype=float).reshape(self.no, 1)
        matrix = np.block(
            [
                [self.A, self.Bu],
                [self.Co, np.zeros((self.no, self.nu))],
            ]
        )
        rhs = np.vstack((np.zeros((self.nx, 1)), reference))
        rank = np.linalg.matrix_rank(matrix)
        unknown_count = self.nx + self.nu
        if matrix.shape[0] == matrix.shape[1] and rank == unknown_count:
            solution = np.linalg.solve(matrix, rhs)
        else:
            solution, _, _, _ = np.linalg.lstsq(matrix, rhs, rcond=None)
        residual = np.linalg.norm(matrix @ solution - rhs)
        state = solution[: self.nx]
        control = solution[self.nx :]
        return EquilibriumPoint(
            state=state.copy(),
            control=control.copy(),
            output=(self.Co @ state).copy(),
            residual_norm=float(residual),
            is_unique=rank == unknown_count,
        )

    def measured_state(self, measured_output: ArrayLike) -> np.ndarray:
        measured_output = np.asarray(measured_output, dtype=float).reshape(self.nx, 1)
        return np.linalg.solve(self.Cm, measured_output)

    def step(
        self, reference: ArrayLike, measured_state: ArrayLike, t: float | None = None
    ) -> np.ndarray:
        del t
        state = self.measured_state(measured_state)
        equilibrium = self.solve_equilibrium(reference)
        unsaturated = equilibrium.control + self.K @ (equilibrium.state - state)
        control = np.clip(unsaturated, self.u_min, self.u_max)
        return control.copy()

    def resetState(self, init_state: ArrayLike | None = None) -> None:
        pass

    update_system_and_gain = updateSysAndGain
    update_gain = updateGain
    reset = resetState


NominalBikeController = NominalStateFeedbackController