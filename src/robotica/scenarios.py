"""Cenários de teste e configuração das simulações.

Define os quatro cenários obrigatórios do trabalho para cada robô:

- ``forward``  : deslocamento retilíneo para frente;
- ``reverse``  : deslocamento retilíneo para trás;
- ``left``     : curva/giro para a esquerda;
- ``right``    : curva/giro para a direita.

Cada cenário fornece um comando de entrada ``u(t)`` (segurador de ordem
zero, constante no horizonte) e a postura inicial. Os comandos são
expressos na entrada natural de cada modelo:

- tração diferencial: ``u = [v, omega]``;
- Ackermann:          ``u = [v, phi]``.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from . import models


# ---------------------------------------------------------------------------
# Configuração temporal
# ---------------------------------------------------------------------------
@dataclass(frozen=True)
class SimConfig:
    """Horizonte e passo da simulação."""

    duration: float = 8.0  # [s]
    dt: float = 0.02  # [s]

    @property
    def t_grid(self) -> np.ndarray:
        n = int(round(self.duration / self.dt)) + 1
        return np.linspace(0.0, self.duration, n)


SCENARIOS = ("forward", "reverse", "left", "right")

SCENARIO_LABELS = {
    "forward": "Retilíneo para frente",
    "reverse": "Retilíneo para trás",
    "left": "Curva para a esquerda",
    "right": "Curva para a direita",
}


# ---------------------------------------------------------------------------
# Robôs
# ---------------------------------------------------------------------------
@dataclass
class Robot:
    """Associa um modelo, seus parâmetros e um identificador."""

    name: str
    label: str
    deriv: callable
    params: object
    pose0: np.ndarray
    commands: dict  # scenario -> u (array shape (2,))


def _diff_drive_robot() -> Robot:
    params = models.DiffDriveParams(wheel_radius=0.05, track_width=0.30)
    v = 0.5  # [m/s]
    omega = 0.6  # [rad/s]
    commands = {
        "forward": np.array([v, 0.0]),
        "reverse": np.array([-v, 0.0]),
        # Avança e gira: omega > 0 => giro anti-horário (esquerda).
        "left": np.array([v, omega]),
        "right": np.array([v, -omega]),
    }
    return Robot(
        name="diffdrive",
        label="Tração diferencial",
        deriv=models.diff_drive_deriv,
        params=params,
        pose0=np.array([0.0, 0.0, 0.0]),
        commands=commands,
    )


def _ackermann_robot() -> Robot:
    params = models.AckermannParams(wheelbase=0.50, track_width=0.30)
    v = 0.5  # [m/s]
    phi = np.deg2rad(25.0)  # [rad]
    commands = {
        "forward": np.array([v, 0.0]),
        "reverse": np.array([-v, 0.0]),
        # phi > 0 => rodas para a esquerda => giro anti-horário indo p/ frente.
        "left": np.array([v, phi]),
        "right": np.array([v, -phi]),
    }
    return Robot(
        name="ackermann",
        label="Direção de Ackermann",
        deriv=models.ackermann_deriv,
        params=params,
        pose0=np.array([0.0, 0.0, 0.0]),
        commands=commands,
    )


def build_robots() -> list[Robot]:
    """Retorna a lista de robôs simulados."""
    return [_diff_drive_robot(), _ackermann_robot()]


def command_signal(robot: Robot, scenario: str, t_grid: np.ndarray) -> np.ndarray:
    """Constrói o sinal de comando ``u(t)`` shape (N, 2) para um cenário.

    Aqui os comandos são constantes no tempo; a função existe para
    deixar explícita a interface "velocidades de entrada ao longo do
    tempo" exigida pela cinemática direta e para facilitar a extensão
    a perfis variáveis no tempo.
    """
    u_const = robot.commands[scenario]
    return np.tile(u_const, (t_grid.shape[0], 1))
