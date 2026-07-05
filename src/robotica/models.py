"""Modelos cinemáticos contínuos de robôs móveis.

Cada modelo é uma função pura ``deriv(pose, u, params) -> pose_dot`` que
retorna as derivadas temporais da postura ``(x_dot, y_dot, theta_dot)``.
Essa assinatura desacopla a álgebra do modelo do integrador numérico
(ver :mod:`robotica.integrators`), permitindo trocar Euler por RK4 sem
alterar os modelos.

Convenções
----------
- Postura: ``pose = [x, y, theta]`` em metros e radianos.
- ``theta`` é medido a partir do eixo X global, positivo anti-horário.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np


# ---------------------------------------------------------------------------
# Parâmetros dos robôs
# ---------------------------------------------------------------------------
@dataclass(frozen=True)
class DiffDriveParams:
    """Parâmetros geométricos do robô de tração diferencial.

    Parameters
    ----------
    wheel_radius : raio das rodas ``r`` [m].
    track_width  : distância entre as rodas ``b`` (bitola) [m].
    """

    wheel_radius: float = 0.05
    track_width: float = 0.30


@dataclass(frozen=True)
class AckermannParams:
    """Parâmetros geométricos do robô com direção de Ackermann.

    Parameters
    ----------
    wheelbase : distância entre eixos dianteiro e traseiro ``L`` [m].
    track_width : bitola dianteira ``d`` [m] (usada apenas para deduzir
        os ângulos das rodas interna/externa; não entra nas equações de
        estado do modelo bicicleta).
    max_steer : ângulo de esterçamento máximo ``phi_max`` [rad].
    """

    wheelbase: float = 0.50
    track_width: float = 0.30
    max_steer: float = np.deg2rad(35.0)


# ---------------------------------------------------------------------------
# Tração diferencial
# ---------------------------------------------------------------------------
def diff_drive_deriv(pose, u, params: DiffDriveParams):
    """Cinemática direta (contínua) do robô de tração diferencial.

    Modelo clássico do uniciclo::

        x_dot     = v * cos(theta)
        y_dot     = v * sin(theta)
        theta_dot = omega

    Parameters
    ----------
    pose : array_like shape (3,)
        Postura ``[x, y, theta]``.
    u : array_like shape (2,)
        Entrada ``[v, omega]`` (velocidade linear [m/s] e angular [rad/s]).
    params : DiffDriveParams
        Parâmetros geométricos (não usados nesta forma, mantidos por
        uniformidade de assinatura).

    Returns
    -------
    numpy.ndarray shape (3,)
        Derivada ``[x_dot, y_dot, theta_dot]``.
    """
    theta = pose[2]
    v, omega = u[0], u[1]
    return np.array([v * np.cos(theta), v * np.sin(theta), omega], dtype=float)


def diff_drive_wheel_speeds_to_body(omega_r, omega_l, params: DiffDriveParams):
    """Converte velocidades angulares das rodas em ``(v, omega)`` do corpo.

    A partir do rolamento sem escorregamento de cada roda
    (``v_roda = r * omega_roda``)::

        v     = r * (omega_r + omega_l) / 2
        omega = r * (omega_r - omega_l) / b

    Parameters
    ----------
    omega_r, omega_l : velocidades angulares das rodas direita/esquerda [rad/s].
    params : DiffDriveParams

    Returns
    -------
    (v, omega) : tuple[float, float]
    """
    r = params.wheel_radius
    b = params.track_width
    v = r * (omega_r + omega_l) / 2.0
    omega = r * (omega_r - omega_l) / b
    return v, omega


def diff_drive_body_to_wheel_speeds(v, omega, params: DiffDriveParams):
    """Inversa de :func:`diff_drive_wheel_speeds_to_body`.

    Returns
    -------
    (omega_r, omega_l) : tuple[float, float] [rad/s].
    """
    r = params.wheel_radius
    b = params.track_width
    omega_r = (2.0 * v + omega * b) / (2.0 * r)
    omega_l = (2.0 * v - omega * b) / (2.0 * r)
    return omega_r, omega_l


# ---------------------------------------------------------------------------
# Direção de Ackermann (modelo bicicleta)
# ---------------------------------------------------------------------------
def ackermann_deriv(pose, u, params: AckermannParams):
    """Cinemática direta (contínua) do robô com direção de Ackermann.

    Redução para o modelo bicicleta, com referência no eixo traseiro::

        x_dot     = v * cos(theta)
        y_dot     = v * sin(theta)
        theta_dot = v * tan(phi) / L

    onde ``phi`` é o ângulo de esterçamento da roda dianteira virtual e
    ``L`` é a distância entre eixos. O raio de curvatura instantâneo é
    ``R = L / tan(phi)``.

    Parameters
    ----------
    pose : array_like shape (3,)
        Postura ``[x, y, theta]``.
    u : array_like shape (2,)
        Entrada ``[v, phi]`` (velocidade linear [m/s] e ângulo de
        esterçamento [rad]).
    params : AckermannParams

    Returns
    -------
    numpy.ndarray shape (3,)
        Derivada ``[x_dot, y_dot, theta_dot]``.
    """
    theta = pose[2]
    v, phi = u[0], u[1]
    L = params.wheelbase
    return np.array(
        [v * np.cos(theta), v * np.sin(theta), v * np.tan(phi) / L], dtype=float
    )


def ackermann_turn_radius(phi, params: AckermannParams):
    """Raio de curvatura ``R = L / tan(phi)`` do eixo traseiro [m].

    Retorna ``inf`` quando ``phi`` é (aproximadamente) zero.
    """
    L = params.wheelbase
    if abs(phi) < 1e-12:
        return np.inf
    return L / np.tan(phi)


def ackermann_inner_outer_steer(phi, params: AckermannParams):
    """Ângulos de esterçamento das rodas interna e externa.

    A verdadeira geometria de Ackermann exige que as duas rodas
    dianteiras apontem para o mesmo centro instantâneo de rotação (ICR).
    Com raio do eixo traseiro ``R = L / tan(phi)`` e bitola ``d``::

        tan(phi_interna) = L / (R - d/2)
        tan(phi_externa) = L / (R + d/2)

    Parameters
    ----------
    phi : ângulo de esterçamento equivalente (modelo bicicleta) [rad].
    params : AckermannParams

    Returns
    -------
    (phi_inner, phi_outer) : tuple[float, float] [rad].
        Para ``phi == 0`` retorna ``(0.0, 0.0)``.
    """
    if abs(phi) < 1e-12:
        return 0.0, 0.0
    L = params.wheelbase
    d = params.track_width
    R = L / np.tan(phi)
    R = abs(R)
    phi_inner = np.arctan2(L, R - d / 2.0)
    phi_outer = np.arctan2(L, R + d / 2.0)
    sign = np.sign(phi)
    return sign * phi_inner, sign * phi_outer
