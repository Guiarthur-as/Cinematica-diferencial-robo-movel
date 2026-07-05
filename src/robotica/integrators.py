"""Integradores numéricos para a cinemática direta.

Recebem uma função de derivada ``deriv(pose, u, params)`` (ver
:mod:`robotica.models`), uma postura inicial, um comando de entrada
variável no tempo e uma grade temporal, e retornam a trajetória de
postura integrada ``(x(t), y(t), theta(t))``.

Implementa Integração de Euler (1ª ordem) e Runge-Kutta clássico de 4ª
ordem (RK4). O comando ``u`` é tratado como constante dentro de cada
passo (segurador de ordem zero), o que é apropriado para os perfis de
comando por partes usados nas simulações.
"""

from __future__ import annotations

from typing import Callable

import numpy as np


def _as_u_func(u_of_t, t_grid):
    """Normaliza a entrada de comando para uma função ``u(t)``.

    Aceita:
    - callable ``t -> [.., ..]``;
    - array shape (N, m) alinhado a ``t_grid`` (usa segurador de ordem zero);
    - array shape (m,) constante.
    """
    if callable(u_of_t):
        return u_of_t

    u_arr = np.asarray(u_of_t, dtype=float)
    if u_arr.ndim == 1:
        return lambda t: u_arr
    if u_arr.ndim == 2:
        t_grid = np.asarray(t_grid, dtype=float)

        def u_func(t):
            idx = int(np.searchsorted(t_grid, t, side="right") - 1)
            idx = max(0, min(idx, u_arr.shape[0] - 1))
            return u_arr[idx]

        return u_func
    raise ValueError("u_of_t deve ser callable, array (m,) ou array (N, m).")


def integrate(
    deriv: Callable,
    pose0,
    u_of_t,
    t_grid,
    params=None,
    method: str = "rk4",
):
    """Integra a cinemática direta ao longo de ``t_grid``.

    Parameters
    ----------
    deriv : callable
        Função de derivada ``deriv(pose, u, params) -> pose_dot``.
    pose0 : array_like shape (3,)
        Postura inicial ``[x, y, theta]``.
    u_of_t : callable | array_like
        Comando de entrada. Ver :func:`_as_u_func`.
    t_grid : array_like shape (N,)
        Instantes de tempo (não necessariamente uniformes).
    params : objeto de parâmetros do modelo (repassado a ``deriv``).
    method : {"euler", "rk4"}
        Método de integração.

    Returns
    -------
    numpy.ndarray shape (N, 3)
        Trajetória de postura, uma linha por instante de ``t_grid``.
    """
    method = method.lower()
    if method not in ("euler", "rk4"):
        raise ValueError(f"Método desconhecido: {method!r}. Use 'euler' ou 'rk4'.")

    t_grid = np.asarray(t_grid, dtype=float)
    u_func = _as_u_func(u_of_t, t_grid)

    n = t_grid.shape[0]
    traj = np.empty((n, 3), dtype=float)
    traj[0] = np.asarray(pose0, dtype=float)

    for k in range(n - 1):
        t = t_grid[k]
        dt = t_grid[k + 1] - t
        pose = traj[k]
        u = np.asarray(u_func(t), dtype=float)

        if method == "euler":
            traj[k + 1] = pose + dt * deriv(pose, u, params)
        else:  # rk4
            k1 = deriv(pose, u, params)
            k2 = deriv(pose + 0.5 * dt * k1, u, params)
            k3 = deriv(pose + 0.5 * dt * k2, u, params)
            k4 = deriv(pose + dt * k3, u, params)
            traj[k + 1] = pose + (dt / 6.0) * (k1 + 2 * k2 + 2 * k3 + k4)

    return traj
