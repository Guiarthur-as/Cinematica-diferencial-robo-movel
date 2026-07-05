"""Utilitários de plotagem das simulações.

Gera dois tipos de figura por cenário:

- trajetória no plano ``(X, Y)``;
- evolução temporal das variáveis de estado ``x(t)``, ``y(t)``, ``theta(t)``.

Usa o backend ``Agg`` (sem display) para robustez em execução headless
no Windows.
"""

from __future__ import annotations

from pathlib import Path

import matplotlib

matplotlib.use("Agg")  # backend headless; deve vir antes do pyplot
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402


def plot_trajectory(traj: np.ndarray, title: str, out_path: Path) -> Path:
    """Plota a trajetória ``(x, y)`` e salva em ``out_path``.

    Desenha também a orientação inicial/final e marca início e fim.
    """
    x, y, theta = traj[:, 0], traj[:, 1], traj[:, 2]

    fig, ax = plt.subplots(figsize=(5.0, 5.0))
    ax.plot(x, y, "-", color="#1f77b4", lw=2, label="Trajetória")
    ax.plot(x[0], y[0], "o", color="green", ms=9, label="Início")
    ax.plot(x[-1], y[-1], "s", color="red", ms=9, label="Fim")

    # Setas de orientação (início e fim).
    for idx, color in ((0, "green"), (-1, "red")):
        ax.annotate(
            "",
            xy=(x[idx] + 0.15 * np.cos(theta[idx]), y[idx] + 0.15 * np.sin(theta[idx])),
            xytext=(x[idx], y[idx]),
            arrowprops=dict(arrowstyle="->", color=color, lw=1.5),
        )

    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.set_title(title)
    ax.axis("equal")
    ax.grid(True, ls=":", alpha=0.6)
    ax.legend(loc="best", fontsize=8)
    fig.tight_layout()

    out_path = Path(out_path)
    fig.savefig(out_path, dpi=150)
    plt.close(fig)
    return out_path


def plot_states(traj: np.ndarray, t: np.ndarray, title: str, out_path: Path) -> Path:
    """Plota ``x(t)``, ``y(t)`` e ``theta(t)`` em subplots e salva."""
    x, y, theta = traj[:, 0], traj[:, 1], traj[:, 2]

    fig, axes = plt.subplots(3, 1, figsize=(6.0, 6.0), sharex=True)
    axes[0].plot(t, x, color="#1f77b4", lw=2)
    axes[0].set_ylabel("x [m]")
    axes[1].plot(t, y, color="#ff7f0e", lw=2)
    axes[1].set_ylabel("y [m]")
    axes[2].plot(t, np.rad2deg(theta), color="#2ca02c", lw=2)
    axes[2].set_ylabel(r"$\theta$ [graus]")
    axes[2].set_xlabel("t [s]")

    for ax in axes:
        ax.grid(True, ls=":", alpha=0.6)
    axes[0].set_title(title)
    fig.tight_layout()

    out_path = Path(out_path)
    fig.savefig(out_path, dpi=150)
    plt.close(fig)
    return out_path
