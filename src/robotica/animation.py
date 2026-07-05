"""Animações das trajetórias para uso no vídeo.

Gera GIFs mostrando o robô (um triângulo orientado por ``theta``)
percorrendo a trajetória integrada, com o rastro do caminho crescendo ao
longo do tempo. Usa o ``PillowWriter`` (Pillow já é dependência do
matplotlib), evitando a necessidade de ffmpeg.
"""

from __future__ import annotations

from pathlib import Path

import matplotlib

matplotlib.use("Agg")  # backend headless; deve vir antes do pyplot
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib.animation import FuncAnimation, PillowWriter  # noqa: E402
from matplotlib.patches import Polygon  # noqa: E402
import numpy as np  # noqa: E402


def _robot_triangle(x, y, theta, size=0.12):
    """Vértices de um triângulo isósceles apontando na direção ``theta``."""
    # Triângulo local: ponta para +x.
    local = np.array([[1.4, 0.0], [-0.8, 0.8], [-0.8, -0.8]]) * size
    c, s = np.cos(theta), np.sin(theta)
    rot = np.array([[c, -s], [s, c]])
    return (local @ rot.T) + np.array([x, y])


#: Cor do triângulo que representa o robô (amarelo com borda preta).
ROBOT_COLOR = "#FFD400"


def animate_trajectory(
    traj: np.ndarray,
    title: str | None,
    out_path: Path,
    n_frames: int = 100,
    fps: int = 20,
    figsize: tuple[float, float] = (5.0, 5.0),
    show_axes: bool = True,
    tri_size: float = 0.12,
    robot_color: str = ROBOT_COLOR,
) -> Path:
    """Cria e salva uma animação GIF da trajetória ``traj`` (N,3).

    Parameters
    ----------
    traj : ndarray (N, 3) com colunas [x, y, theta].
    title : título do gráfico (ou ``None`` para omitir).
    out_path : caminho de saída (``.gif``).
    n_frames : número de quadros (subamostra a trajetória).
    fps : quadros por segundo do GIF.
    figsize : tamanho da figura (polegadas). Use formato largo para banner.
    show_axes : se ``False``, oculta eixos/grade (visual de banner).
    tri_size : tamanho do triângulo do robô.
    robot_color : cor de preenchimento do triângulo do robô.
    """
    x, y, theta = traj[:, 0], traj[:, 1], traj[:, 2]

    # Subamostragem uniforme dos índices para no máximo n_frames quadros.
    n = traj.shape[0]
    idx = np.unique(np.linspace(0, n - 1, min(n_frames, n)).astype(int))

    # Limites com margem.
    pad = 0.2 + 0.1 * max(np.ptp(x), np.ptp(y))
    xlim = (x.min() - pad, x.max() + pad)
    ylim = (y.min() - pad, y.max() + pad)

    fig, ax = plt.subplots(figsize=figsize)
    ax.set_xlim(*xlim)
    ax.set_ylim(*ylim)
    ax.set_aspect("equal")
    if show_axes:
        ax.grid(True, ls=":", alpha=0.6)
        ax.set_xlabel("x [m]")
        ax.set_ylabel("y [m]")
    else:
        ax.axis("off")
    if title:
        ax.set_title(title)

    # Caminho completo (referência tênue) e rastro crescente.
    ax.plot(x, y, ls="--", color="0.8", lw=1, zorder=1)
    (trail,) = ax.plot([], [], color="#1f77b4", lw=2, zorder=2)
    ax.plot(x[0], y[0], "o", color="green", ms=8, zorder=3)  # início

    robot = Polygon(_robot_triangle(x[0], y[0], theta[0], size=tri_size),
                    closed=True, fc=robot_color, ec="k", zorder=4)
    ax.add_patch(robot)

    def update(k):
        i = idx[k]
        trail.set_data(x[: i + 1], y[: i + 1])
        robot.set_xy(_robot_triangle(x[i], y[i], theta[i], size=tri_size))
        return trail, robot

    anim = FuncAnimation(fig, update, frames=len(idx), interval=1000 / fps,
                         blit=True)
    out_path = Path(out_path)
    fig.tight_layout()
    anim.save(out_path, writer=PillowWriter(fps=fps))
    plt.close(fig)
    return out_path
