"""Orquestrador de animações — gera GIFs das trajetórias para o vídeo.

Para cada robô (tração diferencial e Ackermann) e cada cenário (frente,
ré, esquerda, direita), integra a cinemática direta e salva uma animação:

    outputs/animations/{robo}_{cenario}.gif

Uso
---
    uv run python run_animations.py                    # gera todos os GIFs
    uv run python run_animations.py --integrator euler
    uv run python run_animations.py --out outputs/animations
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parent
SRC = ROOT / "src"
if str(SRC) not in sys.path:
    sys.path.insert(0, str(SRC))

from robotica import animation, integrators, models, scenarios  # noqa: E402


def make_hero_banner(out_dir: Path, method: str) -> Path:
    """Gera um banner horizontal: robô seguindo um trajeto sinuoso.

    Usa o robô diferencial com velocidade linear constante e velocidade
    angular ``omega(t) = A*cos(w t)``, de modo que a orientação oscila
    simetricamente em torno de zero e o robô avança em ``+x`` desenhando um
    caminho serpenteante (formato largo, ideal para banner).
    """
    params = models.DiffDriveParams()
    dt, duration = 0.02, 22.0
    t = np.arange(0.0, duration + dt, dt)

    v = 0.45
    amp = 0.62           # amplitude de omega [rad/s]
    w = 2 * np.pi / 5.0  # frequência (período 5 s)
    u = np.column_stack([np.full_like(t, v), amp * np.cos(w * t)])

    traj = integrators.integrate(
        models.diff_drive_deriv, [0.0, 0.0, 0.0], u, t, params, method=method
    )

    out_dir.mkdir(parents=True, exist_ok=True)
    path = out_dir / "hero_banner.gif"
    animation.animate_trajectory(
        traj, title=None, out_path=path,
        n_frames=150, fps=25, figsize=(12.0, 3.2),
        show_axes=False, tri_size=0.18,
    )
    return path


def run(out_dir: Path, method: str) -> list[Path]:
    out_dir.mkdir(parents=True, exist_ok=True)
    cfg = scenarios.SimConfig()
    t = cfg.t_grid
    created: list[Path] = []

    for robot in scenarios.build_robots():
        for scen in scenarios.SCENARIOS:
            u = scenarios.command_signal(robot, scen, t)
            traj = integrators.integrate(
                robot.deriv, robot.pose0, u, t, robot.params, method=method
            )
            title = f"{robot.label} — {scenarios.SCENARIO_LABELS[scen]}"
            path = out_dir / f"{robot.name}_{scen}.gif"
            animation.animate_trajectory(traj, title, path)
            created.append(path)
            print(f"  [{method}] {robot.name:9s} {scen:8s} -> {path.name}")

    return created


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description="Animações de robôs móveis.")
    parser.add_argument("--out", default="outputs/animations",
                        help="diretório de saída dos GIFs")
    parser.add_argument("--integrator", choices=["euler", "rk4"], default="rk4",
                        help="método de integração (padrão: rk4)")
    args = parser.parse_args(argv)
    out_dir = (ROOT / args.out).resolve()

    print(f"Gerando animações em: {out_dir}  (integrador: {args.integrator})")
    created = run(out_dir, args.integrator)
    hero = make_hero_banner(out_dir, args.integrator)
    print(f"  [{args.integrator}] hero banner -> {hero.name}")
    print(f"\n{len(created) + 1} animações geradas.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
