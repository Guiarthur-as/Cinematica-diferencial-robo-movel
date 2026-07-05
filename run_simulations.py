"""Orquestrador de simulações — gera todas as figuras do relatório.

Executa, para cada robô (tração diferencial e Ackermann) e cada cenário
(frente, ré, esquerda, direita), a simulação da cinemática direta via
integração numérica e salva:

- ``outputs/{robo}_{cenario}_traj.png``   : trajetória no plano (X, Y);
- ``outputs/{robo}_{cenario}_states.png`` : evolução de x(t), y(t), theta(t).

Uso
---
    uv run python run_simulations.py                 # gera tudo (RK4)
    uv run python run_simulations.py --integrator euler
    uv run python run_simulations.py --out figuras
    uv run python run_simulations.py --check         # verifica as saídas

Como o repositório usa layout ``src/``, este script insere ``src`` no
``sys.path`` para funcionar mesmo sem instalação editável.
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent
SRC = ROOT / "src"
if str(SRC) not in sys.path:
    sys.path.insert(0, str(SRC))

from robotica import integrators, scenarios  # noqa: E402
from robotica import plotting  # noqa: E402


def run(out_dir: Path, method: str) -> list[Path]:
    """Gera todas as figuras e retorna a lista de caminhos criados."""
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

            scen_label = scenarios.SCENARIO_LABELS[scen]
            title = f"{robot.label} — {scen_label}"

            traj_path = out_dir / f"{robot.name}_{scen}_traj.png"
            states_path = out_dir / f"{robot.name}_{scen}_states.png"
            plotting.plot_trajectory(traj, title, traj_path)
            plotting.plot_states(traj, t, title, states_path)
            created.extend([traj_path, states_path])
            print(f"  [{method}] {robot.name:9s} {scen:8s} -> "
                  f"{traj_path.name}, {states_path.name}")

    return created


def expected_files(out_dir: Path) -> list[Path]:
    files = []
    for robot_name in ("diffdrive", "ackermann"):
        for scen in scenarios.SCENARIOS:
            files.append(out_dir / f"{robot_name}_{scen}_traj.png")
            files.append(out_dir / f"{robot_name}_{scen}_states.png")
    return files


def check(out_dir: Path) -> bool:
    """Verifica que todas as figuras esperadas existem e não estão vazias."""
    ok = True
    for f in expected_files(out_dir):
        if not f.exists() or f.stat().st_size == 0:
            print(f"  FALTANDO/VAZIO: {f}")
            ok = False
    print("Verificação:", "OK" if ok else "FALHOU")
    return ok


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description="Simulações de robôs móveis.")
    parser.add_argument("--out", default="outputs", help="diretório de saída")
    parser.add_argument(
        "--integrator", choices=["euler", "rk4"], default="rk4",
        help="método de integração (padrão: rk4)",
    )
    parser.add_argument(
        "--check", action="store_true",
        help="apenas verifica as figuras existentes (não gera)",
    )
    args = parser.parse_args(argv)
    out_dir = (ROOT / args.out).resolve()

    if args.check:
        return 0 if check(out_dir) else 1

    print(f"Gerando figuras em: {out_dir}  (integrador: {args.integrator})")
    created = run(out_dir, args.integrator)
    print(f"\n{len(created)} figuras geradas.")
    return 0 if check(out_dir) else 1


if __name__ == "__main__":
    raise SystemExit(main())
