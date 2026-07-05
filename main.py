"""Ponto de entrada de conveniência.

Delega para o orquestrador de simulações. Equivale a::

    uv run python run_simulations.py
"""

from run_simulations import main

if __name__ == "__main__":
    raise SystemExit(main())
