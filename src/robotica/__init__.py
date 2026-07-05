"""Pacote de cinemática de robôs móveis.

Modelos cinemáticos diferenciais (tração diferencial e Ackermann),
integradores numéricos (Euler e Runge-Kutta 4) e utilitários de
simulação/plotagem para o trabalho de Cinemática Diferencial e
Simulação de Robôs Móveis.
"""

from . import models, integrators, scenarios

__all__ = ["models", "integrators", "scenarios"]
__version__ = "0.1.0"
