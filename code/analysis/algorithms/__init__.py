"""Selectable localization algorithms used by offline replay and simulation."""

from .router import AlgorithmRouter, available_algorithms, register_algorithm, register_algorithm_slot

__all__ = ["AlgorithmRouter", "available_algorithms", "register_algorithm", "register_algorithm_slot"]
