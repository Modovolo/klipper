# Legacy compatibility wrapper.
#
# The math helper for TAM is implemented in tam_mesh_math; this wrapper
# provides a stable import path.

from .tam_mesh_math import gantry_weights, solve_offsets  # noqa: F401

