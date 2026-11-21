# Legacy compatibility wrapper.
#
# The Two-Tool Adaptive Mesh (TAM) tracking module is implemented in
# tam_mesh; this wrapper provides a stable config section name.

from .tam_mesh import BFPDualTracking, load_config  # noqa: F401

