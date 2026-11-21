# Legacy compatibility wrapper.
#
# The Two-Tool Adaptive Mesh (TAM) calibration module is implemented in
# tam; this wrapper provides a stable config section name.

from .tam import BFPDualCalibration, load_config  # noqa: F401

