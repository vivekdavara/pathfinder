"""Multi-agent path finding on grid maps (the C++ library, through pybind11).

Everything is defined in the compiled module ``pathfinder._core``; see DESIGN.md for the model and the rules.
"""

from ._core import *  # noqa: F401,F403
from ._core import __version__  # noqa: F401
