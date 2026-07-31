"""Motion subsystem: pure planning math (s_curve/layer_blender/deadband),
joint metadata/unit conversions (joints), and the 50 Hz MotionService."""

from .joints import JOINTS, JointInfo  # noqa: F401
from .service import MotionService  # noqa: F401
