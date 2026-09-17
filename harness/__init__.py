"""VESC Autonomous Motor Engineering Harness (PROP-SUB-05).

A modular, extensible aerospace harness for VESC FOC parameter calculation,
hardware safety enforcement, automated evaluation, and continuous learning.
"""

from .vesc_config_engine import (
    MotorSpec,
    HardwareLimits,
    OperatingEnvelope,
    VESCConfigEngine,
    SafetyViolationError
)

from .auto_tuner import (
    VESCAutoTuner,
    OperatingMode,
    VESCHardwareTier,
    MOTOR_LIBRARY,
    HARDWARE_TIERS
)

from .eval_harness import (
    EvalHarness,
    MockVESCHardware,
    EvaluationMetric,
    EvaluationReport
)

from .learning_loop import (
    VESCLearningLoop,
    ExperimentRecord,
    TuningRecommendation
)

from .hub_connector import HubConnector
from .agent_interface import VESCMotorEngineerHarness

__all__ = [
    "MotorSpec",
    "HardwareLimits",
    "OperatingEnvelope",
    "VESCConfigEngine",
    "SafetyViolationError",
    "VESCAutoTuner",
    "OperatingMode",
    "VESCHardwareTier",
    "MOTOR_LIBRARY",
    "HARDWARE_TIERS",
    "EvalHarness",
    "MockVESCHardware",
    "EvaluationMetric",
    "EvaluationReport",
    "VESCLearningLoop",
    "ExperimentRecord",
    "TuningRecommendation",
    "HubConnector",
    "VESCMotorEngineerHarness"
]
