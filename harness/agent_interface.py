"""VESC Motor Engineer Agent Interface & Facade (PROP-SUB-05).

Unified high-level facade combining parameter calculation, safety validation,
simulation/hardware evaluation, continuous self-learning, and export into an
immutable, deterministic API.
"""

import sys
import json
from pathlib import Path
from typing import Dict, Any, Optional, List, Union, Tuple

try:
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
    from .eval_harness import EvalHarness, EvaluationReport
    from .learning_loop import VESCLearningLoop, ExperimentRecord, TuningRecommendation
    from .hub_connector import HubConnector
except (ImportError, ValueError):
    import sys
    from pathlib import Path
    harness_dir = Path(__file__).resolve().parent
    if str(harness_dir.parent) not in sys.path:
        sys.path.insert(0, str(harness_dir.parent))
    from harness.vesc_config_engine import (
        MotorSpec,
        HardwareLimits,
        OperatingEnvelope,
        VESCConfigEngine,
        SafetyViolationError
    )
    from harness.auto_tuner import (
        VESCAutoTuner,
        OperatingMode,
        VESCHardwareTier,
        MOTOR_LIBRARY,
        HARDWARE_TIERS
    )
    from harness.eval_harness import EvalHarness, EvaluationReport
    from harness.learning_loop import VESCLearningLoop, ExperimentRecord, TuningRecommendation
    from harness.hub_connector import HubConnector

# Type aliases for clean annotations
Tuple_Result = Tuple[bool, List[str]]
Tuple_Eval = Tuple[EvaluationReport, ExperimentRecord]


class VESCMotorEngineerHarness:
    """Master Agent Harness for VESC Brushless Motor Engineering."""

    def __init__(
        self,
        motor: Optional[Union[str, MotorSpec]] = None,
        hardware: Optional[Union[VESCHardwareTier, HardwareLimits]] = None,
        envelope: Optional[OperatingEnvelope] = None
    ):
        # Resolve motor
        if isinstance(motor, str):
            self.motor = MOTOR_LIBRARY.get(motor.upper().replace(" ", "_"), MOTOR_LIBRARY["SUNNYSKY_V4006_740KV"])
        elif isinstance(motor, MotorSpec):
            self.motor = motor
        else:
            self.motor = MOTOR_LIBRARY["SUNNYSKY_V4006_740KV"]

        # Resolve hardware
        if isinstance(hardware, VESCHardwareTier):
            self.hardware = HARDWARE_TIERS[hardware]
        elif isinstance(hardware, HardwareLimits):
            self.hardware = hardware
        else:
            self.hardware = HARDWARE_TIERS[VESCHardwareTier.VESC_6_MKV]

        self.envelope = envelope or OperatingEnvelope()
        self.tuner = VESCAutoTuner(self.motor, self.hardware, self.envelope)
        self.learning_loop = VESCLearningLoop()

    def generate_profile(self, mode: str = "flight_maxload") -> Dict[str, Any]:
        """Generates a complete VESC profile for the selected mode."""
        op_mode = OperatingMode(mode) if mode in [m.value for m in OperatingMode] else OperatingMode.FLIGHT_MAXLOAD
        return self.tuner.tune_for_mode(op_mode)

    def generate_propeller_profile(
        self,
        propeller_diameter_inch: float = 14.0,
        propeller_pitch_inch: float = 4.8,
        mode: str = "flight_maxload"
    ) -> Dict[str, Any]:
        """Generates a complete VESC profile optimized for a specific propeller."""
        op_mode = OperatingMode(mode) if mode in [m.value for m in OperatingMode] else OperatingMode.FLIGHT_MAXLOAD
        return self.tuner.tune_with_propeller(propeller_diameter_inch, propeller_pitch_inch, op_mode)

    def validate(self, config: Dict[str, Any]) -> Tuple_Result:
        """Validates configuration against all aerospace invariants."""
        violations = VESCConfigEngine.validate_configuration(config, self.hardware)
        return len(violations) == 0, violations

    def run_evaluation(
        self,
        config: Dict[str, Any],
        live_port: Optional[str] = None
    ) -> Tuple_Eval:
        """Evaluates profile in Mock simulation or Live hardware, logging results into the learning loop."""
        live_iface = None
        if live_port:
            from lib.vesc_interface import VESCInterface
            live_iface = VESCInterface(port=live_port, baudrate=115200)
            if not live_iface.connect():
                raise ConnectionError(f"Could not connect to VESC on {live_port}")

        try:
            harness = EvalHarness(config=config, live_interface=live_iface)
            report = harness.run_full_suite()
            record = self.learning_loop.record_run(config, report)
            return report, record
        finally:
            if live_iface:
                live_iface.disconnect()

    def export(self, config: Dict[str, Any], output_dir: Union[str, Path]) -> Dict[str, str]:
        """Exports profile to JSON, MCConfiguration XML, and APPConfiguration XML."""
        out = Path(output_dir)
        out.mkdir(parents=True, exist_ok=True)

        meta = config.get("metadata", {})
        p_name = meta.get("profile_name", "vesc_profile")

        json_path = out / f"{p_name}.json"
        with open(json_path, "w", encoding="utf-8") as f:
            json.dump(config, f, indent=2)

        mc_xml = VESCConfigEngine.export_mcconf_xml(config)
        app_xml = VESCConfigEngine.export_appconf_xml(config)

        mc_path = out / f"{p_name}_mcconf.xml"
        app_path = out / f"{p_name}_appconf.xml"

        with open(mc_path, "w", encoding="utf-8") as f:
            f.write(mc_xml)
        with open(app_path, "w", encoding="utf-8") as f:
            f.write(app_xml)

        return {
            "json_profile": str(json_path),
            "mcconf_xml": str(mc_path),
            "appconf_xml": str(app_path)
        }

    def get_universal_harness(self) -> Any:
        """Returns unified UniversalPropulsionHarness instance sharing motor specifications."""
        try:
            from universal_harness import UniversalPropulsionHarness
        except ImportError:
            import sys
            prop_dir = Path(__file__).resolve().parents[1]
            if str(prop_dir) not in sys.path:
                sys.path.insert(0, str(prop_dir))
            from universal_harness import UniversalPropulsionHarness
        return UniversalPropulsionHarness(pole_pairs=self.motor.pole_pairs)


