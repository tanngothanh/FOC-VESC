"""VESC Multi-Agent Hub & Tool Connector (PROP-SUB-05).

Provides unified OpenAPI and JSON-Schema tool definitions and execution dispatchers
compatible with:
- LangGraph / LangChain agents
- n8n Workflow Webhooks and Function nodes
- OpenAI / Gemini / Claude tool calling protocols
- Direct Command-Line Interface (CLI)
"""

import json
import sys
from typing import Dict, Any, List, Optional
from pathlib import Path

try:
    from .vesc_config_engine import (
        MotorSpec,
        HardwareLimits,
        OperatingEnvelope,
        VESCConfigEngine
    )
    from .auto_tuner import (
        VESCAutoTuner,
        OperatingMode,
        VESCHardwareTier,
        MOTOR_LIBRARY,
        HARDWARE_TIERS
    )
    from .eval_harness import EvalHarness
    from .learning_loop import VESCLearningLoop
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
        VESCConfigEngine
    )
    from harness.auto_tuner import (
        VESCAutoTuner,
        OperatingMode,
        VESCHardwareTier,
        MOTOR_LIBRARY,
        HARDWARE_TIERS
    )
    from harness.eval_harness import EvalHarness
    from harness.learning_loop import VESCLearningLoop


class HubConnector:
    """Universal Plugin Hub Connector."""

    @staticmethod
    def get_tool_schemas() -> List[Dict[str, Any]]:
        """Returns standard declarative tool schemas for multi-agent platforms."""
        return [
            {
                "name": "vesc_calculate_config",
                "description": "Calculates optimal VESC FOC parameters, current limits, and anti-runaway duty clamps for a given motor specification.",
                "parameters": {
                    "type": "object",
                    "properties": {
                        "motor_name": {"type": "string", "description": "Motor model identifier (e.g. 'Sunnysky V4006 740KV')"},
                        "poles": {"type": "integer", "description": "Total rotor poles (e.g. 24)"},
                        "slots": {"type": "integer", "description": "Total stator slots (e.g. 18)"},
                        "kv": {"type": "number", "description": "Nominal velocity constant in RPM/V (e.g. 740.0)"},
                        "r_phase_ohm": {"type": "number", "description": "Phase resistance in Ohms (e.g. 0.047)"},
                        "l_phase_h": {"type": "number", "description": "Phase inductance in Henry (e.g. 1.413e-5)"},
                        "v_bus_nominal": {"type": "number", "description": "Operating DC bus voltage (e.g. 20.5)"},
                        "i_dc_max": {"type": "number", "description": "DC supply max current in Amps (e.g. 27.5)"},
                        "i_motor_max": {"type": "number", "description": "Motor max continuous current in Amps (e.g. 35.0)"},
                        "max_mech_rpm": {"type": "number", "description": "Maximum flight operating mechanical RPM (e.g. 6000.0)"},
                        "min_mech_rpm": {"type": "number", "description": "Minimum flight operating mechanical RPM (e.g. 1000.0)"},
                        "mode": {
                            "type": "string",
                            "enum": ["bench_safe", "flight_nominal", "flight_maxload"],
                            "description": "Operational deployment mode"
                        }
                    },
                    "required": ["motor_name", "poles", "kv", "r_phase_ohm", "l_phase_h"]
                }
            },
            {
                "name": "vesc_validate_config",
                "description": "Audits a VESC configuration against strict physical and aerospace safety invariants (Anti-HFI, Anti-Backfeed, Anti-Runaway).",
                "parameters": {
                    "type": "object",
                    "properties": {
                        "config": {"type": "object", "description": "VESC configuration dictionary"}
                    },
                    "required": ["config"]
                }
            },
            {
                "name": "vesc_run_eval_suite",
                "description": "Executes full aerospace verification test suite on High-Fidelity Mock Simulator or Live Hardware.",
                "parameters": {
                    "type": "object",
                    "properties": {
                        "config": {"type": "object", "description": "VESC configuration dictionary to evaluate"},
                        "live_port": {"type": "string", "description": "Optional serial port for live hardware (e.g. 'COM33')"}
                    },
                    "required": ["config"]
                }
            },
            {
                "name": "vesc_get_learning_history",
                "description": "Retrieves statistical benchmarks, regression alerts, and tuning recommendations from the continuous learning loop.",
                "parameters": {
                    "type": "object",
                    "properties": {}
                }
            },
            {
                "name": "vesc_export_xml",
                "description": "Exports a VESC profile to VESC Tool MCConfiguration and APPConfiguration XML format.",
                "parameters": {
                    "type": "object",
                    "properties": {
                        "config": {"type": "object", "description": "VESC configuration dictionary"},
                        "output_dir": {"type": "string", "description": "Target directory to write XML files"}
                    },
                    "required": ["config", "output_dir"]
                }
            }
        ]

    @classmethod
    def dispatch(cls, tool_name: str, arguments: Dict[str, Any]) -> Dict[str, Any]:
        """Dispatches a tool call with given arguments and returns a structured JSON response."""
        try:
            if tool_name == "vesc_calculate_config":
                motor = MotorSpec(
                    name=arguments.get("motor_name", "Custom Motor"),
                    poles=int(arguments.get("poles", 24)),
                    slots=int(arguments.get("slots", 18)),
                    kv=float(arguments.get("kv", 740.0)),
                    r_phase_ohm=float(arguments.get("r_phase_ohm", 0.047)),
                    l_phase_h=float(arguments.get("l_phase_h", 1.413e-5)),
                    i_max_cont=float(arguments.get("i_motor_max", 35.0)),
                    p_max_cont=float(arguments.get("p_max_cont", 560.0))
                )
                hw = HardwareLimits(
                    v_bus_nominal=float(arguments.get("v_bus_nominal", 20.5)),
                    i_dc_max=float(arguments.get("i_dc_max", 27.5)),
                    i_dc_min=0.0,
                    i_phase_max=float(arguments.get("i_motor_max", 35.0))
                )
                env = OperatingEnvelope(
                    min_mech_rpm=float(arguments.get("min_mech_rpm", 1000.0)),
                    max_mech_rpm=float(arguments.get("max_mech_rpm", 6000.0)),
                    allow_reverse=False,
                    allow_braking=True
                )
                mode_str = arguments.get("mode", "flight_maxload")
                mode = OperatingMode(mode_str) if mode_str in [m.value for m in OperatingMode] else OperatingMode.FLIGHT_MAXLOAD
                
                tuner = VESCAutoTuner(motor=motor, hardware=hw, envelope=env)
                profile = tuner.tune_for_mode(mode=mode)
                return {"status": "success", "profile": profile}

            elif tool_name == "vesc_validate_config":
                config = arguments.get("config", {})
                violations = VESCConfigEngine.validate_configuration(config)
                return {
                    "status": "success",
                    "valid": len(violations) == 0,
                    "violations": violations
                }

            elif tool_name == "vesc_run_eval_suite":
                config = arguments.get("config", {})
                live_port = arguments.get("live_port")
                live_iface = None
                if live_port:
                    from lib.vesc_interface import VESCInterface
                    live_iface = VESCInterface(port=live_port, baudrate=115200)
                    if not live_iface.connect():
                        return {"status": "error", "message": f"Could not connect to live port {live_port}"}

                try:
                    harness = EvalHarness(config=config, live_interface=live_iface)
                    report = harness.run_full_suite()

                    # Feed into learning loop
                    loop = VESCLearningLoop()
                    record = loop.record_run(config, report)

                    return {
                        "status": "success",
                        "all_passed": report.all_passed,
                        "summary": report.summary,
                        "hardware_target": report.hardware_target,
                        "metrics": [
                            {
                                "name": m.name,
                                "target": m.target,
                                "measured": m.measured,
                                "passed": m.passed,
                                "details": m.details
                            }
                            for m in report.metrics
                        ],
                        "recommendations": record.recommendations
                    }
                finally:
                    if live_iface:
                        live_iface.disconnect()

            elif tool_name == "vesc_get_learning_history":
                loop = VESCLearningLoop()
                summary = loop.get_benchmark_summary()
                return {"status": "success", "benchmark_summary": summary}

            elif tool_name == "vesc_export_xml":
                config = arguments.get("config", {})
                output_dir = Path(arguments.get("output_dir", "."))
                output_dir.mkdir(parents=True, exist_ok=True)

                mc_xml = VESCConfigEngine.export_mcconf_xml(config)
                app_xml = VESCConfigEngine.export_appconf_xml(config)

                mc_path = output_dir / "exported_mcconf.xml"
                app_path = output_dir / "exported_appconf.xml"

                with open(mc_path, "w", encoding="utf-8") as f:
                    f.write(mc_xml)
                with open(app_path, "w", encoding="utf-8") as f:
                    f.write(app_xml)

                return {
                    "status": "success",
                    "mcconf_xml_path": str(mc_path),
                    "appconf_xml_path": str(app_path)
                }

            else:
                return {"status": "error", "message": f"Unknown tool name: {tool_name}"}

        except Exception as e:
            return {"status": "error", "message": str(e)}


if __name__ == "__main__":
    # Simple CLI wrapper for tool calling
    if len(sys.argv) > 2 and sys.argv[1] == "call":
        tool_name = sys.argv[2]
        payload = json.loads(sys.argv[3]) if len(sys.argv) > 3 else {}
        result = HubConnector.dispatch(tool_name, payload)
        print(json.dumps(result, indent=2))
    elif len(sys.argv) > 1 and sys.argv[1] == "schemas":
        print(json.dumps(HubConnector.get_tool_schemas(), indent=2))
    else:
        print("Usage: python -m harness.hub_connector schemas")
        print("       python -m harness.hub_connector call <tool_name> '<json_payload>'")
