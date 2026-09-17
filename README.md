# FOC-VESC: Autonomous Tuning & Control Suite for DroneCAN eVTOL Motor Systems

[![NASA SE Standard](https://img.shields.io/badge/Architecture-NASA%20Systems%20Engineering-blue.svg)](https://www.nasa.gov/seh/)
[![DroneCAN / UAVCAN](https://img.shields.io/badge/Bus-DroneCAN%20%2F%20UAVCAN-green.svg)](https://dronecan.github.io/)
[![Safety Verified](https://img.shields.io/badge/Safety%20Invariants-6%2F6%20Enforced-brightgreen.svg)]()
[![Tests](https://img.shields.io/badge/Tests-61%2F61%20PASS-success.svg)]()
[![Hardware](https://img.shields.io/badge/Hardware-VESC%2060%20%2B%20Durandal-orange.svg)]()

Autonomous VESC Field Oriented Control (FOC) tuning, multi-tier parameter synthesis, HIL physics simulation, regression auditing, and DroneCAN / UAVCAN network integration suite for the **CT-2W1 eVTOL Scale 1:6 Prototype** and high-power UAV propulsion systems.

---

## 📌 1. Highlights & Capabilities

- **Universal Multi-Tier Tuning Engine:** Automatically synthesizes optimal FOC parameters (`foc_current_kp`, `foc_current_ki`, `foc_motor_flux_linkage`, `foc_observer_gain`) for arbitrary brushless motors (e.g. Sunnysky V4006, T-Motor U8, U15) across 4 operation tiers:
  - `bench_5a`: Safe lab testing (5A battery clamp, 0.4700 duty clamp, zero regeneration).
  - `flight_25a`: Nominal flight operation (20A continuous battery, 25A motor, -5A brake).
  - `flight_35a_maxload`: Full-envelope flight (27.5A battery, 35A motor/30s, 560W limit).
  - `flight_hv_800v`: High-voltage eVTOL architecture support.
- **6 Iron Hardware Safety Invariants:** Strict mathematical verification preventing DC backfeed, runaway duty cycle, observer destabilization, reverse rotation, and over-power stator burnout.
- **Discrete-Time Physics Simulator (HIL Evaluation):** Discrete motor + VESC plant model verifying speed tracking error ($\le 1.0\%$), settling time ($\le 250\text{ ms}$), and CAN bus watchdog cutoff ($< 300\text{ ms}$).
- **Autonomous Learning & Regression Loop:** Tracks parameter iterations, audits against benchmark baselines, and generates adaptive gain refinements.
- **OpenAPI & Multi-Agent Plugins:** Ready-to-use schemas and adapters for LangGraph, n8n, AutoGen, CrewAI, and Antigravity Subagents.
- **Dual-Interface Bridge:** Direct USB UART (`COM33`) and DroneCAN over Durandal SLCAN (`COM16`).

---

## 📁 2. Repository Layout

```text
FOC-VESC/
├── harness/                           # Universal Autonomous Agent Harness
│   ├── __init__.py                    # Public API exports
│   ├── agent_interface.py             # Master Facade (VESCMotorEngineerHarness)
│   ├── auto_tuner.py                  # Multi-Tier FOC parameter auto-tuner
│   ├── eval_harness.py                # Discrete-time physics simulator & test runner
│   ├── learning_loop.py               # Continuous learning & regression auditor
│   ├── vesc_config_engine.py          # Math synthesizer & 6 Invariant safety filter
│   ├── hub_connector.py               # CLI dispatcher & OpenAPI JSON tool schemas
│   └── plugin_manifest.json           # Agent plugin manifest (LangGraph, n8n, etc.)
├── docs/
│   └── VESC_AGENT_HARNESS_USER_GUIDE.md # Detailed usage manual & integration recipes
├── profiles/                          # Verified motor and UAVCAN profiles
│   ├── v4006_vesc_bench_5a.json
│   ├── v4006_vesc_flight_25a.json
│   ├── v4006_vesc_flight_35a_maxload.json
│   ├── v4006_motor_config.xml         # 1-click XML for VESC Tool
│   └── v4006_app_uavcan.xml           # 1-click UAVCAN XML for VESC Tool
├── lib/
│   ├── vesc_protocol.py               # Binary VESC packet encoder/decoder & CRC16
│   └── vesc_interface.py              # USB and SLCAN interface classes
├── scripts/
│   ├── set_uavcan_duty_mode.py        # Switch UAVCAN throttle mode (Duty vs RPM)
│   ├── vesc_v4006_tuner.py            # CLI monitor, throttle tests, live sweep
│   ├── setup_durandal_slcan.py        # Auto-configure Durandal SLCAN bridge
│   └── export_vesc_xml.py             # Auto-generate XML from SSOT JSON
└── tests/
    ├── test_vesc_protocol.py          # 38 protocol and framing unit tests
    └── test_vesc_harness.py           # 23 agent harness and safety tests
```

---

## 🚀 3. Quick Start (Python & CLI)

### Running All Unit & Harness Tests (61/61 PASS)
```bash
pytest tests/ -v
```

### Python API Example
```python
from harness import VESCMotorEngineerHarness

harness = VESCMotorEngineerHarness()

# 1. Synthesize 35A Flight Profile with Duty Cycle Control for DroneCAN
profile = harness.tune_motor(
    motor_name="Sunnysky_V4006_740KV",
    tier="flight_35a_maxload",
    battery_cells=6,
    uavcan_esc_index=0,
    uavcan_raw_mode=2  # Duty Cycle Control
)

# 2. Verify in Discrete Physics Simulation
eval_result = harness.evaluate_profile(profile, target_rpm=3000.0)
assert eval_result["passed"] == True
print(f"Speed Error: {eval_result['speed_error_pct']:.2f}% | Watchdog Cutoff: {eval_result['watchdog_cutoff_time_ms']:.1f}ms")
```

### CLI Commands (`hub_connector.py`)
```bash
# Query Motor SSOT
python -m harness.hub_connector spec --motor-name Sunnysky_V4006_740KV

# Auto-tune for Flight Maxload
python -m harness.hub_connector tune --motor-name Sunnysky_V4006_740KV --tier flight_35a_maxload --cells 6 --out-json profiles/v4006_flight_35a.json

# Evaluate candidate profile
python -m harness.hub_connector evaluate --profile profiles/v4006_flight_35a.json

# Audit against benchmark baseline
python -m harness.hub_connector audit --candidate profiles/v4006_flight_35a.json --benchmark profiles/v4006_vesc_flight_25a.json
```

---

## 🚁 4. DroneCAN / UAVCAN Setup & Troubleshooting

### DroneCAN GUI Tool: Why didn't the motor spin?
1. **Safety Checkbox (Command Broadcast):** In DroneCAN GUI Tool (`Tool` -> `ESC Management`), you **MUST check `Command broadcast enabled`**. If unchecked, the slider moves on screen but NO CAN packets are sent.
2. **Channel Index:** VESC is configured to `uavcan_esc_index = 0`. Move **Slider 1 (ESC 1 / Index 0)**.
3. **Throttle Mode (Duty vs RPM):**
   - In RPM Mode (`uavcan_raw_mode = 3`), low slider positions ($< 2.5\%$) command speeds below `s_pid_min_erpm` (1800 ERPM), which VESC drops. Sensorless FOC at standstill also suffers from cogging stiction in pure speed closed loop.
   - In **Duty Cycle Mode (`uavcan_raw_mode = 2`)**, the slider directly sets PWM duty ($0.0 - 1.0$), triggering VESC's sensorless open-loop ramp for instant, smooth breakaway.

### Switch VESC to Duty Cycle Mode (1-Click)
```bash
python scripts/set_uavcan_duty_mode.py --port COM33 --mode duty
```

---

## 🛡️ 5. Hardware Safety Guarantees

- **Zero Backfeed Invariant:** `l_in_current_min = 0.0 A` prevents reverse current into bench supplies.
- **Current Clamps:** Bench $\le 5.0\text{ A}$, Flight Nominal $\le 25.0\text{ A}$, Flight Maxload $\le 35.0\text{ A} / 30\text{s}$.
- **Bench Duty Ceiling:** Hard-clamped at $0.4700$ to prevent accidental high-speed runaway.

---

## 📄 License & Standards

Developed under **NASA Systems Engineering Standards** and **SAE ARP4754A / ARP4761A**.  
Licensed under the Apache-2.0 License.
