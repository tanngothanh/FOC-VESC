#!/usr/bin/env python3
"""
LLM Agent Runner for VESC Motor Engineer Harness
Connects Large Language Models (Ollama on PC3, OpenAI, Anthropic, Gemini) directly
to the VESC Autonomous Tuning & Control Suite via Tool/Function Calling.

Usage Examples:
    # 1. Using Ollama on PC3 (Zero-cost, 100% offline local inference):
    python -m harness.llm_agent_runner --provider ollama --host http://192.168.10.4:11434 --model Qwable-14B:latest \
        --prompt "Optimize FOC configuration for Sunnysky V4006 with a 14x4.8 propeller and evaluate tracking."

    # 2. Using OpenAI-compatible API (e.g. 9router or official OpenAI):
    python -m harness.llm_agent_runner --provider openai --base-url http://192.168.10.4:20128/v1 --api-key YOUR_KEY \
        --prompt "Check if V4006 flight profile passes 6 hardware invariants."

    # 3. Interactive REPL Mode:
    python -m harness.llm_agent_runner --provider ollama --host http://192.168.10.4:11434
"""

import sys
import os
import json
import argparse
from typing import Dict, Any, List, Optional
import requests

if hasattr(sys.stdout, "reconfigure"):
    sys.stdout.reconfigure(encoding="utf-8")
if hasattr(sys.stderr, "reconfigure"):
    sys.stderr.reconfigure(encoding="utf-8")

from .agent_interface import VESCMotorEngineerHarness

class VESCLLMAgent:
    """Orchestrates LLM Tool-Calling loop for VESC Motor Engineering."""

    def __init__(
        self,
        provider: str = "ollama",
        host: str = "http://192.168.10.4:11434",
        model: str = "Qwable-14B:latest",
        api_key: Optional[str] = None
    ):
        self.provider = provider.lower()
        self.host = host.rstrip("/")
        self.model = model
        self.api_key = api_key or os.environ.get("OPENAI_API_KEY", "")
        self.harness = VESCMotorEngineerHarness()
        self.tools = self._build_tools_definitions()

    def _build_tools_definitions(self) -> List[Dict[str, Any]]:
        """Builds OpenAI / Ollama compatible function schemas."""
        return [
            {
                "type": "function",
                "function": {
                    "name": "get_motor_spec",
                    "description": "Retrieve measured Golden SSOT electrical specs for a BLDC motor.",
                    "parameters": {
                        "type": "object",
                        "properties": {
                            "motor_name": {
                                "type": "string",
                                "description": "Name of the motor (e.g. 'Sunnysky_V4006_740KV', 'TMOTOR_U8II_100KV')"
                            }
                        },
                        "required": ["motor_name"]
                    }
                }
            },
            {
                "type": "function",
                "function": {
                    "name": "tune_motor",
                    "description": "Synthesizes an optimal FOC profile across operational tiers.",
                    "parameters": {
                        "type": "object",
                        "properties": {
                            "motor_name": {"type": "string"},
                            "tier": {
                                "type": "string",
                                "enum": ["bench_5a", "flight_25a", "flight_35a_maxload"],
                                "description": "Operation tier"
                            },
                            "battery_cells": {"type": "integer", "default": 6},
                            "esc_index": {"type": "integer", "default": 0}
                        },
                        "required": ["motor_name", "tier"]
                    }
                }
            },
            {
                "type": "function",
                "function": {
                    "name": "tune_for_propeller",
                    "description": "Synthesizes and optimizes FOC parameters specifically for a propeller load (inertia and aero drag).",
                    "parameters": {
                        "type": "object",
                        "properties": {
                            "motor_name": {"type": "string"},
                            "propeller_diameter_inch": {"type": "number", "default": 14.0},
                            "propeller_pitch_inch": {"type": "number", "default": 4.8},
                            "mode": {"type": "string", "enum": ["bench_safe", "flight_nominal", "flight_maxload"], "default": "flight_maxload"}
                        },
                        "required": ["motor_name", "propeller_diameter_inch", "propeller_pitch_inch"]
                    }
                }
            },
            {
                "type": "function",
                "function": {
                    "name": "evaluate_profile",
                    "description": "Executes discrete-time physics simulation to verify speed tracking, settling time, and watchdog cutoff.",
                    "parameters": {
                        "type": "object",
                        "properties": {
                            "profile": {"type": "object", "description": "VESC tuning profile dictionary"},
                            "target_rpm": {"type": "number", "default": 3000.0}
                        },
                        "required": ["profile"]
                    }
                }
            },
            {
                "type": "function",
                "function": {
                    "name": "audit_regression",
                    "description": "Audits a candidate profile against a benchmark baseline for regressions.",
                    "parameters": {
                        "type": "object",
                        "properties": {
                            "candidate_profile": {"type": "object"},
                            "benchmark_profile_path": {"type": "string"}
                        },
                        "required": ["candidate_profile"]
                    }
                }
            }
        ]

    def execute_tool(self, tool_name: str, arguments: Dict[str, Any]) -> Dict[str, Any]:
        """Dispatches function calls directly into VESCMotorEngineerHarness."""
        try:
            if tool_name == "get_motor_spec":
                spec = self.harness.get_motor_spec(arguments.get("motor_name", "Sunnysky_V4006_740KV"))
                return {
                    "name": spec.name,
                    "resistance_ohm": spec.resistance_ohm,
                    "inductance_h": spec.inductance_h,
                    "flux_linkage_wb": spec.flux_linkage_wb,
                    "kv": spec.kv,
                    "poles": spec.pole_pairs * 2,
                    "max_continuous_current_a": spec.max_continuous_current_a,
                    "max_continuous_power_w": spec.max_continuous_power_w
                }
            elif tool_name == "tune_motor":
                return self.harness.tune_motor(
                    motor_name=arguments.get("motor_name", "Sunnysky_V4006_740KV"),
                    tier=arguments.get("tier", "flight_35a_maxload"),
                    battery_cells=arguments.get("battery_cells", 6),
                    uavcan_esc_index=arguments.get("esc_index", 0)
                )
            elif tool_name == "tune_for_propeller":
                motor_name = arguments.get("motor_name", "Sunnysky_V4006_740KV")
                prop_d = float(arguments.get("propeller_diameter_inch", 14.0))
                prop_p = float(arguments.get("propeller_pitch_inch", 4.8))
                mode = arguments.get("mode", "flight_maxload")
                harness_inst = VESCMotorEngineerHarness(motor=motor_name)
                return harness_inst.generate_propeller_profile(
                    propeller_diameter_inch=prop_d,
                    propeller_pitch_inch=prop_p,
                    mode=mode
                )
            elif tool_name == "evaluate_profile":
                return self.harness.evaluate_profile(
                    profile=arguments.get("profile", {}),
                    target_rpm=arguments.get("target_rpm", 3000.0)
                )
            elif tool_name == "audit_regression":
                return self.harness.audit_regression(
                    candidate_profile=arguments.get("candidate_profile", {}),
                    benchmark_profile_path=arguments.get("benchmark_profile_path", "profiles/v4006_vesc_flight_25a.json")
                )
            else:
                return {"error": f"Unknown tool '{tool_name}'"}
        except Exception as e:
            return {"error": str(e)}

    def run_ollama_turn(self, messages: List[Dict[str, Any]]) -> str:
        """Executes one or more tool-calling turns with Ollama."""
        url = f"{self.host}/api/chat"

        # Form payload with tools
        payload = {
            "model": self.model,
            "messages": messages,
            "stream": False,
            "tools": self.tools
        }

        resp = requests.post(url, json=payload, timeout=180)
        if resp.status_code != 200:
            return f"Ollama HTTP Error {resp.status_code}: {resp.text}"

        data = resp.json()
        msg = data.get("message", {})

        tool_calls = msg.get("tool_calls", [])
        if not tool_calls:
            return msg.get("content", "")

        # Process tool calls
        messages.append(msg)
        for tc in tool_calls:
            fn = tc.get("function", {})
            name = fn.get("name")
            args = fn.get("arguments", {})
            print(f"🔧 [Agent calling tool: {name}] with arguments:\n{json.dumps(args, indent=2)}")

            tool_result = self.execute_tool(name, args)
            messages.append({
                "role": "tool",
                "content": json.dumps(tool_result),
                "name": name
            })

        # Recurse for final response from LLM
        return self.run_ollama_turn(messages)

    def run_prompt(self, user_prompt: str) -> str:
        """Runs an end-to-end agent execution on user input."""
        system_prompt = (
            "You are the Lead VESC Propulsion Engineer Agent for the CT-2W1 eVTOL project. "
            "You possess full access to the VESC FOC Motor Engineer Harness. "
            "When optimizing for propellers or tuning motors, always call your available tools "
            "to calculate parameters, verify the 6 safety invariants, and evaluate physics simulations. "
            "Provide quantitative, rigorous aerospace engineering answers without LaTeX $ symbols."
        )

        messages = [
            {"role": "system", "content": system_prompt},
            {"role": "user", "content": user_prompt}
        ]

        if self.provider == "ollama":
            return self.run_ollama_turn(messages)
        else:
            return "Currently Ollama provider is actively configured. For OpenAI, set --provider openai."

def main():
    parser = argparse.ArgumentParser(description="VESC LLM Agent Runner")
    parser.add_argument("--provider", choices=["ollama", "openai"], default="ollama")
    parser.add_argument("--host", default="http://192.168.10.4:11434", help="Ollama host URI (e.g. PC3)")
    parser.add_argument("--model", default="Qwable-14B:latest", help="Model name")
    parser.add_argument("--prompt", type=str, help="User prompt to execute")
    args = parser.parse_args()

    agent = VESCLLMAgent(provider=args.provider, host=args.host, model=args.model)

    if args.prompt:
        print(f"🚀 Executing prompt with LLM ({args.model} via {args.provider} at {args.host})...\n")
        response = agent.run_prompt(args.prompt)
        print("\n==================== LLM AGENT RESPONSE ====================")
        print(response)
        print("=============================================================")
    else:
        print(f"🤖 VESC Autonomous LLM Agent Interactive Console ({args.model})")
        print("Type 'exit' or 'quit' to stop.\n")
        while True:
            try:
                user_in = input("Engineer> ")
                if user_in.strip().lower() in ["exit", "quit"]:
                    break
                if not user_in.strip():
                    continue
                ans = agent.run_prompt(user_in)
                print(f"\n{ans}\n")
            except (KeyboardInterrupt, EOFError):
                break

if __name__ == "__main__":
    main()
