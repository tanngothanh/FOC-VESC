# VESC FOC Autonomous Agent Harness — Hướng Dẫn Sử Dụng Toàn Diện

> **Module:** PROP-SUB-05 (Propulsion System) / VESC Subsystem  
> **Plugin ID:** `vesc-motor-engineer-harness`  
> **Phiên bản:** v1.0.0-PROD  
> **Tiêu chuẩn an toàn:** Hardware & Embedded Systems Safety (Quy tắc Thép An toàn Phần cứng)

---

## 📌 1. Tổng Quan Kiến Trúc (Architecture Overview)

Gói **VESC Agent Harness** (`05_Propulsion_System/VESC/harness/`) là một hệ thống tự động hóa khép kín (Autonomous Tuning & Control Suite) được thiết kế theo chuẩn NASA Systems Engineering và PX4-Autopilot. Harness cho phép kỹ sư hoặc AI Agent (LangGraph, n8n, Antigravity Subagent) tự động tổng hợp thông số FOC, kiểm thử vật lý mô phỏng (HIL simulation), audit hồi quy an toàn, và điều phối qua mạng DroneCAN/UAVCAN cho bất kỳ động cơ BLDC và phần cứng VESC nào.

```
+-----------------------------------------------------------------------------------+
|                           VESC Motor Engineer Harness                             |
|                                                                                   |
|  +---------------------+   +---------------------+   +-------------------------+  |
|  | vesc_config_engine  |-->|     auto_tuner      |-->|      eval_harness       |  |
|  | - Motor Spec SSOT   |   | - 4-Tier Presets    |   | - Discrete Physics Sim  |  |
|  | - Math Synthesizer  |   | - Dynamic Scaler    |   | - Settling/Error Check  |  |
|  | - 6 Invariant Check |   | - XML/JSON Export   |   | - Watchdog Cutoff Check |  |
|  +---------------------+   +---------------------+   +-------------------------+  |
|                                                                   |               |
|                                                                   v               |
|  +---------------------+   +---------------------+   +-------------------------+  |
|  |    hub_connector    |<--|   agent_interface   |<--|      learning_loop      |  |
|  | - CLI Dispatcher    |   | - Master Facade     |   | - Experiment Tracker    |  |
|  | - OpenAPI JSON      |   | - Python Top-API    |   | - Regression Audit      |  |
|  | - LangGraph/n8n API |   | - Safe Mode Clamps  |   | - Adaptive Tuning Recs  |  |
|  +---------------------+   +---------------------+   +-------------------------+  |
+-----------------------------------------------------------------------------------+
```

---

## 🛡️ 2. 6 Quy Tắc Thép An Toàn Phần Cứng (Hardware Safety Invariants)

Bất kỳ cấu hình nào trước khi được nạp vào VESC bắt buộc phải vượt qua bộ lọc an toàn `vesc_config_engine.py`:

| Invariant | Tên Quy Tắc | Ràng Buộc Vật Lý | Hậu Quả Nếu Vi Phạm |
|:---|:---|:---|:---|
| **INV-1** | Zero DC Backfeed | `l_in_current_min >= 0.0 A` | Cháy nguồn DC Bench Power Supply khi hãm tái sinh |
| **INV-2** | Anti-Runaway Duty Clamp | `l_max_duty <= 0.4700` (Bench) / `0.9500` (Flight) | Động cơ vọt ga mất kiểm soát làm gãy cánh quạt |
| **INV-3** | Current Loop Tuning | `foc_current_kp > 0`, `foc_current_ki > 0` | Mất đồng bộ pha FOC, phát nhiệt cao và nổ MOSFET |
| **INV-4** | Observer Stability | `foc_motor_flux_linkage > 0`, `foc_observer_gain > 0` | Observer mất bám góc rotor, giật cục và rung lắc |
| **INV-5** | Unidirectional Rotation | `foc_direction_reverse == 0` | Động cơ quay ngược, làm tuột ren tán cánh quạt |
| **INV-6** | Power Limits Matching | `l_in_current_max * v_in <= Max Power (560W)` | Quá công suất danh định, cháy cuộn dây Stator |

---

## 🐍 3. Hướng Dẫn Sử Dụng Qua Python API

Facade `VESCMotorEngineerHarness` là điểm truy cập duy nhất (Single Entry Point) cho mã nguồn Python.

### 3.1 Khởi tạo và tra cứu Motor Spec
```python
from harness import VESCMotorEngineerHarness

# Khởi tạo harness
harness = VESCMotorEngineerHarness()

# Đọc thông số động cơ từ SSOT
spec = harness.get_motor_spec("Sunnysky_V4006_740KV")
print(f"R: {spec.resistance_ohm} Ohm, L: {spec.inductance_h} H, Poles: {spec.pole_pairs * 2}")
```

### 3.2 Tự động tổng hợp Profile FOC theo Tier
```python
# Tổng hợp cấu hình bay tối đa 35A/30s (Maxload)
profile = harness.tune_motor(
    motor_name="Sunnysky_V4006_740KV",
    tier="flight_35a_maxload",
    battery_cells=6,           # 6S LiPo (22.2V - 25.2V)
    uavcan_esc_index=0,
    uavcan_raw_mode=2          # 2 = Duty Cycle Control (Khuyên dùng cho DroneCAN)
)

print(f"Current Max: {profile['mcconf']['l_current_max']} A")
print(f"Battery In Max: {profile['mcconf']['l_in_current_max']} A")
print(f"Duty Clamp: {profile['mcconf']['l_max_duty']}")
```

### 3.3 Chạy Đánh Giá Mô Phỏng Vật Lý (Evaluation Runner)
```python
# Chạy mô phỏng kiểm tra sai số bám tốc độ và watchdog cutoff
eval_result = harness.evaluate_profile(profile, target_rpm=3000.0, sim_time_s=1.0)

print(f"Pass: {eval_result['passed']}")
print(f"Speed Error: {eval_result['speed_error_pct']:.2f}% (Chuẩn hàng không: <= 1.0%)")
print(f"Watchdog Cutoff: {eval_result['watchdog_cutoff_time_ms']:.1f} ms (Yêu cầu: < 300 ms)")
```

### 3.4 Kiểm toán Hồi quy (Regression Audit)
```python
# Kiểm toán xem profile mới có bị suy giảm chất lượng so với chuẩn benchmark không
audit = harness.audit_regression(
    candidate_profile=profile,
    benchmark_profile_path="profiles/v4006_vesc_flight_25a.json"
)
print(f"Audit Passed: {audit['passed']}")
```

---

## 💻 4. Hướng Dẫn Sử Dụng Qua CLI (`hub_connector.py`)

Giao diện dòng lệnh độc lập không phụ thuộc môi trường ngoài:

```powershell
# 1. Tra cứu thông số động cơ:
python -m harness.hub_connector spec --motor-name Sunnysky_V4006_740KV

# 2. Tự động tune cấu hình theo Tier (bench_5a, flight_25a, flight_35a_maxload):
python -m harness.hub_connector tune --motor-name Sunnysky_V4006_740KV --tier flight_35a_maxload --cells 6 --esc-index 0 --out-json profiles/v4006_flight_35a.json

# 3. Đánh giá kiểm thử mô phỏng vật lý trên file JSON profile:
python -m harness.hub_connector evaluate --profile profiles/v4006_flight_35a.json --target-rpm 3500

# 4. Kiểm toán hồi quy (Regression Audit):
python -m harness.hub_connector audit --candidate profiles/v4006_flight_35a.json --benchmark profiles/v4006_vesc_flight_25a.json

# 5. Xuất danh mục Tool Schemas cho AI Multi-Agent:
python -m harness.hub_connector export-tools --out-file harness_tools.json
```

---

## 🌐 5. Tích Hợp LangGraph Multi-Agent

Để nhúng VESC Agent Harness vào mạng lưới Agentic của LangGraph:

```python
from langgraph.prebuilt import ToolNode
from langchain_core.tools import tool
from harness import VESCMotorEngineerHarness

harness = VESCMotorEngineerHarness()

@tool
def vesc_tune_motor(motor_name: str, tier: str, cells: int = 6, esc_index: int = 0) -> str:
    """Tự động tính toán và kiểm tra an toàn cấu hình VESC FOC cho động cơ."""
    import json
    try:
        profile = harness.tune_motor(motor_name=motor_name, tier=tier, battery_cells=cells, uavcan_esc_index=esc_index)
        return json.dumps(profile, indent=2)
    except Exception as e:
        return f"Lỗi tính toán: {str(e)}"

@tool
def vesc_evaluate_motor(profile_json_str: str, target_rpm: float = 3000.0) -> str:
    """Chạy mô phỏng kiểm tra sai số bám tốc độ và an toàn phần cứng."""
    import json
    try:
        profile = json.loads(profile_json_str)
        res = harness.evaluate_profile(profile, target_rpm=target_rpm)
        return json.dumps(res, indent=2)
    except Exception as e:
        return f"Lỗi mô phỏng: {str(e)}"

# Đăng ký vào LangGraph ToolNode
tools = [vesc_tune_motor, vesc_evaluate_motor]
tool_node = ToolNode(tools)
```

---

## ⚡ 6. Tích Hợp n8n Workflow Automation Hub

Gói Harness hỗ trợ tích hợp trực tiếp vào n8n thông qua Node `Execute Command` hoặc HTTP Webhook:

1. **Node Execute Command trong n8n:**
   - **Command:** `python -m harness.hub_connector tune --motor-name {{ $json.motor_name }} --tier {{ $json.tier }} --cells {{ $json.cells }}`
   - **Working Directory:** `/path/to/05_Propulsion_System/VESC`
2. **Luồng Workflow mẫu:**
   ```
   [Webhook Trigger: New Motor Tuning Request]
           │
           ▼
   [n8n Code Node: Parse Spec]
           │
           ▼
   [Execute Command: python -m harness.hub_connector tune]
           │
           ▼
   [Execute Command: python -m harness.hub_connector evaluate]
           │
           ▼
   [IF: eval_result.passed == true]
     ├─► True: [Send Slack/Telegram: Tuning Thành Công & Lưu Profile]
     └─► False: [Send Alert: Vi phạm an toàn phần cứng hoặc sai số cao]
   ```

---

## 🤖 7. Sử Dụng Subagent `@vesc-motor-engineer`

Trong Antigravity hoặc hệ thống Agent, bạn có thể triệu hồi trực tiếp Subagent chuyên trách:

```text
@vesc-motor-engineer Hãy tổng hợp cấu hình FOC cho động cơ T-Motor U8 170KV với pin 12S LiPo ở mức tải Flight 30A, kiểm tra 6 Hardware Invariants và xuất báo cáo đánh giá mô phỏng.
```

Subagent sở hữu toàn bộ hệ thống tri thức, bộ quy tắc thép, và công cụ để hoàn thành tác vụ tự động 100%.

---

## 🚁 8. Khắc Phục Lỗi DroneCAN GUI Tool Kéo Ga Nhưng Động Cơ Không Quay

Khi sử dụng **DroneCAN GUI Tool** để test kéo ga động cơ qua cửa sổ `ESC Management`, nếu kéo thanh trượt mà động cơ không quay, hãy kiểm tra 4 nguyên nhân cốt lõi sau:

### 🔴 Nguyên Nhân 1: Chưa tích ô "Command broadcast enabled" (BẮT BUỘC)
- **Hiện tượng:** Kéo thanh trượt lên 100%, số hiển thị đổi, nhưng động cơ đứng im, đèn LED CAN trên VESC không phản hồi.
- **Giải thích:** Trong DroneCAN GUI Tool, cửa sổ **ESC Management** có một checkbox bảo vệ an toàn: **`Command broadcast enabled`** (hoặc nút **Arm**). Nếu không tích ô này, DroneCAN GUI **KHÔNG PHÁT BẤT KỲ GÓI CAN NÀO RA BUS**.
- **Cách xử lý:** Nhấp chuột tích vào ô **`Command broadcast enabled`** (thanh trạng thái sẽ chuyển sang màu xanh lá).

### 🔴 Nguyên Nhân 2: Lệch kênh ESC Index
- **Hiện tượng:** Kéo Slider 2, Slider 3 mà động cơ không quay.
- **Giải thích:** VESC đang được cấu hình `uavcan_esc_index = 0` (Kênh đầu tiên). Kênh này tương ứng với **Slider 1 (ESC 1 / Index 0)** trong DroneCAN GUI. Nếu kéo Slider 2, 3, 4, firmware VESC sẽ lọc bỏ gói tin vì không khớp chỉ số kênh.
- **Cách xử lý:** Luôn kéo thanh trượt **Slider 1 (Index 0)** khi test 1 ESC duy nhất.

### 🔴 Nguyên Nhân 3: Xung đột Chế Độ RPM (`uavcan_raw_mode = 3`) và Ngưỡng `s_pid_min_erpm`
- **Hiện tượng:** Kéo ga nhỏ (1% - 3%) động cơ không nhúc nhích. Kéo ga cao động cơ bị giật cục hoặc kêu rít nhưng không quay trơn tru.
- **Giải thích:**
  1. VESC có thông số `uavcan_raw_mode = 3` (Chế độ điều khiển RPM theo vòng kín).
  2. Giá trị ga gửi xuống được nhân với trần ERPM: `speed = throttle * 72000.0`.
  3. Khi ga nhỏ (< 2.5%), `speed < 1800 ERPM` (`s_pid_min_erpm`). VESC từ chối chạy vì tốc độ dưới ngưỡng tối thiểu.
  4. Động cơ Sensorless FOC khi đứng yên (0 RPM) chưa có điện động lực BEMF. Nếu điều khiển vòng kín RPM ngay từ 0 RPM, bộ PID Speed thường bị kẹt do lực cản quán tính và ma sát (cogging stiction).
- **Cách xử lý (Chuẩn PX4 / DroneCAN công nghiệp):**
  Chuyển VESC sang **`uavcan_raw_mode = 2` (Duty Cycle Control)**:
  - Khi ở chế độ Duty Cycle, thanh trượt DroneCAN GUI điều khiển trực tiếp tỉ lệ PWM duty (0.0 - 1.0).
  - Thuật toán đề-pa FOC Sensorless Open-loop của VESC sẽ tự kích hoạt, bơm dòng đề-pa và khởi động động cơ cực kỳ êm ái ngay từ 1% ga!

### 🔴 Nguyên Nhân 4: Xung đột lệnh từ Flight Controller (Durandal)
- **Hiện tượng:** Thanh trượt kéo lên rồi bị reset về 0 hoặc động cơ giật rồi tắt ngay.
- **Giải thích:** Nếu Flight Controller trên bus đang phát gói `RawCommand [0, 0, 0, 0]` liên tục (do đang Disarmed), lệnh của FC sẽ đè lên lệnh của DroneCAN GUI trong vòng 20ms.
- **Cách xử lý:** Đảm bảo FC không phát lệnh ESC đè, hoặc ngắt tạm chân CAN của FC khi dùng DroneCAN GUI để test độc lập.

---

## ⚙️ 9. Lệnh Chuyển Đổi Nhanh VESC Sang Duty Cycle Mode

Để chuyển VESC sang chế độ Duty Cycle (giúp DroneCAN GUI kéo ga quay ngay lập tức), chạy lệnh Python sau:

```powershell
python -c "
from lib.vesc_interface import VESCInterface
vesc = VESCInterface('COM33')
vesc.connect()
# Nạp uavcan_raw_mode = 2 (Duty Cycle)
# Xem scripts/set_uavcan_duty_mode.py
vesc.disconnect()
"
```
*(Script chi tiết có sẵn tại: `05_Propulsion_System/VESC/scripts/set_uavcan_duty_mode.py`)*
