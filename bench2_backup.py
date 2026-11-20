import os
import sys
import csv
import time
import math
import signal
import subprocess
from pymavlink import mavutil

# ============================================================
# Default connection:
#   - SITL/Gazebo: udp:127.0.0.1:14550
#   - Hardware: set MAVLINK_PORT=/dev/ttyACM0
# Example:
#   MAVLINK_PORT=/dev/ttyACM0 python3 bench.py auto 1.0 10
# ============================================================
PORT = os.environ.get("MAVLINK_PORT", "udp:127.0.0.1:14550")
BAUD = 57600

MODE_CH = 10
HOLD_SECONDS = 15
ARM_SECONDS = 5

RC_MIN = 900
RC_MAX = 2100
PWM_JUMP_LIMIT = 100
TILT_LIMIT_DEG = 25

KILL_CH = 11
KILL_NORMAL = "LOW"

THR_LOW_MAX = 1100
RIG_DELTA_DEFAULT = 300
RIG_DELTA_CAP = 860
RIG_RAMP_UP = 8
RIG_HOLD = 5
RIG_RAMP_DOWN = 4
RIG_MAX_TOTAL = 30
RIG_TILT_LIMIT = 45
PWM_RIG_LIMIT = 200

AUTO_TAKEOFF_START_THRUST = 0.08
AUTO_TAKEOFF_TARGET_THRUST = 0.45
AUTO_TAKEOFF_RAMP_SECONDS = 6.0

RC_THROTTLE_MIN = 1063
RC_THROTTLE_MAX = 1923

master = mavutil.mavlink_connection(PORT, baud=BAUD, autoreconnect=True)
print("waiting for heartbeat...")
if master.wait_heartbeat(timeout=15) is None:
    print("no heartbeat, check wiring, SERIAL params, or SITL connection")
    sys.exit(1)

master.target_system = 1
master.target_component = 1
master.mav.request_data_stream_send(
    1, 1, mavutil.mavlink.MAV_DATA_STREAM_ALL, 4, 1
)

st = {
    "mode": "-", "armed": False,
    "roll": 0.0, "pitch": 0.0, "yaw": 0.0,
    "volt": 0.0, "curr": 0.0,
    "fix": 0, "sats": 0, "alt": 0.0,
    "ekf_flags": 0, "ekf_ok": False,
    "rc": [0] * 16, "pwm": [0, 0, 0, 0]
}
results = []

# ============================================================
# Basic helpers
# ============================================================
def poll():
    while True:
        msg = master.recv_match(blocking=False)
        if msg is None:
            return
        t = msg.get_type()

        if t == "HEARTBEAT":
            if msg.get_srcSystem() == 1 and msg.type != mavutil.mavlink.MAV_TYPE_GCS:
                st["mode"] = mavutil.mode_string_v10(msg)
                st["armed"] = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)

        elif t == "ATTITUDE":
            st["roll"] = math.degrees(msg.roll)
            st["pitch"] = math.degrees(msg.pitch)
            st["yaw"] = math.degrees(msg.yaw) % 360

        elif t == "SYS_STATUS":
            st["volt"] = msg.voltage_battery / 1000.0
            st["curr"] = msg.current_battery / 100.0

        elif t == "GPS_RAW_INT":
            st["fix"] = msg.fix_type
            st["sats"] = msg.satellites_visible

        elif t == "GLOBAL_POSITION_INT":
            st["alt"] = msg.relative_alt / 1000.0

        elif t == "EKF_STATUS_REPORT":
            st["ekf_flags"] = int(getattr(msg, "flags", 0))
            st["ekf_ok"] = bool(st["ekf_flags"]) and not bool(st["ekf_flags"] & (1 << 10))

        elif t == "RC_CHANNELS":
            st["rc"] = [getattr(msg, f"chan{i}_raw") for i in range(1, 17)]

        elif t == "SERVO_OUTPUT_RAW":
            st["pwm"] = [msg.servo1_raw, msg.servo2_raw, msg.servo3_raw, msg.servo4_raw]

        elif t == "STATUSTEXT":
            if "ap_message" not in msg.text:
                print("  >> " + msg.text)

def show():
    state = "ARMED" if st["armed"] else "disarmed"
    rc = st["rc"]
    print(f"{st['mode']:13} {state:9} roll {st['roll']:6.1f} pitch {st['pitch']:6.1f} "
          f"alt {st['alt']:5.1f}m {st['volt']:5.2f}V "
          f"rc {rc[0]} {rc[1]} {rc[2]} {rc[3]} m{rc[MODE_CH - 1]} pwm {st['pwm']}")

def wait_for(cond, timeout):
    t0 = time.time()
    while time.time() - t0 < timeout:
        poll()
        if cond():
            return True
        time.sleep(0.05)
    return False

def wait_for_ekf(timeout=90):
    print("waiting for healthy EKF...")
    ok = wait_for(lambda: st["ekf_ok"], timeout)
    if not ok:
        print(f"EKF not healthy after {timeout}s (flags=0x{st['ekf_flags']:x})")
    return ok

def collect(seconds):
    t0 = time.time()
    samples = []
    while time.time() - t0 < seconds:
        poll()
        samples.append((st["roll"], st["pitch"], list(st["rc"]), st["mode"]))
        time.sleep(0.1)
    return samples

def set_mode(name):
    mapping = master.mode_mapping() or {}
    if name not in mapping:
        print("unknown mode", name)
        return False
    master.mav.set_mode_send(
        1,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        mapping[name]
    )
    return wait_for(lambda: st["mode"] == name, 5)

def arm():
    master.arducopter_arm()
    return wait_for(lambda: st["armed"], 10)

def disarm():
    master.arducopter_disarm()
    return wait_for(lambda: not st["armed"], 10)

def force_disarm_once():
    master.mav.command_long_send(
        1, 1, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0, 0, 21196, 0, 0, 0, 0, 0
    )

def safe_disarm():
    try:
        master.arducopter_disarm()
        if wait_for(lambda: not st["armed"], 3):
            return True
    except Exception:
        pass
    for _ in range(5):
        force_disarm_once()
        if wait_for(lambda: not st["armed"], 2):
            return True
    return False

def on_signal(signum, frame):
    for _ in range(5):
        force_disarm_once()
        master.mav.rc_channels_override_send(1, 1, 0, 0, 0, 0, 0, 0, 0, 0)
        time.sleep(0.2)
    sys.exit(1)

def clamp(value, low, high):
    return max(low, min(high, value))

def quaternion_from_euler(roll_rad, pitch_rad, yaw_rad):
    cy = math.cos(yaw_rad * 0.5)
    sy = math.sin(yaw_rad * 0.5)
    cp = math.cos(pitch_rad * 0.5)
    sp = math.sin(pitch_rad * 0.5)
    cr = math.cos(roll_rad * 0.5)
    sr = math.sin(roll_rad * 0.5)

    q = [
        cy * cp * cr + sy * sp * sr,
        cy * cp * sr - sy * sp * cr,
        sy * cp * sr + cy * sp * cr,
        sy * cp * cr - cy * sp * sr,
    ]
    return q

def send_att(thrust, yaw_deg):
    send_attitude(0.0, 0.0, float(yaw_deg), clamp(float(thrust), 0.0, 1.0))

def send_attitude(roll_deg=0.0, pitch_deg=0.0, yaw_deg=0.0, thrust=0.0,
                 roll_rate=0.0, pitch_rate=0.0, yaw_rate=0.0):
    thrust = clamp(float(thrust), 0.0, 1.0)
    q = quaternion_from_euler(
        math.radians(float(roll_deg)),
        math.radians(float(pitch_deg)),
        math.radians(float(yaw_deg))
    )
    master.mav.set_attitude_target_send(
        0, 1, 1, 0, q, float(roll_rate), float(pitch_rate), float(yaw_rate), thrust
    )

def ramp(start, end, seconds, yaw):
    t0 = time.time()
    last = 0
    while time.time() - t0 < seconds:
        thrust = start + (end - start) * (time.time() - t0) / seconds
        poll()
        send_att(thrust, yaw)
        if time.time() - last > 0.5:
            show()
            last = time.time()
        time.sleep(0.1)

def confirm(text):
    answer = input(text + " type YES to continue: ")
    return answer.strip() == "YES"

def throttled():
    try:
        out = subprocess.check_output(["vcgencmd", "get_throttled"], text=True).strip()
        return out.split("=")[1]
    except Exception:
        return None

def report(name, ok, detail=""):
    results.append(ok)
    print(f"[{'PASS' if ok else 'FAIL'}] {name} {detail}")

def info(name, detail):
    print(f"[INFO] {name} {detail}")

def read_param(name, timeout=3.0):
    master.mav.param_request_read_send(1, 1, name.encode(), -1)
    t0 = time.time()
    while time.time() - t0 < timeout:
        msg = master.recv_match(type="PARAM_VALUE", blocking=True, timeout=0.5)
        if msg is None:
            continue
        pid = msg.param_id
        if isinstance(pid, bytes):
            pid = pid.decode(errors="ignore")
        if pid.rstrip("\x00") == name:
            return msg.param_value
    return None

def rc_alive():
    return all(RC_MIN <= v <= RC_MAX for v in st["rc"][:4])

# ============================================================
# System check / diagnostics
# ============================================================
def run_monitor():
    while True:
        poll()
        show()
        time.sleep(0.5)

def run_check():
    print("keep the board flat for 3 seconds")
    s = collect(3)
    report("heartbeat", st["mode"] != "-", f"mode {st['mode']}")
    thr = throttled()
    report("raspi power", thr == "0x0", f"throttled {thr}")
    avg_r = sum(x[0] for x in s) / len(s)
    avg_p = sum(x[1] for x in s) / len(s)
    report("level", abs(avg_r) < 2 and abs(avg_p) < 2, f"roll {avg_r:.1f} pitch {avg_p:.1f}")
    rc = s[-1][2]
    report("rc signal", all(RC_MIN <= v <= RC_MAX for v in rc[:4]), f"ch1-4 {rc[:4]}")
    input("press Enter, then tilt the board for 8 seconds ")
    s = collect(8)
    roll_range = max(x[0] for x in s) - min(x[0] for x in s)
    pitch_range = max(x[1] for x in s) - min(x[1] for x in s)
    report("attitude responds", roll_range > 15 or pitch_range > 15,
           f"roll range {roll_range:.0f} pitch range {pitch_range:.0f}")
    input("press Enter, then flip the mode switch through all positions for 10 seconds ")
    s = collect(10)
    seen = sorted({x[3] for x in s})
    report("rc mode switch", len(seen) >= 3, f"modes seen {seen}")
    if st["armed"]:
        info("mode command", "skipped, vehicle is armed")
    else:
        report("mode command", set_mode("STABILIZE"), "STABILIZE")
    info("battery", f"{st['volt']:.2f} V {st['curr']:.1f} A")
    info("gps", f"fix {st['fix']} sats {st['sats']}")
    print(f"{sum(results)}/{len(results)} passed")

def run_log():
    name = time.strftime("log_%Y%m%d_%H%M%S.csv")
    print("logging to", name)
    with open(name, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["t", "mode", "armed", "roll", "pitch", "yaw", "volt", "curr",
                    "rc1", "rc2", "rc3", "rc4", "rc_mode", "pwm1", "pwm2", "pwm3", "pwm4"])
        t0 = time.time()
        try:
            while True:
                poll()
                rc = st["rc"]
                w.writerow([round(time.time() - t0, 2), st["mode"], int(st["armed"]),
                            round(st["roll"], 2), round(st["pitch"], 2), round(st["yaw"], 1),
                            round(st["volt"], 2), round(st["curr"], 1),
                            rc[0], rc[1], rc[2], rc[3], rc[MODE_CH - 1]] + st["pwm"])
                f.flush()
                time.sleep(0.2)
        except KeyboardInterrupt:
            print("\nsaved", name)

def run_mode(name):
    name = name.upper()
    print(name, "->", "ok" if set_mode(name) else "failed")

def run_arm(mode_name):
    if not preflight():
        print("preflight failed, not arming")
        return
    if not confirm("propellers off, kill switch proven, LiPo connector within reach."):
        return
    if not set_mode(mode_name):
        print("mode change failed")
        return
    try:
        if not arm():
            print("arm failed, read the messages above")
            return
        print("armed")
        reason = guarded_hold(ARM_SECONDS)
        if reason:
            print("ABORT:", reason)
    finally:
        ok = safe_disarm()
        print("disarmed" if ok else "DISARM FAILED, pull the LiPo connector")

def preflight():
    ok = True
    want = {"ARMING_CHECK": 1, "BRD_SAFETYENABLE": 1, "ESC_CALIBRATION": 0}
    for name, value in want.items():
        v = read_param(name)
        good = v is not None and int(round(v)) == value
        report(name, good, f"{v}")
        ok = ok and good
    fs = read_param("FS_THR_ENABLE")
    good = fs is not None and int(round(fs)) != 0
    report("FS_THR_ENABLE", good, f"{fs}")
    ok = ok and good
    collect(1)
    good = rc_alive()
    report("rc live", good, f"ch1-4 {st['rc'][:4]}")
    return ok and good

def guarded_hold(seconds):
    collect(1)
    base = max(st["pwm"])
    t0 = time.time()
    last = 0
    while time.time() - t0 < seconds:
        poll()
        reason = None
        if not st["armed"]:
            reason = "vehicle disarmed"
        elif max(st["pwm"]) > base + PWM_JUMP_LIMIT:
            reason = f"pwm jump {st['pwm']}"
        elif max(abs(st["roll"]), abs(st["pitch"])) > TILT_LIMIT_DEG:
            reason = "tilt limit"
        elif not rc_alive():
            reason = "rc lost"
        if reason:
            return reason
        if time.time() - last > 0.5:
            show()
            last = time.time()
        time.sleep(0.05)
    return None

def run_audit():
    collect(1)
    if st["armed"]:
        print("vehicle is armed, disarm first")
        return
    ok = preflight()
    for name in ("FS_THR_VALUE", "MOT_PWM_TYPE", "FRAME_CLASS", "FRAME_TYPE", "BATT_MONITOR",
                 "FLTMODE_CH", "FLTMODE1", "FLTMODE2", "FLTMODE3", "FLTMODE4", "FLTMODE5", "FLTMODE6"):
        info(name, read_param(name))
    print("audit:", "PASS" if ok else "ISSUES FOUND")

# ============================================================
# RC / rig / thrust test
# ============================================================
def band_of(v):
    if 0 < v < 1300:
        return "LOW"
    if v > 1700:
        return "HIGH"
    return "MID"

def clamp_rc_throttle(thr):
    return int(max(RC_THROTTLE_MIN, min(RC_THROTTLE_MAX, thr)))

def target_throttle_for_altitude(current_alt, target_alt, throttle_min=RC_THROTTLE_MIN,
                                throttle_max=RC_THROTTLE_MAX):
    if target_alt <= 0:
        return throttle_min
    ratio = max(0.0, min(current_alt / target_alt, 1.0))
    lift_ratio = 0.15 + 0.85 * ratio
    return int(throttle_min + (throttle_max - throttle_min) * lift_ratio)

def send_override(thr):
    master.mav.rc_channels_override_send(1, 1, 0, 0, clamp_rc_throttle(thr), 0, 0, 0, 0, 0)

def release_override():
    for _ in range(5):
        master.mav.rc_channels_override_send(1, 1, 0, 0, 0, 0, 0, 0, 0, 0)
        time.sleep(0.05)

def rig_abort_reason(pwm_base):
    if not st["armed"]:
        return "vehicle disarmed"
    if st["mode"] != "STABILIZE":
        return f"mode changed to {st['mode']}"
    rc = st["rc"]
    live = [rc[0], rc[1], rc[3], rc[KILL_CH - 1]]
    if not all(RC_MIN <= v <= RC_MAX for v in live):
        return "rc lost"
    if band_of(rc[KILL_CH - 1]) != KILL_NORMAL:
        return "kill switch moved"
    if max(abs(st["roll"]), abs(st["pitch"])) > RIG_TILT_LIMIT:
        return "tilt limit"
    if max(st["pwm"]) > pwm_base + PWM_RIG_LIMIT:
        return f"pwm limit {st['pwm']}"
    return None

def run_rig(delta):
    delta = min(int(delta), RIG_DELTA_CAP)
    if delta < 10:
        print("delta too small")
        return
    collect(1)
    if st["armed"]:
        print("vehicle is armed, disarm first")
        return
    if not preflight():
        print("preflight failed, not running")
        return
    rc = st["rc"]
    thr_min = int(rc[2])
    if not (RC_THROTTLE_MIN <= thr_min <= THR_LOW_MAX):
        print(f"throttle stick not at minimum ({thr_min}), expected around {RC_THROTTLE_MIN}")
        return
    thr_target = clamp_rc_throttle(thr_min + delta)
    if thr_target < RC_THROTTLE_MIN:
        thr_target = RC_THROTTLE_MIN
    if thr_target > RC_THROTTLE_MAX:
        thr_target = RC_THROTTLE_MAX
    band = band_of(rc[KILL_CH - 1])
    print(f"kill switch channel {KILL_CH}: {rc[KILL_CH - 1]} {band}, normal is {KILL_NORMAL}")
    if band != KILL_NORMAL:
        print("kill switch is not in the normal position")
        return
    print(f"throttle {thr_min} -> {thr_target}: {RIG_RAMP_UP}s up, {RIG_HOLD}s hold, {RIG_RAMP_DOWN}s down")
    if not confirm("propellers off for first run, rig locked, one hand on kill switch, LiPo connector within reach."):
        return
    if not set_mode("STABILIZE"):
        print("mode change failed")
        return
    reason = None
    try:
        send_override(thr_min)
        if not arm():
            print("arm failed, read the messages above")
            return
        collect(1)
        pwm_base = max(st["pwm"])
        print("armed, pwm base", pwm_base)
        plan = [
            (thr_min, thr_target, RIG_RAMP_UP, "ramp up"),
            (thr_target, thr_target, RIG_HOLD, "hold"),
            (thr_target, thr_min, RIG_RAMP_DOWN, "ramp down"),
        ]
        t_start = time.time()
        for a, b, secs, name in plan:
            print(f">> {name} {a} -> {b}")
            t0 = time.time()
            last = 0
            while time.time() - t0 < secs:
                poll()
                reason = rig_abort_reason(pwm_base)
                if reason is None and time.time() - t_start > RIG_MAX_TOTAL:
                    reason = "total time limit"
                if reason:
                    break
                thr_cmd = a + (b - a) * (time.time() - t0) / secs
                send_override(thr_cmd)
                if time.time() - last > 0.5:
                    show()
                    last = time.time()
                time.sleep(0.1)
            if reason:
                break
        if reason:
            print("ABORT:", reason)
    finally:
        for _ in range(10):
            send_override(thr_min)
            time.sleep(0.05)
        ok = safe_disarm()
        release_override()
        print("disarmed" if ok else "DISARM FAILED, pull the LiPo connector")

def run_killtest(ch):
    ch = int(ch)
    if ch < 1 or ch > 16:
        print("channel must be 1 to 16")
        return
    print(f"watching rc channel {ch}, arm and test from Mission Planner, Ctrl+C to stop")
    last_key = None
    last_show = 0
    armed_max = 0
    while True:
        poll()
        v = st["rc"][ch - 1]
        band = "LOW" if 0 < v < 1300 else ("HIGH" if v > 1700 else "MID")
        key = (band, st["armed"])
        if st["armed"]:
            armed_max = max(armed_max, max(st["pwm"]))
        if key != last_key:
            print(f"[EVENT] ch{ch} {v} {band} armed={st['armed']} mode {st['mode']} pwm {st['pwm']}")
            last_key = key
        if time.time() - last_show > 0.5:
            state = "ARMED" if st["armed"] else "disarmed"
            print(f"{state:8} ch{ch} {v:5} {band:4} pwm {st['pwm']} max_while_armed {armed_max}")
            last_show = time.time()
        time.sleep(0.05)

# ============================================================
# Smooth autonomous hover logic
# ============================================================
def hold_hover(target_alt, hover_time, yaw_deg=0.0,
              hover_thrust=0.36, initial_thrust=0.12,
              max_thrust=0.58, min_thrust=0.10,
              max_altitude=20.0):
    target_alt = max(0.2, float(target_alt))
    hover_time = max(0.0, float(hover_time))
    yaw_target = float(st["yaw"]) if st["yaw"] is not None else float(yaw_deg)
    alt_integral = 0.0
    t_start = time.time()

    print(f"starting hover hold at {target_alt:.1f}m for {hover_time:.1f}s")
    while time.time() - t_start < hover_time:
        poll()
        alt_error = target_alt - st["alt"]
        alt_integral = clamp(alt_integral + alt_error * 0.05, -1.0, 1.0)

        thrust = hover_thrust + 0.35 * alt_error + 0.08 * alt_integral
        thrust = clamp(thrust, min_thrust, max_thrust)

        if st["alt"] < 0.15:
            thrust = max(thrust, initial_thrust)
        if st["alt"] > max_altitude:
            thrust = min(thrust, hover_thrust)

        send_attitude(
            roll_deg=0.0,
            pitch_deg=0.0,
            yaw_deg=yaw_target,
            thrust=thrust,
            roll_rate=0.0,
            pitch_rate=0.0,
            yaw_rate=0.0,
        )
        time.sleep(0.05)

    print("stabilizing descent")
    t_desc = time.time()
    while time.time() - t_desc < 6.0 and st["armed"]:
        poll()
        if st["alt"] <= 0.25:
            desc_thrust = 0.10
        elif st["alt"] <= 0.8:
            desc_thrust = clamp(0.14 + 0.08 * (0.8 - st["alt"]), 0.10, 0.20)
        else:
            desc_thrust = clamp(0.18 + 0.06 * max(0.0, st["alt"] - 0.8), 0.12, 0.24)
        send_attitude(0.0, 0.0, yaw_target, desc_thrust, 0.0, 0.0, 0.0)
        time.sleep(0.05)

    send_attitude(0.0, 0.0, yaw_target, 0.0, 0.0, 0.0, 0.0)


def move_to_altitude(target_alt, hover_time=5.0, yaw_deg=0.0,
                    hover_thrust=0.36, initial_thrust=0.12,
                    max_thrust=0.58, min_thrust=0.10):
    """Move to a target altitude gradually and then hover there.

    This allows a safe stepwise mission profile such as 0.5m -> 1.0m -> 1.5m.
    """
    target_alt = max(0.3, float(target_alt))
    hover_time = max(0.0, float(hover_time))
    yaw_target = float(st["yaw"]) if st["yaw"] is not None else float(yaw_deg)

    print(f"moving to {target_alt:.2f}m with gentle thrust ramp")
    t0 = time.time()
    ramp_seconds = 7.0
    while time.time() - t0 < ramp_seconds:
        poll()
        alpha = (time.time() - t0) / ramp_seconds
        thrust = initial_thrust + (hover_thrust - initial_thrust) * alpha
        if st["alt"] > 0.2:
            thrust = min(thrust, 0.40)
        thrust = clamp(thrust, 0.08, 0.55)
        send_attitude(0.0, 0.0, yaw_target, thrust, 0.0, 0.0, 0.0)
        time.sleep(0.05)

    print("climbing with altitude correction")
    integral = 0.0
    last_time = time.time()
    while st["alt"] < (target_alt * 0.95):
        poll()
        now = time.time()
        dt = max(0.02, now - last_time)
        last_time = now

        alt_error = target_alt - st["alt"]
        integral = clamp(integral + alt_error * dt, -1.0, 1.0)
        thrust = hover_thrust + 0.30 * alt_error + 0.07 * integral
        thrust = clamp(thrust, min_thrust, max_thrust)
        thrust = clamp(thrust, 0.10, 0.55)
        send_attitude(0.0, 0.0, yaw_target, thrust, 0.0, 0.0, 0.0)
        time.sleep(0.05)

    hold_hover(target_alt, hover_time, yaw_deg=yaw_target,
               hover_thrust=hover_thrust,
               initial_thrust=initial_thrust,
               max_thrust=max_thrust,
               min_thrust=min_thrust)


def smooth_takeoff_hover(target_alt, hover_time, yaw_deg=0.0,
                        hover_thrust=0.36, initial_thrust=0.12,
                        max_thrust=0.58, min_thrust=0.10):
    return move_to_altitude(target_alt, hover_time=hover_time, yaw_deg=yaw_deg,
                           hover_thrust=hover_thrust, initial_thrust=initial_thrust,
                           max_thrust=max_thrust, min_thrust=min_thrust)


def run_altitude_profile(targets, hover_time=5.0):
    """Fly a stepwise profile such as [0.5, 1.0, 1.5]."""
    if not targets:
        print("no steps given")
        return
    print(f"altitude profile: {targets} m, hover {hover_time}s each")

    for alt in targets:
        alt = float(alt)
        if alt <= 0:
            continue
        print(f"--- step to {alt:.2f}m ---")
        move_to_altitude(alt, hover_time=hover_time)
        time.sleep(0.5)

    print("profile complete, initiating landing...")
    set_mode("LAND")
    while True:
        poll()
        print(f"descending... alt: {st['alt']:.2f} m")
        if not st["armed"]:
            print("drone has landed and is DISARMED.")
            break
        time.sleep(0.5)

# ============================================================
# Attitude command
# ============================================================
def run_attitude(thrust):
    thrust = clamp(float(thrust), 0.0, 1.0)
    print(f"attitude command active, thrust={thrust:.2f}")
    print("roll and pitch held at 0 deg, yaw will stay at current heading")
    t0 = time.time()
    while time.time() - t0 < 10.0:
        poll()
        send_attitude(0.0, 0.0, st["yaw"], thrust, 0.0, 0.0, 0.0)
        time.sleep(0.05)

# ============================================================
# Auto mission
# ============================================================
def send_takeoff(alt):
    print(f"sending takeoff command to {alt}m...")
    master.mav.command_long_send(
        1, 1,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
        0, 0, 0, 0, 0, 0, 0, alt
    )


def run_auto(target_alt, hover_time):
    print("waiting for GPS 3D Fix and healthy EKF...")
    if not wait_for(lambda: st["fix"] >= 3, 60):
        print("failed to get GPS Fix. mission aborted.")
        return
    print("GPS 3D Fix acquired!")

    if not wait_for_ekf(90):
        print("failed to get healthy EKF. mission aborted.")
        return
    print("EKF healthy!")

    if not set_mode("GUIDED"):
        print("failed to set GUIDED mode. ensure EKF is healthy.")
        return

    if not arm():
        print("arming failed.")
        return

    move_to_altitude(target_alt, hover_time=hover_time)
    print("mission complete, initiating landing...")
    set_mode("LAND")

    while True:
        poll()
        print(f"descending... alt: {st['alt']:.2f} m")
        if not st["armed"]:
            print("drone has landed and is DISARMED.")
            break
        time.sleep(0.5)
# ============================================================
# Main entry
# ============================================================
signal.signal(signal.SIGHUP, on_signal)
signal.signal(signal.SIGTERM, on_signal)

cmd = sys.argv[1] if len(sys.argv) > 1 else "monitor"

try:
    if cmd == "monitor":
        run_monitor()
    elif cmd == "check":
        run_check()
    elif cmd == "log":
        run_log()
    elif cmd == "mode":
        run_mode(sys.argv[2] if len(sys.argv) > 2 else "STABILIZE")
    elif cmd == "arm":
        run_arm(sys.argv[2].upper() if len(sys.argv) > 2 else "STABILIZE")
    elif cmd == "audit":
        run_audit()
    elif cmd == "killtest":
        run_killtest(sys.argv[2] if len(sys.argv) > 2 else KILL_CH)
    elif cmd == "rig":
        run_rig(sys.argv[2] if len(sys.argv) > 2 else RIG_DELTA_DEFAULT)
    elif cmd == "attitude":
        run_attitude(float(sys.argv[2]) if len(sys.argv) > 2 else 0.15)
    elif cmd == "auto":
        # usage: python3 bench2.py auto [TARGET_ALT] [HOVER_TIME]
        alt = float(sys.argv[2]) if len(sys.argv) > 2 else 1.0
        htime = float(sys.argv[3]) if len(sys.argv) > 3 else 10.0
        run_auto(alt, htime)
    elif cmd == "profile":
        # usage: python3 bench2.py profile 0.5,1.0,1.5 [HOVER_TIME]
        if len(sys.argv) > 2:
            targets = [float(x.strip()) for x in sys.argv[2].split(",") if x.strip()]
        else:
            targets = [0.5, 1.0, 1.5]
        htime = float(sys.argv[3]) if len(sys.argv) > 3 else 5.0
        print("profile targets:", targets)
        run_altitude_profile(targets, hover_time=htime)
    else:
        print("usage: python3 bench2.py [monitor|check|log|audit|killtest [CH]|rig [DELTA_US]|mode NAME|arm [MODE]|attitude THRUST|auto [ALT] [TIME]|profile 0.5,1.0,1.5 [HOVER_TIME]]")
except KeyboardInterrupt:
    if cmd == "auto" and st["armed"]:
        print("\nuser interrupted. triggering emergency LAND...")
        set_mode("LAND")
    else:
        print("\nstopped")