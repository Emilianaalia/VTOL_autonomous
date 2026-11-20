import os
import sys
import csv
import time
import math
import signal
import subprocess
from pymavlink import mavutil

PORT = os.environ.get('MAVLINK_PORT', 'udp:127.0.0.1:14550')
BAUD = 57600

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
    'mode': '-', 'armed': False,
    'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0,
    'volt': 0.0, 'curr': 0.0,
    'fix': 0, 'sats': 0, 'alt': 0.0,
    'ekf_flags': 0, 'ekf_ok': False,
    'rc': [0] * 16, 'pwm': [0, 0, 0, 0]
}

def poll():
    while True:
        msg = master.recv_match(blocking=False)
        if msg is None:
            return
        t = msg.get_type()

        if t == 'HEARTBEAT':
            if msg.get_srcSystem() == 1 and msg.type != mavutil.mavlink.MAV_TYPE_GCS:
                st['mode'] = mavutil.mode_string_v10(msg)
                st['armed'] = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)

        elif t == 'ATTITUDE':
            st['roll'] = math.degrees(msg.roll)
            st['pitch'] = math.degrees(msg.pitch)
            st['yaw'] = math.degrees(msg.yaw) % 360

        elif t == 'GLOBAL_POSITION_INT':
            st['alt'] = msg.relative_alt / 1000.0

        elif t == 'GPS_RAW_INT':
            st['fix'] = msg.fix_type
            st['sats'] = msg.satellites_visible

        elif t == 'EKF_STATUS_REPORT':
            st['ekf_ok'] = bool(msg.flags) and not bool(msg.flags & (1 << 10))

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
    return wait_for(lambda: st['mode'] == name, 5)

def arm():
    master.arducopter_arm()
    return wait_for(lambda: st['armed'], 10)

def wait_for(cond, timeout):
    t0 = time.time()
    while time.time() - t0 < timeout:
        poll()
        if cond():
            return True
        time.sleep(0.05)
    return False

def hold_hover(target_alt, hover_time, yaw_deg=0.0,
              hover_thrust=0.36, initial_thrust=0.12,
              max_thrust=0.58, min_thrust=0.10,
              max_altitude=20.0):
    target_alt = max(0.2, float(target_alt))
    hover_time = max(0.0, float(hover_time))
    yaw_target = float(st['yaw']) if st['yaw'] is not None else float(yaw_deg)
    alt_integral = 0.0
    last_alt = st['alt']
    t_start = time.time()

    while time.time() - t_start < hover_time:
        poll()

        alt_error = target_alt - st['alt']
        alt_integral = clamp(alt_integral + alt_error * 0.05, -1.0, 1.0)

        thrust = hover_thrust + 0.35 * alt_error + 0.08 * alt_integral
        thrust = clamp(thrust, min_thrust, max_thrust)

        if st['alt'] < 0.15:
            thrust = max(thrust, initial_thrust)
        if st['alt'] > max_altitude:
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
        last_alt = st['alt']

    send_attitude(0.0, 0.0, yaw_target, 0.0, 0.0, 0.0, 0.0)

def smooth_takeoff_hover(target_alt, hover_time, yaw_deg=0.0,
                        hover_thrust=0.36, initial_thrust=0.12,
                        max_thrust=0.58, min_thrust=0.10):
    target_alt = max(0.3, float(target_alt))
    hover_time = max(0.0, float(hover_time))
    yaw_target = float(st['yaw']) if st['yaw'] is not None else float(yaw_deg)

    print(f"starting smooth takeoff to {target_alt:.1f}m")
    t0 = time.time()
    ramp_seconds = 6.0

    while time.time() - t0 < ramp_seconds:
        poll()
        alpha = (time.time() - t0) / ramp_seconds
        thrust = initial_thrust + (hover_thrust - initial_thrust) * alpha
        if st['alt'] > 0.2:
            thrust = min(thrust, 0.42)
        send_attitude(0.0, 0.0, yaw_target, thrust, 0.0, 0.0, 0.0)
        time.sleep(0.05)

    print("climbing with gentle altitude correction")
    while st['alt'] < (target_alt * 0.92):
        poll()
        alt_error = target_alt - st['alt']
        thrust = hover_thrust + 0.28 * alt_error
        thrust = clamp(thrust, min_thrust, max_thrust)
        send_attitude(0.0, 0.0, yaw_target, thrust, 0.0, 0.0, 0.0)
        time.sleep(0.05)

    hold_hover(target_alt, hover_time, yaw_deg=yaw_target,
               hover_thrust=hover_thrust,
               initial_thrust=initial_thrust,
               max_thrust=max_thrust,
               min_thrust=min_thrust)

def run_auto(target_alt, hover_time):
    print("waiting for GPS 3D Fix and healthy EKF...")
    if not wait_for(lambda: st['fix'] >= 3, 60):
        print("failed to get GPS Fix. mission aborted.")
        return

    if not wait_for(lambda: st['ekf_ok'], 90):
        print("failed to get healthy EKF. mission aborted.")
        return

    if not set_mode('GUIDED'):
        print("failed to set GUIDED mode")
        return

    if not arm():
        print("arming failed.")
        return

    smooth_takeoff_hover(target_alt, hover_time)
    set_mode('LAND')

if __name__ == "__main__":
    # usage: python3 bench.py auto [TARGET_ALT] [HOVER_TIME]
    alt = float(sys.argv[2]) if len(sys.argv) > 2 else 1.0
    htime = float(sys.argv[3]) if len(sys.argv) > 3 else 10.0
    run_auto(alt, htime)