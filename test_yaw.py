import asyncio
import math

from mavsdk import System
from mavsdk.offboard import VelocityNedYaw

TAKEOFF_ALTITUDE_M = 2.0
SIDE_LENGTH_M = 1.0
MOVE_SPEED_M_S = 0.4
YAW_RATE_DEG_S = 15.0
YAW_UPDATE_INTERVAL_S = 0.1
LAND_DESCENT_SPEED_M_S = 0.2
LAND_HANDOFF_ALTITUDE_M = 0.4


def normalize_yaw(yaw_deg: float) -> float:
    return (yaw_deg + 180.0) % 360.0 - 180.0


async def turn_to_yaw(
    drone: System, current_yaw_deg: float, target_yaw_deg: float
) -> float:
    yaw_delta = (target_yaw_deg - current_yaw_deg + 180.0) % 360.0 - 180.0
    if abs(yaw_delta) < 0.01:
        return normalize_yaw(target_yaw_deg)

    turn_duration_s = abs(yaw_delta) / YAW_RATE_DEG_S
    steps = math.ceil(turn_duration_s / YAW_UPDATE_INTERVAL_S)
    step_duration_s = turn_duration_s / steps

    for step in range(1, steps + 1):
        yaw_deg = normalize_yaw(
            current_yaw_deg + yaw_delta * step / steps
        )
        await drone.offboard.set_velocity_ned(
            VelocityNedYaw(0.0, 0.0, 0.0, yaw_deg)
        )
        await asyncio.sleep(step_duration_s)

    return normalize_yaw(target_yaw_deg)


async def wait_for_takeoff_altitude(drone: System) -> None:
    async def wait_for_altitude() -> None:
        async for position in drone.telemetry.position():
            if position.relative_altitude_m >= TAKEOFF_ALTITUDE_M - 0.2:
                return

    try:
        await asyncio.wait_for(wait_for_altitude(), timeout=30.0)
    except asyncio.TimeoutError:
        print("Takeoff altitude was not reached; landing.")
        await drone.action.land()
        raise


async def land_slowly(drone: System, yaw_deg: float) -> None:
    async def wait_for_handoff_altitude() -> None:
        async for position in drone.telemetry.position():
            if position.relative_altitude_m <= LAND_HANDOFF_ALTITUDE_M:
                return

    print("Descending slowly...")
    await drone.offboard.set_velocity_ned(
        VelocityNedYaw(0.0, 0.0, LAND_DESCENT_SPEED_M_S, yaw_deg)
    )

    try:
        await asyncio.wait_for(wait_for_handoff_altitude(), timeout=30.0)
    except asyncio.TimeoutError:
        print("Low landing altitude was not reached; handing over to autopilot.")
        await drone.offboard.set_velocity_ned(
            VelocityNedYaw(0.0, 0.0, 0.0, yaw_deg)
        )
        await drone.offboard.stop()
        await drone.action.land()
        return

    await drone.offboard.set_velocity_ned(
        VelocityNedYaw(0.0, 0.0, 0.0, yaw_deg)
    )
    await asyncio.sleep(0.5)
    await drone.offboard.stop()

    print("LAND!")
    await drone.action.land()


async def main():
    drone = System()
    await drone.connect(system_address="udp://:14540")

    print("Waiting connecting..")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print("Connected!")
            break

    print("ARM!")
    await drone.action.arm()

    print("TAKEOFF!")
    await drone.action.set_takeoff_altitude(TAKEOFF_ALTITUDE_M)
    await drone.action.takeoff()
    await wait_for_takeoff_altitude(drone)
    await asyncio.sleep(2.0)

    async for attitude in drone.telemetry.attitude_euler():
        start_yaw_deg = attitude.yaw_deg
        break

    current_yaw_deg = start_yaw_deg
    print("MOVE SQUARE!")
    await drone.offboard.set_velocity_ned(
        VelocityNedYaw(0.0, 0.0, 0.0, current_yaw_deg)
    )
    await drone.offboard.start()
    await asyncio.sleep(0.5)

    square_sides = (
        (0.4, 0.0, "NORTH", 0.0),
        (0.0, 0.4, "EAST", 90.0),
        (-0.4, 0.0, "SOUTH", 180.0),
        (0.0, -0.4, "WEST", -90.0),
    )
    move_duration_s = SIDE_LENGTH_M / MOVE_SPEED_M_S

    for north_speed, east_speed, direction, target_yaw_deg in square_sides:
        print(f"Turning toward {direction}...")
        await drone.offboard.set_velocity_ned(
            VelocityNedYaw(0.0, 0.0, 0.0, current_yaw_deg)
        )
        await asyncio.sleep(0.5)

        current_yaw_deg = await turn_to_yaw(
            drone, current_yaw_deg, target_yaw_deg
        )
        await asyncio.sleep(0.5)

        print(f"Moving {direction}...")
        await drone.offboard.set_velocity_ned(
            VelocityNedYaw(north_speed, east_speed, 0.0, current_yaw_deg)
        )
        await asyncio.sleep(move_duration_s)

    await drone.offboard.set_velocity_ned(
        VelocityNedYaw(0.0, 0.0, 0.0, current_yaw_deg)
    )
    await asyncio.sleep(1.0)
    await land_slowly(drone, current_yaw_deg)


asyncio.run(main())
