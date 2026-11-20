import asyncio
from mavsdk import System
from mavsdk.offboard import VelocityNedYaw

async def main():
    drone = System()
    await drone.connect(system_address="udp://:14540")

    print("Waiting connecting..")
    async for move in drone.core.connection_state():
      if move.is_connected:
       print("Connected!")
       break
      
    print ("ARM!")
    await drone.action.arm()
    await asyncio.sleep(5)

    
    print ("TAKEOFF!")
    await drone.action.set_takeoff_altitude(2)
    await drone.action.takeoff()
    await asyncio.sleep(5)

    async for attitude in drone.telemetry.attitude_euler():
      break

    start_yaw = attitude.yaw_deg

    print("MOVE SQUARE!")
    await drone.offboard.set_velocity_ned(
        VelocityNedYaw(0.0, 0.0, 0.0, start_yaw)
    )
    await drone.offboard.start()

    square_sides = (
        (0.5, 0.0, "NORTH"),
        (0.0, 0.5, "EAST"),
        (-0.5, 0.0, "SOUTH"),
        (0.0, -0.5, "WEST"),
    )
    for north_speed, east_speed, direction in square_sides:
        print(f"Moving {direction}")
        await drone.offboard.set_velocity_ned(
            VelocityNedYaw(north_speed, east_speed, 0.0, start_yaw)
        )
        await asyncio.sleep(6)

    await drone.offboard.set_velocity_ned(
        VelocityNedYaw(0.0, 0.0, 0.0, start_yaw)
    )
    await asyncio.sleep(1)
    await drone.offboard.stop()

    print("LAND!")
    await drone.action.land()
    await asyncio.sleep(10)

asyncio.run(main())
 
