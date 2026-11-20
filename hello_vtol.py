import asyncio
from mavsdk import System
async def main():
    drone = System()
    await drone.connect(system_address="udp://:14540")

    print("Waiting connecting..")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print("Connected!")
            break

    async for pos in drone.telemetry.position():
        print(f"Tinggi: {pos.relative_altitude_m:.2f} m")
        break

asyncio.run(main()) 

