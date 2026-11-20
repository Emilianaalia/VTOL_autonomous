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

 print ("ARM!")
 await drone.action.arm()

 
 print ("TAKEOFF!")
 await drone.action.set_takeoff_altitude(2)
 await drone.action.takeoff()
 await asyncio.sleep(10)

 HOVER_DURATION_S = 20
 print(f"HOVERING for {HOVER_DURATION_S} seconds...")
 await asyncio.sleep(HOVER_DURATION_S)

 print ("LAND!")
 await drone.action.land()
 await asyncio.sleep(10)

asyncio.run(main())


