import asyncio
from mavsdk import System
async def main():
 drone = System()
 await drone.connect(system_address="udp://:14540")

 print("Waiting connecting..")
 async for arm in drone.core.connection_state():
  if arm.is_connected:
   print("Connected!")
   break

 print ("ARM!")
 await drone.action.arm()
 await asyncio.sleep(10)
 
 print ("DISARM!")
 await drone.action.disarm() 

asyncio.run(main())


