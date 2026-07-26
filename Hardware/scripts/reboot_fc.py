import asyncio, sys, os
sys.path.insert(0, os.path.expanduser("~/ws/scripts/precise_landing"))
from flight_controller import FC

async def main():
    fc = FC()
    await fc.start()

    # Safety check: never reboot an armed vehicle.
    armed = None
    async for a in fc.vehicle.telemetry.armed():
        armed = a
        break
    print(f"Armed state before reboot: {armed}")
    if armed:
        print("ABORT: vehicle reports ARMED - not rebooting.")
        return

    print("Rebooting flight controller...")
    await fc.vehicle.action.reboot()
    print("Reboot command sent.")

asyncio.run(main())
