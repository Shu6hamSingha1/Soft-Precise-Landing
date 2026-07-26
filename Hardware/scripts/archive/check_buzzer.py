import asyncio, sys, os
sys.path.insert(0, os.path.expanduser("~/ws/scripts/precise_landing"))
from flight_controller import FC

async def main():
    fc = FC()
    await fc.start()
    rb = await fc.set_px4_param_int("CBRK_BUZZER", 782097, verify=True)
    print(f"CBRK_BUZZER after reboot = {rb}")

    armed = None
    async for a in fc.vehicle.telemetry.armed():
        armed = a
        break
    print(f"Armed: {armed}")

    print("Listening for status/health messages for 5s...")
    async def listen():
        async for h in fc.vehicle.telemetry.health():
            print(f"HEALTH: {h}")
            break
    try:
        await asyncio.wait_for(listen(), timeout=5)
    except asyncio.TimeoutError:
        pass

    await fc.close()

asyncio.run(main())
