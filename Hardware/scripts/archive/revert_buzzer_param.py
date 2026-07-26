import asyncio, sys, os
sys.path.insert(0, os.path.expanduser("~/ws/scripts/precise_landing"))
from flight_controller import FC

async def main():
    fc = FC()
    await fc.start()
    rb = await fc.set_px4_param_int("CBRK_BUZZER", 0, verify=True)
    print(f"RESULT: CBRK_BUZZER readback = {rb}")
    await fc.close()

asyncio.run(main())
