import asyncio
from bleak import BleakScanner


async def main():
    print("scanning for 5 seconds, please wait...")

    devices = await BleakScanner.discover(return_adv=True)

    for d, a in devices.values():  # TODO: What data type is devices?
        print()
        print(d)
        print("-" * len(str(d)))
        print(a)


if __name__ == "__main__":
    asyncio.run(main())