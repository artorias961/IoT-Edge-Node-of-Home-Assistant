"""
Since our ble_test_service.c file uses BT_GATT_CHRC_NOTIFY and bt_gatt_notify(),
  we have to use bleak's start_notify() feature instead of the serial GATT read/write 
  features.

"""


import asyncio
from bleak import BleakClient
import bleak

hw_address = "FF:EE:DE:AD:BE:EF"  # NOTE: replace with your hw_address
service_uuid = '00001523-1212-efde-8282-785fdeadbeef'  # NOTE: replace with your service uuid


async def print_notification(sender: bleak.BleakGATTCharacteristic, data: bytearray) -> None:
    print(f"received: {data}")


async def main(hw_address: str):
    try:
        # Connecting to the bleak client (using the MAC Address)
        client = BleakClient(hw_address)

        # Wait for the signal until it connects
        await client.connect()

        # Making an empty variable
        ptr_to_char = None

        # Get the information from the BLE Client
        our_service = [x for x in client.services if x.uuid == service_uuid]
        
        # Get the serboice of the device
        our_service = our_service[0]
        
        # Get the characteristics of the device (is Notify or Read/Write)
        our_char = our_service.characteristics[0]

        # Finally advertise the information what you get from the MAC Address or in our case the nRF52dk
        await client.start_notify(our_char.uuid, print_notification)

        # Create an infinite loop to recieve signal from the board
        while True:
            # Print in the terminal
            print("hello")

            # Every two seconds get the information that is being advertise from GATT
            await asyncio.sleep(2)

    except Exception as e:
        print(e)
    finally:
        await client.disconnect()

asyncio.run(main(hw_address))