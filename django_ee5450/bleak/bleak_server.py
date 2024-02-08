"""
Since our ble_test_service.c file uses BT_GATT_CHRC_NOTIFY and bt_gatt_notify(),
  we have to use bleak's start_notify() feature instead of the serial GATT read/write 
  features.

"""


import asyncio
import channels.layers
from bleak import BleakClient
import bleak
import os


# hw_address = "FF:EE:DE:AD:BE:AD"  # NOTE: replace with your hw_address
# service_uuid = '00001523-1212-efde-1999-785fdeadbeef'  # NOTE: replace with your service uuid

class BLECommandServer(object):
    def __init__(self) -> None:
        self.channel_layer = channels.layers.get_channel_layer()
        self.target_service_uuid = '00001523-1212-efde-1999-785fdeadbeef'
        self.example_homework_address = "FF:EE:DE:AD:BE:AD" 


    async def print_notification(self, sender: bleak.BleakGATTCharacteristic, data: bytearray) -> None:
        print(f"received: {data}")


    async def main(self, hw_address: str):
        try:
            client = BleakClient(hw_address)
            await client.connect()
            ptr_to_char = None
            our_service = [x for x in client.services if x.uuid == self.target_service_uuid]
            our_service = our_service[0]
            our_char = our_service.characteristics[0]

            await client.start_notify(our_char.uuid, self.print_notification)
            while True:
                print("hello")
                await asyncio.sleep(2)

        except Exception as e:
            print(e)
        finally:
            await client.disconnect()

    # asyncio.run(main(self.example_homework_address))

if __name__ == "__main__":
    # Need to define the settings.py file for the django server to work 
    os.environ['DJANGO_SETTINGS_MODULE'] = "django_ee5450.settings"

    # Calling the class 
    the_server = BLECommandServer()

    # Run the entire server
    asyncio.run(the_server.main())


