"""
Since our ble_test_service.c file uses BT_GATT_CHRC_NOTIFY and bt_gatt_notify(),
  we have to use bleak's start_notify() feature instead of the serial GATT read/write 
  features.

It turns out that using Channels Layers doesn't quite allow for bleak's callbacks,
    so we are going to use Redis directly for inter-process communication.

IMPORTANT: Make sure to "mamba install redis" if you haven't already.
"""
# My Docker Container ID
# 838a6787ea553a72b0261c4dbf10fd874e9d55babbb8956ab257fbd52183eeac

import asyncio
import redis.asyncio as redis
from bleak import BleakClient, BleakScanner
import bleak
import os
from pathlib import Path
import sys
from typing import Dict, Tuple, List
import json


class BLECommandServer(object):
    """
    Our BLECommandServer is quite straightforward.  There is an infinite 
        loop that is predicated on a message request being received
        from the Channels Layer Group with the name given by group_name.
        By default, this is BLECommand.

    We keep track of which channel consumers are connected to which
        BLE nodes using a set.
        You'll probably only have one since we only have one client 
        connected to your Hub, but we make it a set just in case you 
        have more than one client connected to your Hub in the future.
    """
    def __init__(self, redis_server: str = 'localhost') -> None:
        self.redis_connection = redis.from_url(f'redis://{redis_server}', decode_responses=True)
        self.redis_channel = self.redis_connection.pubsub()
        self.target_service_uuid = '00001523-1212-efde-1999-785fdeadbeef'  # NOTE: replace with your service uuid
        self.example_hw_address = "FF:EE:DE:AD:BE:AD"  # just for reference
        # store BleakClient objects by hw_addr, in a 
        # tuple with the client, subscribed clients set, and characteristic handle integer
        self.clients: Dict[str, Tuple[BleakClient, set, int]] = {}  

    async def cb_handle_ble_receive_notify(self, sender: bleak.BleakGATTCharacteristic, data: bytearray) -> None:
        """
        Callback for sending data and sender from BLE notify to Layer Group.
        
        :param sender: service/characteristic sending the BLE notification
        :param data: characteristic data received from BLE notification
        """
        print(f"received: {data}")
        subscribers = list([x[1] for x in self.clients.values() if x[2] == sender.handle][0])
        await self.redis_connection.publish(
            "BLECommandResponses",
            {
                'type': "BLENotifyReceive",
                'service_uuid': sender.service_uuid,
                'char_uuid': sender.uuid,
                'data': data,
                'senders': subscribers
            }
        )

    async def handle_ble_notify_start_request(self, sender: str, hw_address: str, char_uuid: str) -> None:
        """
        Method to handle a BLE notify subscribe request (BLENotifyStartRequest) from Layer Group.
        Responds with an acknowledgement of BLENotifyStart type to the Layer Group.

        :param hw_address: HW address of the BLE edge node
        :param char_uuid: characteristic to subscribe to
        """
        try:
            if self.clients[hw_address][2] == -9001:
                # double-check the characteristic's handle
                self.clients[hw_address][2] = (
                    self.clients[hw_address][0].services.get_characteristic(char_uuid).handle)
                await self.clients[hw_address][0].start_notify(char_uuid, self.handle_ble_receive_notify)
            await self.redis_connection.publish(
                "BLECommandResponses",
                {
                    'type': "BLENotifyStartResponse",
                    'hw_address': hw_address,
                    'sender': sender,
                    'char_uuid': char_uuid,
                    'success': True
                }
            )
        except Exception as e:
            await self.redis_connection.publish(
                "BLECommandResponses",
                {
                    'type': "BLENotifyStartResponse",
                    'hw_address': hw_address,
                    'sender': sender,
                    'char_uuid': char_uuid,
                    'success': False,
                    'error': str(e)
                }
            )

    async def handle_ble_discover_request(self, sender: str) -> None:
        """
        Method to handle BLE device discovery request (BLEDiscoverRequest) from Layer Group.
        """
        try:
            dict_devices = await BleakScanner.discover(return_adv=True)
            # devices is a dictionary that is {hw_addr: (hw_addr, adv_data)}
            # adv_data is an AdvertisementData with field service_uuids
            await self.redis_connection.publish(
                "BLECommandResponses",
                {
                    'type': "BLEDiscoverResponse",
                    'devices': {hw_addr: adv_data.service_uuids 
                                for hw_addr, adv_data in dict_devices.values()},
                    'sender': sender,
                    'success': True
                }
            )
        except Exception as e:
            await self.redis_connection.publish(
                "BLECommandResponses",
                {
                    'type': "BLEDiscoverResponse",
                    'success': False,
                    'sender': sender,
                    'error': str(e)
                }
            )

    async def cb_handle_ble_disconnect(self, the_client: BleakClient) -> None:
        """
        Callback to handle sudden disconnect of BLE node.  Disconnect the 
            address that fails.
        """
        await self.redis_connection.publish(
            "BLECommandResponses",
            {
                'type': "BLEDisconnect",
                'senders': list(self.clients[the_client.address][1])
            }
        )
        del self.clients[the_client.address]
        
    async def handle_ble_connect_request(self, sender: str, hw_address: str) -> None:
        """
        Method to handle BLE device connection request (BLEConnectRequest) from Layer Group.

        :param hw_address: hardware address of the device to connect to.
        """
        try:
            if hw_address not in self.clients:
                self.clients[hw_address] = (
                    await BleakClient(
                        hw_address, disconnected_callback=self.cb_handle_ble_disconnect),
                    set((sender, )),
                    -9001  # unconnected placeholder handle
                )
            else:
                self.clients[hw_address][1].add(sender)
            # devices is a dictionary that is {hw_addr: (hw_addr, adv_data)}
            # adv_data is an AdvertisementData with field service_uuids
            await self.redis_connection.publish(
                "BLECommandResponses",
                {
                    'type': "BLEConnectResponse",
                    'hw_address': hw_address,
                    'sender': sender,
                    'success': True
                }
            )
        except Exception as e:
            await self.redis_connection.publish(
                "BLECommandResponses",
                {
                    'type': "BLEConnectResponse",
                    'hw_addr': hw_address,
                    'success': False,
                    'sender': sender,
                    'error': str(e)
                }
            )

    async def main(self):
        await self.redis_channel.subscribe("BLECommandRequests")
        while True:
            try:
                incoming_message = await self.redis_channel.get_message(
                    ignore_subscribe_messages=True, timeout=None)
                if incoming_message is None:
                    print("No message in inbox (probably the initial run).")
                    continue
                data_dict = json.loads(incoming_message["data"])
                request_type = data_dict["type"]
                originator = data_dict["sender"]

                if request_type == "BLEDiscoverRequest":
                    await self.handle_ble_discover_request(originator)
                elif request_type == "BLEConnectRequest":
                    await self.handle_ble_connect_request(
                        originator, data_dict["hw_address"])
                elif request_type == "BLENotifyStartRequest":
                    await self.handle_ble_notify_start_request(
                        originator, data_dict["hw_address"], data_dict["char_uuid"])
                else:
                    print(f"Unknown request type: {request_type}")
            except json.JSONDecodeError:
                print(f"Incoming message is not JSON, but is: {incoming_message}")
            except Exception as e:
                print(e)
            finally:
                for client in self.clients:
                    await client.disconnect()
    """
        try:
            client = BleakClient(self.example_hw_address)
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
    """


if __name__ == '__main__':
    # now we can start the actual server
    the_server = BLECommandServer("0.0.0.0")  # NOTE: replace with your redis IP
    asyncio.run(the_server.main())