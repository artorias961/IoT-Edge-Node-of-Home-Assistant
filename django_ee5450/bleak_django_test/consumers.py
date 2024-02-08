import json
from channels.generic.websocket import AsyncWebsocketConsumer
from bleak import BleakScanner


"""
Async WS Consumer for the Bleak Scanner component
"""
class BleakScannerConsumer(AsyncWebsocketConsumer):
    async def connect(self):
        return await super().connect()
    
    async def disconnect(self, code):
        return await super().disconnect(code)
    
    async def receive(self, text_data=None, bytes_data=None):
        str_scan_request = json.loads(text_data)["message"]
        print(str_scan_request)
        time_scan_request_s = int(str_scan_request.split("scan.")[1])
        devices = await BleakScanner.discover(
            timeout=time_scan_request_s, return_adv=True
        )
        out_dict = {}
        for hw_addr, hw_adv_tuple in devices.items():
            if len(hw_adv_tuple[1].service_uuids) > 0:
                out_dict[hw_addr] = hw_adv_tuple[1].service_uuids
        return await self.send(text_data=json.dumps({"message": out_dict}))

"""
Async WS Consumer for the Bleak Device Subscription Control Component
"""
class BleakStreamerSubscriptionControlConsumer(AsyncWebsocketConsumer):
    async def connect(self):
        return await super().connect()
