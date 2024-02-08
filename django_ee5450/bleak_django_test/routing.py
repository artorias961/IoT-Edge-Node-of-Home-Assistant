from django.urls import path

from . import consumers

websocket_urlpatterns = [
    path("bleak/scan_control/", consumers.BleakScannerConsumer.as_asgi()),
]