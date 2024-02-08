"""
This management command is actually a server that is the
    "plumbing" that will subscribe to our Redis "BLECommandResponses"
    channel, and then route it to the appropriate Django Channels
    consumer via Channels Layer.  This scheme is needed due to the
    BLE Notification callback from bleak (and BLE in general),
    in which messages can be sent from BLE nodes at any time, which
    makes it tricky to use Channels Layer Groups directly in the
    bleak server.

Run this via python manage.py pubsub_channels_server

Based on code from: https://gist.github.com/morlandi/bb915db7acef0ee0e4cb070921208610
"""

# -*- coding: UTF-8 -*-
import signal
import sys
import traceback
import time
import logging
import json
import channels.layers
from asgiref.sync import async_to_sync
from redis.exceptions import ConnectionError
from django.core.management.base import BaseCommand
from django.conf import settings
import redis


logger = logging.getLogger(__name__)


class Command(BaseCommand):
    help = u'Opens a connection to Redis and listens for messages, and then whenever it gets one, sends the message onto a channel in the Django channel system'

    def __init__(self, *args, **kwargs):
        super(Command, self).__init__(*args, **kwargs)
        signal.signal(signal.SIGINT, signal_handler)
        self.logger = logger or logging.getLogger(__name__)
        self.redis_server_ip = "localhost" #"localhost" #"192.168.122.1"  # TODO: replace this with localhost
        self.redis_pubsub_channel = "BLECommandResponses"
        self.layer_group = "BLECommandGroup"

    def handle(self, *args, **options):
        # self.set_logger(options.get('verbosity'))
        self.channel = options.get('channel')
        self.logger.info('Initializing redis listener...[subscribing channel: "%s"]' % self.channel)
        self.redis_client = None
        self.redis_pubsub = None
        self.loop()

    def connect(self):
        while True:
            self.logger.debug('Trying to connect to redis ...')
            try:
                self.redis = redis.from_url(
                    f'redis://{self.redis_server_ip}',
                    decode_responses=True)
                self.redis.ping()
            except (ConnectionError, ConnectionRefusedError):
                time.sleep(1)
            else:
                break
        self.pubsub = self.redis.pubsub()
        self.pubsub.subscribe(self.redis_pubsub_channel)
        self.logger.info('Connected to redis.')

    def loop(self):
        self.connect()
        while True:
            try:
                for item in self.pubsub.listen():
                    data = json.loads(str(item['data']))
                    print(data)
                    self.broadcast_message(item, data)
                    print(data)
            except ConnectionError:
                self.logger.error('Lost connections to redis.')
                self.connect()

    def broadcast_message(self, message: dict, data: dict):
        channel_layer = channels.layers.get_channel_layer()
        async_to_sync(channel_layer.group_send)(
            self.layer_group, {
                # TODO: replace with the correct message
                "type": data['BLECommandResponses'],                # Commands
                "params": data['data'],                 # Parameters
            })


def signal_handler(signal, frame):
    sys.exit(0)


