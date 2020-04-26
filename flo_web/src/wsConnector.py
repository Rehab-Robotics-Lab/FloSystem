#! /usr/bin/env python

# Software License Agreement (BSD License)
#
# Copyright (c) 2012, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

import rospy
import uuid

from rosauth.srv import Authentication

import sys
import threading
import traceback
from functools import wraps

from autobahn.twisted.websocket import WebSocketClientProtocol
from twisted.internet import interfaces, reactor
from zope.interface import implementer

from rosbridge_library.rosbridge_protocol import RosbridgeProtocol
from rosbridge_library.util import json, bson
import sys

from twisted.python import log
from twisted.internet import reactor

from autobahn.twisted.websocket import WebSocketClientFactory


def _log_exception():
    """Log the most recent exception to ROS."""
    exc = traceback.format_exception(*sys.exc_info())
    rospy.logerr(''.join(exc))


def log_exceptions(f):
    """Decorator for logging exceptions to ROS."""
    @wraps(f)
    def wrapper(*args, **kwargs):
        try:
            return f(*args, **kwargs)
        except:
            _log_exception()
            raise
    return wrapper


@implementer(interfaces.IPushProducer)
class OutgoingValve:
    """Allows the Autobahn transport to pause outgoing messages from rosbridge.

    The purpose of this valve is to connect backpressure from the WebSocket
    client
    back to the rosbridge protocol, which depends on backpressure for queueing.
    Without this flow control, rosbridge will happily keep writing messages to
    the WebSocket until the system runs out of memory.

    This valve is closed and opened automatically by the Twisted TCP server.
    In practice, Twisted should only close the valve when its userspace write
    buffer
    is full and it should only open the valve when that buffer is empty.

    When the valve is closed, the rosbridge protocol instance's outgoing writes
    must block until the valve is opened.
    """

    def __init__(self, proto):
        self._proto = proto
        self._valve = threading.Event()
        self._finished = False

    @log_exceptions
    def relay(self, message):
        self._valve.wait()
        if self._finished:
            return
        reactor.callFromThread(self._proto.outgoing, message)

    def pauseProducing(self):
        if not self._finished:
            self._valve.clear()

    def resumeProducing(self):
        self._valve.set()

    def stopProducing(self):
        self._finished = True
        self._valve.set()


class RosbridgeWebSocket(WebSocketClientProtocol):
    client_id_seed = 0
    clients_connected = 0
    authenticate = False

    # The following are passed on to RosbridgeProtocol
    # defragmentation.py:
    fragment_timeout = 600                  # seconds
    # protocol.py:
    delay_between_messages = 0              # seconds
    max_message_size = None                 # bytes
    unregister_timeout = 10.0               # seconds
    bson_only_mode = False

    def onOpen(self):
        cls = self.__class__
        parameters = {
            "fragment_timeout": cls.fragment_timeout,
            "delay_between_messages": cls.delay_between_messages,
            "max_message_size": cls.max_message_size,
            "unregister_timeout": cls.unregister_timeout,
            "bson_only_mode": cls.bson_only_mode
        }
        try:
            self.protocol = RosbridgeProtocol(
                cls.client_id_seed, parameters=parameters)
            producer = OutgoingValve(self)
            self.transport.registerProducer(producer, True)
            producer.resumeProducing()
            self.protocol.outgoing = producer.relay
            self.authenticated = False
            cls.client_id_seed += 1
            cls.clients_connected += 1
            self.client_id = uuid.uuid4()
            self.peer = self.transport.getPeer().host
            if cls.client_manager:
                cls.client_manager.add_client(self.client_id, self.peer)

        except Exception as exc:
            rospy.logerr(
                "Unable to accept incoming connection.  Reason: %s", str(exc))
        rospy.loginfo("Client connected.  %d clients total.",
                      cls.clients_connected)

    def onMessage(self, message, binary):
        cls = self.__class__
        if not binary:
            message = message.decode('utf-8')
        self.protocol.incoming(message)

    def outgoing(self, message):
        if type(message) == bson.BSON:
            binary = True
            message = bytes(message)
        elif type(message) == bytearray:
            binary = True
            message = bytes(message)
        else:
            binary = False
            message = message.encode('utf-8')

        self.sendMessage(message, binary)

    def onClose(self, was_clean, code, reason):
        if not hasattr(self, 'protocol'):
            return  # Closed before connection was opened.
        cls = self.__class__
        cls.clients_connected -= 1
        self.protocol.finish()
        if cls.client_manager:
            cls.client_manager.remove_client(self.client_id, self.peer)
        rospy.loginfo("Client disconnected. %d clients total.",
                      cls.clients_connected)


class MyClientProtocol(WebSocketClientProtocol):

    def onConnect(self, response):
        print("Server connected: {0}".format(response.peer))

    def onConnecting(self, transport_details):
        print("Connecting; transport details: {}".format(transport_details))
        return None  # ask for defaults

    def onOpen(self):
        print("WebSocket connection open.")

        def hello():
            self.sendMessage("Hello, world!".encode('utf8'))
            self.sendMessage(b"\x00\x01\x03\x04", isBinary=True)
            self.factory.reactor.callLater(1, hello)

        # start sending messages every second ..
        hello()

    def onMessage(self, payload, isBinary):
        if isBinary:
            print("Binary message received: {0} bytes".format(len(payload)))
        else:
            print("Text message received: {0}".format(payload.decode('utf8')))

    def onClose(self, wasClean, code, reason):
        print("WebSocket connection closed: {0}".format(reason))


if __name__ == '__main__':

    log.startLogging(sys.stdout)

    FACTORY = WebSocketClientFactory("ws://localhost:8080/host")
    FACTORY.protocol = RosbridgeWebSocket
    # FACTORY.protocol = MyClientProtocol

    reactor.connectTCP("localhost", 8080, FACTORY)
    reactor.run()
