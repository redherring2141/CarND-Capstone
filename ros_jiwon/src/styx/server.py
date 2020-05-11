#!/usr/bin/env python

from gevent import pywsgi
from geventwebsocket.handler import WebSocketHandler

import socketio
import time

from bridge import Bridge
from conf import conf

#Added
import eventlet
eventlet.monkey_patch(socket=True, select=True, time=True)
import eventlet.wsgi
from flask import Flask, render_template
from bridge import Bridge
from conf import conf

#sio = socketio.Server(async_mode='gevent')
sio = socketio.Server()#modified
app = Flask(__name__)#added
msgs = []

dbw_enable = False

@sio.on('connect')
def connect(sid, environ):
    print("connect ", sid)

def send(topic, data):
    s = 1#added
    msgs.append((topic, data))#added
    #sio.emit(topic, data=data, skip_sid=True)

bridge = Bridge(conf, send)

@sio.on('telemetry')
def telemetry(sid, data):
    global dbw_enable
    if data["dbw_enable"] != dbw_enable:
        dbw_enable = data["dbw_enable"]
        bridge.publish_dbw_status(dbw_enable)
    bridge.publish_odometry(data)
    #Added
    for i in range(len(msgs)):
        topic, data = msgs.pop(0)
        sio.emit(topic, data=data, skip_sid=True)

@sio.on('control')
def control(sid, data):
    bridge.publish_controls(data)

@sio.on('obstacle')
def obstacle(sid, data):
    bridge.publish_obstacles(data)

@sio.on('lidar')
def obstacle(sid, data):
    bridge.publish_lidar(data)

@sio.on('trafficlights')
def trafficlights(sid, data):
    bridge.publish_traffic(data)

@sio.on('image')
def image(sid, data):
    bridge.publish_camera(data)

if __name__ == '__main__':

    # Create socketio WSGI application
    #app = socketio.WSGIApp(sio)
    app = socketio.Middleware(sio, app)

    # deploy as an gevent WSGI server
    #pywsgi.WSGIServer(('', 4567), app, handler_class=WebSocketHandler).serve_forever()
    eventlet.wsgi.server(eventlet.listen(('', 4567)), app)