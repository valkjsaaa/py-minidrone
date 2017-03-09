#!/usr/bin/env python
import json
import threading
import minidrone
import zmq
import time
import math

mutex = threading.Lock()

S_DISCONNECTED = 0
S_CONNECTING = 1
S_CONNECTED = 2

DRONE_MAC = 'E0:14:D6:A9:3D:28'
CB_MSG = 0
CB_BATTERY = 1
CB_DATA_UPDATE = 2
CB_SPEED = 3
CB_STATE = 4

VICON_TIMEOUT = 0.5
DRONE_TIMEOUT = 1
LOOP_TIMEOUT = 0.05
LIFT_DELAY = 3

ROTATION_HALT = 0.2
ROTATION_FAILED = 1

GROUNDED_SPEED = 10

VICON_SERVER_SOCKET = "tcp://*:5555"
UNITY_SERVER_SOCKET = "tcp://*:5556"


class ViconServerThread(minidrone.StoppableThread):
    def __init__(self, context, process, feedback, cleanup):
        super().__init__()
        self.context = context
        self.process = process
        self.feedback = feedback
        self.cleanup = cleanup

    def run(self):
        socket = self.context.socket(zmq.REP)
        socket.bind(VICON_SERVER_SOCKET)
        while True:
            if not self.stop_event.is_set():
                control_message = socket.recv()
                control_message_json = json.loads(control_message)
                # noinspection PyBroadException
                try:
                    translation_json = control_message_json["translation"]
                    translation = (translation_json['x'],
                                   translation_json['y'],
                                   translation_json['z'])
                    rotation_json = control_message_json["rotation"]
                    rotation = (rotation_json['w'],
                                rotation_json['x'],
                                rotation_json['y'],
                                rotation_json['z'])
                    reset = control_message["reset"] == 1
                    self.process(translation, rotation, reset)
                except:
                    self.cleanup()
                feedback_message = self.feedback()
                socket.send(feedback_message)

            else:
                socket.close()
                break


class UnityServerThread(minidrone.StoppableThread):
    def __init__(self, context, process, feedback, cleanup):
        super().__init__()
        self.context = context
        self.process = process
        self.feedback = feedback
        self.cleanup = cleanup

    def run(self):
        socket = self.context.socket(zmq.REP)
        socket.bind(UNITY_SERVER_SOCKET)
        while True:
            if not self.stop_event.is_set():
                control_message = socket.recv()
                control_message_json = json.loads(control_message)
                # noinspection PyBroadException
                try:
                    translation_json = control_message_json["translation"]
                    translation = (translation_json['x'],
                                   translation_json['y'],
                                   translation_json['z'])
                    rotation_json = control_message_json["rotation"]
                    rotation = (rotation_json['w'],
                                rotation_json['x'],
                                rotation_json['y'],
                                rotation_json['z'])
                    self.process(translation, rotation)
                except:
                    self.cleanup()
                feedback_message = self.feedback()
                socket.send(feedback_message)

            else:
                socket.close()
                break


def angluar_difference(quad1, quad2):
    w1, x1, y1, z1 = quad1
    w2, x2, y2, z2 = quad2
    len_quad1 = math.sqrt(x1 ** 2 + y1 ** 2 + z1 ** 2)
    len_quad2 = math.sqrt(x2 ** 2 + y2 ** 2 + z2 ** 2)
    dot_product = x1 * x2 + y1 * y2 + z1 * z2
    return math.acos(dot_product / len_quad1 / len_quad2)


class ControllerThread(minidrone.StoppableThread):
    def __init__(self):
        super().__init__()
        self.zmqContext = zmq.Context()
        self.viconServerThread = \
            ViconServerThread(self.zmqContext, self.receive_vicon_data, self.status_report, self.halt)
        self.unityServerThread = \
            UnityServerThread(self.zmqContext, self.receive_unity_data, self.status_report, self.halt)
        self.drone = minidrone.MiniDrone(mac=DRONE_MAC, callback=self.receive_drone_data)
        self.state = S_DISCONNECTED
        self.message = self.speed = self.battery = ''
        self.config = dict()
        self.new_changes = threading.Semaphore(0)
        self.drone_translation = (0, 0, 0)
        self.drone_rotation = (0, 1, 0, 0)
        self.target_translation = (0, 0, 0)
        self.target_rotation = (0, 1, 0, 0)
        self.last_drone_update = time.time()
        self.last_vicon_update = time.time()
        self.lifted_time = 0
        self.failed = False

    def emergency(self):
        print("emergency!")
        self.drone.emergency()

    def halt(self):
        print("halted!")
        self.drone.still()

    def status_report(self):
        mutex.acquire()
        # TODO: status report
        mutex.release()
        return "I'm fine"

    def receive_drone_data(self, t, data):
        if t == CB_MSG:
            mutex.acquire()
            self.message = data
            mutex.release()
        elif t == CB_BATTERY:
            mutex.acquire()
            self.battery = data
            mutex.release()
        elif t == CB_SPEED:
            mutex.acquire()
            self.speed = data
            mutex.release()
        elif t == CB_DATA_UPDATE:
            mutex.acquire()
            self.config = data
            mutex.release()
        elif t == CB_STATE:
            mutex.acquire()
            self.state = S_CONNECTED if data == 'y' else S_DISCONNECTED
            mutex.release()
        self.last_drone_update = time.time()
        self.new_changes.release()

    def receive_vicon_data(self, translation, rotation, reset):
        self.drone_translation = translation
        self.drone_rotation = rotation
        if reset:
            self.failed = False
        self.last_vicon_update = time.time()
        self.new_changes.release()

    def receive_unity_data(self, translation, rotation):
        self.target_translation = translation
        self.target_rotation = rotation
        self.new_changes.release()

    def make_decision(self):
        now = time.time()
        if self.failed:
            return
        if self.state == S_DISCONNECTED:
            self.drone.connect()
        if now - self.last_drone_update > DRONE_TIMEOUT:
            self.halt()
            return
        if now - self.last_vicon_update > VICON_TIMEOUT:
            self.halt()
            return
        if self.state == S_CONNECTED:
            if self.speed < GROUNDED_SPEED:
                if now - self.lifted_time > LIFT_DELAY:
                    self.drone.takeoff()

            else:
                angle = angluar_difference(self.drone_rotation, self.target_rotation)
                if angle > ROTATION_FAILED:
                    self.emergency()
                    self.failed = True
                else:
                    hor_lr = hor_fb = vertical = 0
                    rotation_factor = -1
                    horizontal_factor = -1
                    vertical_factor = -1
                    rotation = rotation_factor * (self.target_rotation[0] - self.drone_rotation[0])
                    if angle < ROTATION_HALT:
                        hor_lr = horizontal_factor * (self.target_translation[0] - self.drone_translation[0])
                        hor_fb = horizontal_factor * (self.target_translation[2] - self.drone_translation[2])
                        vertical = vertical_factor * (self.target_translation[1] - self.drone_translation[1])
                    print((hor_lr, hor_fb, rotation, vertical))
                    self.drone.send_joy(hor_lr, hor_fb, rotation, vertical)

    def run(self):
        self.viconServerThread.start()
        self.drone.connect()
        while True:
            if not self.stop_event.is_set():
                self.new_changes.acquire(blocking=True, timeout=LOOP_TIMEOUT)
                self.make_decision()

            else:
                self.emergency()
                self.drone.die()
                break


if __name__ == '__main__':
    mainThread = ControllerThread()
    mainThread.start()
    mainThread.join()
