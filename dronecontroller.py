#!/usr/bin/env python
import json
import threading
import minidrone
import zmq
import time
import math
from pid_controller.pid import PID

OFFLINE_DEBUG = True

MAX_SPEED = 100

mutex = threading.Lock()

S_DISCONNECTED = 0
S_CONNECTING = 1
S_CONNECTED = 2

# DRONE_MAC = 'E0:14:1E:C2:3D:47' # white minidrone
DRONE_MAC = 'E0:14:2C:AA:3D:4F'  # blue minidrone
CB_MSG = 0
CB_BATTERY = 1
CB_DATA_UPDATE = 2
CB_SPEED = 3
CB_STATE = 4

VICON_TIMEOUT = 0.5
DRONE_TIMEOUT = 3
LOOP_TIMEOUT = 0.05
LIFT_DELAY = 3

ROTATION_HALT = 0.4
ROTATION_FAILED = 1.0

GROUNDED_SPEED = 10

VICON_SERVER_SOCKET = "tcp://*:5555"
UNITY_SERVER_SOCKET = "tcp://*:5556"


class ViconServerThread(minidrone.StoppableThread):
    def __init__(self, context, process, feedback, cleanup):
        minidrone.StoppableThread.__init__(self)
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
                control_message_json = json.loads(str(control_message))
                # noinspection PyBroadException
                try:
                    translation_json = control_message_json["translation"]
                    translation = (translation_json['x'],
                                   translation_json['y'],
                                   translation_json['z'])
                    rotation_json = control_message_json["rotation"]
                    rotation = (rotation_json['x'],
                                rotation_json['y'],
                                rotation_json['z'],
                                rotation_json['w'])
                    reset = control_message_json['reset'] == 1
                    tracking = control_message_json['tracking']
                    self.process(translation, rotation, reset, tracking)
                except:
                    self.cleanup()
                feedback_message = self.feedback()
                socket.send(feedback_message)

            else:
                socket.close()
                break


class UnityServerThread(minidrone.StoppableThread):
    def __init__(self, context, process, feedback, cleanup):
        minidrone.StoppableThread.__init__(self)
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
                    state_json = control_message_json['state']
                    state = {
                        'takeoff': state_json['takeoff']
                    }
                    self.process(translation, rotation, state)
                except:
                    self.cleanup()
                feedback_message = self.feedback()
                socket.send(feedback_message)

            else:
                socket.close()
                break


def angluar_difference(quad1, quad2):
    quad_diff = add_quaternion(quad1, negate_quaternion(quad2))
    return quaternion_to_yaw(quad_diff)


def negate_quaternion(q):
    x, y, z, w = q
    return (-x, -y, -z, w)


def add_quaternion(q1, q2):
    return (q1[3] * q2[0] + q1[0] * q2[3] + q1[1] * q2[2] - q1[2] * q2[1],
            q1[3] * q2[1] - q1[0] * q2[2] + q1[1] * q2[3] + q1[2] * q2[0],
            q1[3] * q2[2] + q1[0] * q2[1] - q1[1] * q2[0] + q1[2] * q2[3],
            q1[3] * q2[3] - q1[0] * q2[0] - q1[1] * q2[1] - q1[2] * q2[2])


def quaternion_to_roll(q):
    x, y, z, w = q
    return math.asin(2 * x * y + 2 * z * w)

def quaternion_to_pitch(q):
    x, y, z, w = q
    return math.atan2(2 * x * w - 2 * y * z, 1 - 2 * x * x - 2 * z * z)


def quaternion_to_yaw(q):
    x, y, z, w = q
    return math.atan2(2 * y * w - 2 * x * z, 1 - 2 * y * y - 2 * z * z)


class ControllerThread(minidrone.StoppableThread):
    def __init__(self):
        minidrone.StoppableThread.__init__(self)
        self.zmqContext = zmq.Context()
        self.new_changes = threading.Semaphore(0)
        self.viconServerThread = \
            ViconServerThread(self.zmqContext, self.receive_vicon_data, self.status_report, self.halt)
        self.unityServerThread = \
            UnityServerThread(self.zmqContext, self.receive_unity_data, self.status_report, self.halt)
        self.drone = minidrone.MiniDrone(mac=DRONE_MAC, callback=self.receive_drone_data)
        self.state = S_DISCONNECTED
        self.message = self.battery = ''
        self.speed = 0
        self.config = dict()
        self.drone_translation = (0, 0, 0)
        self.drone_rotation = (0, 0, 0, 1)
        self.drone_tracking = False
        self.target_translation = (0.0, 1.5, 0.0)
        self.target_rotation = (0, 0, 0, 1)
        self.target_takeoff = True
        self.last_drone_update = time.time()
        self.last_vicon_update = time.time()
        self.lifted_time = 0
        self.failed = False
        self.took_off = False
        self.pid_lr = PID(p=300.0, i=500.0, d=100.0)
        self.pid_fb = PID(p=300.0, i=500.0, d=100.0)
        self.pid_vertical = PID(p=70.0, i=10.0, d=0.0)
        self.pid_rotation = PID(p=200.0, i=0.0, d=50.0)
        self.joy_update = time.time()

    def emergency(self):
        print("emergency!")
        self.drone.emergency()

    def crash(self):
        self.failed = True
        print("crash!")
        self.drone.land()

    def halt(self):
        print("halted!")
        self.drone.still()

    def status_report(self):
        mutex.acquire()
        status = {
            "battery": self.battery
        }
        mutex.release()
        return json.dumps(status)

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
            self.speed = int(data)
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

    def receive_vicon_data(self, translation, rotation, reset, tracking):
        self.drone_translation = translation
        self.drone_rotation = rotation
        self.drone_tracking = tracking
        if reset:
            self.failed = False
        self.last_vicon_update = time.time()
        self.new_changes.release()

    def receive_unity_data(self, translation, rotation, state):
        self.target_translation = translation
        self.target_rotation = rotation
        self.target_takeoff = state['takeoff']
        self.new_changes.release()

    def make_decision(self):
        now = time.time()
        if self.failed and not OFFLINE_DEBUG:
            return
        if self.state == S_DISCONNECTED and not OFFLINE_DEBUG:
            self.state = S_CONNECTING
            print("Connecting...")
            self.drone.connect()
        if self.state == S_CONNECTING and not OFFLINE_DEBUG:
            return
        # if now - self.last_drone_update > DRONE_TIMEOUT:
        #     self.halt()
        #     return
        # if now - self.last_vicon_update > VICON_TIMEOUT:
        #     self.halt()
        #     return
        if self.state == S_CONNECTED or OFFLINE_DEBUG:
        # if True:
            if not self.took_off and not OFFLINE_DEBUG:
            # if False:
                if now - self.lifted_time > LIFT_DELAY:
                    self.took_off = True
                    self.drone.takeoff()
                    time.sleep(1)
                    pass

            else:
                if self.drone_tracking or OFFLINE_DEBUG:
                    if time.time() - self.joy_update > 0.1:
                        roll = quaternion_to_roll(self.drone_rotation)
                        yaw = quaternion_to_yaw(self.drone_rotation)
                        if math.fabs(roll) > ROTATION_FAILED or math.fabs(yaw) > ROTATION_FAILED:
                            self.crash()
                            return
                        angle = angluar_difference(self.drone_rotation, self.target_rotation)

                        def scale(x):
                            return math.copysign(math.sqrt(math.fabs(x)), x)

                        def clip(x, x_min, x_max):
                            return max(x_min, min(x_max, x))

                        trans_angle = quaternion_to_yaw(self.drone_rotation)
                        diff_x = (self.drone_translation[0] - self.target_translation[0])
                        diff_z = (self.drone_translation[2] - self.target_translation[2])
                        pos_lr = diff_x * math.cos(trans_angle) - diff_z * math.sin(trans_angle)
                        pos_fb = diff_z * math.cos(trans_angle) + diff_x * math.sin(trans_angle)
                        hor_lr = scale(self.pid_lr(-pos_lr)) - 5
                        hor_fb = scale(self.pid_fb(-pos_fb)) + 10
                        rotation = scale(self.pid_rotation(-angle))
                        vertical = self.pid_vertical(-(self.drone_translation[1] - self.target_translation[1]))

                        hor_lr = clip(hor_lr, -MAX_SPEED, MAX_SPEED)
                        hor_fb = clip(hor_fb, -MAX_SPEED, MAX_SPEED)
                        vertical = clip(vertical, -MAX_SPEED, MAX_SPEED)
                        rotation = clip(rotation, -MAX_SPEED, MAX_SPEED)
                        print(self.drone_rotation)
                        print(self.drone_translation)
                        print(hor_lr, hor_fb, rotation, vertical)
                        if not OFFLINE_DEBUG:
                            self.drone.send(self.drone.send_joy_fast, int(hor_lr), int(hor_fb), int(rotation),
                                        int(vertical))
                        self.joy_update = time.time()
                else:
                    self.drone.send(self.drone.send_joy, 0, 0, 0, -10)

    def run(self):
        self.viconServerThread.start()
        self.state = S_CONNECTING
        self.drone.connect()
        while True:
            if not self.stop_event.is_set():
                current_time = time.time()
                while time.time() - current_time < LOOP_TIMEOUT:
                    if self.new_changes.acquire(blocking=False):
                        break
                self.make_decision()

            else:
                self.emergency()
                self.drone.die()
                break


if __name__ == '__main__':
    mainThread = ControllerThread()
    mainThread.start()
    mainThread.join()
