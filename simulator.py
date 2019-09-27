#!/usr/bin/python3
"""simulator.py: Visualizes Java robot simulator output."""
__author__      = "Gavin Uberti"

import json
import math
import os
import queue
import socket
import socketserver
import sys
import threading
import time
from operator import attrgetter

# Hide the Python boot message
os.environ["PYGAME_HIDE_SUPPORT_PROMPT"] = "hide"
import pygame

import controller
import write_video

JAVA_SERVER_HOST = "localhost"
JAVA_SERVER_PORT = 4445
PYTHON_SERVER_PORT = 4446

SCREEN_DIMS = (720, 720)
FIELD_WIDTH_IN = 12 * 12 - 3
ROBOT_WIDTH_IN = 18
MAX_TRAIL_LENGTH = 1024
TRAIL_THICKNESS = 20

SCREEN_CENTER = tuple(x / 2 for x in SCREEN_DIMS)
SCALE_FACTOR = SCREEN_DIMS[0] / FIELD_WIDTH_IN

RR2_BACKGROUND_FIELD = pygame.image.load("res/ss_field_real.png")
RR2_BACKGROUND_FIELD = pygame.transform.scale(RR2_BACKGROUND_FIELD, SCREEN_DIMS)

ROBOT_DIMS = tuple(ROBOT_WIDTH_IN * x // FIELD_WIDTH_IN for x in SCREEN_DIMS)
ROBOT = pygame.image.load("res/mecanum.png")
ROBOT = pygame.transform.scale(ROBOT, (ROBOT_DIMS))

SECS_BETWEEN_FRAMES = 1 / 60

class RobotPosition:
    def __init__(self, x, y, h, index, framerate, json=""):
        self.x = x
        self.y = y
        self.h = h
        self.index = index
        self.framerate = framerate
        self.json = json

    @classmethod
    def from_json(cls, json_data):
        attrs = json.loads(json_data)
        return cls(attrs["x"], attrs["y"], attrs["heading"], attrs["index"], attrs["framerate"], json_data)

    def __str__(self):
        return str(self.json)

    # Self's time should always be less than position
    # Frames 105, 103, current time is 104
    # time_frac = 104 - 103 / 105 - 103 = 1/2
    def interpolate(self, position, frac):
        return RobotPosition(
            frac * (position.x - self.x) + self.x,
            frac * (position.y - self.y) + self.y,
            frac * (position.h - self.h) + self.h,
            frac * (position.index - self.index) + self.index,
            self.framerate
        )

    def get_screen_center(self):
        # We flip X and Y to correspond to how we've laid out field
        return (  # Coords for center of robot
            SCREEN_CENTER[1] - SCALE_FACTOR * self.y,
            SCREEN_CENTER[0] - SCALE_FACTOR * self.x,
        )

    def draw(self, screen):
        rotated = pygame.transform.rotate(ROBOT, math.degrees(self.h))
        robot_size = rotated.get_rect().size

        # Calculate the corner of the robot
        center = self.get_screen_center()
        corner = (center[0] - robot_size[1] / 2, center[1] - robot_size[0] / 2)

        screen.blit(rotated, corner)


class FrameQueue:
    def __init__(self):
        self.frames = []
        self.trail = []
        self.last_frame_update = time.time()
        self.prev_start = None
        self.prev_end = None
        self.real_time = False

    def add(self, frame: RobotPosition):
        if frame.framerate == -1:
            while len(self.frames) > 1:
                self._pop_min()
        self.frames.append(frame)
        self.trail.append(frame)
        while len(self.trail) > MAX_TRAIL_LENGTH:
            self.trail.pop(0)

    def _pop_min(self):
        m = min(self.frames,key=attrgetter('index'))
        self.frames.remove(m)
        return m

    def get(self):
        if not self.prev_start or not self.prev_end:
            if len(self.frames) < 2:
                return None
            self.prev_start = self._pop_min()
            self.prev_end = self._pop_min()
            self.last_frame_update = time.time()

        # If we should and can frame update, do so
        if (
            time.time() >= self.last_frame_update + SECS_BETWEEN_FRAMES
            and len(self.frames) >= 1
        ):
            self.prev_start = self.prev_end
            self.prev_end = self._pop_min()
            self.last_frame_update = time.time()

        elapsed_time = time.time() - self.last_frame_update
        interpolate_frac = elapsed_time / SECS_BETWEEN_FRAMES
        return self.prev_start.interpolate(self.prev_end, interpolate_frac)

    def draw_trail(self, screen):
        if len(self.trail) > 0:
            start = tuple(map(int, self.trail[0].get_screen_center()))
            for i in range(1, len(self.trail)):
                end = tuple(map(int, self.trail[i].get_screen_center()))
                pygame.draw.line(screen, (255, 0, 0), start, end, 4)
                start = end

robot_frames = FrameQueue()


class ThreadedUDPRequestHandler(socketserver.BaseRequestHandler):
    def handle(self):
        data = self.request[0].strip()
        socket = self.request[1]
        robot_frames.add(RobotPosition.from_json(data))


class ThreadedUDPServer(socketserver.ThreadingMixIn, socketserver.UDPServer):
    pass

def sendMessage(s):
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.connect((JAVA_SERVER_HOST, JAVA_SERVER_PORT))
    try:
        sock.sendall(s.encode())
    finally:
        sock.close()

def gen_filename():
    return "video/" + str(int(time.time() * 1000)) + ".mp4"

def main(argv):
    server = ThreadedUDPServer(
        ("localhost", PYTHON_SERVER_PORT), ThreadedUDPRequestHandler
    )
    server_thread = threading.Thread(target=server.serve_forever)
    server_thread.daemon = True
    server_thread.start()
    pygame.init()

    gamepad = controller.Controller()
    #pipeline = write_video.FramePipeline(gen_filename())
    screen = pygame.display.set_mode(SCREEN_DIMS)

    done = False
    time_offset = None
    while not done:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                done = True

        # We always need to blit game field and send controller inputs
        screen.blit(RR2_BACKGROUND_FIELD, [0, 0])
        robot_frames.draw_trail(screen)
        gamepad.update()
        sendMessage(gamepad.toJSON())

        frame = robot_frames.get()
        if frame:
            frame.draw(screen)
            #pipeline.write_surface(screen)
        pygame.display.flip()

    #pipeline.close()
    server.shutdown()
    server.server_close()



if __name__ == "__main__":
    main(sys.argv)
