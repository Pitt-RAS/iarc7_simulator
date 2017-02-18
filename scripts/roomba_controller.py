#! /usr/bin/env python3

from pymorse import Morse
from roomba import Roomba, Obstacle
import time

with Morse() as simu:
    roombas = []
    obstacles = []
    for robot_name in simu.robots:
        if 'Roomba' in robot_name:
            roombas.append(Roomba(getattr(simu, robot_name)))
        elif 'Obstacle' in robot_name:
            obstacles.append(Obstacle(getattr(simu, robot_name)))

    for roomba in roombas:
        roomba.send_start_signal()
    for obstacle in obstacles:
        obstacle.send_start_signal()

    last_time = 0
    while True:
        while time.time() - last_time < 0.02: pass
        last_time = time.time()
        for roomba in roombas:
            roomba.loop()
        for obstacle in obstacles:
            obstacle.loop()
