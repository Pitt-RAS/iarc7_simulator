import time

import quadcopter

KP_ALTITUDE = 5
KI_ALTITUDE = 1
KD_ALTITUDE = 0.75

altitude_setpoint = 2

def constrain(x, a, b):
    return max(a, min(x, b))

integral = 0
last_time = time.time()
last_altitude = 0

start_time = time.time()

def loop():
    altitude = quadcopter.get_altitude()
    error = altitude_setpoint - altitude

    global last_time
    curr_time = time.time()
    dt = curr_time - last_time
    last_time = curr_time

    global integral
    integral += KI_ALTITUDE * dt * error

    if integral > 1:
        integral = 1
    elif integral < -1:
        integral = -1

    global last_altitude
    altitude_change = altitude - last_altitude
    last_altitude = altitude

    global altitude_setpoint
    if curr_time - start_time > 3:
        quadcopter.set_pitch(0)
        altitude_setpoint = 1
    elif curr_time - start_time > 2:
        quadcopter.set_pitch(-10)
        altitude_setpoint = 4
    elif curr_time - start_time > 1:
        quadcopter.set_pitch(10)
        altitude_setpoint = 3

    throttle = KP_ALTITUDE * error
    throttle += integral
    throttle -= KD_ALTITUDE * altitude_change / dt
    quadcopter.set_throttle(constrain(throttle, 0, 1))
    quadcopter.get_camera_image()
