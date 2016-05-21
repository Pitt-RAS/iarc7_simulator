import math
import time

KP_ALTITUDE = 0.3
KI_ALTITUDE = 0.1
KD_ALTITUDE = 0.1

altitude_setpoint = 2

def constrain(x, a, b):
    return max(a, min(x, b))

integral = 0.58 # estimated holding throttle
last_time = time.time()

altitude_buffer_size = 1
altitude_buffer = [0 for i in range(altitude_buffer_size)]

start_time = time.time()

initialized = False
def _init(quad):
    global start_time, last_time, initialized
    start_time = time.time()
    last_time = time.time()
    initialized = True

def loop(quad):
    if not initialized: _init(quad)

    global altitude_setpoint
    altitude = quad.get_current_altitude()
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

    altitude_change = (altitude - altitude_buffer.pop(0)) / altitude_buffer_size
    altitude_buffer.append(altitude)

    if curr_time - start_time > 15:
        quad.set_target_pitch(0)
        quad.set_target_heading(1)
        altitude_setpoint = -0.01
    elif curr_time - start_time > 10:
        quad.set_target_pitch(-math.pi / 18)
        quad.set_target_heading(-1)
        altitude_setpoint = 4
    elif curr_time - start_time > 5:
        quad.set_target_pitch(math.pi / 18)
        altitude_setpoint = 3

    throttle = KP_ALTITUDE * error
    throttle += integral
    throttle -= KD_ALTITUDE * altitude_change / dt
    print(('%.3f\t'*5)%(error, integral, altitude_change, dt, throttle))
    quad.set_throttle(constrain(throttle, 0.01, 1))
