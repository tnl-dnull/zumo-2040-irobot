# Implementation of iRobot Open Create interface (v2.0)
#  see https://edu.irobot.com/learning-library/create-2-oi-spec for details

# The interface takes in command bytes and may respond or react.

# This implementation originally built with the Zumo 2040 in mind.  Due to this,
#  some iRobot commands may be left out (for example brush motor, clean, etc.)

from zumo_2040_robot import robot
from machine import UART, Pin
import time
import irobot_handler_display
import irobot_interface

# Interface uart is 115200 8n1 by default
uart = UART(0, baudrate=115200, tx=Pin(28), rx=Pin(29))
uart.write("irobot alt v1.0\r\n")

button_a = robot.ButtonA()
button_b = robot.ButtonB()
button_c = robot.ButtonC()
motors = robot.Motors()
encoders = robot.Encoders()

irobot_handler_display.init_handler_display()

arrows = ["v", None, "^"]
left_dir = 1
right_dir = 1
left_speed = 0
right_speed = 0
last_update_time = 0
button_count_a = 0
button_count_c = 0

while True:
    
    # Update the LCD and motors every 50 ms.
    if time.ticks_diff(time.ticks_ms(), last_update_time) > 50:
        # get command on enter 
        # TODO: change to read in buffer, process within x ms?
        command = uart.read()

        # quick command comparison
        if command is not None:
            response = irobot_interface_process(command)

            if response is not None:
                uart.write(response)

        last_update_time = time.ticks_ms()

        if button_a.is_pressed():
            # debounce TODO: can these be combined into a button handler? (a,b,c)
            if button_count_a < 4:
                button_count_a += 1
            else:
                # do something on b-press
        else:
            button_count_a = 0

        if button_c.is_pressed():
            # debounce...
            if button_count_c < 4:
                button_count_c += 1
            else:
                # do something on b-press
        else:
            button_count_c = 0

        if left_speed < 0: left_speed = 0
        if left_speed > motors.MAX_SPEED: left_speed = motors.MAX_SPEED
        if right_speed < 0: right_speed = 0
        if right_speed > motors.MAX_SPEED: right_speed = motors.MAX_SPEED

        motors.set_speeds(left_dir * left_speed, right_dir * right_speed)

        irobot_handler_display.update_handler_display()

