# iRobot Handler display helper.  Init and update display.

from zumo_2040_robot import robot

display = robot.Display()

def init_handler_display():
    display.fill_rect(0, 0, 8, 64, 0)
    display.text(f"Start Test...", 10, 10)
    display.show()

def update_handler_display(text):
    #
    #display.fill_rect(40, 48, 64, 16, 0)
    display.text(f"{text}", 40, 48)
    display.show()