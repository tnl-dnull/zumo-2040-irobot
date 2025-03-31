# iRobot Handler display helper.  Init and update display.

from zumo_2040_robot import robot

display = robot.Display()

def init_handler_display():
    display.fill_rect(0, 0, 8, 64, 0)
    display.text(f"Start Test...", 10, 10)
    display.show()

def display_centered_text(text, y=56, show=True):
    display.fill_rect(0, y, 128, 16, 0)
    display.text(text, (128 - len(text) * 8) // 2, y)
    if show: display.show()

def update_handler_display(text):
    #
    display_centered_text(f"{text}")