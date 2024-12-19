# iRobot Handler display helper.  Init and update display.

from zumo_2040_robot import robot

display = robot.Display()

def init_handler_display():
    display.text("Hold=run", 32, 0)
    display.text("Tap=flip", 32, 8)
    display.text("A", 8, 28)
    display.text("C", 112, 28)
    display.text("L: ", 24, 48)
    display.text("R: ", 24, 56)

def update_handler_display()
    display.fill_rect(0, 0, 8, 64, 0)
    y = 28 + -left_dir * left_speed // 225
    display.text(arrows[left_dir + 1], 0, y)

    display.fill_rect(120, 0, 8, 64, 0)
    y = 28 + -right_dir * right_speed // 225
    display.text(arrows[right_dir + 1], 120, y)

    display.fill_rect(40, 48, 64, 16, 0)
    left_encoder, right_encoder = encoders.get_counts()
    display.text(f"{left_encoder:>8}", 40, 48)
    display.text(f"{right_encoder:>8}", 40, 56)

    display.show()