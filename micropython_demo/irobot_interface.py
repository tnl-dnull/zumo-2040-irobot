# iRobot interface details...

# ... call into the interface with byte array / command sequence, 
#   interface will react and may return response?  
import irobot_display

def irobot_interface_process(command):

    # pull out the command byte...
    # if len(command) < 1:
    #     return (False,'')
    # command and data
    #if len(command) > 1:
        
    # command
    #cmd_byte = command[0]
    if command == 128:
        irobot_display.update_handler_display("cmd 128") 
    # reset
    elif command == 7:
        irobot_display.update_handler_display("cmd 7")
    # stop
    elif command == 173:
        irobot_display.update_handler_display("cmd 173")
    # full
    elif command == 132:
        irobot_display.update_handler_display("cmd 132")
    # drive
    elif command == 137:
        irobot_display.update_handler_display("cmd 137")
    # drive direct
    elif command == 145:
        irobot_display.update_handler_display("cmd 145")
    else :
        irobot_display.update_handler_display("rx byte " + str(command))
    # other commands TBD...
    #
    # drive pwm?
    # leds
    # digit leds ascii  - aka display text...
    # song
    # play - only have piezo i think on zumo?  this may be limited?
    # sensors
    # query list
    # stream
    #return (True,"response")
    # (False,)

# define list of supported status responses...