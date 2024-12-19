# iRobot interface details...

# ... call into the interface with byte array / command sequence, 
#   interface will react and may return response?  


def irobot_interface_process(command):

    # pull out the command byte...
    if len(command) < 1:
        return (False,)
    # command and data
    #if len(command) > 1:
        
    # command
    cmd_byte = command[0]
    match cmd_byte:
        # start
        case 128:
            
        # reset
        case 7:
            
        # stop
        case 173:

        # full
        case 132:
            
        # drive
        case 137:

        # drive direct
        case 145:
            
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

        case _:
            return (False,)

    return (True,"response")
    # (False,)

# define list of supported status responses...