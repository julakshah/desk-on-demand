from inputs import get_gamepad


"""
INPUT LOGGING NOTES FOR 8BitDo Ultimate 2C
event.ev_type: the category of input
  "Absolute": joystick inputs, d-pad, triggers
    D-pad
      Pressed = -1
      Unpressed = 0
    Trigger
      Value = 0-255
    Joystick
      value_xy = int_16
  "Key": abxy, bumpers, other buttons
    Pressed = 1
    Unpressed = 0
event.code: the code denoting each sensor
  Left Joystick, gives two outputs for the hall effect sensors:
    ABS_X
    ABS_Y
  Right Joystick:
    ABS_RX
    ABS_RY
  Left Trigger:
    ABS_Z
  Right Trigger:
    ABS_RZ
  ABXY, in that order:
    BTN_SOUTH
    BTN_EAST
    BTN_NORTH
    BTN_WEST

"""


def main():
    """Just print out some event infomation when the gamepad is used."""
    while 1:
        events = get_gamepad()
        for event in events:
            print(event.ev_type, event.code, event.state)


if __name__ == "__main__":
    main()
