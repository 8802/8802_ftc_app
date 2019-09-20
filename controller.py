"""controller.py: Converts controller input to FTC JSON."""
__author__      = "Gavin Uberti"

import json

from libs import inputs

# -1 maps to dpad_up and dpad_left,
# 1 maps to dpad_right and dpad_down
HAT_MAPPINGS = {
    "ABS_HAT0X": ("dpad_left", "dpad_right"),
    "ABS_HAT0Y": ("dpad_up", "dpad_down")
}

STICK_MAPPINGS = {
    "ABS_X": ("left_stick_x", 32767, -1),
    "ABS_Y": ("left_stick_y", 32767, 1),
    "ABS_RX": ("right_stick_x", 32767, -1),
    "ABS_RY": ("right_stick_y", 32767, 1),
    "ABS_Z": ("left_trigger", 255, 1),
    "ABS_RZ": ("right_trigger", 255, 1),
}

BUTTON_MAPPINGS = {
    "BTN_NORTH": "y",
    "BTN_SOUTH": "a",
    "BTN_EAST": "b",
    "BTN_WEST": "x",
    "BTN_TL": "left_bumper",
    "BTN_TR": "right_bumper",
    "BTN_THUMBL": "left_stick_button",
    "BTN_THUMBR": "right_stick_button"
}

class Controller:
    def _set_prop(self, prop, value):
        self.properties[prop] = value

    def _reset_all_props(self):
        for props in HAT_MAPPINGS.values():
            for prop in props:
                self._set_prop(prop, False)

        for prop in STICK_MAPPINGS.values():
            self._set_prop(prop[0], 0)

        for prop in BUTTON_MAPPINGS.values():
            self._set_prop(prop, False)

    def __init__(self, gamepad_id=0):
        # Set gamepad reference
        self.gamepad_id = gamepad_id
        self.properties = {}
        self._reset_all_props()

    def update(self):
        gamepads = inputs.devices.gamepads
        if (len(gamepads) - 1) < self.gamepad_id:
            self._reset_all_props()
            return

        # If we do have a gamepad, continue as planned
        event_list = gamepads[self.gamepad_id].read()
        while len(event_list):
            event = event_list[0]

            # First do hat
            if event.code in HAT_MAPPINGS:
                mapping = HAT_MAPPINGS[event.code]
                self._set_prop(mapping[0], event.state == -1)
                self._set_prop(mapping[1], event.state == 1)
            # Now do sticks
            elif event.code in STICK_MAPPINGS:
                mapping = STICK_MAPPINGS[event.code]
                value = min(1, max(-1, event.state / mapping[1])) * mapping[2]
                self._set_prop(mapping[0], float(value))
            # Now do buttons
            elif event.code in BUTTON_MAPPINGS:
                self._set_prop(BUTTON_MAPPINGS[event.code], bool(event.state))
            event_list = gamepads[self.gamepad_id].read()

    def toJSON(self):
        return json.dumps(self.properties)

# For testing
def main():
    controller = Controller()
    while True:
        controller.update()
        print(controller.toJSON())

if __name__ == "__main__":
    main()
