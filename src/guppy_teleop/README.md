# guppy_teleop

Input service for translating multi-device input to Twist and widget-based GUI for monitoring and controlling guppy. 

### Input

Start input individually with `ros2 run guppy_teleop ...`. The entry-points for individual input are `input`/`keyboard`/`controller`. The ChangeState service must be up in order to send state changes.

If an input device is being initialized that you don't want to be used, you can blacklist devices by adding them to the respective lists in the respective classes from `guppy_teleop.input.controller` and `guppy_teleop.input.keyboard`. There are additional options like `DEADZONE`, `LINEAR_MULTIPLIER`, and `ANGULAR_MULTIPLIER` for devices as well.

#### Adding New Input Devices

Input devices are made to be easily extensible by just inheriting either the InputDevice class from `guppy_teleop.input.input_device` or the nearest subclass of InputDevice like the Controller class from `guppy_teleop.input.controller`. After creating a new class, write a discovery function (refer to the `find_controllers()` function from `guppy_teleop.input.controller`) and add it to the parent classes discovery function or tie it into the discovery process when defining the InputHandler (refer to the `main()` function of `guppy_teleop.input_handler` or the `InputWidget.__init__()` method from `guppy_teleop.frontend.widgets.input_widget`)

#### Adding New Commands

The implementations of the `Controller` and `Keyboard` classes both allow you to define commands. To do so, ensure the "command key" (the key(s) that designate an input as a command, ie. `KEY_LEFTCTRL`) you use is added to the `COMMAND_KEYS` list. Then, add a new entry in the `COMMAND_MAP` dict in the form; `dict[Callable, list[int]]`. ie.

```
self.COMMAND_MAP: dict[Callable, list[int]] = {
    ...
    lambda: self.handler.push_state("FAULT"): [ecodes.BTN_SELECT, ecodes.BTN_B]
    ...
}
```
