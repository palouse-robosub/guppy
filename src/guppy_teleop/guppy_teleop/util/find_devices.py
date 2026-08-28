from evdev import InputDevice as EvdevDevice  # , categorize
from evdev import ecodes, list_devices

from guppy_teleop.input.controller import Controller
from guppy_teleop.input.input_device import InputDevice
from guppy_teleop.input.joystick import Joystick
from guppy_teleop.input.keyboard import Keyboard
from guppy_teleop.util.device_mode import DeviceMode
from guppy_teleop.util.device_priority import DevicePriority


def find_joysticks(
    handler,
    mode: DeviceMode = DeviceMode.DISABLED,
    priority: DevicePriority = DevicePriority.MEDIUM,
) -> list[Joystick]:
    sub_devices: list[InputDevice] = []

    sub_paths: list[str] = [device._device.path for device in sub_devices]

    paths: list[str] = []

    for path in list_devices():
        if path in sub_paths:
            continue

        device = EvdevDevice(path)

        capabilities = device.capabilities()

        if (
            ecodes.EV_ABS in capabilities
            and ecodes.EV_KEY in capabilities
            and ecodes.BTN_PINKIE in capabilities[ecodes.EV_KEY]
        ):
            paths.append(path)

        # print("\n\n")
        # print(device.name)
        # print(device.capabilities(verbose=True))

    devices: list[InputDevice] = []

    devices.extend(sub_devices)
    devices.extend([Joystick(handler, path, mode, None, priority) for path in paths])
    # TODO no need to pass path if you just created the freakin device,
    # check type if it's string or not in constructor

    return devices


def find_controllers(
    handler,
    mode: DeviceMode = DeviceMode.DISABLED,
    priority: DevicePriority = DevicePriority.MEDIUM,
    find_subclasses: bool = True,
) -> list[Controller]:
    sub_devices: list[InputDevice] = []

    if find_subclasses:  # subclasses
        sub_devices.extend(find_joysticks(handler, mode, priority, find_subclasses))

    sub_paths: list[str] = [device._device.path for device in sub_devices]

    paths: list[str] = []

    for path in list_devices():
        if path in sub_paths:
            continue

        device = EvdevDevice(path)

        capabilities = device.capabilities()

        if (
            ecodes.EV_ABS in capabilities
            and ecodes.EV_KEY in capabilities
            and ecodes.BTN_MODE in capabilities[ecodes.EV_KEY]
        ):
            paths.append(path)

            # print("\n\n")
            # print(device.name)
            # print(device.capabilities(verbose=True))

    devices: list[InputDevice] = []

    devices.extend(sub_devices)
    devices.extend([Controller(handler, path, mode, None, priority) for path in paths])
    # TODO no need to pass path if you just created the freakin device,
    # check type if it's string or not in constructor

    return devices


def find_keyboards(
    handler,
    mode: DeviceMode = DeviceMode.DISABLED,
    priority=DevicePriority.MEDIUM,
) -> list[Keyboard]:
    sub_devices: list[InputDevice] = []

    sub_paths: list[str] = [device._device.path for device in sub_devices]

    paths: list[str] = []

    for path in list_devices():
        if path in sub_paths:
            continue

        device = EvdevDevice(path)

        capabilities = device.capabilities()

        if (
            ecodes.EV_KEY in capabilities
            and ecodes.KEY_SPACE in capabilities[ecodes.EV_KEY]
            and device.name not in Keyboard.BLACKLISTED_DEVICES
        ):
            paths.append(path)

            # print("\n\n")
            # print(device.name)
            # print(device.capabilities(verbose=True))

    devices: list[InputDevice] = []

    devices.extend(sub_devices)
    devices.extend([Keyboard(handler, path, mode, None, priority) for path in paths])
    # TODO no need to pass path if you just created the freakin
    # device,check type if it's string or not in constructor

    return devices
