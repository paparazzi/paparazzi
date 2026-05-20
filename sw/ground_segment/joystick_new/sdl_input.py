from __future__ import annotations

import ctypes
import math
from dataclasses import dataclass

import sdl2

from config import JoystickState

STICK_INPUT_DEV_MAX = 15
STICK_AXIS_COUNT = 32
STICK_BUTTON_COUNT = 32


class JoystickError(RuntimeError):
    pass


@dataclass(frozen=True)
class JoystickInfo:
    index: int
    name: str
    axes: int
    buttons: int
    hats: int


def _axis_to_pprz(value: int) -> int:
    return int(((value - -32768) * 254) / (32768 - -32768)) - 127


def list_joysticks() -> list[JoystickInfo]:
    if sdl2.SDL_InitSubSystem(sdl2.SDL_INIT_JOYSTICK | sdl2.SDL_INIT_EVENTS) != 0:
        raise JoystickError(sdl2.SDL_GetError().decode("utf-8", "replace"))
    infos: list[JoystickInfo] = []
    for index in range(sdl2.SDL_NumJoysticks()):
        joystick = sdl2.SDL_JoystickOpen(index)
        if not joystick:
            continue
        raw_name = sdl2.SDL_JoystickName(joystick)
        infos.append(
            JoystickInfo(
                index=index,
                name=raw_name.decode("utf-8", "replace") if raw_name else "",
                axes=sdl2.SDL_JoystickNumAxes(joystick),
                buttons=sdl2.SDL_JoystickNumButtons(joystick),
                hats=sdl2.SDL_JoystickNumHats(joystick),
            )
        )
        sdl2.SDL_JoystickClose(joystick)
    return infos


class JoystickDevice:
    def __init__(self, index: int):
        if sdl2.SDL_Init(sdl2.SDL_INIT_JOYSTICK | sdl2.SDL_INIT_EVENTS) != 0:
            raise JoystickError(sdl2.SDL_GetError().decode("utf-8", "replace"))
        sdl2.SDL_JoystickEventState(sdl2.SDL_ENABLE)
        self.joystick = self._open_or_find(index)
        self.axis_count = min(sdl2.SDL_JoystickNumAxes(self.joystick), STICK_AXIS_COUNT)
        self.button_count = min(sdl2.SDL_JoystickNumButtons(self.joystick), STICK_BUTTON_COUNT)
        self.hat_count = sdl2.SDL_JoystickNumHats(self.joystick)
        self.axis = [0] * self.axis_count
        self.buttons = 0
        self.hat = 0
        self.moved_axis = [False] * self.axis_count
        self.state = JoystickState(axis=list(self.axis), buttons=self.buttons, hat=self.hat, button_count=self.button_count)

    @property
    def name(self) -> str:
        raw_name = sdl2.SDL_JoystickName(self.joystick)
        return raw_name.decode("utf-8", "replace") if raw_name else ""

    def close(self) -> None:
        if self.joystick:
            sdl2.SDL_JoystickClose(self.joystick)
            self.joystick = None
        sdl2.SDL_QuitSubSystem(sdl2.SDL_INIT_JOYSTICK | sdl2.SDL_INIT_EVENTS)

    def wait_until_all_axes_moved(self) -> None:
        while not all(self.moved_axis):
            self.wait_state(None)

    def wait_state(self, timeout: float | None) -> JoystickState | None:
        event = sdl2.SDL_Event()
        if timeout is None:
            has_event = sdl2.SDL_WaitEvent(ctypes.byref(event))
        else:
            timeout_ms = math.ceil(timeout * 1000) if timeout > 0 else 0
            has_event = sdl2.SDL_WaitEventTimeout(ctypes.byref(event), timeout_ms)
        if not has_event:
            return None
        self._process_event(event)
        self.state = JoystickState(axis=list(self.axis), buttons=self.buttons, hat=self.hat, button_count=self.button_count)
        return self.state

    def _open_or_find(self, index: int):
        joystick = sdl2.SDL_JoystickOpen(index)
        if joystick:
            return joystick
        for fallback in range(STICK_INPUT_DEV_MAX):
            joystick = sdl2.SDL_JoystickOpen(fallback)
            if joystick:
                return joystick
        raise JoystickError("no suitable joystick found")

    def _process_event(self, event) -> None:
        event_type = event.type
        if event_type in (sdl2.SDL_JOYBUTTONDOWN, sdl2.SDL_JOYBUTTONUP):
            button = int(event.jbutton.button)
            if button < self.button_count:
                if event.jbutton.state == sdl2.SDL_PRESSED:
                    self.buttons |= 1 << button
                else:
                    self.buttons &= ~(1 << button)
        elif event_type == sdl2.SDL_JOYHATMOTION:
            if event.jhat.hat == 0:
                self.hat = int(event.jhat.value)
        elif event_type == sdl2.SDL_JOYAXISMOTION:
            axis = int(event.jaxis.axis)
            if axis < self.axis_count:
                self.axis[axis] = _axis_to_pprz(int(event.jaxis.value))
                self.moved_axis[axis] = True
        elif event_type == sdl2.SDL_QUIT:
            raise KeyboardInterrupt
