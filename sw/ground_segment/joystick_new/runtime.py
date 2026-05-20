from __future__ import annotations

import math
import time
from typing import Callable

from config import (
    Actions,
    AxisInput,
    ButtonInput,
    HatInput,
    JoystickState,
    MessageAction,
)
from expressions import (
    MAX_INPUT,
    MIN_INPUT,
    EvalContext,
)


def transformed_axis(raw: int, input_value: AxisInput) -> int:
    value = 0 if abs(raw) < input_value.deadband else raw
    cubic = (float(value) ** 3) / (float(MAX_INPUT) ** 2)
    shaped = (float(value) * (1.0 - input_value.exponent)) + (cubic * input_value.exponent)
    limited = shaped * input_value.limit
    trimmed = int(limited + input_value.trim)
    return max(MIN_INPUT, min(MAX_INPUT, trimmed))


def input_value(state: JoystickState, input_def: AxisInput | ButtonInput | HatInput) -> int:
    if isinstance(input_def, AxisInput):
        try:
            return transformed_axis(state.axis[input_def.index], input_def)
        except IndexError as exc:
            raise IndexError(f"axis {input_def.index} not available") from exc
    if isinstance(input_def, ButtonInput):
        return (state.buttons >> input_def.index) & 0x1
    if isinstance(input_def, HatInput):
        return state.hat
    raise TypeError(input_def)


class JoystickRuntime:
    def __init__(
        self,
        actions: Actions,
        ac_id: int,
        send_message: Callable[[str, str, dict[str, object]], None],
        joystick_id: int,
        verbose: bool = False,
    ):
        self.actions = actions
        self.ac_id = ac_id
        self.send_message = send_message
        self.joystick_id = joystick_id
        self.verbose = verbose
        self.last_values: dict[tuple[str, str], tuple[bool, tuple[tuple[str, object], ...]]] = {}

    def execute_once(self, state: JoystickState) -> None:
        names = self._names(state)
        context = EvalContext(names=names, hat=state.hat, joystick_id=self.joystick_id)
        self._update_variables(context)
        names.update({name: variable.value for name, variable in self.actions.variables.items()})
        context = EvalContext(names=names, hat=state.hat, joystick_id=self.joystick_id)

        if self.verbose:
            print(self._format_state(state, names))

        for message in self.actions.messages:
            self._execute_message(message, context)

    def run(self, device: "JoystickDevice") -> None:
        next_tick = time.monotonic()
        while True:
            timeout = max(0.0, next_tick - time.monotonic())
            state = device.wait_state(timeout)
            now = time.monotonic()
            if state is not None and self.verbose:
                self._format_state(state, self._names(state))
            if now >= next_tick:
                self.execute_once(device.state)
                next_tick += self.actions.period
                if now - next_tick > self.actions.period:
                    next_tick = now + self.actions.period

    def _names(self, state: JoystickState) -> dict[str, int]:
        values = {name: input_value(state, input_def) for name, input_def in self.actions.inputs.items()}
        values.update({name: variable.value for name, variable in self.actions.variables.items()})
        return values

    def _update_variables(self, context: EvalContext) -> None:
        for variable in self.actions.variables.values():
            for value, expression in variable.events:
                if expression.eval(context) != 0:
                    variable.value = value

    def _execute_message(self, message: MessageAction, context: EvalContext) -> None:
        values: dict[str, object] = {}
        for field in message.fields:
            evaluated = [expr.eval(context) for expr in field.expressions]
            values[field.name] = evaluated if _is_array_type(field.type_name) else evaluated[0]

        on_event = True if message.on_event is None else message.on_event.eval(context) != 0
        comparable = tuple(values.items())
        key = (message.msg_class, message.name)
        previous = self.last_values.get(key)
        should_send = (((on_event, comparable) != previous) or message.send_always) and on_event
        if should_send:
            if message.has_ac_id:
                values = {"ac_id": str(self.ac_id) if message.msg_class == "ground" else self.ac_id, **values}
            self.send_message(message.msg_class, message.name, values)
        self.last_values[key] = (on_event, comparable)

    @staticmethod
    def _format_state(state: JoystickState, names: dict[str, int]) -> str:
        buttons = " ".join(f"{i}:{(state.buttons >> i) & 1}" for i in range(state.button_count))
        axes = " ".join(f"{i}:{value}" for i, value in enumerate(state.axis))
        named = " ".join(f"{name}:{value}" for name, value in sorted(names.items()))
        return f"buttons: {buttons}\nhat: {state.hat}\naxis: {axes}\ninputs: {named}"


def _is_array_type(type_name: str) -> bool:
    return "[" in type_name or type_name == "string"
