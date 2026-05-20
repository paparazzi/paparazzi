from __future__ import annotations

import xml.etree.ElementTree as ET
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

from env import ensure_pprz_paths, paparazzi_home
from expressions import Expression, parse_many


ensure_pprz_paths()
from pprzlink import messages_xml_map

messages_xml_map.parse_messages(str(paparazzi_home() / "var" / "messages.xml"))

MESSAGE_CLASSES = {"datalink", "ground"}


class ConfigError(ValueError):
    pass


@dataclass
class AxisInput:
    index: int
    deadband: int = 0
    limit: float = 1.0
    exponent: float = 0.0
    trim: float = 0.0


@dataclass(frozen=True)
class ButtonInput:
    index: int


@dataclass(frozen=True)
class HatInput:
    index: int


Input = AxisInput | ButtonInput | HatInput


@dataclass
class Variable:
    value: int
    events: list[tuple[int, Any]] = field(default_factory=list)


@dataclass(frozen=True)
class MessageField:
    name: str
    type_name: str
    expressions: list[Any]


@dataclass(frozen=True)
class MessageAction:
    msg_class: str
    name: str
    fields: list[MessageField]
    on_event: Any | None
    send_always: bool
    has_ac_id: bool


@dataclass
class Actions:
    period: float
    inputs: dict[str, Input]
    variables: dict[str, Variable]
    messages: list[MessageAction]


@dataclass(frozen=True)
class JoystickState:
    axis: list[int]
    buttons: int
    hat: int
    button_count: int


@dataclass(frozen=True)
class MessageDef:
    msg_class: str
    name: str
    fields: dict[str, str]
    enums: dict[str, list[str]]


def parse_config(
    xml_file: Path,
    setting_indices: dict[str, int],
    block_indices: dict[str, int],
) -> Actions:
    root = ET.parse(xml_file).getroot()
    input_node = _required(root, "input", xml_file)
    messages_node = _required(root, "messages", xml_file)

    inputs = _parse_inputs(input_node)
    variables = _parse_variables(root.find("variables"))
    messages = [
        _parse_message(node, setting_indices, block_indices)
        for node in list(messages_node)
        if node.tag == "message"
    ]
    period = float(messages_node.attrib["period"])
    return Actions(period=period, inputs=inputs, variables=variables, messages=messages)


def _required(root: ET.Element, tag: str, xml_file: Path) -> ET.Element:
    node = root.find(tag)
    if node is None:
        raise ConfigError(f"{xml_file} has no <{tag}> node")
    return node


def _parse_inputs(input_node: ET.Element) -> dict[str, AxisInput | ButtonInput | HatInput]:
    inputs: dict[str, AxisInput | ButtonInput | HatInput] = {}
    for node in list(input_node):
        name = node.attrib["name"]
        index = int(node.attrib["index"])
        if name in inputs:
            continue
        if node.tag == "axis":
            inputs[name] = AxisInput(
                index=index,
                deadband=int(node.attrib.get("deadband", "0")),
                limit=float(node.attrib.get("limit", "1.0")),
                exponent=float(node.attrib.get("exponent", "0.0")),
                trim=float(node.attrib.get("trim", "0.0")),
            )
        elif node.tag == "button":
            inputs[name] = ButtonInput(index=index)
        elif node.tag == "hat":
            inputs[name] = HatInput(index=index)
        else:
            raise ConfigError(f"unexpected input tag <{node.tag}>")
    return inputs


def _parse_variables(variables_node: ET.Element | None) -> dict[str, Variable]:
    if variables_node is None:
        return {}
    variables: dict[str, Variable] = {}
    for node in list(variables_node):
        if node.tag == "var":
            name = node.attrib["name"]
            if name in variables:
                raise ConfigError(f"variable {name!r} is declared twice")
            variables[name] = Variable(value=int(node.attrib["default"]))
    for node in list(variables_node):
        if node.tag == "set":
            name = node.attrib["var"]
            if name not in variables:
                raise ConfigError(f"<set> references unknown variable {name!r}")
            variables[name].events.append((int(node.attrib["value"]), Expression.parse(node.attrib["on_event"])))
    return variables


def _parse_message(
    node: ET.Element,
    setting_indices: dict[str, int],
    block_indices: dict[str, int],
) -> MessageAction:
    msg_class = node.attrib["class"]
    name = node.attrib["name"]
    send_always = node.attrib.get("send_always") == "true"
    on_event = Expression.parse(node.attrib["on_event"]) if "on_event" in node.attrib else None

    if msg_class not in MESSAGE_CLASSES:
        raise ConfigError(f"unsupported message class {msg_class!r}")

    msg_def = _message_def(msg_class, name)

    fields: list[MessageField] = []
    for field_node in list(node):
        if field_node.tag != "field":
            continue
        field_name = field_node.attrib["name"]
        if field_name not in msg_def.fields:
            raise ConfigError(f"{msg_class}.{name} has no field {field_name!r}")
        def resolver(func: str, args: list[object], field_name: str = field_name) -> int | None:
            return _resolve_static_call(func, args, msg_def, field_name, setting_indices, block_indices)
        expressions = [expr.resolve(resolver) for expr in parse_many(field_node.attrib["value"])]
        fields.append(MessageField(field_name, msg_def.fields[field_name], expressions))

    return MessageAction(msg_class, name, fields, on_event, send_always, "ac_id" in msg_def.fields)


def _message_def(msg_class: str, name: str) -> MessageDef:
    try:
        msg_id = messages_xml_map.get_msg_id(msg_class, name)
        field_names = messages_xml_map.get_msg_fields(msg_class, name)
        field_types = messages_xml_map.get_msg_fieldtypes(msg_class, msg_id)
        enum_values = messages_xml_map.get_msg_values_enum(msg_class, msg_id)
    except Exception as exc:
        raise ConfigError(f"message {msg_class}.{name} not found in pprzlink message definitions") from exc

    fields = dict(zip(field_names, field_types))
    enums = {
        field_name: values
        for field_name, values in zip(field_names, enum_values)
        if values is not None
    }
    return MessageDef(msg_class, name, fields, enums)


def _resolve_static_call(
    func: str,
    args: list[object],
    msg_def: MessageDef,
    field_name: str,
    setting_indices: dict[str, int],
    block_indices: dict[str, int],
) -> int | None:
    if func == "IndexOfSetting" and len(args) == 1:
        return setting_indices[str(args[0])]
    if func == "IndexOfBlock" and len(args) == 1:
        return block_indices[str(args[0])]
    if func == "IndexOfEnum" and len(args) == 1:
        enum_name = str(args[0])
        values = msg_def.enums.get(field_name, [])
        if enum_name in values:
            return values.index(enum_name)
        raise ConfigError(f"{msg_def.msg_class}.{msg_def.name}.{field_name} has no enum value {enum_name!r}")
    return None
