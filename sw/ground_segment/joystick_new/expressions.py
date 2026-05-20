from __future__ import annotations

import operator
from dataclasses import dataclass
from typing import Callable

from lark import Lark, Token, Transformer, UnexpectedInput, v_args
from lark.exceptions import VisitError


class ExpressionError(ValueError):
    pass


MIN_INPUT = -127
MAX_INPUT = 127


def split_expressions(value: str) -> list[str]:
    parts: list[str] = []
    current: list[str] = []
    quote: str | None = None
    for char in value:
        if quote:
            current.append(char)
            if char == quote:
                quote = None
        elif char in ("'", '"'):
            quote = char
            current.append(char)
        elif char == ";":
            part = "".join(current).strip()
            if part:
                parts.append(part)
            current = []
        else:
            current.append(char)
    part = "".join(current).strip()
    if part:
        parts.append(part)
    return parts


Node = tuple[object, ...]


GRAMMAR = r"""
    ?start: comparison
    ?comparison: bit_or (COMP_OP bit_or)* -> comparison
    ?bit_or: bit_and ("or" bit_and)* -> bit_or
    ?bit_and: sum ("and" sum)* -> bit_and
    ?sum: product ((PLUS | MINUS) product)* -> bin_chain
    ?product: unary ((STAR | SLASH) unary)* -> bin_chain
    ?unary: PLUS unary -> pos
          | MINUS unary -> neg
          | atom
    ?atom: INT -> int_value
         | STRING -> string_value
         | NAME "(" [args] ")" -> call
         | NAME -> name
         | "(" comparison ")"
    args: comparison ("," comparison)*

    COMP_OP: "lt" | "le" | "gt" | "ge" | "eq" | "ne" | ">=" | "==" | "!=" | ">"
    PLUS: "+"
    MINUS: "-"
    STAR: "*"
    SLASH: "/"
    INT: /0x[0-9a-fA-F]+|[0-9]+/
    NAME: /[A-Za-z_][A-Za-z0-9_]*(\.[A-Za-z_][A-Za-z0-9_]*)*/
    STRING: /'[^']*'|"[^"]*"/

    %import common.WS_INLINE
    %ignore WS_INLINE
"""


PARSER = Lark(GRAMMAR, parser="lalr", maybe_placeholders=False)


@v_args(inline=True)
class _ToNode(Transformer):
    def int_value(self, token: Token) -> Node:
        return ("int", int(str(token), 0))

    def string_value(self, token: Token) -> Node:
        text = str(token)
        return ("string", text[1:-1])

    def name(self, token: Token) -> Node:
        return ("name", str(token))

    def args(self, *items: Node) -> list[Node]:
        return list(items)

    def call(self, name: Token, args: list[Node] | None = None) -> Node:
        return ("call", str(name), args or [])

    def pos(self, _op: Token, value: Node) -> Node:
        return value

    def neg(self, _op: Token, value: Node) -> Node:
        return ("neg", value)

    def bin_chain(self, first: Node, *rest: object) -> Node:
        node = first
        for op, value in zip(rest[0::2], rest[1::2]):
            node = ("bin", str(op), node, value)
        return node

    def bit_and(self, first: Node, *rest: Node) -> Node:
        node = first
        for value in rest:
            node = ("bin", "and", node, value)
        return node

    def bit_or(self, first: Node, *rest: Node) -> Node:
        node = first
        for value in rest:
            node = ("bin", "or", node, value)
        return node

    def comparison(self, first: Node, *rest: object) -> Node:
        if not rest:
            return first
        pairs = [(str(op), value) for op, value in zip(rest[0::2], rest[1::2])]
        return ("compare", first, pairs)


@dataclass(frozen=True)
class Expression:
    text: str
    tree: Node

    @classmethod
    def parse(cls, text: str) -> "Expression":
        try:
            tree = _ToNode().transform(PARSER.parse(text))
        except UnexpectedInput as exc:
            raise ExpressionError(f"invalid expression {text!r}: {exc}") from exc
        except VisitError as exc:
            if isinstance(exc.orig_exc, ExpressionError):
                raise exc.orig_exc from exc
            raise ExpressionError(f"invalid expression {text!r}: {exc.orig_exc}") from exc
        return cls(text=text, tree=tree)

    def resolve(self, resolver: Callable[[str, list[object]], int | None]) -> "Expression":
        return Expression(text=self.text, tree=_resolve_constants(self.tree, resolver))

    def eval(self, context: "EvalContext") -> int:
        return int(_eval_node(self.tree, context))


@dataclass(frozen=True)
class EvalContext:
    names: dict[str, int]
    hat: int
    joystick_id: int


def parse_many(value: str) -> list[Expression]:
    return [Expression.parse(part) for part in split_expressions(value)]


def _resolve_constants(node: Node, resolver: Callable[[str, list[object]], int | None]) -> Node:
    if node[0] == "call" and node[1] in {"IndexOfSetting", "IndexOfBlock", "IndexOfEnum"}:
        args = [_literal_or_name(arg) for arg in node[2]]
        value = resolver(str(node[1]), args)
        if value is not None:
            return ("int", int(value))
    if node[0] in {"neg"}:
        return (node[0], _resolve_constants(node[1], resolver))
    if node[0] == "bin":
        return ("bin", node[1], _resolve_constants(node[2], resolver), _resolve_constants(node[3], resolver))
    if node[0] == "compare":
        return ("compare", _resolve_constants(node[1], resolver), [(op, _resolve_constants(value, resolver)) for op, value in node[2]])
    if node[0] == "call":
        return ("call", node[1], [_resolve_constants(arg, resolver) for arg in node[2]])
    return node


def _literal_or_name(node: Node) -> object:
    if node[0] in {"int", "string", "name"}:
        return node[1]
    if node[0] == "neg":
        value = _literal_or_name(node[1])
        if isinstance(value, int):
            return -value
    raise ExpressionError(f"unsupported constant argument {node!r}")


def scale(x: int, minimum: int, maximum: int) -> int:
    return minimum + ((x - MIN_INPUT) * (maximum - minimum)) // (MAX_INPUT - MIN_INPUT)


def bound(x: int, minimum: int, maximum: int) -> int:
    return max(minimum, min(maximum, x))


def fit(x: int, minimum: int, maximum: int, input_minimum: int, input_maximum: int) -> int:
    if maximum == minimum:
        raise ExpressionError("Fit range cannot have equal min and max")
    value = input_minimum + ((x - minimum) * (input_maximum - input_minimum)) // (maximum - minimum)
    return bound(value, input_minimum, input_maximum)


def pprz_mode(mode: int) -> int:
    threshold = MAX_INPUT // 2
    if mode > threshold:
        return 2
    if mode < -threshold:
        return 0
    return 1


HAT_VALUES = {
    "HatCentered": 0,
    "HatUp": 1,
    "HatRight": 2,
    "HatRightUp": 3,
    "HatDown": 4,
    "HatRightDown": 6,
    "HatLeft": 8,
    "HatLeftUp": 9,
    "HatLeftDown": 12,
}


BIN_OPS = {
    "+": operator.add,
    "-": operator.sub,
    "*": operator.mul,
    "/": operator.floordiv,
    "and": operator.and_,
    "or": operator.or_,
}


COMPARE_OPS = {
    ">": operator.gt,
    ">=": operator.ge,
    "==": operator.eq,
    "!=": operator.ne,
    "lt": operator.lt,
    "le": operator.le,
    "gt": operator.gt,
    "ge": operator.ge,
    "eq": operator.eq,
    "ne": operator.ne,
}


def _eval_node(node: Node, context: EvalContext) -> int:
    kind = node[0]
    if kind == "int":
        return int(node[1])
    if kind == "string":
        raise ExpressionError(f"unsupported literal {node[1]!r}")
    if kind == "name":
        name = str(node[1])
        try:
            return context.names[name]
        except KeyError as exc:
            raise ExpressionError(f"unknown input or variable {name!r}") from exc
    if kind == "neg":
        return -_eval_node(node[1], context)
    if kind == "bin":
        op = str(node[1])
        return int(BIN_OPS[op](_eval_node(node[2], context), _eval_node(node[3], context)))
    if kind == "compare":
        left = _eval_node(node[1], context)
        for op, value in node[2]:
            right = _eval_node(value, context)
            if not COMPARE_OPS[op](left, right):
                return 0
            left = right
        return 1
    if kind == "call":
        func = str(node[1])
        if func in HAT_VALUES:
            return 1 if context.hat == HAT_VALUES[func] else 0
        args = [_eval_node(arg, context) for arg in node[2]]
        if func == "Scale" and len(args) == 3:
            return scale(*args)
        if func == "Fit" and len(args) == 5:
            return fit(*args)
        if func == "Bound" and len(args) == 3:
            return bound(*args)
        if func == "PprzMode" and len(args) == 1:
            return pprz_mode(args[0])
        if func == "JoystickID" and not args:
            return context.joystick_id
        raise ExpressionError(f"unknown function {func!r}")
    raise ExpressionError(f"unsupported expression {node!r}")
