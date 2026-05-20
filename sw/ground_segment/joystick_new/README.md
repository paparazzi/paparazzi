# Headless Joystick Ivy Bridge

This is a Python rewrite of `sw/ground_segment/joystick/input2ivy.ml`.
It reads joystick XML profiles from `conf/joystick`, applies the same input
transforms and expression helpers, and sends Paparazzi Ivy messages.

## Running

```sh
python3 sw/ground_segment/joystick_new/pprz_joystick.py --list
python3 sw/ground_segment/joystick_new/pprz_joystick.py -d 0 my_joystick.xml
```

## Dependencies

Runtime dependencies:

- `PySDL2` plus the SDL2 shared library, for joystick input.
- `ivy-python` and Paparazzi's generated `pprzlink` Python package, for Ivy bus sending.
- `lxml`, required by `pprzlink` message catalog parsing.
- `lark`, for parsing joystick XML expressions.

## Compatibility Notes

- Ivy messages are sent with the legacy `input2ivy` sender token, for example
  `input2ivy JOYSTICK_RAW ...`.
- If duplicate input names are declared, the first declaration wins.
- Expressions use XML-safe `and` and `or` operators. Less-than comparisons use
  `lt` or `le` instead of raw XML `<` or `<=`.
- Message de-duplication is keyed by `(class, name)`, not only by name.

Hardware and Ivy integration still need validation on a machine with a real
joystick and `ivy-python` installed.
