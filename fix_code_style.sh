#!/bin/sh
astyle \
    --style=kr   \
    --indent=spaces=2  \
    --convert-tabs \
    --indent-switches    \
    --indent-preprocessor \
    --pad-oper      \
    --pad-header    \
    --unpad-paren   \
    --keep-one-line-blocks  \
    --keep-one-line-statements  \
    --align-pointer=type  \
    --suffix=none   \
    --lineend=linux   \
    --add-brackets    \
    $*

# options that don't seem to be supported by astyle 2.01
# --attach-extern-c --align-reference=name --ignore-exclude-errors-x --max-code-length=120
